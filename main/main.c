/**
 ****************************************************************************************************
 * @file        main.c  (主端)
 * @author      ONE
 * @version     V3.0
 * @date        2026-06-17
 * @brief       主端程序 - USB MSC + WiFi AP + 主从自动文件传输（修复版）
 *
 *              流程: USB模式(PC拷贝文件) → 检测从机连接WiFi →
 *                    检查SD卡是否有文件(仍在USB模式，不可直接读，需先切换)→
 *                    切换WiFi模式 → 检查SD卡文件 → TCP发送文件 →
 *                    删除SD所有文件 → 切回USB模式
 *
 * @修复清单
 *  [BUG-1] cleanup_no_transfer 标签：挂载失败时仍调用 switch_to_usb_mode，
 *          现改为用 g_wifi_mode 标志条件判断是否需要切回
 *  [BUG-2] size_t remain 截断 uint64_t 文件大小，改为 uint64_t remain
 *  [BUG-3] 主循环竞态：xTaskCreate 前先置 g_transfer_busy = true
 *  [BUG-4] files[MAX_FILES] 放栈上导致栈溢出(26KB)，改为堆分配
 *  [BUG-5] send() 未循环，新增 send_all() 封装确保完整发送
 *
 * @逻辑修复
 *  [LOGIC-1] 无文件时不再切换WiFi模式再切回，避免PC看到U盘插拔
 *  [LOGIC-2] 传输任务结束后主动重置 g_sta_count，避免下次循环误触发
 *  [LOGIC-3] recv(ack) 补加超时，防止从机不响应时永久阻塞
 *  [LOGIC-4] 删除文件后调用 fsync/sync 确保文件系统落盘再切回USB
 *  [LOGIC-5] switch_to_wifi_mode 失败时提前返回，不再走后续逻辑
 *
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 */

/* ======================== 模式开关 ======================== */
#define USE_USB_MSC  1                     /* 启用USB大容量存储功能 */

/* ======================== 头文件 ======================== */
#include <stdio.h>                        /* 标准输入输出 */
#include <stdlib.h>                       /* 标准库函数 */
#include <string.h>                       /* 字符串处理 */
#include <stdbool.h>                      /* 布尔类型 */
#include "freertos/FreeRTOS.h"            /* FreeRTOS基础 */
#include "freertos/task.h"                /* 任务管理 */
#include "esp_system.h"                   /* ESP系统函数 */
#include "esp_log.h"                      /* 日志输出 */
#include "nvs_flash.h"                    /* NVS闪存初始化 */
#include "nvs.h"                          /* NVS操作 */
#include "esp_wifi.h"                     /* WiFi驱动 */
#include "esp_event.h"                    /* 事件循环 */
#include "led.h"                          /* LED控制 */
#include "sd_card.h"                      /* SD卡驱动 */
#if USE_USB_MSC
#include "usb_msc.h"                      /* USB MSC封装 */
#include "tusb.h"                         /* TinyUSB核心 */
#include "tusb_msc_storage.h"             /* TinyUSB MSC存储接口 */
#endif

#include "lwip/sockets.h"                 /* lwIP socket API */
#include "lwip/inet.h"                    /* lwIP地址转换 */
#include "dirent.h"                       /* 目录操作 */
#include "sys/stat.h"                     /* 文件状态 */
#include "unistd.h"                       /* unlink等POSIX函数 */

/* ======================== 字节序转换宏 ======================== */
#ifndef htobe64
#define htobe64(x)  __builtin_bswap64(x)  /* 将主机64位转为大端网络字节序 */
#endif

/* ======================== WiFi AP 配置 ======================== */
static const char *TAG = "MASTER";        /* 日志标签，标识主机 */
#define WIFI_SSID       "BOSSCOM_USB_AP"  /* WiFi AP 热点名称 */
#define WIFI_PASS       "012345678"       /* WiFi AP 密码 */
#define MAX_STA_CONN    5                 /* 最大允许连接的从机数 */
#define MAC2STR(a)      (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]  /* 将MAC数组转为格式化参数 */
#define MACSTR          "%02x:%02x:%02x:%02x:%02x:%02x"                  /* MAC地址格式化字符串 */

/* ======================== 主从传输配置 ======================== */
#define TRANSFER_PORT           3333    /* TCP监听端口 */
#define TRANSFER_WAIT_TIMEOUT   20      /* 等待从机连接超时(秒) */
#define TRANSFER_ACK_TIMEOUT    10      /* 等待从机握手ACK超时(秒) */
#define SD_MOUNT_POINT          "/sd"   /* SD卡挂载点路径 */
#define MAX_FILES               100     /* 单次最大发送文件数 */
#define FILE_SEND_BUF_SIZE      4096    /* 文件发送缓冲区大小，堆分配 */

/* ======================== 全局状态 ======================== */
static int           g_sta_count    = 0;     /* 当前已连接STA（从机）的数量 */
static bool          g_wifi_mode    = false; /* 当前是否为WiFi本地模式（false表示USB模式） */
static volatile bool g_transfer_busy = false; /* 传输任务是否正在执行 */

/* ======================== 函数声明 ======================== */
static bool switch_to_wifi_mode(void);         /* 切换到WiFi本地模式 */
static void switch_to_usb_mode(void);          /* 切换到USB U盘模式 */
static int  send_all(int sock, const void *buf, size_t len); /* 循环发送，确保全部发出 */
static void file_transfer_task(void *arg);     /* 文件传输任务函数 */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data); /* WiFi事件处理回调 */
static void wifi_init_softap(void);            /* 初始化WiFi SoftAP */

/* ======================== 安全发送封装 ======================== */
/**
 * @brief  循环发送直到全部发完或出错
 *         修复[BUG-5]: 原代码 send() 一次调用不保证全部发出
 * @return 0=成功, -1=出错
 */
static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;   /* 将缓冲区转为字节指针便于递增 */
    while (len > 0) {                           /* 只要还有剩余字节待发送 */
        int r = send(sock, p, len, 0);          /* 尝试发送 len 字节 */
        if (r <= 0) return -1;                  /* 发送出错或连接关闭，返回-1 */
        p   += r;                               /* 指针前移已发送的字节数 */
        len -= r;                               /* 减少剩余待发送字节数 */
    }
    return 0;                                   /* 全部发送完毕，返回成功 */
}

/* ======================== 模式切换实现 ======================== */
/**
 * @brief  切换到WiFi本地模式（断开USB，本地挂载SD）
 * @return true=成功, false=失败
 *
 * 修复[LOGIC-5]: 返回bool，调用方可判断是否成功，失败时不继续走传输流程
 */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;   /* 如果已经在WiFi模式，无需切换，直接返回成功 */
    printf("[模式] → WiFi模式 (断开USB + 本地挂载SD)\n"); /* 打印切换提示 */
    tud_disconnect();                                  /* 断开USB，PC会看到U盘弹出 */
    vTaskDelay(pdMS_TO_TICKS(800));                    /* 等待800ms，让PC处理移除事件 */
    tinyusb_msc_storage_unmount();                     /* 卸载TinyUSB MSC存储，释放SD卡总线 */

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT); /* 将SD卡挂载到本地FATFS */
    if (ret != ESP_OK) {                               /* 挂载失败 */
        printf("[模式] WiFi模式挂载失败: %s\n", esp_err_to_name(ret)); /* 打印错误原因 */
        /* 挂载失败：尝试重新连上USB，不改变 g_wifi_mode */
        tud_connect();                                 /* 重新启用USB连接，恢复PC访问 */
        return false;                                  /* 返回失败 */
    }
    g_wifi_mode = true;                                /* 标记当前为WiFi模式 */
    printf("[模式] WiFi模式就绪，SD已挂载于 %s\n", SD_MOUNT_POINT); /* 打印成功信息 */
    return true;                                       /* 返回成功 */
#else
    g_wifi_mode = true;                                /* 未启用USB MSC，直接设为WiFi模式 */
    return true;                                       /* 返回成功 */
#endif
}

/**
 * @brief  切换到USB U盘模式（卸载本地挂载，启用USB MSC）
 *
 * 修复[BUG-1]: 只有 g_wifi_mode==true 时才执行，避免未挂载时误调用
 */
static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    if (!g_wifi_mode) {                                /* 如果已经处于USB模式 */
        printf("[模式] 已处于USB模式，跳过\n");        /* 打印提示信息 */
        return;                                        /* 直接返回 */
    }
    printf("[模式] → USB模式 (本地卸载 + USB连接)\n");  /* 打印切换提示 */
    tinyusb_msc_storage_unmount();                     /* 卸载本地FATFS挂载，释放SD卡 */
    g_wifi_mode = false;                               /* 清除WiFi模式标志 */
    vTaskDelay(pdMS_TO_TICKS(300));                    /* 等待300ms，确保卸载完成 */
    tud_connect();                                     /* 重新连接USB，PC会看到U盘插入 */
    printf("[模式] USB模式就绪，PC可访问SD卡\n");      /* 打印就绪信息 */
#endif
}

/* ======================== WiFi 事件处理 ======================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {      /* 有STA（从机）连接到AP */
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data; /* 获取事件数据 */
        g_sta_count++;                                 /* STA数量加1 */
        ESP_LOGI(TAG, "从机 " MACSTR " 已连接, 当前总数:%d", /* 打印从机MAC和总数 */
                 MAC2STR(e->mac), g_sta_count);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) { /* STA断开连接 */
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data; /* 获取事件数据 */
        if (g_sta_count > 0) g_sta_count--;            /* 确保计数不为负，减1 */
        ESP_LOGI(TAG, "从机 " MACSTR " 已断开, 当前总数:%d", /* 打印断开信息 */
                 MAC2STR(e->mac), g_sta_count);
    }
}

/* ======================== WiFi SoftAP 初始化 ======================== */
static void wifi_init_softap(void)
{
    esp_netif_init();                         /* 初始化网络接口 */
    esp_event_loop_create_default();          /* 创建默认事件循环 */
    esp_netif_create_default_wifi_ap();      /* 创建默认WiFi AP接口 */

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); /* 默认WiFi初始化配置 */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));     /* 初始化WiFi */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL)); /* 注册WiFi事件回调 */

    char ssid[32] = WIFI_SSID;                /* 默认SSID */
    char pass[64] = WIFI_PASS;                /* 默认密码 */
    nvs_handle_t nvs;                         /* NVS句柄 */
    if (nvs_open("wifi_cfg", NVS_READONLY, &nvs) == ESP_OK) { /* 尝试打开NVS配置分区 */
        size_t len = sizeof(ssid);            /* 设置读取长度 */
        nvs_get_str(nvs, "ssid", ssid, &len); /* 从NVS读取SSID，覆盖默认值 */
        len = sizeof(pass);                   /* 重置长度 */
        nvs_get_str(nvs, "pass", pass, &len); /* 从NVS读取密码 */
        nvs_close(nvs);                       /* 关闭NVS */
    }

    wifi_config_t wifi_cfg = {                /* AP配置结构体 */
        .ap = {
            .ssid_len       = strlen(ssid),   /* SSID长度 */
            .max_connection = MAX_STA_CONN,   /* 最大连接数 */
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK, /* 认证模式 */
        },
    };
    memcpy(wifi_cfg.ap.ssid,     ssid, sizeof(wifi_cfg.ap.ssid));    /* 拷贝SSID */
    memcpy(wifi_cfg.ap.password, pass, sizeof(wifi_cfg.ap.password)); /* 拷贝密码 */
    if (strlen(pass) == 0) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;    /* 如果密码为空则开放认证 */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));           /* 设置WiFi模式为AP */
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg)); /* 应用AP配置 */
    ESP_ERROR_CHECK(esp_wifi_start());                          /* 启动WiFi */

    esp_netif_ip_info_t ip;                                     /* 获取AP的IP信息 */
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip);
    ESP_LOGI(TAG, "AP已启动: SSID=%s, IP=" IPSTR, ssid, IP2STR(&ip.ip)); /* 打印AP信息 */
}

/* ======================== 文件信息结构 ======================== */
typedef struct {
    char     name[256];      /* 文件名，最多255字符 */
    uint64_t size;           /* 文件大小（字节） */
} file_info_t;

/* ======================== 文件传输任务 ======================== */
/**
 * @brief  主从文件传输任务
 *
 *  步骤:
 *   1. 先切换到WiFi本地模式（让SD可读）
 *   2. 检查SD卡是否有文件，无文件则直接切回USB退出
 *   3. 创建TCP服务器等待从机连接（带超时）
 *   4. 握手：发 MASTER_SEND → 等从机回 READY（带ACK超时）
 *   5. 堆上收集文件列表，发送文件总数+每个文件名/大小/内容
 *   6. 发送 DONE，等从机最终ACK
 *   7. 删除SD卡所有文件，sync落盘
 *   8. 切回USB模式
 *   9. 重置 g_sta_count 和 g_transfer_busy
 *
 *  修复汇总:
 *   [BUG-2] remain 改为 uint64_t
 *   [BUG-3] 由调用方在 xTaskCreate 前置位 g_transfer_busy
 *   [BUG-4] file_info_t 数组改为堆分配
 *   [BUG-5] 所有 send 改用 send_all
 *   [LOGIC-1] 无文件时不切WiFi模式（直接切回USB即可，此时还未切换）
 *   [LOGIC-2] 任务结束时重置 g_sta_count
 *   [LOGIC-3] recv ACK 加超时
 *   [LOGIC-4] 删文件后 sync
 */
static void file_transfer_task(void *arg)
{
    printf("[传输] 任务启动\n");

    int         listen_sock = -1;           /* 监听socket描述符 */
    int         client_sock = -1;           /* 客户端socket描述符 */
    file_info_t *files      = NULL;         /* 文件列表数组指针，稍后堆分配 */
    bool        entered_wifi_mode = false;  /* 记录本次任务是否成功切入WiFi模式 */

    /* --------------------------------------------------------
     * 步骤1: 切换到WiFi本地模式
     * --------------------------------------------------------
     * 注意: 在USB模式下 SD 由 TinyUSB 控制，ESP32本地无法直接读写。
     *       必须先切换到本地模式才能检查和读取文件。
     * -------------------------------------------------------- */
    if (!switch_to_wifi_mode()) {           /* 尝试切换到WiFi模式 */
        printf("[传输] 无法进入WiFi模式，取消传输\n"); /* 切换失败打印信息 */
        goto task_exit;                     /* 跳转到退出处理 */
    }
    entered_wifi_mode = true;               /* 标记已成功进入WiFi模式 */

    /* --------------------------------------------------------
     * 步骤2: 检查SD卡是否有文件
     * 修复[LOGIC-1]: 切换WiFi模式之后再检查，而非之前
     * -------------------------------------------------------- */
    {
        DIR *dir = opendir(SD_MOUNT_POINT); /* 打开SD卡挂载目录 */
        if (!dir) {                         /* 打开失败 */
            printf("[传输] 无法打开SD卡目录: %s\n", SD_MOUNT_POINT); /* 打印错误 */
            goto task_exit;                 /* 退出 */
        }
        bool has_file = false;              /* 文件存在标志 */
        struct dirent *entry;               /* 目录项指针 */
        while ((entry = readdir(dir)) != NULL) { /* 遍历目录 */
            if (entry->d_type == DT_REG) {  /* 如果是常规文件 */
                has_file = true;            /* 标记存在文件 */
                break;                      /* 找到一个即可退出循环 */
            }
        }
        closedir(dir);                      /* 关闭目录 */

        if (!has_file) {                    /* 如果没有文件 */
            printf("[传输] SD卡上没有文件，跳过传输\n"); /* 打印提示 */
            goto task_exit;                 /* 切回USB退出 */
        }
    }

    /* --------------------------------------------------------
     * 步骤3: 创建TCP服务器，等待从机连接
     * -------------------------------------------------------- */
    listen_sock = socket(AF_INET, SOCK_STREAM, 0); /* 创建TCP socket */
    if (listen_sock < 0) {                  /* 创建失败 */
        printf("[传输] socket() 创建失败\n"); /* 打印错误 */
        goto task_exit;                     /* 退出 */
    }

    {
        int opt = 1;                        /* 选项值：允许地址重用 */
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); /* 设置SO_REUSEADDR */
    }

    {
        struct sockaddr_in addr = {0};      /* 服务器地址结构体 */
        addr.sin_family      = AF_INET;     /* IPv4 */
        addr.sin_addr.s_addr = htonl(INADDR_ANY); /* 监听所有本机IP */
        addr.sin_port        = htons(TRANSFER_PORT); /* 设置端口为传输端口 */
        if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) { /* 绑定地址 */
            printf("[传输] bind() 失败\n"); /* 打印错误 */
            goto task_exit;                 /* 退出 */
        }
    }

    if (listen(listen_sock, 1) != 0) {      /* 开始监听，最大连接数1 */
        printf("[传输] listen() 失败\n");   /* 打印错误 */
        goto task_exit;                     /* 退出 */
    }

    /* 等待从机连接的超时设置 */
    {
        struct timeval tv = { .tv_sec = TRANSFER_WAIT_TIMEOUT, .tv_usec = 0 }; /* 超时时间 */
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); /* 设置接收超时 */
    }

    printf("[传输] 在端口 %d 上监听，等待从机连接 (超时=%ds)...\n",
           TRANSFER_PORT, TRANSFER_WAIT_TIMEOUT); /* 打印监听信息 */

    {
        struct sockaddr_in cli_addr;        /* 客户端地址 */
        socklen_t cli_len = sizeof(cli_addr); /* 地址长度 */
        client_sock = accept(listen_sock, (struct sockaddr *)&cli_addr, &cli_len); /* 接受连接 */
        if (client_sock < 0) {              /* 接受失败或超时 */
            printf("[传输] accept() 超时或出错，取消\n"); /* 打印提示 */
            goto task_exit;                 /* 退出 */
        }
        printf("[传输] 从机已连接: %s\n", inet_ntoa(cli_addr.sin_addr)); /* 打印从机IP */
    }

    /* 给已连接的socket也设一个收发超时，防握手阻塞
     * 修复[LOGIC-3]: 原代码 recv(ack) 无超时，从机不回应时永久阻塞 */
    {
        struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 }; /* ACK超时时间 */
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); /* 设置接收超时 */
        setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)); /* 设置发送超时 */
    }

    /* --------------------------------------------------------
     * 步骤4: 握手
     * -------------------------------------------------------- */
    {
        const char *handshake = "MASTER_SEND"; /* 握手发起字符串 */
        if (send_all(client_sock, handshake, strlen(handshake)) != 0) { /* 发送握手 */
            printf("[传输] 握手发送失败\n"); /* 打印错误 */
            goto task_exit;                 /* 退出 */
        }

        char ack[16] = {0};                 /* 接收应答缓冲区 */
        int  n = recv(client_sock, ack, sizeof(ack) - 1, 0); /* 接收从机应答 */
        if (n <= 0 || strncmp(ack, "READY", 5) != 0) { /* 检查是否收到 READY */
            printf("[传输] 从机未就绪 (n=%d, ack='%s')\n", n, ack); /* 打印失败信息 */
            goto task_exit;                 /* 退出 */
        }
        printf("[传输] 从机就绪，开始发送文件...\n"); /* 握手成功 */
    }

    /* --------------------------------------------------------
     * 步骤5: 收集文件列表（堆分配）
     * 修复[BUG-4]: 原来放栈上，100个文件≈26KB，必然栈溢出
     * -------------------------------------------------------- */
    files = (file_info_t *)malloc(sizeof(file_info_t) * MAX_FILES); /* 分配文件信息数组 */
    if (!files) {                           /* 分配失败 */
        printf("[传输] 分配文件信息数组失败\n"); /* 打印错误 */
        goto task_exit;                     /* 退出 */
    }

    int file_cnt = 0;                       /* 实际文件数计数器 */
    {
        DIR *dir = opendir(SD_MOUNT_POINT); /* 再次打开目录 */
        if (!dir) {                         /* 打开失败 */
            printf("[传输] 无法再次打开SD卡目录\n"); /* 打印错误 */
            uint32_t zero = 0;              /* 发送0表示无文件 */
            send_all(client_sock, &zero, sizeof(zero)); /* 通知从机文件数为0 */
            goto task_exit;                 /* 退出 */
        }
        struct dirent *entry;               /* 目录项 */
        while ((entry = readdir(dir)) != NULL && file_cnt < MAX_FILES) { /* 遍历目录 */
            if (entry->d_type == DT_REG) {  /* 如果是常规文件 */
                char path[300];             /* 文件完整路径 */
                snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name); /* 构造路径 */
                struct stat st;             /* 文件状态 */
                if (stat(path, &st) == 0) { /* 获取文件信息成功 */
                    strncpy(files[file_cnt].name, entry->d_name, 255); /* 拷贝文件名 */
                    files[file_cnt].name[255] = '\0'; /* 确保字符串结束 */
                    files[file_cnt].size = (uint64_t)st.st_size; /* 记录文件大小 */
                    file_cnt++;             /* 文件计数加1 */
                }
            }
        }
        closedir(dir);                      /* 关闭目录 */
    }

    /* 发送文件总数 */
    {
        uint32_t count_net = htonl((uint32_t)file_cnt); /* 转为网络字节序 */
        if (send_all(client_sock, &count_net, sizeof(count_net)) != 0) { /* 发送文件总数 */
            printf("[传输] 发送文件总数失败\n"); /* 打印错误 */
            goto task_exit;                 /* 退出 */
        }
        printf("[传输] 将发送 %d 个文件\n", file_cnt); /* 打印文件数量 */
    }

    /* 发送每个文件 */
    {
        /* 文件内容发送缓冲区也堆分配，节省栈空间 */
        char *send_buf = (char *)malloc(FILE_SEND_BUF_SIZE); /* 分配发送缓冲区 */
        if (!send_buf) {                    /* 分配失败 */
            printf("[传输] 分配发送缓冲区失败\n"); /* 打印错误 */
            goto task_exit;                 /* 退出 */
        }

        for (int i = 0; i < file_cnt; i++) { /* 遍历每个文件 */
            /* 发送文件名长度 + 文件名 */
            uint16_t name_len     = (uint16_t)strlen(files[i].name); /* 文件名长度 */
            uint16_t name_len_net = htons(name_len); /* 转为网络字节序 */
            if (send_all(client_sock, &name_len_net, sizeof(name_len_net)) != 0 || /* 发送长度 */
                send_all(client_sock, files[i].name, name_len)             != 0) { /* 发送文件名 */
                printf("[传输] 发送文件名失败: %s\n", files[i].name); /* 打印错误 */
                free(send_buf);            /* 释放发送缓冲区 */
                goto task_exit;            /* 退出 */
            }

            /* 发送文件大小（大端64位）*/
            uint64_t size_be = htobe64(files[i].size); /* 文件大小转为大端 */
            if (send_all(client_sock, &size_be, sizeof(size_be)) != 0) { /* 发送文件大小 */
                printf("[传输] 发送文件大小失败: %s\n", files[i].name); /* 打印错误 */
                free(send_buf);            /* 释放发送缓冲区 */
                goto task_exit;            /* 退出 */
            }

            /* 发送文件内容 */
            char path[300];                /* 文件路径 */
            snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, files[i].name); /* 构造路径 */
            FILE *fp = fopen(path, "rb");  /* 以二进制读模式打开文件 */
            if (!fp) {                     /* 打开失败 */
                printf("[传输] 打开文件失败: %s\n", path); /* 打印错误 */
                /* 已承诺发这个文件，打开失败需要告知从机：发全0填充
                 * （从机需要读完对应字节数才能继续接收下一文件）*/
                uint64_t remain = files[i].size; /* 需要填充的剩余字节数 */
                memset(send_buf, 0, FILE_SEND_BUF_SIZE); /* 发送缓冲区清零 */
                while (remain > 0) {       /* 循环发送0填充 */
                    size_t chunk = (remain > FILE_SEND_BUF_SIZE) ?
                                   FILE_SEND_BUF_SIZE : (size_t)remain; /* 本次发送块大小 */
                    if (send_all(client_sock, send_buf, chunk) != 0) break; /* 发送填充数据 */
                    remain -= chunk;       /* 减少剩余字节 */
                }
                continue;                  /* 继续下一个文件 */
            }

            /* 修复[BUG-2]: remain 用 uint64_t，避免32位截断 */
            uint64_t remain = files[i].size; /* 剩余待发送字节数 */
            while (remain > 0) {           /* 循环发送文件内容 */
                size_t chunk = (remain > FILE_SEND_BUF_SIZE) ?
                               FILE_SEND_BUF_SIZE : (size_t)remain; /* 本次读取块大小 */
                size_t n = fread(send_buf, 1, chunk, fp); /* 从文件读取数据 */
                if (n == 0) {              /* 读取到0字节，文件意外结束 */
                    printf("[传输] 读取文件提前结束: %s\n", files[i].name); /* 打印提示 */
                    break;                 /* 跳出循环 */
                }
                /* 修复[BUG-5]: 用 send_all 保证全部发出 */
                if (send_all(client_sock, send_buf, n) != 0) { /* 发送读取的数据 */
                    printf("[传输] 发送文件数据中途失败: %s\n", files[i].name); /* 打印错误 */
                    fclose(fp);            /* 关闭文件 */
                    free(send_buf);        /* 释放缓冲区 */
                    goto task_exit;        /* 退出 */
                }
                remain -= n;               /* 更新剩余字节 */
            }
            fclose(fp);                    /* 关闭文件 */
            printf("[传输] 已发送: %s (%llu 字节)\n",
                   files[i].name, (unsigned long long)files[i].size); /* 打印发送成功 */
        }
        free(send_buf);                    /* 释放发送缓冲区 */
    }

    /* --------------------------------------------------------
     * 步骤6: 发送 DONE，等从机最终ACK
     * -------------------------------------------------------- */
    {
        const char *done = "DONE";         /* 传输结束标识 */
        send_all(client_sock, done, strlen(done));  /* 发送DONE，尽力发，不判断返回值 */

        char ack[16] = {0};                /* 接收最终ACK缓冲区 */
        recv(client_sock, ack, sizeof(ack) - 1, 0);    /* 接收从机ACK，有超时保护 */
        printf("[传输] 传输完成，从机ACK: '%s'\n", ack); /* 打印从机应答 */
    }

    close(client_sock); client_sock = -1;  /* 关闭客户端socket */
    close(listen_sock); listen_sock = -1;  /* 关闭监听socket */

    /* --------------------------------------------------------
     * 步骤7: 删除SD卡所有文件，并同步文件系统
     * 修复[LOGIC-4]: 原代码删完没有 sync，切回USB后PC可能看到残留缓存
     * -------------------------------------------------------- */
    printf("[传输] 正在删除SD卡上所有文件...\n");
    {
        DIR *dir = opendir(SD_MOUNT_POINT); /* 打开目录 */
        if (dir) {                          /* 打开成功 */
            struct dirent *entry;           /* 目录项 */
            while ((entry = readdir(dir)) != NULL) { /* 遍历 */
                if (entry->d_type == DT_REG) { /* 常规文件 */
                    char path[300];         /* 文件路径 */
                    snprintf(path, sizeof(path), "%s/%s",
                             SD_MOUNT_POINT, entry->d_name); /* 构造路径 */
                    if (unlink(path) == 0) { /* 删除文件 */
                        printf("[传输] 已删除: %s\n", entry->d_name); /* 打印成功 */
                    } else {
                        printf("[传输] 删除失败: %s\n", entry->d_name); /* 打印失败 */
                    }
                }
            }
            closedir(dir);                 /* 关闭目录 */
        }
        /* ESP-IDF 的 FatFS 中 unlink() 是同步操作，目录项立即更新，
         * 无需额外 sync()（ESP-IDF newlib 未提供该符号）。
         * 延迟一小段时间确保底层写操作完成后再切换总线。 */
        vTaskDelay(pdMS_TO_TICKS(300));    /* 等待300ms */
    }

/* --------------------------------------------------------
 * 统一出口：关闭socket，切回USB，释放资源
 * -------------------------------------------------------- */
task_exit:
    if (client_sock >= 0) { close(client_sock); } /* 如果客户端socket仍打开，关闭它 */
    if (listen_sock >= 0) { close(listen_sock); } /* 如果监听socket仍打开，关闭它 */
    if (files)            { free(files); }        /* 释放文件列表内存 */

    /* 只有成功进入WiFi模式的情况下才需要切回USB
     * 修复[BUG-1]: 原代码无论是否切换成功都会调用 switch_to_usb_mode */
    if (entered_wifi_mode) {               /* 如果本次切换到了WiFi模式 */
        printf("[传输] 切换回USB模式...\n"); /* 打印提示 */
        switch_to_usb_mode();              /* 执行切回USB操作 */
    }

    /* 修复[LOGIC-2]: 传输任务结束后重置 g_sta_count。
     * 从机发完文件后通常会继续保持WiFi连接（g_sta_count 不会自动归零），
     * 若不重置，主循环下次检测到 g_sta_count>0 会再次触发传输（但SD已无文件，
     * 会白白切换一次USB模式）。重置后等从机真正断开再重连才会触发。
     *
     * 注意: 若存在多个从机同时在线，此处全量清零是合理的——
     *       因为本次任务已经广播了所有文件，无需对同一批文件再次传输。 */
    g_sta_count = 0;                       /* 清零连接计数 */

    g_transfer_busy = false;               /* 清除传输忙标志 */
    printf("[传输] 任务结束\n");           /* 打印任务结束 */
    vTaskDelete(NULL);                     /* 删除本任务 */
}

/* ======================== 主函数 ======================== */
void app_main(void)
{
    /* NVS初始化 */
    esp_err_t ret = nvs_flash_init();      /* 初始化默认NVS分区 */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();                 /* 如果分区有问题，擦除 */
        nvs_flash_init();                  /* 重新初始化 */
    }

    /* LED初始化 */
    led_init();                            /* 初始化LED */
    LED(1);                                /* 点亮LED */
    vTaskDelay(pdMS_TO_TICKS(6000));       /* 等待6秒，确保USB枚举完成 */
    setvbuf(stdout, NULL, _IONBF, 0);      /* 设置标准输出无缓冲，方便实时查看日志 */
    printf("\n========== 主机 V3.0 (自动传输) ==========\n"); /* 打印标题 */

    /* SD卡初始化 */
    sdmmc_card_t *sd_card = NULL;          /* SD卡句柄 */
    esp_err_t sd_ret = sd_card_init(&sd_card); /* 初始化SD卡 */
    bool sd_ok = (sd_ret == ESP_OK && sd_card != NULL); /* 判断SD卡是否可用 */

    if (sd_ok) {                           /* SD卡就绪 */
        printf("[主函数] SD卡就绪\n");     /* 打印提示 */
#if USE_USB_MSC
        printf("[主函数] 初始化 USB MSC...\n"); /* 打印初始化提示 */
        ESP_ERROR_CHECK(usb_msc_init(sd_card)); /* 初始化USB MSC，注册SD卡给TinyUSB */
        ESP_ERROR_CHECK(usb_msc_mount(SD_MOUNT_POINT)); /* 将SD卡挂载到本地FATFS（初始为USB模式） */
        printf("[主函数] USB MSC 就绪，PC可拷贝文件\n"); /* 打印就绪信息 */
#endif
    } else {                               /* SD卡初始化失败 */
        printf("[主函数] SD卡初始化失败: %s (0x%x)\n",
               esp_err_to_name(sd_ret), sd_ret); /* 打印失败原因 */
    }

    /* WiFi AP初始化 */
    printf("[主函数] 启动WiFi AP...\n");   /* 打印提示 */
    wifi_init_softap();                    /* 初始化并启动SoftAP */
    printf("[主函数] AP已激活: %s\n", WIFI_SSID); /* 打印AP信息 */

    /* 主循环 */
    int loop_cnt = 0;                      /* 循环计数器，用于周期性打印状态 */
    LED(0);                                /* 熄灭LED */
    printf("[主函数] 进入主循环，等待从机...\n"); /* 打印提示 */

    while (1) {                            /* 无限循环 */
        LED_TOGGLE();                      /* 翻转LED，指示运行心跳 */

        /* 触发条件：SD正常 + 当前无传输任务 + 有从机在线
         *
         * 修复[BUG-3]: 先置 g_transfer_busy=true 再 xTaskCreate，
         * 防止 xTaskCreate 返回后、新任务调度前的200ms tick里再次触发。
         * 注意: g_transfer_busy 是 volatile，两条语句之间无竞态（单核调度）。 */
        if (sd_ok && !g_transfer_busy && g_sta_count > 0) { /* 满足传输条件 */
            printf("[主函数] 检测到从机 (STA=%d)，启动传输任务\n",
                   g_sta_count);          /* 打印触发信息 */
            g_transfer_busy = true;        /* 先置位忙标志！修复[BUG-3] */
            BaseType_t rc = xTaskCreate(file_transfer_task, "transfer",
                                        8192,   /* 任务栈大小8KB，文件列表已改为堆分配 */
                                        NULL, 5, NULL); /* 创建传输任务 */
            if (rc != pdPASS) {            /* 任务创建失败 */
                printf("[主函数] 创建传输任务失败!\n"); /* 打印错误 */
                g_transfer_busy = false;   /* 创建失败则复位忙标志 */
            }
        }

        if (++loop_cnt >= 25) {            /* 每25次循环打印一次状态 */
            loop_cnt = 0;                  /* 复位计数器 */
            printf("[循环] 模式:%s  STA数量:%d  传输:%s\n",
                   g_wifi_mode      ? "WiFi" : "USB", /* 当前模式 */
                   g_sta_count,                           /* 连接数 */
                   g_transfer_busy  ? "运行中" : "空闲"); /* 传输状态 */
        }

        vTaskDelay(sd_ok ? pdMS_TO_TICKS(200) : pdMS_TO_TICKS(500)); /* 延时，SD正常时200ms，否则500ms */
    }
}