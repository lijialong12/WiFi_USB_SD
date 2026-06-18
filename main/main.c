/**
 ****************************************************************************************************
 * @file        main.c  (从端)
 * @author      ONE
 * @version     V2.0
 * @date        2026-06-17
 * @brief       从端程序 - USB MSC + WiFi STA + 主从自动文件接收
 *
 *              流程: 上电 → USB U盘模式待机(PC可访问SD) →
 *                    连接主机WiFi热点 → 轮询TCP连接主机 →
 *                    TCP连上后：切换到WiFi本地模式(USB断开) →
 *                    握手 → 接收文件写入SD → 回ACK →
 *                    切回USB U盘模式 → 等待下一轮
 *
 *              关键点: 只有在实际传输期间才切换到本地模式，
 *                      平时保持USB U盘模式，PC可以正常访问SD卡。
 *
 * @协议说明（与主端对齐）
 *  主→从: "MASTER_SEND"        (握手发起，11字节)
 *  从→主: "READY"              (握手应答，5字节)
 *  主→从: uint32_t file_count  (大端，文件总数)
 *  循环 file_count 次:
 *    主→从: uint16_t name_len  (大端，文件名字节数)
 *    主→从: char name[name_len](文件名，无终止符)
 *    主→从: uint64_t file_size (大端，文件字节数)
 *    主→从: uint8_t data[file_size] (文件内容)
 *  主→从: "DONE"               (所有文件发完，4字节)
 *  从→主: "ACK"                (最终确认，3字节)
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
#include "freertos/event_groups.h"        /* 事件组 */
#include "esp_system.h"                   /* ESP系统函数 */
#include "esp_log.h"                      /* 日志输出 */
#include "esp_wifi.h"                     /* WiFi驱动 */
#include "esp_event.h"                    /* 事件循环 */
#include "nvs_flash.h"                    /* NVS闪存初始化 */
#include "nvs.h"                          /* NVS操作 */
#include "led.h"                          /* LED控制 */
#include "sd_card.h"                      /* SD卡驱动 */
#if USE_USB_MSC
#include "usb_msc.h"                      /* USB MSC封装 */
#include "tusb.h"                         /* TinyUSB核心 */
#include "tusb_msc_storage.h"             /* TinyUSB MSC存储接口 */
#endif

#include "lwip/sockets.h"                 /* lwIP socket API */
#include "lwip/inet.h"                    /* lwIP地址转换 */
#include "sys/stat.h"                     /* 文件状态 */
#include "unistd.h"                       /* unlink等POSIX函数 */

/* ======================== 字节序转换宏 ======================== */
#ifndef be64toh
#define be64toh(x)  __builtin_bswap64(x)  /* 将大端64位转为主机字节序 */
#endif

/* ======================== WiFi STA 配置 ======================== */
static const char *TAG = "SLAVE";         /* 日志标签，标识从机 */

#define MASTER_SSID             "BOSSCOM_USB_AP"  /* 主机热点名，需与主端一致 */
#define MASTER_PASS             "012345678"        /* 主机热点密码 */
#define MASTER_IP               "192.168.4.1"      /* 主机固定IP（SoftAP默认） */
#define MASTER_PORT             3333               /* 与主端 TRANSFER_PORT 一致 */

#define WIFI_CONNECT_TIMEOUT_S  30                 /* WiFi连接超时(秒) */
#define WIFI_MAX_RETRY          10                 /* STA连接最大重试次数 */
#define TCP_CONNECT_RETRY       5                  /* TCP连接失败重试次数 */
#define TCP_CONNECT_RETRY_MS    2000               /* 每次重试间隔(ms) */
#define TCP_RECV_TIMEOUT_S      30                 /* TCP收数据超时(秒) */

#define SD_MOUNT_POINT          "/sd"              /* SD卡挂载点路径 */
#define FILE_RECV_BUF_SIZE      4096               /* 接收缓冲区大小（堆分配） */

/* ======================== WiFi 事件组 ======================== */
#define WIFI_CONNECTED_BIT  BIT0                   /* 事件位：WiFi已连接 */
#define WIFI_FAIL_BIT       BIT1                   /* 事件位：WiFi连接失败 */

static EventGroupHandle_t s_wifi_event_group = NULL;   /* WiFi事件组句柄 */
static int                s_retry_num        = 0;      /* 当前WiFi重试计数 */

/* ======================== 全局状态 ======================== */
static bool g_sd_ok          = false;         /* SD卡是否初始化成功 */
static bool g_wifi_connected = false;         /* WiFi是否已连接 */
static bool g_wifi_mode      = false;         /* 当前模式：true=WiFi本地挂载, false=USB MSC */

/* ======================== 函数声明 ======================== */
static bool switch_to_wifi_mode(void);        /* 切换到WiFi本地模式 */
static void switch_to_usb_mode(void);         /* 切换回USB U盘模式 */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data); /* WiFi事件处理回调 */
static bool wifi_init_sta(void);              /* 初始化WiFi STA并连接 */
static int  recv_all(int sock, void *buf, size_t len); /* 循环接收直到收满指定字节 */
static bool do_file_receive_after_handshake(int sock); /* 执行文件接收协议 */

/* ======================== 模式切换 ======================== */
/**
 * @brief  切换到WiFi本地模式（断开USB，ESP32本地挂载SD）
 *         在TCP连上、准备接收文件前调用
 * @return true=成功, false=挂载失败
 */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;   /* 已在本地模式，直接返回成功，幂等操作 */
    printf("[模式] → WiFi模式 (断开USB + 本地挂载SD)\n"); /* 打印切换提示 */
    tud_disconnect();                                  /* 断开USB，PC看到U盘移除 */
    vTaskDelay(pdMS_TO_TICKS(800));                    /* 等待PC处理弹出事件，延迟800ms */
    tinyusb_msc_storage_unmount();                     /* 卸载TinyUSB MSC存储，释放SD卡总线 */

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT); /* 将SD卡挂载到本地FATFS */
    if (ret != ESP_OK) {                               /* 判断挂载是否成功 */
        printf("[模式] WiFi模式挂载失败: %s\n", esp_err_to_name(ret)); /* 打印失败原因 */
        tud_connect();                                 /* 挂载失败则重新连接USB，恢复到USB模式 */
        return false;                                  /* 返回失败 */
    }
    g_wifi_mode = true;                                /* 标记当前为WiFi模式 */
    printf("[模式] WiFi模式就绪，SD已挂载于 %s\n", SD_MOUNT_POINT); /* 打印成功信息 */
    return true;                                       /* 返回成功 */
#else
    g_wifi_mode = true;                                /* 未启用USB MSC时，直接设为WiFi模式 */
    return true;                                       /* 返回成功 */
#endif
}

/**
 * @brief  切换回USB U盘模式（卸载本地挂载，重新连接USB）
 *         在文件接收完成后调用
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
    vTaskDelay(pdMS_TO_TICKS(500));                    /* 延迟500ms，确保卸载完成 */
    tud_connect();                                     /* 重新连接USB，PC会看到U盘插入 */
    vTaskDelay(pdMS_TO_TICKS(1000));                    /* 等待1秒，让USB稳定 */
    printf("[模式] USB模式就绪 (connected=%d)\n", tud_connected()); /* 打印就绪信息+状态 */
#endif
}

/* ======================== 安全接收封装 ======================== */
/**
 * @brief  循环接收直到收满 len 字节或出错/超时
 * @return 0=成功, -1=出错或对端关闭
 */
static int recv_all(int sock, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;           /* 将缓冲区指针转为字节指针便于递增 */
    while (len > 0) {                      /* 只要还有剩余字节未接收 */
        int r = recv(sock, p, len, 0);     /* 尝试接收len字节 */
        if (r <= 0) return -1;             /* 接收出错或连接关闭则返回-1 */
        p   += r;                          /* 指针前移已接收的字节数 */
        len -= r;                          /* 剩余待接收字节数减少 */
    }
    return 0;                              /* 全部收完，返回成功 */
}

/* ======================== WiFi 事件处理 ======================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) { /* WiFi STA启动完成事件 */
        esp_wifi_connect();                  /* 立即发起连接 */
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) { /* STA断开连接 */
        g_wifi_connected = false;            /* 更新连接状态为假 */
        if (s_retry_num < WIFI_MAX_RETRY) {  /* 如果重试次数未达上限 */
            esp_wifi_connect();              /* 再次尝试连接 */
            s_retry_num++;                   /* 重试计数加1 */
            ESP_LOGI(TAG, "重试连接WiFi (%d/%d)...", s_retry_num, WIFI_MAX_RETRY); /* 打印重试信息 */
        } else {                             /* 已达最大重试次数 */
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); /* 设置失败事件位 */
            ESP_LOGE(TAG, "WiFi连接失败，已重试%d次", WIFI_MAX_RETRY); /* 打印错误信息 */
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) { /* STA获取到IP地址 */
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data; /* 获取事件数据 */
        ESP_LOGI(TAG, "获取到IP: " IPSTR, IP2STR(&e->ip_info.ip)); /* 打印获得的IP */
        s_retry_num      = 0;               /* 重试计数清零 */
        g_wifi_connected = true;             /* 标记已连接 */
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); /* 设置连接成功事件位 */
    }
}

/* ======================== WiFi STA 初始化 ======================== */
static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); /* 创建事件组 */

    esp_netif_init();                         /* 初始化网络接口 */
    esp_event_loop_create_default();          /* 创建默认事件循环 */
    esp_netif_create_default_wifi_sta();      /* 创建默认WiFi STA接口 */

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); /* 使用默认WiFi初始化配置 */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));     /* 初始化WiFi */

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL)); /* 注册WiFi事件处理 */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL)); /* 注册IP获取事件处理 */

    char ssid[32] = MASTER_SSID;              /* 默认SSID */
    char pass[64] = MASTER_PASS;              /* 默认密码 */
    nvs_handle_t nvs;                         /* NVS句柄 */
    if (nvs_open("wifi_cfg", NVS_READONLY, &nvs) == ESP_OK) { /* 尝试打开NVS配置分区 */
        size_t len = sizeof(ssid);            /* 读取长度变量 */
        nvs_get_str(nvs, "ssid", ssid, &len); /* 从NVS读取SSID */
        len = sizeof(pass);                   /* 重置长度变量 */
        nvs_get_str(nvs, "pass", pass, &len); /* 从NVS读取密码 */
        nvs_close(nvs);                       /* 关闭NVS */
    }

    wifi_config_t wifi_cfg = {                /* 定义WiFi STA配置结构体 */
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK, /* 认证模式 */
            .pmf_cfg = { .capable = true, .required = false }, /* 管理帧保护 */
        },
    };
    strncpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));    /* 拷贝SSID */
    strncpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password)); /* 拷贝密码 */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA)); /* 设置WiFi模式为STA */
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg)); /* 应用STA配置 */
    ESP_ERROR_CHECK(esp_wifi_start());      /* 启动WiFi */

    ESP_LOGI(TAG, "正在连接热点: %s ...", ssid); /* 打印连接提示 */

    EventBits_t bits = xEventGroupWaitBits(  /* 等待连接结果 */
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, /* 等待连接成功或失败位 */
        pdFALSE, pdFALSE,                   /* 不自动清除标志，逻辑或 */
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_S * 1000) /* 超时时间 */
    );

    if (bits & WIFI_CONNECTED_BIT) {         /* 如果连接成功 */
        ESP_LOGI(TAG, "WiFi已连接: %s", ssid); /* 打印成功信息 */
        return true;                         /* 返回成功 */
    }
    ESP_LOGE(TAG, "WiFi连接超时/失败");     /* 打印失败信息 */
    return false;                            /* 返回失败 */
}

/* ======================== 文件接收核心逻辑 ======================== */
/**
 * @brief  握手已在主循环完成后，接收文件数+文件内容+DONE/ACK
 *         调用前：switch_to_wifi_mode() 已成功，READY 已发出
 */
static bool do_file_receive_after_handshake(int sock)
{
    char *recv_buf = NULL;                  /* 接收缓冲区指针，稍后堆分配 */
    FILE *fp       = NULL;                  /* 文件指针 */
    bool  success  = false;                 /* 最终成功标志 */

    /* ---------- 1. 收文件总数 ---------- */
    uint32_t file_count = 0;                /* 文件总数变量 */
    {
        uint32_t count_net = 0;             /* 网络字节序的文件数 */
        if (recv_all(sock, &count_net, sizeof(count_net)) != 0) { /* 接收4字节文件数 */
            ESP_LOGE(TAG, "接收文件总数失败"); /* 打印错误 */
            goto recv_exit;                 /* 跳转到退出清理 */
        }
        file_count = ntohl(count_net);       /* 转换为主机字节序 */
        ESP_LOGI(TAG, "期望接收 %lu 个文件", (unsigned long)file_count); /* 打印文件数量 */
        if (file_count == 0) {               /* 如果文件数为0 */
            success = true;                  /* 视为正常结束（无文件） */
            goto recv_exit;                  /* 跳转到退出 */
        }
    }

    /* ---------- 3. 堆分配接收缓冲区 ---------- */
    recv_buf = (char *)malloc(FILE_RECV_BUF_SIZE); /* 分配接收缓冲区 */
    if (!recv_buf) {                         /* 分配失败 */
        ESP_LOGE(TAG, "malloc recv_buf 失败"); /* 打印错误 */
        goto recv_exit;                      /* 跳转到退出 */
    }

    /* ---------- 4. 循环接收每个文件 ---------- */
    for (uint32_t i = 0; i < file_count; i++) { /* 遍历每个文件 */

        /* 4a. 收文件名长度 */
        uint16_t name_len_net = 0;           /* 网络字节序的文件名长度 */
        if (recv_all(sock, &name_len_net, sizeof(name_len_net)) != 0) { /* 接收2字节长度 */
            ESP_LOGE(TAG, "接收文件名长度失败，第%lu个文件", (unsigned long)i); /* 打印错误 */
            goto recv_exit;                  /* 退出 */
        }
        uint16_t name_len = ntohs(name_len_net); /* 转换为主机字节序 */
        if (name_len == 0 || name_len > 255) {   /* 验证文件名长度合法性 */
            ESP_LOGE(TAG, "无效的文件名长度=%u", name_len); /* 打印错误 */
            goto recv_exit;                  /* 退出 */
        }

        /* 4b. 收文件名 */
        char file_name[256] = {0};           /* 文件名缓冲区，初始化为0 */
        if (recv_all(sock, file_name, name_len) != 0) { /* 接收文件名 */
            ESP_LOGE(TAG, "接收文件名失败，第%lu个文件", (unsigned long)i); /* 打印错误 */
            goto recv_exit;                  /* 退出 */
        }
        file_name[name_len] = '\0';          /* 添加字符串终止符 */

        /* 4c. 收文件大小（大端64位） */
        uint64_t file_size_be = 0;           /* 大端64位文件大小 */
        if (recv_all(sock, &file_size_be, sizeof(file_size_be)) != 0) { /* 接收8字节大小 */
            ESP_LOGE(TAG, "接收文件大小失败: %s", file_name); /* 打印错误 */
            goto recv_exit;                  /* 退出 */
        }
        uint64_t file_size = be64toh(file_size_be); /* 转换为主机字节序 */
        ESP_LOGI(TAG, "[%lu/%lu] %s (%llu 字节)", /* 打印文件名和大小 */
                 (unsigned long)(i + 1), (unsigned long)file_count,
                 file_name, (unsigned long long)file_size);

        /* 4d. 创建目标文件（先删旧文件，避免残留） */
        char path[300];                      /* 文件完整路径缓冲区 */
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, file_name); /* 构造路径 */
        unlink(path);                        /* 删除可能存在的旧文件 */

        fp = fopen(path, "wb");              /* 以二进制写入模式打开文件 */
        if (!fp) {                           /* 打开失败 */
            ESP_LOGE(TAG, "无法创建文件: %s, 丢弃 %llu 字节", /* 打印错误 */
                     path, (unsigned long long)file_size);
            /* 文件无法创建，仍需把字节全部读完保持协议同步 */
            uint64_t discard = file_size;    /* 剩余要丢弃的字节数 */
            while (discard > 0) {            /* 循环丢弃 */
                size_t chunk = (discard > FILE_RECV_BUF_SIZE) ?
                               FILE_RECV_BUF_SIZE : (size_t)discard; /* 本次丢弃块大小 */
                if (recv_all(sock, recv_buf, chunk) != 0) goto recv_exit; /* 接收并丢弃 */
                discard -= chunk;            /* 减少剩余字节 */
            }
            continue;                        /* 跳过后续写入，处理下一个文件 */
        }

        /* 4e. 循环接收并写入SD */
        uint64_t remain = file_size;         /* 剩余要写入的字节数 */
        while (remain > 0) {                 /* 只要还有数据未写入 */
            size_t chunk = (remain > FILE_RECV_BUF_SIZE) ?
                           FILE_RECV_BUF_SIZE : (size_t)remain; /* 本次接收块大小 */
            if (recv_all(sock, recv_buf, chunk) != 0) { /* 接收一块数据 */
                ESP_LOGE(TAG, "接收数据丢失: %s", file_name); /* 打印错误 */
                fclose(fp); fp = NULL;       /* 关闭文件并置空指针 */
                goto recv_exit;              /* 退出 */
            }
            size_t written = fwrite(recv_buf, 1, chunk, fp); /* 将接收到的数据写入文件 */
            if (written != chunk) {          /* 写入字节数与期望不符 */
                ESP_LOGE(TAG, "SD卡写入失败: %s", file_name); /* 打印错误 */
                fclose(fp); fp = NULL;       /* 关闭文件并置空指针 */
                goto recv_exit;              /* 退出 */
            }
            remain -= chunk;                 /* 更新剩余字节 */
        }
        fclose(fp); fp = NULL;               /* 文件接收完成，关闭文件 */
        ESP_LOGI(TAG, "已保存: %s", path);   /* 打印保存成功 */
    }

    /* ---------- 5. 收 "DONE"，回 "ACK" ---------- */
    {
        char done_buf[8] = {0};              /* 接收DONE标记的缓冲区 */
        if (recv_all(sock, done_buf, 4) != 0) { /* 接收4字节"DONE" */
            ESP_LOGE(TAG, "接收DONE失败");   /* 打印错误 */
            goto recv_exit;                  /* 退出 */
        }
        done_buf[4] = '\0';                  /* 添加终止符以便字符串比较 */
        if (strncmp(done_buf, "DONE", 4) != 0) { /* 检查是否为"DONE" */
            ESP_LOGE(TAG, "期望DONE，实际收到'%s'", done_buf); /* 打印意外数据 */
            goto recv_exit;                  /* 退出 */
        }
        send(sock, "ACK", 3, 0);             /* 发送确认ACK */
        ESP_LOGI(TAG, "全部完成，ACK已发送"); /* 打印完成信息 */

        /* 等待主端收到ACK后主动关闭连接，不能从端抢先close。
         * 若从端先close，主端recv(ack)拿到连接断开(n=0)而非ACK字符串。
         * 这里用recv等主端关闭（返回0），超时由SO_RCVTIMEO保护。*/
        {
            char drain[16];                  /* 用于接收任何剩余数据的缓冲区 */
            int n = recv(sock, drain, sizeof(drain), 0); /* 等待主端关闭连接 */
            ESP_LOGI(TAG, "主端已关闭连接 (n=%d)，传输结束", n); /* 打印连接关闭信息 */
        }
        success = true;                      /* 标记传输成功 */
    }

recv_exit:                                   /* 统一退出点 */
    if (fp)       { fclose(fp); }            /* 如果文件还开着，关闭它 */
    if (recv_buf) { free(recv_buf); }        /* 释放接收缓冲区 */
    return success;                          /* 返回传输是否成功 */
}

/* ======================== 主函数 ======================== */
void app_main(void)
{
    /* NVS初始化 */
    esp_err_t ret = nvs_flash_init();        /* 初始化NVS闪存 */
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();                   /* 擦除NVS分区 */
        nvs_flash_init();                    /* 重新初始化 */
    }

    /* LED初始化 */
    led_init();                              /* 初始化LED */
    LED(1);                                  /* 点亮LED */
    setvbuf(stdout, NULL, _IONBF, 0);        /* 设置标准输出为无缓冲，便于实时查看日志 */
    printf("\n========== 从机 V2.0 (自动接收) ==========\n"); /* 打印标题 */

    /* ---------- SD卡初始化 ---------- */
    sdmmc_card_t *sd_card = NULL;            /* SD卡句柄 */
    esp_err_t sd_ret = sd_card_init(&sd_card); /* 初始化SD卡 */
    g_sd_ok = (sd_ret == ESP_OK && sd_card != NULL); /* 记录SD卡是否就绪 */

    if (!g_sd_ok) {                          /* SD卡初始化失败 */
        printf("[主函数] SD卡初始化失败: %s\n", esp_err_to_name(sd_ret)); /* 打印失败原因 */
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }  /* 死机 */
    }
    printf("[主函数] SD卡就绪\n");           /* 打印成功信息 */

    /* ---------- USB MSC 初始化（优先，确保PC能识别U盘）---------- */
#if USE_USB_MSC
    printf("[主函数] 初始化 USB MSC...\n"); /* 打印USB MSC初始化提示 */
    ESP_ERROR_CHECK(usb_msc_init(sd_card)); /* 初始化USB MSC，注册SD卡给TinyUSB */
    printf("[主函数] USB MSC驱动已安装\n");
    vTaskDelay(pdMS_TO_TICKS(100));         /* 短暂延迟 */
    tud_connect();                           /* 立即连接USB，让PC能识别U盘 */
    vTaskDelay(pdMS_TO_TICKS(2000));         /* 等待2秒，让USB稳定，PC识别 */
    printf("[主函数] USB MSC 就绪，PC可访问SD卡 (connected=%d)\n", tud_connected()); /* 打印就绪信息 */
#endif

    LED(0);                                  /* 关闭LED */

    /* ---------- WiFi STA 初始化（最后初始化，避免影响USB）---------- */
    printf("[主函数] 启动WiFi STA (主机热点可能尚未开启)...\n"); /* 打印WiFi初始化提示 */
    wifi_init_sta();   /* 初始化WiFi STA，尝试连接主端热点 */
    printf("[主函数] 准备就绪 (USB模式)。等待主机WiFi...\n"); /* 打印等待提示 */

    /* ======================== 主循环 ========================
     * 正常待机：USB U盘模式，PC可访问SD。
     * 轮询TCP连接主端：
     *   连上 → 切WiFi本地模式 → 接收文件 → 切回USB模式
     *   未连上 → 等待后继续轮询（主端SD无文件时不开Server，属正常）
     * ======================================================== */
    int loop_cnt = 0;                        /* 循环计数器，用于间隔打印 */

    while (1) {                              /* 无限循环 */
        LED_TOGGLE();                        /* 翻转LED状态，指示心跳 */

        /* WiFi未连接：等待主端热点出现，驱动后台自动重连 */
        if (!g_wifi_connected) {             /* 如果WiFi未连接 */
            if (++loop_cnt >= 10) {          /* 每10次循环打印一次信息 */
                loop_cnt = 0;                /* 重置计数器 */
                printf("[循环] 等待主机WiFi (%s)...\n", MASTER_SSID); /* 打印等待信息 */
                /* 重试次数耗尽后驱动停止自动重连，手动踢一下 */
                s_retry_num = 0;             /* 重置重试计数 */
                esp_wifi_connect();          /* 手动触发连接 */
            }
            vTaskDelay(pdMS_TO_TICKS(2000)); /* 延迟2秒继续循环 */
            continue;                        /* 跳过本次循环后续逻辑 */
        }
        loop_cnt = 0;                        /* WiFi已连接，重置循环计数器 */
        printf("[循环] WiFi已连接，正在轮询主机TCP...\n"); /* 打印轮询提示 */

        if (!g_sd_ok) {                      /* 如果SD卡不可用 */
            printf("[循环] SD卡不可用\n");   /* 打印提示 */
            vTaskDelay(pdMS_TO_TICKS(3000)); /* 延迟3秒 */
            continue;                        /* 继续循环 */
        }

        /* ---------- 尝试TCP连接主端 ----------
         * 此时仍处于USB模式，connect() 只是建立网络连接，不涉及SD操作，安全。
         * 只有连接成功后才切换模式。 */
        int  sock      = -1;                 /* socket文件描述符 */
        bool connected = false;              /* 连接成功标志 */

        for (int attempt = 0; attempt < TCP_CONNECT_RETRY; attempt++) { /* 重试连接 */
            sock = socket(AF_INET, SOCK_STREAM, 0); /* 创建TCP socket */
            if (sock < 0) {                  /* 创建失败 */
                printf("[TCP] socket() 失败\n"); /* 打印错误 */
                break;                       /* 退出重试循环 */
            }

            struct timeval tv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 }; /* 超时结构体 */
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); /* 设置接收超时 */
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)); /* 设置发送超时 */

            struct sockaddr_in master_addr = {0}; /* 主机地址结构体 */
            master_addr.sin_family = AF_INET;     /* IPv4 */
            master_addr.sin_port   = htons(MASTER_PORT); /* 设置端口，转网络字节序 */
            inet_pton(AF_INET, MASTER_IP, &master_addr.sin_addr); /* 将IP字符串转为二进制 */

            if (connect(sock, (struct sockaddr *)&master_addr,
                        sizeof(master_addr)) == 0) { /* 尝试连接 */
                connected = true;            /* 连接成功 */
                printf("[TCP] 已连接到主机 (第%d次尝试)\n", attempt + 1); /* 打印成功信息 */
                break;                       /* 跳出重试循环 */
            }

            printf("[TCP] 连接失败 (%d/%d), %dms后重试\n",
                   attempt + 1, TCP_CONNECT_RETRY, TCP_CONNECT_RETRY_MS); /* 打印失败信息 */
            close(sock);                     /* 关闭失败的socket */
            sock = -1;                       /* 重置socket描述符 */
            vTaskDelay(pdMS_TO_TICKS(TCP_CONNECT_RETRY_MS)); /* 等待重试间隔 */
        }

        if (!connected) {                    /* 如果重试后仍未连接 */
            /* 主端SD无文件时不开Server，TCP连接失败是正常情况。
             * 等待10秒再轮询，避免频繁切换USB模式骚扰PC。 */
            vTaskDelay(pdMS_TO_TICKS(10000)); /* 等待10秒 */
            continue;                        /* 继续主循环 */
        }

        /* ---------- TCP已连上，先做握手验证再切模式 ----------
         * 关键: 不能TCP一连上就切WiFi模式断USB。
         * 必须先收到主端发来的 "MASTER_SEND" 确认对方身份，
         * 握手失败说明连的不是主端（或主端状态不对），直接断开，USB不动。 */
        {
            /* 握手超时独立设短一点（5秒），不影响后续传输超时 */
            struct timeval htv = { .tv_sec = 5, .tv_usec = 0 }; /* 5秒超时 */
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &htv, sizeof(htv)); /* 设置接收超时 */
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &htv, sizeof(htv)); /* 设置发送超时 */

            char greeting[16] = {0};         /* 握手消息缓冲区 */
            int  n = recv(sock, greeting, 11, MSG_WAITALL); /* 接收11字节 "MASTER_SEND" */
            if (n != 11 || strncmp(greeting, "MASTER_SEND", 11) != 0) { /* 检查握手内容 */
                printf("[TCP] 握手失败 (n=%d, 数据='%.*s'), 断开连接\n",
                       n, (n > 0 ? n : 0), greeting); /* 打印失败信息 */
                close(sock);                 /* 关闭socket */
                vTaskDelay(pdMS_TO_TICKS(10000)); /* 等待10秒 */
                continue;                    /* 回主循环，不切模式 */
            }
            printf("[TCP] 握手成功，主机已确认\n"); /* 握手成功 */

            /* 握手通过，现在才切WiFi本地模式断USB */
            printf("[传输] 切换到WiFi模式...\n"); /* 打印切换提示 */
            if (!switch_to_wifi_mode()) {    /* 切换到WiFi本地模式 */
                printf("[传输] 模式切换失败，取消传输\n"); /* 打印失败 */
                /* 已收到MASTER_SEND但没回READY，主端会超时退出，没关系 */
                close(sock);                 /* 关闭socket */
                vTaskDelay(pdMS_TO_TICKS(3000)); /* 等待3秒 */
                continue;                    /* 继续主循环 */
            }

            /* 切换成功后恢复传输超时，回复READY */
            struct timeval ttv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 }; /* 传输超时 */
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ttv, sizeof(ttv)); /* 设置接收超时 */
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ttv, sizeof(ttv)); /* 设置发送超时 */

            if (send(sock, "READY", 5, 0) != 5) { /* 发送READY应答 */
                printf("[传输] 发送READY失败\n"); /* 打印错误 */
                close(sock);                 /* 关闭socket */
                switch_to_usb_mode();        /* 切回USB模式 */
                vTaskDelay(pdMS_TO_TICKS(3000)); /* 等待3秒 */
                continue;                    /* 继续主循环 */
            }
            printf("[传输] 已发送READY，开始接收文件...\n"); /* 打印提示 */
        }

        /* ---------- 执行文件接收（握手已在上面完成，跳过内部握手）---------- */
        LED(1);                              /* 点亮LED，指示传输中 */
        bool ok = do_file_receive_after_handshake(sock); /* 执行文件接收协议 */
        close(sock);                         /* 传输完毕，关闭socket */
        LED(0);                              /* 熄灭LED */

        if (ok) {                            /* 根据传输结果打印信息 */
            printf("[传输] 所有文件接收成功\n"); /* 成功 */
        } else {
            printf("[传输] 接收失败或不完整\n"); /* 失败 */
        }

        /* ---------- 切回USB模式 ----------
         * 无论接收成功与否都要切回，保证PC能正常访问SD。 */
        printf("[传输] 切换回USB模式...\n"); /* 打印切换提示 */
        switch_to_usb_mode();                /* 切换回USB U盘模式 */

        /* ---------- 断开并重连WiFi ----------
         * 主动断开再重连，让主端检测到新STA连接，触发新一轮传输。
         * 否则主端看到g_sta_count不变，不会再次启动传输任务。 */
        printf("[传输] 重连WiFi以触发下一轮传输...\n"); /* 打印提示 */
        esp_wifi_disconnect();               /* 断开WiFi */
        s_retry_num = 0;                     /* 重置重试计数 */
        esp_wifi_connect();                  /* 重新连接 */
        g_wifi_connected = false;            /* 重置连接状态，等待重连成功 */
        printf("[传输] WiFi重连中...\n");     /* 打印提示 */

        /* 等待WiFi重连完成+主端处理文件后再进入下一轮 */
        vTaskDelay(pdMS_TO_TICKS(5000));     /* 等待5秒 */
    }
}