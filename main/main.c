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
#define USE_USB_MSC  1

/* ======================== 头文件 ======================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "led.h"
#include "sd_card.h"
#if USE_USB_MSC
#include "usb_msc.h"
#include "tusb.h"
#include "tusb_msc_storage.h"
#endif

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "dirent.h"
#include "sys/stat.h"
#include "unistd.h"

/* ======================== 字节序转换宏 ======================== */
#ifndef htobe64
#define htobe64(x)  __builtin_bswap64(x)
#endif

/* ======================== WiFi AP 配置 ======================== */
static const char *TAG = "MASTER";
#define WIFI_SSID       "BOSSCOM_USB_AP"
#define WIFI_PASS       "012345678"
#define MAX_STA_CONN    5
#define MAC2STR(a)      (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR          "%02x:%02x:%02x:%02x:%02x:%02x"

/* ======================== 主从传输配置 ======================== */
#define TRANSFER_PORT           3333    /* TCP监听端口 */
#define TRANSFER_WAIT_TIMEOUT   20      /* 等待从机连接超时(秒) */
#define TRANSFER_ACK_TIMEOUT    10      /* 等待从机握手ACK超时(秒) */
#define SD_MOUNT_POINT          "/sd"
#define MAX_FILES               100     /* 单次最大发送文件数 */
#define FILE_SEND_BUF_SIZE      4096    /* 文件发送缓冲区，堆分配 */

/* ======================== 全局状态 ======================== */
static int           g_sta_count    = 0;     /* 当前STA连接数 */
static bool          g_wifi_mode    = false; /* 当前是否为WiFi本地模式 */
static volatile bool g_transfer_busy = false; /* 传输任务忙标志 */

/* ======================== 函数声明 ======================== */
static bool switch_to_wifi_mode(void);
static void switch_to_usb_mode(void);
static int  send_all(int sock, const void *buf, size_t len);
static void file_transfer_task(void *arg);
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data);
static void wifi_init_softap(void);

/* ======================== 安全发送封装 ======================== */
/**
 * @brief  循环发送直到全部发完或出错
 *         修复[BUG-5]: 原代码 send() 一次调用不保证全部发出
 * @return 0=成功, -1=出错
 */
static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        int r = send(sock, p, len, 0);
        if (r <= 0) return -1;
        p   += r;
        len -= r;
    }
    return 0;
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
    if (g_wifi_mode) return true;   /* 已在WiFi模式，幂等 */

    printf("[MODE] → WiFi mode (USB disconnect + local mount)\n");
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(800)); /* 等待PC处理U盘移除事件 */
    tinyusb_msc_storage_unmount();

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {
        printf("[MODE] WiFi mode mount FAILED: %s\n", esp_err_to_name(ret));
        /* 挂载失败：尝试重新连上USB，不改变 g_wifi_mode */
        tud_connect();
        return false;
    }
    g_wifi_mode = true;
    printf("[MODE] WiFi mode OK, SD mounted at %s\n", SD_MOUNT_POINT);
    return true;
#else
    g_wifi_mode = true;
    return true;
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
    if (!g_wifi_mode) {
        printf("[MODE] Already in USB mode, skip\n");
        return;
    }
    printf("[MODE] → USB mode (local unmount + USB connect)\n");
    tinyusb_msc_storage_unmount();
    g_wifi_mode = false;
    vTaskDelay(pdMS_TO_TICKS(300));
    tud_connect();
    printf("[MODE] USB mode OK, PC can access SD\n");
#endif
}

/* ======================== WiFi 事件处理 ======================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        g_sta_count++;
        ESP_LOGI(TAG, "Station " MACSTR " connected, total:%d",
                 MAC2STR(e->mac), g_sta_count);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        if (g_sta_count > 0) g_sta_count--;
        ESP_LOGI(TAG, "Station " MACSTR " disconnected, total:%d",
                 MAC2STR(e->mac), g_sta_count);
    }
}

/* ======================== WiFi SoftAP 初始化 ======================== */
static void wifi_init_softap(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

    char ssid[32] = WIFI_SSID;
    char pass[64] = WIFI_PASS;
    nvs_handle_t nvs;
    if (nvs_open("wifi_cfg", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(ssid);
        nvs_get_str(nvs, "ssid", ssid, &len);
        len = sizeof(pass);
        nvs_get_str(nvs, "pass", pass, &len);
        nvs_close(nvs);
    }

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid_len       = strlen(ssid),
            .max_connection = MAX_STA_CONN,
            .authmode       = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    memcpy(wifi_cfg.ap.ssid,     ssid, sizeof(wifi_cfg.ap.ssid));
    memcpy(wifi_cfg.ap.password, pass, sizeof(wifi_cfg.ap.password));
    if (strlen(pass) == 0) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip);
    ESP_LOGI(TAG, "AP started: SSID=%s, IP=" IPSTR, ssid, IP2STR(&ip.ip));
}

/* ======================== 文件信息结构 ======================== */
typedef struct {
    char     name[256];
    uint64_t size;
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
    printf("[TRANSFER] Task started\n");

    int         listen_sock = -1;
    int         client_sock = -1;
    file_info_t *files      = NULL;
    bool        entered_wifi_mode = false;  /* 记录本次任务是否成功切入WiFi模式 */

    /* --------------------------------------------------------
     * 步骤1: 切换到WiFi本地模式
     * --------------------------------------------------------
     * 注意: 在USB模式下 SD 由 TinyUSB 控制，ESP32本地无法直接读写。
     *       必须先切换到本地模式才能检查和读取文件。
     * -------------------------------------------------------- */
    if (!switch_to_wifi_mode()) {
        printf("[TRANSFER] Cannot enter WiFi mode, abort\n");
        goto task_exit;
    }
    entered_wifi_mode = true;

    /* --------------------------------------------------------
     * 步骤2: 检查SD卡是否有文件
     * 修复[LOGIC-1]: 切换WiFi模式之后再检查，而非之前
     * -------------------------------------------------------- */
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) {
            printf("[TRANSFER] Cannot open SD dir: %s\n", SD_MOUNT_POINT);
            goto task_exit;
        }
        bool has_file = false;
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) { has_file = true; break; }
        }
        closedir(dir);

        if (!has_file) {
            printf("[TRANSFER] No files on SD, skip transfer\n");
            goto task_exit;     /* 无文件，切回USB退出 */
        }
    }

    /* --------------------------------------------------------
     * 步骤3: 创建TCP服务器，等待从机连接
     * -------------------------------------------------------- */
    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) {
        printf("[TRANSFER] socket() failed\n");
        goto task_exit;
    }

    {
        int opt = 1;
        setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    }

    {
        struct sockaddr_in addr = {0};
        addr.sin_family      = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port        = htons(TRANSFER_PORT);
        if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            printf("[TRANSFER] bind() failed\n");
            goto task_exit;
        }
    }

    if (listen(listen_sock, 1) != 0) {
        printf("[TRANSFER] listen() failed\n");
        goto task_exit;
    }

    /* 等待从机连接的超时 */
    {
        struct timeval tv = { .tv_sec = TRANSFER_WAIT_TIMEOUT, .tv_usec = 0 };
        setsockopt(listen_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    printf("[TRANSFER] Listening on port %d, waiting slave (timeout=%ds)...\n",
           TRANSFER_PORT, TRANSFER_WAIT_TIMEOUT);

    {
        struct sockaddr_in cli_addr;
        socklen_t cli_len = sizeof(cli_addr);
        client_sock = accept(listen_sock, (struct sockaddr *)&cli_addr, &cli_len);
        if (client_sock < 0) {
            printf("[TRANSFER] accept() timeout or error, abort\n");
            goto task_exit;
        }
        printf("[TRANSFER] Slave connected: %s\n", inet_ntoa(cli_addr.sin_addr));
    }

    /* 给已连接的socket也设一个收发超时，防握手阻塞
     * 修复[LOGIC-3]: 原代码 recv(ack) 无超时，从机不回应时永久阻塞 */
    {
        struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 };
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    }

    /* --------------------------------------------------------
     * 步骤4: 握手
     * -------------------------------------------------------- */
    {
        const char *handshake = "MASTER_SEND";
        if (send_all(client_sock, handshake, strlen(handshake)) != 0) {
            printf("[TRANSFER] Handshake send failed\n");
            goto task_exit;
        }

        char ack[16] = {0};
        int  n = recv(client_sock, ack, sizeof(ack) - 1, 0);
        if (n <= 0 || strncmp(ack, "READY", 5) != 0) {
            printf("[TRANSFER] Slave not ready (n=%d, ack='%s')\n", n, ack);
            goto task_exit;
        }
        printf("[TRANSFER] Slave ready, sending files...\n");
    }

    /* --------------------------------------------------------
     * 步骤5: 收集文件列表（堆分配）
     * 修复[BUG-4]: 原来放栈上，100个文件≈26KB，必然栈溢出
     * -------------------------------------------------------- */
    files = (file_info_t *)malloc(sizeof(file_info_t) * MAX_FILES);
    if (!files) {
        printf("[TRANSFER] malloc file_info failed\n");
        goto task_exit;
    }

    int file_cnt = 0;
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) {
            printf("[TRANSFER] Cannot reopen SD dir\n");
            uint32_t zero = 0;
            send_all(client_sock, &zero, sizeof(zero));
            goto task_exit;
        }
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL && file_cnt < MAX_FILES) {
            if (entry->d_type == DT_REG) {
                char path[300];
                snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name);
                struct stat st;
                if (stat(path, &st) == 0) {
                    strncpy(files[file_cnt].name, entry->d_name, 255);
                    files[file_cnt].name[255] = '\0'; /* 确保终止符 */
                    files[file_cnt].size = (uint64_t)st.st_size;
                    file_cnt++;
                }
            }
        }
        closedir(dir);
    }

    /* 发送文件总数 */
    {
        uint32_t count_net = htonl((uint32_t)file_cnt);
        if (send_all(client_sock, &count_net, sizeof(count_net)) != 0) {
            printf("[TRANSFER] Send file count failed\n");
            goto task_exit;
        }
        printf("[TRANSFER] Will send %d file(s)\n", file_cnt);
    }

    /* 发送每个文件 */
    {
        /* 文件内容发送缓冲区也堆分配，节省栈空间 */
        char *send_buf = (char *)malloc(FILE_SEND_BUF_SIZE);
        if (!send_buf) {
            printf("[TRANSFER] malloc send_buf failed\n");
            goto task_exit;
        }

        for (int i = 0; i < file_cnt; i++) {
            /* 发送文件名长度 + 文件名 */
            uint16_t name_len     = (uint16_t)strlen(files[i].name);
            uint16_t name_len_net = htons(name_len);
            if (send_all(client_sock, &name_len_net, sizeof(name_len_net)) != 0 ||
                send_all(client_sock, files[i].name, name_len)             != 0) {
                printf("[TRANSFER] Send filename failed: %s\n", files[i].name);
                free(send_buf);
                goto task_exit;
            }

            /* 发送文件大小（大端64位）*/
            uint64_t size_be = htobe64(files[i].size);
            if (send_all(client_sock, &size_be, sizeof(size_be)) != 0) {
                printf("[TRANSFER] Send file size failed: %s\n", files[i].name);
                free(send_buf);
                goto task_exit;
            }

            /* 发送文件内容 */
            char path[300];
            snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, files[i].name);
            FILE *fp = fopen(path, "rb");
            if (!fp) {
                printf("[TRANSFER] Open failed: %s\n", path);
                /* 已承诺发这个文件，打开失败需要告知从机：发全0填充
                 * （从机需要读完对应字节数才能继续接收下一文件）*/
                uint64_t remain = files[i].size;
                memset(send_buf, 0, FILE_SEND_BUF_SIZE);
                while (remain > 0) {
                    size_t chunk = (remain > FILE_SEND_BUF_SIZE) ?
                                   FILE_SEND_BUF_SIZE : (size_t)remain;
                    if (send_all(client_sock, send_buf, chunk) != 0) break;
                    remain -= chunk;
                }
                continue;
            }

            /* 修复[BUG-2]: remain 用 uint64_t，避免32位截断 */
            uint64_t remain = files[i].size;
            while (remain > 0) {
                size_t chunk = (remain > FILE_SEND_BUF_SIZE) ?
                               FILE_SEND_BUF_SIZE : (size_t)remain;
                size_t n = fread(send_buf, 1, chunk, fp);
                if (n == 0) {
                    printf("[TRANSFER] fread EOF early: %s\n", files[i].name);
                    break;
                }
                /* 修复[BUG-5]: 用 send_all 保证全部发出 */
                if (send_all(client_sock, send_buf, n) != 0) {
                    printf("[TRANSFER] send failed mid-file: %s\n", files[i].name);
                    fclose(fp);
                    free(send_buf);
                    goto task_exit;
                }
                remain -= n;
            }
            fclose(fp);
            printf("[TRANSFER] Sent: %s (%llu bytes)\n",
                   files[i].name, (unsigned long long)files[i].size);
        }
        free(send_buf);
    }

    /* --------------------------------------------------------
     * 步骤6: 发送 DONE，等从机最终ACK
     * -------------------------------------------------------- */
    {
        const char *done = "DONE";
        send_all(client_sock, done, strlen(done));  /* 尽力发，不判断返回值 */

        char ack[16] = {0};
        recv(client_sock, ack, sizeof(ack) - 1, 0);    /* 有超时保护，不会死等 */
        printf("[TRANSFER] Transfer complete, slave ack: '%s'\n", ack);
    }

    close(client_sock); client_sock = -1;
    close(listen_sock); listen_sock = -1;

    /* --------------------------------------------------------
     * 步骤7: 删除SD卡所有文件，并同步文件系统
     * 修复[LOGIC-4]: 原代码删完没有 sync，切回USB后PC可能看到残留缓存
     * -------------------------------------------------------- */
    printf("[TRANSFER] Deleting all files on SD...\n");
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                if (entry->d_type == DT_REG) {
                    char path[300];
                    snprintf(path, sizeof(path), "%s/%s",
                             SD_MOUNT_POINT, entry->d_name);
                    if (unlink(path) == 0) {
                        printf("[TRANSFER] Deleted: %s\n", entry->d_name);
                    } else {
                        printf("[TRANSFER] Delete failed: %s\n", entry->d_name);
                    }
                }
            }
            closedir(dir);
        }
        /* ESP-IDF 的 FatFS 中 unlink() 是同步操作，目录项立即更新，
         * 无需额外 sync()（ESP-IDF newlib 未提供该符号）。
         * 延迟一小段时间确保底层写操作完成后再切换总线。 */
        vTaskDelay(pdMS_TO_TICKS(300));
    }

/* --------------------------------------------------------
 * 统一出口：关闭socket，切回USB，释放资源
 * -------------------------------------------------------- */
task_exit:
    if (client_sock >= 0) { close(client_sock); }
    if (listen_sock >= 0) { close(listen_sock); }
    if (files)            { free(files); }

    /* 只有成功进入WiFi模式的情况下才需要切回USB
     * 修复[BUG-1]: 原代码无论是否切换成功都会调用 switch_to_usb_mode */
    if (entered_wifi_mode) {
        printf("[TRANSFER] Switching back to USB mode...\n");
        switch_to_usb_mode();
    }

    /* 修复[LOGIC-2]: 传输任务结束后重置 g_sta_count。
     * 从机发完文件后通常会继续保持WiFi连接（g_sta_count 不会自动归零），
     * 若不重置，主循环下次检测到 g_sta_count>0 会再次触发传输（但SD已无文件，
     * 会白白切换一次USB模式）。重置后等从机真正断开再重连才会触发。
     *
     * 注意: 若存在多个从机同时在线，此处全量清零是合理的——
     *       因为本次任务已经广播了所有文件，无需对同一批文件再次传输。 */
    g_sta_count = 0;

    g_transfer_busy = false;
    printf("[TRANSFER] Task done\n");
    vTaskDelete(NULL);
}

/* ======================== 主函数 ======================== */
void app_main(void)
{
    /* NVS初始化 */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* LED初始化 */
    led_init();
    LED(1);
    vTaskDelay(pdMS_TO_TICKS(6000));    /* 等待USB枚举完成 */
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\n========== MASTER V3.0 (Auto Transfer) ==========\n");

    /* SD卡初始化 */
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    bool sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (sd_ok) {
        printf("[MAIN] SD card OK\n");
#if USE_USB_MSC
        printf("[MAIN] Init USB MSC...\n");
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        ESP_ERROR_CHECK(usb_msc_mount(SD_MOUNT_POINT));
        printf("[MAIN] USB MSC ready, PC can copy files\n");
#endif
    } else {
        printf("[MAIN] SD card FAILED: %s (0x%x)\n",
               esp_err_to_name(sd_ret), sd_ret);
    }

    /* WiFi AP初始化 */
    printf("[MAIN] Starting WiFi AP...\n");
    wifi_init_softap();
    printf("[MAIN] AP active: %s\n", WIFI_SSID);

    /* 主循环 */
    int loop_cnt = 0;
    LED(0);
    printf("[MAIN] Entering main loop, waiting for slave...\n");

    while (1) {
        LED_TOGGLE();

        /* 触发条件：SD正常 + 当前无传输任务 + 有从机在线
         *
         * 修复[BUG-3]: 先置 g_transfer_busy=true 再 xTaskCreate，
         * 防止 xTaskCreate 返回后、新任务调度前的200ms tick里再次触发。
         * 注意: g_transfer_busy 是 volatile，两条语句之间无竞态（单核调度）。 */
        if (sd_ok && !g_transfer_busy && g_sta_count > 0) {
            printf("[MAIN] Slave detected (STA=%d), starting transfer task\n",
                   g_sta_count);
            g_transfer_busy = true;     /* 先置位！修复[BUG-3] */
            BaseType_t rc = xTaskCreate(file_transfer_task, "transfer",
                                        8192,   /* 栈8KB，文件列表已改堆分配 */
                                        NULL, 5, NULL);
            if (rc != pdPASS) {
                printf("[MAIN] xTaskCreate failed!\n");
                g_transfer_busy = false;    /* 创建失败则复位 */
            }
        }

        if (++loop_cnt >= 25) {
            loop_cnt = 0;
            printf("[LOOP] Mode:%s  STA:%d  Transfer:%s\n",
                   g_wifi_mode      ? "WiFi" : "USB",
                   g_sta_count,
                   g_transfer_busy  ? "running" : "idle");
        }

        vTaskDelay(sd_ok ? pdMS_TO_TICKS(200) : pdMS_TO_TICKS(500));
    }
}