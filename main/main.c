/**
 ****************************************************************************************************
 * @file        main.c  (从端)
 * @author      ONE
 * @version     V3.2
 * @date        2026-06-22
 * @brief       从端程序 - USB MSC + WiFi STA常驻 + TCP轮询连接
 *
 *              核心设计:
 *               - WiFi STA始终保持连接（主端热点不存在时等待）
 *               - USB MSC始终在线（PC可访问SD卡）
 *               - 轮询TCP连接主端，连上后才断USB传输
 *               - 传输完成后切回USB，继续轮询
 *
 *              配合主端V3.3: 按键触发传输，按需开WiFi热点
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
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "led.h"
#include "sd_card.h"
#if USE_USB_MSC
#include "usb_msc.h"
#include "tusb.h"
#include "tusb_msc_storage.h"
#endif

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "sys/stat.h"
#include "unistd.h"

/* ======================== 字节序转换宏 ======================== */
#ifndef be64toh
#define be64toh(x)  __builtin_bswap64(x)
#endif

/* ======================== 配置 ======================== */
static const char *TAG = "SLAVE";

#define MASTER_SSID             "BOSSCOM_USB_AP"
#define MASTER_PASS             "012345678"
#define MASTER_IP               "192.168.4.1"
#define MASTER_PORT             3333

#define WIFI_CONNECT_TIMEOUT_S  30      /* 连接热点超时(秒) */
#define TCP_POLL_INTERVAL_S     2       /* TCP轮询间隔(秒) */
#define TCP_CONNECT_TIMEOUT_S   5       /* TCP连接超时 */
#define TCP_RECV_TIMEOUT_S      30      /* 数据接收超时 */

#define SD_MOUNT_POINT          "/sd"
#define FILE_RECV_BUF_SIZE      4096

/* ======================== 全局状态 ======================== */
static bool g_sd_ok      = false;
static bool g_wifi_mode  = false;
static bool g_wifi_ok    = false;    /* WiFi是否已连接 */
static int  g_wifi_retry = 0;        /* WiFi重试计数 */

/* ======================== 函数声明 ======================== */
static bool switch_to_wifi_mode(void);
static void switch_to_usb_mode(void);
static int  recv_all(int sock, void *buf, size_t len);
static bool do_file_receive_after_handshake(int sock);
static bool wifi_init_sta(void);
static bool wifi_connect(void);

/* ======================== 模式切换 ======================== */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;
    printf("[模式] → WiFi模式 (断开USB + 本地挂载SD)\n");
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(800));
    tinyusb_msc_storage_unmount();

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {
        printf("[模式] WiFi模式挂载失败: %s\n", esp_err_to_name(ret));
        tud_connect();
        return false;
    }
    g_wifi_mode = true;
    printf("[模式] WiFi模式就绪，SD挂载于 %s\n", SD_MOUNT_POINT);
    return true;
#else
    g_wifi_mode = true;
    return true;
#endif
}

static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    if (!g_wifi_mode) return;
    printf("[模式] → USB模式 (本地卸载 + USB连接)\n");
    tinyusb_msc_storage_unmount();
    g_wifi_mode = false;
    vTaskDelay(pdMS_TO_TICKS(500));
    tud_connect();
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("[模式] USB模式就绪\n");
#endif
}

/* ======================== 安全接收封装 ======================== */
static int recv_all(int sock, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len > 0) {
        int r = recv(sock, p, len, 0);
        if (r <= 0) return -1;
        p   += r;
        len -= r;
    }
    return 0;
}

/* ======================== 文件接收核心 ======================== */
static bool do_file_receive_after_handshake(int sock)
{
    char *recv_buf = NULL;
    FILE *fp       = NULL;
    bool  success  = false;

    uint32_t file_count = 0;
    {
        uint32_t count_net = 0;
        if (recv_all(sock, &count_net, sizeof(count_net)) != 0) {
            printf("[传输] 接收文件总数失败\n");
            goto recv_exit;
        }
        file_count = ntohl(count_net);
        printf("[传输] 期望接收 %lu 个文件\n", (unsigned long)file_count);
        if (file_count == 0) { success = true; goto recv_exit; }
    }

    recv_buf = (char *)malloc(FILE_RECV_BUF_SIZE);
    if (!recv_buf) { printf("[传输] malloc recv_buf 失败\n"); goto recv_exit; }

    for (uint32_t i = 0; i < file_count; i++) {
        uint16_t name_len_net = 0;
        if (recv_all(sock, &name_len_net, sizeof(name_len_net)) != 0) {
            printf("[传输] 接收文件名长度失败，第%lu个文件\n", (unsigned long)i);
            goto recv_exit;
        }
        uint16_t name_len = ntohs(name_len_net);
        if (name_len == 0 || name_len > 255) { goto recv_exit; }

        char file_name[256] = {0};
        if (recv_all(sock, file_name, name_len) != 0) { goto recv_exit; }
        file_name[name_len] = '\0';

        uint64_t file_size_be = 0;
        if (recv_all(sock, &file_size_be, sizeof(file_size_be)) != 0) { goto recv_exit; }
        uint64_t file_size = be64toh(file_size_be);
        printf("[传输] [%lu/%lu] %s (%llu 字节)\n",
               (unsigned long)(i + 1), (unsigned long)file_count,
               file_name, (unsigned long long)file_size);

        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, file_name);
        unlink(path);

        fp = fopen(path, "wb");
        if (!fp) {
            printf("[传输] 无法创建文件: %s\n", path);
            uint64_t discard = file_size;
            while (discard > 0) {
                size_t chunk = (discard > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)discard;
                if (recv_all(sock, recv_buf, chunk) != 0) goto recv_exit;
                discard -= chunk;
            }
            continue;
        }

        uint64_t remain = file_size;
        while (remain > 0) {
            size_t chunk = (remain > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)remain;
            if (recv_all(sock, recv_buf, chunk) != 0) { fclose(fp); goto recv_exit; }
            size_t written = fwrite(recv_buf, 1, chunk, fp);
            if (written != chunk) { fclose(fp); goto recv_exit; }
            remain -= chunk;
        }
        fclose(fp); fp = NULL;
        printf("[传输] 已保存: %s\n", path);
    }

    {
        char done_buf[8] = {0};
        if (recv_all(sock, done_buf, 4) != 0) { goto recv_exit; }
        done_buf[4] = '\0';
        if (strncmp(done_buf, "DONE", 4) != 0) { goto recv_exit; }
        send(sock, "OK", 2, 0);
        printf("[传输] 全部完成，OK已发送\n");

        char drain[16];
        recv(sock, drain, sizeof(drain), 0); /* 等主端关闭 */
        success = true;
    }

recv_exit:
    if (fp)       { fclose(fp); }
    if (recv_buf) { free(recv_buf); }
    return success;
}

/* ======================== WiFi STA 初始化（只做一次）======================= */
static bool g_wifi_init_done = false;

static bool wifi_init_sta(void)
{
    if (g_wifi_init_done) return true;

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    g_wifi_init_done = true;
    return true;
}

/* ======================== WiFi 连接主端热点 ======================== */
static bool wifi_connect(void)
{
    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, MASTER_SSID, sizeof(sta_cfg.sta.ssid));
    strncpy((char *)sta_cfg.sta.password, MASTER_PASS, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_cfg));
    esp_wifi_start();

    printf("[WiFi] 正在连接 %s ...\n", MASTER_SSID);

    int timeout_ms = WIFI_CONNECT_TIMEOUT_S * 1000;
    while (timeout_ms > 0) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            printf("[WiFi] 已连接到 %s\n", MASTER_SSID);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        timeout_ms -= 500;
    }

    printf("[WiFi] 连接超时\n");
    esp_wifi_stop();
    return false;
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

    /* SD卡初始化 */
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    g_sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (g_sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        printf("[主函数] 初始化 USB MSC...\n");
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        tud_connect();
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("[主函数] USB MSC 就绪，PC可访问SD卡\n");
#endif
    } else {
        printf("[主函数] SD卡初始化失败\n");
    }

    /* WiFi STA一次性初始化 */
    printf("[主函数] 初始化 WiFi STA...\n");
    wifi_init_sta();

    vTaskDelay(pdMS_TO_TICKS(500));
    setvbuf(stdout, NULL, _IONBF, 0);
    LED(0);

    printf("\n========== 从机 V3.2 (WiFi常驻 + TCP轮询) ==========\n");

    int loop_cnt = 0;
    int trans_cnt = 0;

    /* ======================== 主循环 ======================== */
    while (1) {
        LED_TOGGLE();

        /* ---- WiFi管理 ---- */
        if (!g_wifi_ok) {
            /* 等待一段时间再重试 */
            if (g_wifi_retry > 0 && g_wifi_retry < 5) {
                printf("[WiFi] %ds后重试...\n", g_wifi_retry * 5);
                for (int s = 0; s < g_wifi_retry * 5; s++) {
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    LED_TOGGLE();
                }
            }

            printf("[WiFi] 尝试连接 %s...\n", MASTER_SSID);
            g_wifi_ok = wifi_connect();
            g_wifi_retry++;

            if (!g_wifi_ok) {
                continue;  /* 连接失败，重试 */
            }
            g_wifi_retry = 0;
        }

        /* ---- WiFi已连接，尝试TCP ---- */
        printf("[TCP] 尝试连接主机 %s:%d ...\n", MASTER_IP, MASTER_PORT);

        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            printf("[TCP] socket()失败\n");
            vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_S * 1000));
            continue;
        }

        struct timeval tv = { .tv_sec = TCP_CONNECT_TIMEOUT_S, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(MASTER_PORT);
        inet_pton(AF_INET, MASTER_IP, &addr.sin_addr);

        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            /* 主端WiFi未开（热点关闭），正常 */
            close(sock);
            if (++loop_cnt >= 10) {
                loop_cnt = 0;
                printf("[TCP] 主机未响应，继续轮询...\n");
            }
            vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_S * 1000));
            continue;
        }
        loop_cnt = 0;

        printf("[TCP] 已连接到主机！\n");

        /* ---- 握手 ---- */
        {
            struct timeval htv = { .tv_sec = 10, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &htv, sizeof(htv));

            char greeting[16] = {0};
            int n = recv(sock, greeting, 11, MSG_WAITALL);
            if (n != 11 || strncmp(greeting, "MASTER_SEND", 11) != 0) {
                printf("[握手] 失败，断开\n");
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_S * 1000));
                continue;
            }
            printf("[握手] 收到 MASTER_SEND\n");

            /* 握手通过，切WiFi模式断USB */
            if (!switch_to_wifi_mode()) {
                printf("[传输] 无法切WiFi模式\n");
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_S * 1000));
                continue;
            }

            /* 恢复超时，回READY */
            struct timeval ttv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ttv, sizeof(ttv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ttv, sizeof(ttv));

            if (send(sock, "READY", 5, 0) != 5) {
                printf("[传输] 发送READY失败\n");
                close(sock);
                switch_to_usb_mode();
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_S * 1000));
                continue;
            }
        }

        /* ---- 接收文件 ---- */
        LED(1);
        bool ok = do_file_receive_after_handshake(sock);
        close(sock);
        LED(0);

        printf(ok ? "[传输] 所有文件接收成功\n" : "[传输] 接收失败\n");
        trans_cnt++;

        /* ---- 切回USB模式 ---- */
        switch_to_usb_mode();

        /* ---- 等待下一轮 ---- */
        printf("[传输] 等待下一轮 (共%d次传输)\n", trans_cnt);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
