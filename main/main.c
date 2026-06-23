/**
 ****************************************************************************************************
 * @file        main.c  (从端)
 * @author      ONE
 * @version     V3.6
 * @date        2026-06-22
 * @brief       从端程序 - 后台持续扫描WiFi，扫描期间不影响USB挂载，
 *              待TCP握手成功后才拔出USB进行文件传输
 ****************************************************************************************************
 */

#define USE_USB_MSC  1

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

#ifndef be64toh
#define be64toh(x)  __builtin_bswap64(x)
#endif

static const char *TAG = "SLAVE";

#define MASTER_SSID             "BOSSCOM_USB_AP"
#define MASTER_PASS             "012345678"
#define MASTER_IP               "192.168.4.1"
#define MASTER_PORT             3333

#define WIFI_RETRY_INTERVAL_MS  3000
#define WIFI_GOT_IP_TIMEOUT_MS  15000
#define TCP_POLL_INTERVAL_MS    5000
#define TCP_CONNECT_TIMEOUT_S   5
#define TCP_RECV_TIMEOUT_S      30

#define SD_MOUNT_POINT          "/sd"
#define FILE_RECV_BUF_SIZE      4096

#define WIFI_GOT_IP_BIT         BIT0
#define WIFI_DISCONNECTED_BIT   BIT1

static bool               g_sd_ok          = false;
static bool               g_wifi_mode      = false;
static bool               g_wifi_init_done = false;
static EventGroupHandle_t g_wifi_eg        = NULL;

static void  switch_to_wifi_mode(void);
static void  switch_to_usb_mode(void);
static int   recv_all(int sock, void *buf, size_t len);
static bool  do_file_receive_after_handshake(int sock);
static void  wifi_init_sta(void);
static bool  wifi_try_connect(void);

/* ======================== LED 错误码 ========================
 *
 *  错误码含义（从端专用）：
 *   1闪 —— SD卡初始化失败 / USB MSC初始化失败
 *   2闪 —— WiFi始终无法连接（保留，当前主循环不用此码）
 *   3闪 —— TCP握手失败（收到的不是 MASTER_SEND）
 *   4闪 —— 接收文件总数失败
 *   5闪 —— 接收文件名或文件大小失败
 *   6闪 —— recv_all 数据接收中断
 *   7闪 —— fwrite 写SD卡失败
 *   8闪 —— 未收到 DONE 包
 *   快闪10下 —— 传输全部成功
 * =========================================================== */
static void led_error_blink(int code)
{
    for (int round = 0; round < 5; round++) {
        for (int i = 0; i < code; i++) {
            LED(1); vTaskDelay(pdMS_TO_TICKS(200));
            LED(0); vTaskDelay(pdMS_TO_TICKS(200));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void led_success_blink(void)
{
    for (int i = 0; i < 10; i++) {
        LED(1); vTaskDelay(pdMS_TO_TICKS(80));
        LED(0); vTaskDelay(pdMS_TO_TICKS(80));
    }
}

/* ======================== WiFi 事件处理 ======================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            xEventGroupClearBits(g_wifi_eg, WIFI_GOT_IP_BIT);
            xEventGroupSetBits(g_wifi_eg, WIFI_DISCONNECTED_BIT);
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            xEventGroupClearBits(g_wifi_eg, WIFI_DISCONNECTED_BIT);
            xEventGroupSetBits(g_wifi_eg, WIFI_GOT_IP_BIT);
        }
    }
}

/* ======================== WiFi STA 初始化 ======================== */
static void wifi_init_sta(void)
{
    if (g_wifi_init_done) return;
    g_wifi_eg = xEventGroupCreate();
    configASSERT(g_wifi_eg);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, MASTER_SSID, sizeof(sta_cfg.sta.ssid));
    strncpy((char *)sta_cfg.sta.password, MASTER_PASS, sizeof(sta_cfg.sta.password));
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    sta_cfg.sta.failure_retry_cnt  = 1;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_cfg));
    g_wifi_init_done = true;
}

static bool wifi_try_connect(void)
{
    xEventGroupClearBits(g_wifi_eg, WIFI_GOT_IP_BIT | WIFI_DISCONNECTED_BIT);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(g_wifi_eg, WIFI_GOT_IP_BIT | WIFI_DISCONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_GOT_IP_TIMEOUT_MS));
    return (bits & WIFI_GOT_IP_BIT);
}

static bool wifi_is_connected(void)
{
    EventBits_t bits = xEventGroupGetBits(g_wifi_eg);
    return (bits & WIFI_GOT_IP_BIT) && !(bits & WIFI_DISCONNECTED_BIT);
}

/* ======================== 模式切换 ======================== */
static void switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return;
    printf("[模式] 握手成功，拔出USB，挂载SD到FatFS...\n");

    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));           /* 等PC感知断开 */

    tinyusb_msc_storage_unmount();            /* 释放TinyUSB对SD卡的控制 */
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);  /* 挂载到FatFS */
    if (ret != ESP_OK) {
        printf("[模式] FatFS挂载失败: %s\n", esp_err_to_name(ret));
        /* 挂载失败就重连USB，不进入传输 */
        tud_connect();
        return;
    }

    g_wifi_mode = true;
    printf("[模式] SD卡已挂载到FatFS，可以写文件\n");
#endif
}

static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    if (!g_wifi_mode) return;
    printf("[模式] 传输完毕，把SD卡还给TinyUSB...\n");

    tinyusb_msc_storage_unmount();            /* 从FatFS卸载 */
    vTaskDelay(pdMS_TO_TICKS(100));

    g_wifi_mode = false;
    tud_connect();
    vTaskDelay(pdMS_TO_TICKS(1500));
    printf("[模式] USB已恢复连接\n");
#endif
}

static int recv_all(int sock, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len > 0) {
        int r = recv(sock, p, len, 0);
        if (r <= 0) return -1;
        p += r; len -= r;
    }
    return 0;
}

/* ======================== 文件接收（握手后） ======================== */
static bool do_file_receive_after_handshake(int sock)
{
    char *recv_buf = NULL;
    FILE *fp       = NULL;
    int   err_code = 0;   /* 0=成功 */

    /* 接收文件总数 */
    uint32_t file_count = 0;
    {
        uint32_t count_net = 0;
        if (recv_all(sock, &count_net, sizeof(count_net)) != 0) {
            printf("[传输] 接收文件总数失败\n");
            err_code = 4; goto recv_exit;
        }
        file_count = ntohl(count_net);
        printf("[传输] 期望接收 %lu 个文件\n", (unsigned long)file_count);
        if (file_count == 0) { goto recv_exit; /* success，err_code=0 */ }
    }

    recv_buf = (char *)malloc(FILE_RECV_BUF_SIZE);
    if (!recv_buf) { printf("[传输] malloc失败\n"); err_code = 4; goto recv_exit; }

    for (uint32_t i = 0; i < file_count; i++) {

        /* 文件名长度 */
        uint16_t name_len_net = 0;
        if (recv_all(sock, &name_len_net, sizeof(name_len_net)) != 0) {
            printf("[传输] 接收文件名长度失败 [%lu]\n", (unsigned long)i);
            err_code = 5; goto recv_exit;
        }
        uint16_t name_len = ntohs(name_len_net);
        if (name_len == 0 || name_len > 255) {
            printf("[传输] 文件名长度非法=%u\n", name_len);
            err_code = 5; goto recv_exit;
        }

        /* 文件名 */
        char file_name[256] = {0};
        if (recv_all(sock, file_name, name_len) != 0) {
            printf("[传输] 接收文件名失败\n");
            err_code = 5; goto recv_exit;
        }
        file_name[name_len] = '\0';

        /* 文件大小 */
        uint64_t file_size_be = 0;
        if (recv_all(sock, &file_size_be, sizeof(file_size_be)) != 0) {
            printf("[传输] 接收文件大小失败: %s\n", file_name);
            err_code = 5; goto recv_exit;
        }
        uint64_t file_size = be64toh(file_size_be);
        printf("[传输] [%lu/%lu] %s (%llu字节)\n",
               (unsigned long)(i+1), (unsigned long)file_count,
               file_name, (unsigned long long)file_size);

        /* 创建文件 */
        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, file_name);
        unlink(path);

        fp = fopen(path, "wb");
        if (!fp) {
            /* 无法创建：丢弃数据，继续下一个 */
            printf("[传输] 无法创建 %s，丢弃数据\n", path);
            uint64_t discard = file_size;
            while (discard > 0) {
                size_t chunk = (discard > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)discard;
                if (recv_all(sock, recv_buf, chunk) != 0) { err_code = 6; goto recv_exit; }
                discard -= chunk;
            }
            continue;
        }

        /* 接收并写入文件内容 */
        uint64_t remain = file_size;
        while (remain > 0) {
            size_t chunk = (remain > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)remain;
            if (recv_all(sock, recv_buf, chunk) != 0) {
                printf("[传输] 接收数据中断: %s\n", file_name);
                fclose(fp); fp = NULL;
                err_code = 6; goto recv_exit;
            }
            if (fwrite(recv_buf, 1, chunk, fp) != chunk) {
                printf("[传输] 写SD失败: %s\n", file_name);
                fclose(fp); fp = NULL;
                err_code = 7; goto recv_exit;
            }
            remain -= chunk;
        }
        fclose(fp); fp = NULL;
        printf("[传输] 已保存: %s\n", path);
    }

    /* 等待 DONE */
    {
        char done_buf[8] = {0};
        if (recv_all(sock, done_buf, 4) != 0) {
            printf("[传输] 接收DONE失败\n");
            err_code = 8; goto recv_exit;
        }
        done_buf[4] = '\0';
        if (strncmp(done_buf, "DONE", 4) != 0) {
            printf("[传输] DONE包内容错误: %s\n", done_buf);
            err_code = 8; goto recv_exit;
        }
        send(sock, "OK", 2, 0);
        printf("[传输] 完成，OK已发送\n");
        char drain[16]; recv(sock, drain, sizeof(drain), 0);
        /* err_code 保持 0 = 成功 */
    }

recv_exit:
    if (fp)       fclose(fp);
    if (recv_buf) free(recv_buf);

    if (err_code == 0) {
        led_success_blink();        /* 快闪10下 = 成功 */
        return true;
    } else {
        led_error_blink(err_code);  /* 闪N下 = 错误码 */
        return false;
    }
}

/* ======================== 主函数 ======================== */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    led_init(); LED(1);

    /* 1. SD卡及 USB MSC 初始化 */
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    g_sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (g_sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        esp_err_t msc_ret = usb_msc_init(sd_card);
        if (msc_ret != ESP_OK) {
            printf("[主函数] USB MSC 初始化失败!\n");
            led_error_blink(1);   /* 1闪 = 初始化失败 */
        } else {
            tud_connect();
            vTaskDelay(pdMS_TO_TICKS(1000));
            printf("[主函数] USB MSC连接正常，U盘已挂载\n");
        }
#endif
    } else {
        printf("[主函数] SD卡失败\n");
        led_error_blink(1);       /* 1闪 = SD卡失败 */
    }

    /* 2. WiFi初始化 */
    wifi_init_sta();
    esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(500));
    setvbuf(stdout, NULL, _IONBF, 0);
    LED(0);

    printf("\n========== 从机 V3.6 (扫描不挂载模式) ==========\n");
    printf("USB始终保持连接，后台持续扫描主端热点...\n");

    int trans_cnt = 0;
    int log_cnt   = 0;

    /* ======================== 主循环 ======================== */
    while (1) {
        LED_TOGGLE();

        /* 阶段1：WiFi扫描与连接 */
        if (!wifi_is_connected()) {
            if (++log_cnt >= 3) { log_cnt = 0; printf("[WiFi] 扫描中，U盘仍挂载...\n"); }
            wifi_try_connect();
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
            continue;
        }
        log_cnt = 0;

        /* 阶段2：TCP 端口轮询 */
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS)); continue; }

        struct timeval tv = { .tv_sec = TCP_CONNECT_TIMEOUT_S, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(MASTER_PORT);
        inet_pton(AF_INET, MASTER_IP, &addr.sin_addr);

        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS));
            continue;
        }

        printf("[TCP] 已连接到主机！准备握手...\n");

        /* 阶段3：等待主端握手 */
        {
            struct timeval htv = { .tv_sec = 10, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &htv, sizeof(htv));

            char greeting[16] = {0};
            int n = recv(sock, greeting, 11, MSG_WAITALL);
            if (n != 11 || strncmp(greeting, "MASTER_SEND", 11) != 0) {
                printf("[握手] 握手失败，U盘保持连接\n");
                led_error_blink(3);   /* 3闪 = 握手失败 */
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS));
                continue;
            }
            printf("[握手] 收到 MASTER_SEND，立刻准备拔出U盘！\n");

            switch_to_wifi_mode();

            struct timeval ttv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ttv, sizeof(ttv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ttv, sizeof(ttv));

            if (send(sock, "READY", 5, 0) != 5) {
                printf("[传输] 发READY失败\n");
                close(sock);
                switch_to_usb_mode();
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS));
                continue;
            }
        }

        /* 阶段4：接收文件 */
        LED(1);
        bool ok = do_file_receive_after_handshake(sock);
        close(sock);
        LED(0);

        switch_to_usb_mode();

        if (ok) {
            printf("[传输] 文件接收成功，已完成%d次\n", ++trans_cnt);
            vTaskDelay(pdMS_TO_TICKS(3000));
        } else {
            printf("[传输] 传输失败，冷却10秒...\n");
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }
}