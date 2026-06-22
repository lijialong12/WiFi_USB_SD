/**
 ****************************************************************************************************
 * @file        main.c  (主端)
 * @author      ONE
 * @version     V3.5
 * @date        2026-06-22
 * @brief       主端程序 - USB MSC + GPIO0按键触发 + WiFi AP按需开启
 *
 *              【V3.5 修复记录】
 *               - 修复因 TCP 10秒超时过短导致大文件传输失败、从机产生空文件的问题
 *               - 宏定义 TRANSFER_ACK_TIMEOUT 从 10 提升至 60 秒
 *               - 完善 fread 读取失败的报错处理逻辑
 ****************************************************************************************************
 */

#define USE_USB_MSC  1

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
#include "driver/gpio.h"
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

#ifndef htobe64
#define htobe64(x)  __builtin_bswap64(x)
#endif

static const char *TAG = "MASTER";
#define WIFI_SSID       "BOSSCOM_USB_AP"
#define WIFI_PASS       "012345678"
#define MAX_STA_CONN    5

#define TRANSFER_PORT           3333
#define ACCEPT_TIMEOUT_S        60      /* 等待从机连接超时(秒) */
/* 🔴【重要修改】超时时间从 10 秒加大到 60 秒，防止大文件传输中断 */
#define TRANSFER_ACK_TIMEOUT    60
#define SD_MOUNT_POINT          "/sd"
#define MAX_FILES               100
#define FILE_SEND_BUF_SIZE      4096

#define BUTTON_GPIO             GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS      200

static bool g_wifi_mode = false;
static volatile bool g_button_pressed = false;

static bool switch_to_wifi_mode(void);
static void switch_to_usb_mode(void);
static int  send_all(int sock, const void *buf, size_t len);
static void handle_transfer(int client_sock);
static void wifi_ap_start(void);
static void wifi_ap_stop(void);
static void IRAM_ATTR button_isr_handler(void *arg);

typedef struct { char name[256]; uint64_t size; } file_info_t;

static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        int r = send(sock, p, len, 0);
        if (r <= 0) return -1;
        p += r; len -= r;
    }
    return 0;
}

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
        printf("[模式] 挂载失败: %s\n", esp_err_to_name(ret));
        tud_connect();
        return false;
    }
    g_wifi_mode = true;
    printf("[模式] WiFi模式就绪\n");
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
    printf("[模式] → USB模式\n");
    tinyusb_msc_storage_unmount();
    g_wifi_mode = false;
    vTaskDelay(pdMS_TO_TICKS(500));
    tud_connect();
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("[模式] USB就绪\n");
#endif
}

static void IRAM_ATTR button_isr_handler(void *arg)
{
    g_button_pressed = true;
}

static void button_init(void)
{
    gpio_config_t io = {
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
}

static bool g_wifi_init_once = false;

static void wifi_init_once(void)
{
    if (g_wifi_init_once) return;
    esp_netif_init();
    esp_event_loop_create_default();
    g_wifi_init_once = true;
}

static void wifi_ap_start(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();

    wifi_init_once();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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
        .ap = { .ssid_len = strlen(ssid), .max_connection = MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK },
    };
    memcpy(wifi_cfg.ap.ssid, ssid, sizeof(wifi_cfg.ap.ssid));
    memcpy(wifi_cfg.ap.password, pass, sizeof(wifi_cfg.ap.password));
    if (strlen(pass) == 0) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip);
    printf("[WiFi] AP已启动: SSID=%s, IP=" IPSTR "\n", ssid, IP2STR(&ip.ip));
}

static void wifi_ap_stop(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();
    printf("[WiFi] AP已停止\n");
}

static void handle_transfer(int client_sock)
{
    file_info_t *files = NULL;
    char *send_buf = NULL;

    /* 1. 检查SD文件 */
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) { printf("[传输] 无法打开SD\n"); return; }
        bool has_file = false;
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) { has_file = true; break; }
        }
        closedir(dir);
        if (!has_file) { printf("[传输] SD无文件\n"); return; }
    }

    /* 2. 设置超时 */
    struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 };
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    /* 3. 握手 */
    if (send_all(client_sock, "MASTER_SEND", 11) != 0) { printf("[传输] 握手失败\n"); return; }
    char ack[16] = {0};
    if (recv(client_sock, ack, 15, 0) <= 0 || strncmp(ack, "READY", 5) != 0)
    { printf("[传输] 从机未就绪\n"); return; }
    printf("[传输] 从机就绪\n");

    /* 4. 收集文件 */
    files = malloc(sizeof(file_info_t) * MAX_FILES);
    if (!files) return;
    int file_cnt = 0;
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) { uint32_t z = 0; send_all(client_sock, &z, 4); free(files); return; }
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL && file_cnt < MAX_FILES) {
            if (entry->d_type != DT_REG) continue;
            char path[300];
            snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name);
            struct stat st;
            if (stat(path, &st) == 0) {
                strncpy(files[file_cnt].name, entry->d_name, 255);
                files[file_cnt].name[255] = 0;
                files[file_cnt].size = (uint64_t)st.st_size;
                file_cnt++;
            }
        }
        closedir(dir);
    }

    /* 5. 发文件数 */
    uint32_t cnt_net = htonl((uint32_t)file_cnt);
    send_all(client_sock, &cnt_net, 4);
    printf("[传输] %d个文件\n", file_cnt);

    /* 6. 逐个发送 */
    send_buf = malloc(FILE_SEND_BUF_SIZE);
    if (!send_buf) { free(files); return; }

    for (int i = 0; i < file_cnt; i++) {
        uint16_t nl = htons((uint16_t)strlen(files[i].name));
        send_all(client_sock, &nl, 2);
        send_all(client_sock, files[i].name, strlen(files[i].name));

        uint64_t sb = htobe64(files[i].size);
        send_all(client_sock, &sb, 8);

        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, files[i].name);
        FILE *fp = fopen(path, "rb");
        if (!fp) {
            memset(send_buf, 0, FILE_SEND_BUF_SIZE);
            uint64_t r = files[i].size;
            while (r > 0) {
                size_t c = (r > FILE_SEND_BUF_SIZE) ? FILE_SEND_BUF_SIZE : (size_t)r;
                send_all(client_sock, send_buf, c);
                r -= c;
            }
            continue;
        }
        uint64_t remain = files[i].size;
        while (remain > 0) {
            size_t c = (remain > FILE_SEND_BUF_SIZE) ? FILE_SEND_BUF_SIZE : (size_t)remain;
            size_t n = fread(send_buf, 1, c, fp);
            
            // 🔴【重要修改】读取到 0 字节时认为是错误，立刻终止，防止发送空文件
            if (n == 0) { 
                fclose(fp); 
                printf("[传输] 本地读取文件 %s 失败\n", files[i].name);
                goto cleanup; 
            }
            
            if (send_all(client_sock, send_buf, n) != 0) { 
                fclose(fp); 
                goto cleanup; 
            }
            remain -= n;
        }
        fclose(fp);
        printf("[传输] %s (%llu字节)\n", files[i].name, files[i].size);
    }

    /* 7. DONE + 等OK */
    send_all(client_sock, "DONE", 4);
    bool should_delete = false;
    {
        char a[16] = {0};
        int n = recv(client_sock, a, 15, 0);
        if (n > 0) { a[n] = 0; printf("[传输] ACK:%s\n", a); }
        if (strncmp(a, "OK", 2) == 0) should_delete = true;
    }

    /* 8. 删文件 */
    if (should_delete) {
        printf("[传输] 删除SD文件...\n");
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                if (entry->d_type == DT_REG) {
                    char path[300];
                    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name);
                    unlink(path);
                }
            }
            closedir(dir);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

cleanup:
    free(send_buf);
    free(files);
    printf("[传输] 完成\n");
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    led_init();
    LED(1);

    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    bool sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        tud_connect();
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("[主函数] USB MSC 就绪，PC可访问SD卡\n");
#endif
    } else {
        printf("[主函数] SD卡失败\n");
    }

    button_init();

    vTaskDelay(pdMS_TO_TICKS(500));
    setvbuf(stdout, NULL, _IONBF, 0);
    LED(0);

    printf("\n========== 主机 V3.5 (按键触发) ==========\n");
    printf("U盘已就绪，按下 GPIO0 按键开始传输\n");

    int trans_cnt = 0;

    while (1) {
        LED_TOGGLE();

        if (g_button_pressed) {
            g_button_pressed = false;
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            printf("[按键] 用户按下，检查SD卡...\n");

            if (!switch_to_wifi_mode()) {
                printf("[按键] 切模式失败\n");
                switch_to_usb_mode();
                continue;
            }

            bool has_file = false;
            DIR *dir = opendir(SD_MOUNT_POINT);
            if (dir) {
                struct dirent *entry;
                while ((entry = readdir(dir)) != NULL) {
                    if (entry->d_type == DT_REG) { has_file = true; break; }
                }
                closedir(dir);
            }
            if (!has_file) {
                printf("[按键] SD无文件\n");
                switch_to_usb_mode();
                continue;
            }

            printf("[按键] 发现文件，启动WiFi热点...\n");
            wifi_ap_start();

            int lsock = socket(AF_INET, SOCK_STREAM, 0);
            if (lsock < 0) { wifi_ap_stop(); switch_to_usb_mode(); continue; }

            int opt = 1;
            setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            struct sockaddr_in addr = {0};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            addr.sin_port = htons(TRANSFER_PORT);
            if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
                close(lsock); wifi_ap_stop(); switch_to_usb_mode(); continue;
            }
            listen(lsock, 1);

            struct timeval tv = { .tv_sec = ACCEPT_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(lsock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            printf("[按键] 等待从机连接(超时%ds)...\n", ACCEPT_TIMEOUT_S);

            struct sockaddr_in cli;
            socklen_t cl = sizeof(cli);
            int csock = accept(lsock, (struct sockaddr *)&cli, &cl);

            if (csock < 0) {
                printf("[按键] 从机连接超时\n");
            } else {
                printf("[按键] 从机已连接: %s\n", inet_ntoa(cli.sin_addr));
                handle_transfer(csock);
                close(csock);
                trans_cnt++;
            }

            close(lsock);
            wifi_ap_stop();
            switch_to_usb_mode();

            printf("[按键] 完成(共%d次传输)，可再按按键\n", trans_cnt);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}