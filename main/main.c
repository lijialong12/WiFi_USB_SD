/**
 ****************************************************************************************************
 * @file        main.c  (主端 - 纯AP强制发送版)
 * @author      ONE
 * @version     V3.9
 * @date        2026-06-29
 * @brief       纯 AP 模式：启动即开热点，从机连上 TCP 端口后，无视从机状态，直接发送 SD 卡所有文件。
 ****************************************************************************************************
 */

#define USE_USB_MSC 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// ======================== 字节序转换宏 ========================
#ifndef htobe64
#define htobe64(x)  __builtin_bswap64(x)
#endif

// ======================== 核心参数配置宏定义 ========================
#define WIFI_SSID           "BOSSCOM_USB_AP"
#define WIFI_PASS           "012345678"
#define MAX_STA_CONN        5
#define TRANSFER_PORT       3333
#define ACCEPT_TIMEOUT_S    60
#define TRANSFER_ACK_TIMEOUT 60
#define SD_MOUNT_POINT      "/sd"
#define MAX_FILES           100
#define FILE_SEND_BUF_SIZE  4096

#define USB_IDLE_TIMEOUT_MS 3000  // 保留，防止未使用变量警告

// 🔴【灯语辅助宏】
#define LED_BLINK(times, period_ms) for(int _i=0; _i<(times); _i++){ LED_TOGGLE(); vTaskDelay(pdMS_TO_TICKS((period_ms))); }

// ======================== 全局状态变量 ========================
static bool g_wifi_mode = false;

// 原方案B的标记变量，需要靠修改底层 tinyusb_msc_storage.c 来赋值
// ✅ 修改后（全局可见）
volatile bool g_usb_busy = false; 
bool g_usb_written = false;       

typedef struct { 
    char name[256];
    uint64_t size;
} file_info_t;

// ======================== 函数声明 ========================
static bool switch_to_wifi_mode(void);
static void switch_to_usb_mode(void);
static int send_all(int sock, const void *buf, size_t len);
static void handle_transfer(int client_sock);
static void wifi_ap_start(void);
static void wifi_ap_stop(void);

// 纯自动化检测辅助函数（保留声明）
static void wait_usb_idle_stable(uint32_t check_duration_ms);

// ======================== 核心业务函数实现 ========================

static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        int r = send(sock, p, len, 0);
        if (r <= 0) return -1;
        p += r;
        len -= r;
    }
    return 0;
}

static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;
    printf("[模式] → WiFi模式 (断开USB + 本地挂载SD)\n");
    LED_BLINK(5, 100);
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(800));
    tinyusb_msc_storage_unmount();
    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {
        printf("[模式] 挂载失败: %s\n", esp_err_to_name(ret));
        LED(0); vTaskDelay(pdMS_TO_TICKS(3000));
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
    LED_BLINK(5, 100);
    vTaskDelay(pdMS_TO_TICKS(500));
    printf("[模式] USB就绪\n");
#endif
}

// ======================== WiFi AP 按需启停逻辑 ========================
static bool g_wifi_init_once = false;
static esp_netif_t *g_ap_netif = NULL;

static void wifi_init_once(void)
{
    if (g_wifi_init_once) return;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    g_ap_netif = esp_netif_create_default_wifi_ap();
    g_wifi_init_once = true;
}

static void wifi_ap_start(void)
{
    wifi_init_once();
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
        .ap = {
            .ssid_len = strlen(ssid),
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    memcpy(wifi_cfg.ap.ssid, ssid, sizeof(wifi_cfg.ap.ssid));
    memcpy(wifi_cfg.ap.password, pass, sizeof(wifi_cfg.ap.password));
    if (strlen(pass) == 0) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip;
    if (g_ap_netif) {
        esp_netif_get_ip_info(g_ap_netif, &ip);
        printf("[WiFi] AP已启动: SSID=%s, IP=" IPSTR "\n", ssid, IP2STR(&ip.ip));
    }
}

static void wifi_ap_stop(void)
{
    if (esp_wifi_stop() == ESP_OK) {
        esp_wifi_deinit();
        printf("[WiFi] AP已停止\n");
    }
}

// 保留原函数，防止编译器告警
static void wait_usb_idle_stable(uint32_t check_duration_ms)
{
    // 此函数在新版 AP 模式下已不再被调用
    return;
}

// ======================== 核心修改：无握手、无ACK的暴力发送 ========================
static void handle_transfer(int client_sock)
{
    file_info_t *files = NULL;
    char *send_buf = NULL;

    // 【1. 检查SD卡是否有文件】
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) { printf("[传输] 无法打开SD\n"); return; }
        bool has_file = false;
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {
            if (entry->d_type == DT_REG) {
                has_file = true; break;
            }
        }
        closedir(dir);
        if (!has_file) { printf("[传输] SD无文件\n"); return; }
    }

    // 【2. 只设置发送超时，移除接收超时】
    struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 };
    // setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); // 不等待接收
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));     // 发送失败自动超时断开

    // 【3. 直接宣告，无视从机是否接受】
    if (send_all(client_sock, "MASTER_SEND", 11) != 0) { printf("[传输] 连接异常\n"); return; }
    printf("[传输] 宣告成功，无视从端状态，直接发送数据...\n");

    // 【4. 收集文件】
    files = malloc(sizeof(file_info_t) * MAX_FILES);
    if (!files) return;
    int file_cnt = 0;
    {
        DIR *dir = opendir(SD_MOUNT_POINT);
        if (!dir) {
            uint32_t z = 0;
            send_all(client_sock, &z, 4);
            free(files); return;
        }
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

    // 【5. 发文件数】
    uint32_t cnt_net = htonl((uint32_t)file_cnt);
    send_all(client_sock, &cnt_net, 4);
    printf("[传输] 向从机发送 %d 个文件\n", file_cnt);

    // 【6. 发文件】
    send_buf = malloc(FILE_SEND_BUF_SIZE);
    if (!send_buf) { free(files); return; }
    LED(1);

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
            if (n == 0) {
                fclose(fp);
                printf("[传输] 本地读取文件 %s 失败\n", files[i].name);
                LED(0);
                goto cleanup;
            }
            if (send_all(client_sock, send_buf, n) != 0) {
                fclose(fp);
                LED(0);
                goto cleanup;
            }
            remain -= n;
        }
        fclose(fp);
        printf("[传输] 发送完成: %s (%llu字节)\n", files[i].name, files[i].size);
    }
    LED(0);

    // 【7. 发送结束标记 (只发送，不接收OK)】
    send_all(client_sock, "DONE", 4);
    printf("[传输] 所有数据已发出（未等待从机回复）\n");
    // 不删除文件，确保主端数据安全

cleanup:
    if (send_buf) free(send_buf);
    if (files) free(files);
    printf("[传输] 传输结束，释放连接\n");
}


// ======================== 主函数入口 (纯 AP 模式) ========================
void app_main(void)
{
    // ======= 1. 基础系统与存储初始化 =======
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    led_init();
    LED(1);

    // ======= 2. SD 卡与 USB MSC 初始化 (USB 仍然是U盘) =======
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    bool sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        tud_connect();
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("[主函数] USB MSC 就绪，PC端依然可以通过USB拷贝文件到卡内\n");
#endif
    } else {
        printf("[主函数] SD卡失败\n");
    }

    // ======= 3. 日志输出 =======
    vTaskDelay(pdMS_TO_TICKS(500));
    setvbuf(stdout, NULL, _IONBF, 0);
    LED(0);

    printf("\n========== 主机 V3.9 (纯AP自动化版) ==========\n");
    printf("永久开启热点，从机连接WiFi后自动触发下发文件。\n");

    // ======= 4. 直接开启 WiFi AP (常驻，不再关闭) =======
    wifi_ap_start();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ======= 5. 建立 TCP 监听服务 =======
    int lsock = socket(AF_INET, SOCK_STREAM, 0);
    if (lsock < 0) {
        printf("[主端] 创建套接字失败\n");
        return;
    }
    int opt = 1;
    setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(TRANSFER_PORT);
    
    if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        close(lsock);
        printf("[主端] 端口绑定失败\n");
        return;
    }
    listen(lsock, 1);
    printf("[主端] TCP服务已启动，等待从机连接(端口%d)...\n", TRANSFER_PORT);

    // ======= 6. 主循环：纯粹的 TCP 触发机制 =======
    while (1) {
        /* 🔴【灯语逻辑】：正常待机状态，1.2秒一个周期慢速翻转 */
        LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(800));

        struct sockaddr_in cli;
        socklen_t cl = sizeof(cli);
        
        // 阻塞式等待。只要从机连接上此端口，accept 就会立刻返回 > 0
        int csock = accept(lsock, (struct sockaddr *)&cli, &cl);

        if (csock > 0) {
            printf("[触发] 从机已连接，立即开始下发文件！\n");
            LED_BLINK(5, 100); // 闪烁提示正在传输
            
            handle_transfer(csock); // 执行暴力无回复发送
            
            close(csock);
            printf("[主端] 传输结束，继续等待下一次从机连接...\n");
        } else {
            // 极少出现，但若 accept 出错，等待 1 秒后重新进入监听
            printf("[主端] Accept异常，重启监听\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}