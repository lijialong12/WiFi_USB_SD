/**
 ****************************************************************************************************
 * @file        main.c  (主端 - 绝对纯自动化版，无需触发文件)
 * @author      ONE
 * @version     V3.9
 * @date        2026-06-29
 * @brief       绝对纯自动化 - 只要向U盘拷贝文件，检测写入结束且空闲3秒后，
 *              自动拔U盘，走WiFi发给从机。
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

#define USB_IDLE_TIMEOUT_MS 3000  // USB底层连续空闲 3 秒视为传输结束

// 🔴【灯语辅助宏】
#define LED_BLINK(times, period_ms) for(int _i=0; _i<(times); _i++){ LED_TOGGLE(); vTaskDelay(pdMS_TO_TICKS((period_ms))); }

// ======================== 全局状态变量 ========================
static bool g_wifi_mode = false;

// 原方案B的标记变量，需要靠第一步中修改底层 tinyusb_msc_storage.c 来赋值
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

// 纯自动化检测辅助函数
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

// ======================== 文件传输逻辑与自动化检测 ========================

// 等待 USB 底层持续空闲一定时长
static void wait_usb_idle_stable(uint32_t check_duration_ms)
{
    printf("[自动化] 检测到写入，等待USB底层传输完全停止...\n");
    uint32_t stable_time = 0;
    while (stable_time < check_duration_ms) {
        if (g_usb_busy) {
            stable_time = 0;      // 底层还在发数据，计时器归零
        } else {
            stable_time += 100;   // 空闲了 100ms
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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

    // 【2. 设置 Socket 超时】
    struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 };
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // 【3. 握手】
    if (send_all(client_sock, "MASTER_SEND", 11) != 0) { printf("[传输] 握手失败\n"); return; }
    char ack[16] = {0};
    if (recv(client_sock, ack, 15, 0) <= 0 || strncmp(ack, "READY", 5) != 0) {
        printf("[传输] 从机未就绪\n"); return;
    }
    printf("[传输] 从机就绪\n");

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
    printf("[传输] %d个文件\n", file_cnt);

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
        printf("[传输] %s (%llu字节)\n", files[i].name, files[i].size);
    }
    LED(0);

    // 【7. 发DONE等ACK】
    send_all(client_sock, "DONE", 4);
    bool should_delete = false;
    {
        char a[16];
        int retry = 0;
        const int max_retries = 12;
        const int wait_per_retry = 10;
        struct timeval short_tv = { .tv_sec = wait_per_retry, .tv_usec = 0 };
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &short_tv, sizeof(short_tv));

        bool ack_received = false;
        while (retry < max_retries) {
            memset(a, 0, sizeof(a));
            int n = recv(client_sock, a, sizeof(a) - 1, 0);
            if (n > 0) {
                a[n] = '\0';
                printf("[传输] 第 %d 次尝试收到 ACK: %s\n", retry + 1, a);
                if (strncmp(a, "OK", 2) == 0) {
                    should_delete = true;
                    ack_received = true;
                    break;
                }
            } else {
                printf("[传输] 等待 ACK 超时 (%d/%d)\n", retry + 1, max_retries);
            }
            retry++;
        }
        struct timeval old_tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 };
        setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &old_tv, sizeof(old_tv));
    }

    // 【8. 删除文件】
    if (should_delete) {
        printf("[传输] 删除SD文件...\n");
        LED_BLINK(5, 100);
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


// ======================== 主函数入口 ========================
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

    // ======= 2. SD 卡与 USB MSC 初始化 =======
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

    // ======= 3. 日志与防误触初始化 =======
    vTaskDelay(pdMS_TO_TICKS(500));
    setvbuf(stdout, NULL, _IONBF, 0);
    LED(0);

    printf("\n========== 主机 V3.9 (绝对纯自动化) ==========\n");
    printf("U盘已就绪，只等电脑向U盘拷贝文件。\n");

    // 开机后等待10秒防误触（防止Windows刚识别时写入的卷标触发）
    printf("[启动] 等待10秒防误触屏蔽期，请勿操作...\n");
    vTaskDelay(pdMS_TO_TICKS(10000));
    g_usb_written = false; // 强行消除系统刚开机时自动产生的误触发

    int trans_cnt = 0;

    // ======= 4. 主循环 (绝对纯自动化事件轮询) =======
    while (1) {
        /* 🔴【灯语逻辑】：正常待机状态，1.2秒一个周期慢速翻转 */
        LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(800));

        if (!g_wifi_mode) {
            
            // 核心触发条件：【曾写入过数据】 + 【现在底层不忙】
            if (g_usb_written && !g_usb_busy) {
                
                // 等到 USB 确确实实停下来了 3 秒钟，说明文件彻底传输结束
                wait_usb_idle_stable(USB_IDLE_TIMEOUT_MS); 

                printf("[自动化] 触发条件完全满足，自动开始传输！\n");
                LED_BLINK(3, 100);

                // 4.1 切换 WiFi 模式 (拔出 U 盘)
                if (!switch_to_wifi_mode()) {
                    printf("[自动化] 切模式失败\n");
                    switch_to_usb_mode(); 
                    continue;
                }

                // 4.2 检查真实文件是否存在
                bool has_file = false;
                DIR *dir = opendir(SD_MOUNT_POINT);
                if (dir) {
                    struct dirent *entry;
                    while ((entry = readdir(dir)) != NULL) {
                        if (entry->d_type == DT_REG) { 
                            has_file = true; break; 
                        }
                    }
                    closedir(dir);
                }
                if (!has_file) { 
                    printf("[自动化] 无有效数据文件\n");
                    switch_to_usb_mode(); 
                    continue; 
                }

                // 4.3 启动 WiFi AP
                printf("[自动化] 发现文件，启动WiFi热点...\n");
                wifi_ap_start();

                // 4.4 建立 TCP Socket (参照原逻辑)
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

                printf("[自动化] 等待从机连接(超时%ds)...\n", ACCEPT_TIMEOUT_S);

                struct sockaddr_in cli;
                socklen_t cl = sizeof(cli);
                int csock = accept(lsock, (struct sockaddr *)&cli, &cl);

                if (csock < 0) {
                    printf("[自动化] 从机连接超时\n");
                } else {
                    printf("[自动化] 从机已连接: %s\n", inet_ntoa(cli.sin_addr));
                    handle_transfer(csock);
                    close(csock);
                    trans_cnt++;
                }

                close(lsock);
                wifi_ap_stop();
                switch_to_usb_mode();

                // ======== 【绝对纯自动化专属收尾】 ========
                // 1. 重置写入触发标记，等待下一次电脑接入
                g_usb_written = false;
                // 2. 给电脑 5 秒钟恢复枚举时间，防止刚插回去又马上触发
                vTaskDelay(pdMS_TO_TICKS(5000));

                printf("[自动化] 完成(共%d次传输)，等待下一次电脑写入文件\n", trans_cnt);
                continue;
            }
        }
    }
}