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

/* ======================== WiFi STA 配置 ======================== */
static const char *TAG = "SLAVE";

#define MASTER_SSID             "BOSSCOM_USB_AP"  /* 主机热点名，需与主端一致 */
#define MASTER_PASS             "012345678"        /* 主机热点密码 */
#define MASTER_IP               "192.168.4.1"      /* 主机固定IP（SoftAP默认） */
#define MASTER_PORT             3333               /* 与主端 TRANSFER_PORT 一致 */

#define WIFI_CONNECT_TIMEOUT_S  30                 /* WiFi连接超时(秒) */
#define WIFI_MAX_RETRY          10                 /* STA连接最大重试次数 */
#define TCP_CONNECT_RETRY       5                  /* TCP连接失败重试次数 */
#define TCP_CONNECT_RETRY_MS    2000               /* 每次重试间隔(ms) */
#define TCP_RECV_TIMEOUT_S      30                 /* TCP收数据超时(秒) */

#define SD_MOUNT_POINT          "/sd"
#define FILE_RECV_BUF_SIZE      4096               /* 接收缓冲区（堆分配） */

/* ======================== WiFi 事件组 ======================== */
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static int                s_retry_num        = 0;

/* ======================== 全局状态 ======================== */
static bool g_sd_ok          = false;
static bool g_wifi_connected = false;
static bool g_wifi_mode      = false;  /* true=本地挂载模式, false=USB MSC模式 */

/* ======================== 函数声明 ======================== */
static bool switch_to_wifi_mode(void);
static void switch_to_usb_mode(void);
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data);
static bool wifi_init_sta(void);
static int  recv_all(int sock, void *buf, size_t len);
static bool do_file_receive_after_handshake(int sock);

/* ======================== 模式切换 ======================== */
/**
 * @brief  切换到WiFi本地模式（断开USB，ESP32本地挂载SD）
 *         在TCP连上、准备接收文件前调用
 * @return true=成功, false=挂载失败
 */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;   /* 已在本地模式，幂等 */

    printf("[MODE] → WiFi mode (USB disconnect + local mount)\n");
    tud_disconnect();                                  /* PC看到U盘移除 */
    vTaskDelay(pdMS_TO_TICKS(800));                    /* 等待PC处理弹出事件 */
    tinyusb_msc_storage_unmount();

    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {
        printf("[MODE] WiFi mode mount FAILED: %s\n", esp_err_to_name(ret));
        tud_connect();    /* 挂载失败，重新连上USB */
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
 * @brief  切换回USB U盘模式（卸载本地挂载，重新连接USB）
 *         在文件接收完成后调用
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
    tud_connect();                                     /* PC重新看到U盘插入 */
    printf("[MODE] USB mode OK\n");
#endif
}

/* ======================== 安全接收封装 ======================== */
/**
 * @brief  循环接收直到收满 len 字节或出错/超时
 * @return 0=成功, -1=出错或对端关闭
 */
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

/* ======================== WiFi 事件处理 ======================== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retry WiFi (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "WiFi failed after %d retries", WIFI_MAX_RETRY);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_num      = 0;
        g_wifi_connected = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ======================== WiFi STA 初始化 ======================== */
static bool wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    char ssid[32] = MASTER_SSID;
    char pass[64] = MASTER_PASS;
    nvs_handle_t nvs;
    if (nvs_open("wifi_cfg", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(ssid);
        nvs_get_str(nvs, "ssid", ssid, &len);
        len = sizeof(pass);
        nvs_get_str(nvs, "pass", pass, &len);
        nvs_close(nvs);
    }

    wifi_config_t wifi_cfg = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    strncpy((char *)wifi_cfg.sta.ssid,     ssid, sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, pass, sizeof(wifi_cfg.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to AP: %s ...", ssid);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_S * 1000)
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected: %s", ssid);
        return true;
    }
    ESP_LOGE(TAG, "WiFi connect timeout/failed");
    return false;
}

/* ======================== 文件接收核心逻辑 ======================== */
/**
 * @brief  握手已在主循环完成后，接收文件数+文件内容+DONE/ACK
 *         调用前：switch_to_wifi_mode() 已成功，READY 已发出
 */
static bool do_file_receive_after_handshake(int sock)
{
    char *recv_buf = NULL;
    FILE *fp       = NULL;
    bool  success  = false;

    /* ---------- 1. 收文件总数 ---------- */
    uint32_t file_count = 0;
    {
        uint32_t count_net = 0;
        if (recv_all(sock, &count_net, sizeof(count_net)) != 0) {
            ESP_LOGE(TAG, "Recv file count failed");
            goto recv_exit;
        }
        file_count = ntohl(count_net);
        ESP_LOGI(TAG, "Expecting %lu file(s)", (unsigned long)file_count);
        if (file_count == 0) {
            success = true;     /* 主端无文件，正常结束 */
            goto recv_exit;
        }
    }

    /* ---------- 3. 堆分配接收缓冲区 ---------- */
    recv_buf = (char *)malloc(FILE_RECV_BUF_SIZE);
    if (!recv_buf) {
        ESP_LOGE(TAG, "malloc recv_buf failed");
        goto recv_exit;
    }

    /* ---------- 4. 循环接收每个文件 ---------- */
    for (uint32_t i = 0; i < file_count; i++) {

        /* 4a. 收文件名长度 */
        uint16_t name_len_net = 0;
        if (recv_all(sock, &name_len_net, sizeof(name_len_net)) != 0) {
            ESP_LOGE(TAG, "Recv name_len failed at file %lu", (unsigned long)i);
            goto recv_exit;
        }
        uint16_t name_len = ntohs(name_len_net);
        if (name_len == 0 || name_len > 255) {
            ESP_LOGE(TAG, "Invalid name_len=%u", name_len);
            goto recv_exit;
        }

        /* 4b. 收文件名 */
        char file_name[256] = {0};
        if (recv_all(sock, file_name, name_len) != 0) {
            ESP_LOGE(TAG, "Recv filename failed at file %lu", (unsigned long)i);
            goto recv_exit;
        }
        file_name[name_len] = '\0';

        /* 4c. 收文件大小（大端64位） */
        uint64_t file_size_be = 0;
        if (recv_all(sock, &file_size_be, sizeof(file_size_be)) != 0) {
            ESP_LOGE(TAG, "Recv file_size failed: %s", file_name);
            goto recv_exit;
        }
        uint64_t file_size = be64toh(file_size_be);
        ESP_LOGI(TAG, "[%lu/%lu] %s (%llu bytes)",
                 (unsigned long)(i + 1), (unsigned long)file_count,
                 file_name, (unsigned long long)file_size);

        /* 4d. 创建目标文件（先删旧文件，避免残留） */
        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, file_name);
        unlink(path);

        fp = fopen(path, "wb");
        if (!fp) {
            ESP_LOGE(TAG, "Cannot create: %s, discarding %llu bytes",
                     path, (unsigned long long)file_size);
            /* 文件无法创建，仍需把字节全部读完保持协议同步 */
            uint64_t discard = file_size;
            while (discard > 0) {
                size_t chunk = (discard > FILE_RECV_BUF_SIZE) ?
                               FILE_RECV_BUF_SIZE : (size_t)discard;
                if (recv_all(sock, recv_buf, chunk) != 0) goto recv_exit;
                discard -= chunk;
            }
            continue;
        }

        /* 4e. 循环接收并写入SD */
        uint64_t remain = file_size;
        while (remain > 0) {
            size_t chunk = (remain > FILE_RECV_BUF_SIZE) ?
                           FILE_RECV_BUF_SIZE : (size_t)remain;
            if (recv_all(sock, recv_buf, chunk) != 0) {
                ESP_LOGE(TAG, "Recv data lost: %s", file_name);
                fclose(fp); fp = NULL;
                goto recv_exit;
            }
            size_t written = fwrite(recv_buf, 1, chunk, fp);
            if (written != chunk) {
                ESP_LOGE(TAG, "SD write failed: %s", file_name);
                fclose(fp); fp = NULL;
                goto recv_exit;
            }
            remain -= chunk;
        }
        fclose(fp); fp = NULL;
        ESP_LOGI(TAG, "Saved: %s", path);
    }

    /* ---------- 5. 收 "DONE"，回 "ACK" ---------- */
    {
        char done_buf[8] = {0};
        if (recv_all(sock, done_buf, 4) != 0) {         /* "DONE" = 4字节 */
            ESP_LOGE(TAG, "Recv DONE failed");
            goto recv_exit;
        }
        done_buf[4] = '\0';
        if (strncmp(done_buf, "DONE", 4) != 0) {
            ESP_LOGE(TAG, "Expected DONE, got '%s'", done_buf);
            goto recv_exit;
        }
        send(sock, "ACK", 3, 0);
        ESP_LOGI(TAG, "All done, ACK sent");

        /* 等待主端收到ACK后主动关闭连接，不能从端抢先close。
         * 若从端先close，主端recv(ack)拿到连接断开(n=0)而非ACK字符串。
         * 这里用recv等主端关闭（返回0），超时由SO_RCVTIMEO保护。*/
        {
            char drain[16];
            int n = recv(sock, drain, sizeof(drain), 0);
            ESP_LOGI(TAG, "Master closed connection (n=%d), all done", n);
        }
        success = true;
    }

recv_exit:
    if (fp)       { fclose(fp); }
    if (recv_buf) { free(recv_buf); }
    return success;
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
    vTaskDelay(pdMS_TO_TICKS(6000));    /* 等待USB枚举完成（与主端保持一致） */
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\n========== SLAVE V2.0 (Auto Receive) ==========\n");

    /* ---------- SD卡 + USB MSC 初始化 ----------
     * 默认以USB U盘模式启动，PC可直接访问SD卡。
     * 接收文件时再切换到本地模式，接收完毕后切回。 */
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);
    g_sd_ok = (sd_ret == ESP_OK && sd_card != NULL);

    if (g_sd_ok) {
        printf("[MAIN] SD card OK\n");
#if USE_USB_MSC
        /* 注意: 只调用 usb_msc_init()，不调用 usb_msc_mount()。
         *
         * usb_msc_init() 完成:
         *   - 注册 SDMMC 到 TinyUSB MSC 存储层
         *   - 安装 TinyUSB 驱动，USB PHY 使能，PC 可识别 U 盘
         *   此时 SD 卡由 TinyUSB/PC 控制，ESP32 本地不可读写，这是正确的待机状态。
         *
         * usb_msc_mount() 的作用是把 SD 卡挂载给 ESP32 本地 FATFS 用，
         * 那是"WiFi本地模式"下才需要做的事（switch_to_wifi_mode 里会调用
         * tinyusb_msc_storage_mount），现在不能提前挂载，否则和 TinyUSB 冲突。*/
        printf("[MAIN] Init USB MSC...\n");
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        /* 不在这里 mount，保持 USB 模式让 PC 访问 */
        printf("[MAIN] USB MSC ready, PC can access SD\n");
#endif
    } else {
        printf("[MAIN] SD card FAILED: %s\n", esp_err_to_name(sd_ret));
    }

    /* ---------- WiFi STA 初始化 ----------
     * 注意: 主端可能还没开机，热点不存在是正常的。
     * 连接失败不重启，进主循环后持续等待热点出现。
     * WiFi驱动会在后台自动重连（事件回调里有重试逻辑）。 */
    printf("[MAIN] Starting WiFi STA (master may not be on yet)...\n");
    wifi_init_sta();   /* 无论成功失败都继续，g_wifi_connected标志后续判断 */

    LED(0);
    printf("[MAIN] Ready (USB mode). Waiting for master WiFi...\n");

    /* ======================== 主循环 ========================
     * 正常待机：USB U盘模式，PC可访问SD。
     * 轮询TCP连接主端：
     *   连上 → 切WiFi本地模式 → 接收文件 → 切回USB模式
     *   未连上 → 等待后继续轮询（主端SD无文件时不开Server，属正常）
     * ======================================================== */
    int loop_cnt = 0;

    while (1) {
        LED_TOGGLE();

        /* WiFi未连接：等待主端热点出现，驱动后台自动重连 */
        if (!g_wifi_connected) {
            if (++loop_cnt >= 10) {
                loop_cnt = 0;
                printf("[LOOP] Waiting for master WiFi (%s)...\n", MASTER_SSID);
                /* 重试次数耗尽后驱动停止自动重连，手动踢一下 */
                s_retry_num = 0;
                esp_wifi_connect();
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        loop_cnt = 0;
        printf("[LOOP] WiFi OK, polling master TCP...\n");

        if (!g_sd_ok) {
            printf("[LOOP] SD not available\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* ---------- 尝试TCP连接主端 ----------
         * 此时仍处于USB模式，connect() 只是建立网络连接，不涉及SD操作，安全。
         * 只有连接成功后才切换模式。 */
        int  sock      = -1;
        bool connected = false;

        for (int attempt = 0; attempt < TCP_CONNECT_RETRY; attempt++) {
            sock = socket(AF_INET, SOCK_STREAM, 0);
            if (sock < 0) {
                printf("[TCP] socket() failed\n");
                break;
            }

            struct timeval tv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

            struct sockaddr_in master_addr = {0};
            master_addr.sin_family = AF_INET;
            master_addr.sin_port   = htons(MASTER_PORT);
            inet_pton(AF_INET, MASTER_IP, &master_addr.sin_addr);

            if (connect(sock, (struct sockaddr *)&master_addr,
                        sizeof(master_addr)) == 0) {
                connected = true;
                printf("[TCP] Connected to master (attempt %d)\n", attempt + 1);
                break;
            }

            printf("[TCP] Connect failed (%d/%d), retry in %dms\n",
                   attempt + 1, TCP_CONNECT_RETRY, TCP_CONNECT_RETRY_MS);
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(TCP_CONNECT_RETRY_MS));
        }

        if (!connected) {
            /* 主端SD无文件时不开Server，TCP连接失败是正常情况。
             * 等待10秒再轮询，避免频繁切换USB模式骚扰PC。 */
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        /* ---------- TCP已连上，先做握手验证再切模式 ----------
         * 关键: 不能TCP一连上就切WiFi模式断USB。
         * 必须先收到主端发来的 "MASTER_SEND" 确认对方身份，
         * 握手失败说明连的不是主端（或主端状态不对），直接断开，USB不动。 */
        {
            /* 握手超时独立设短一点（5秒），不影响后续传输超时 */
            struct timeval htv = { .tv_sec = 5, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &htv, sizeof(htv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &htv, sizeof(htv));

            char greeting[16] = {0};
            int  n = recv(sock, greeting, 11, MSG_WAITALL);
            if (n != 11 || strncmp(greeting, "MASTER_SEND", 11) != 0) {
                printf("[TCP] Handshake failed (n=%d, data='%.*s'), drop\n",
                       n, (n > 0 ? n : 0), greeting);
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(10000));
                continue;   /* 回主循环，不切模式 */
            }
            printf("[TCP] Handshake OK, master confirmed\n");

            /* 握手通过，现在才切WiFi本地模式断USB */
            printf("[TRANSFER] Switching to WiFi mode...\n");
            if (!switch_to_wifi_mode()) {
                printf("[TRANSFER] Mode switch failed, abort\n");
                /* 已收到MASTER_SEND但没回READY，主端会超时退出，没关系 */
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }

            /* 切换成功后恢复传输超时，回复READY */
            struct timeval ttv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ttv, sizeof(ttv));
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ttv, sizeof(ttv));

            if (send(sock, "READY", 5, 0) != 5) {
                printf("[TRANSFER] Send READY failed\n");
                close(sock);
                switch_to_usb_mode();
                vTaskDelay(pdMS_TO_TICKS(3000));
                continue;
            }
            printf("[TRANSFER] Sent READY, starting receive...\n");
        }

        /* ---------- 执行文件接收（握手已在上面完成，跳过内部握手）---------- */
        LED(1);
        bool ok = do_file_receive_after_handshake(sock);
        close(sock);
        LED(0);

        if (ok) {
            printf("[TRANSFER] All files received OK\n");
        } else {
            printf("[TRANSFER] Receive failed or incomplete\n");
        }

        /* ---------- 切回USB模式 ----------
         * 无论接收成功与否都要切回，保证PC能正常访问SD。 */
        printf("[TRANSFER] Switching back to USB mode...\n");
        switch_to_usb_mode();

        /* 等待主端完成删文件+切USB操作后再进入下一轮。
         * 主端删完文件约需300ms+切USB约需300ms，留5秒余量。 */
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}