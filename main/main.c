/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ONE
 * @version     V1.8
 * @date        2026-06-16
 * @brief       双角色 — 纯WiFi+SD, 无USB (调试通过后再加USB)
 *
 *   主机: WiFi AP → SD直接挂载 → Web文件服务器 → 等从机注册 →
 *         文件监控扫描 → HTTP推送 → 删除源文件
 *   从机: WiFi STA连接主机 → SD直接挂载 → HTTP接收服务器 →
 *         收到文件 → fwrite保存
 *
 *   核心流程: 从机连WiFi → 注册 → 主机检测文件 → 推送 → 从机保存 → 主机删除
 ****************************************************************************************************
 */

/* ======================== 角色宏开关 ======================== */
#define DEVICE_ROLE_MASTER  0
#define DEVICE_ROLE_SLAVE   1
#define DEVICE_ROLE  DEVICE_ROLE_SLAVE   /* ← 改这里切换角色 */

/* ======================== 头文件 ======================== */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "led.h"
#include "sd_card.h"
#include <dirent.h>
#include <sys/stat.h>

#if DEVICE_ROLE == DEVICE_ROLE_MASTER
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "web_server.h"
#include "file_monitor.h"
#include "file_transfer.h"
#elif DEVICE_ROLE == DEVICE_ROLE_SLAVE
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "slave.h"
#endif

/* ======================== 配置 ======================== */
#define AP_SSID     "BOSSCOM_USB_AP"
#define AP_PASS     "012345678"
#define AP_MAX_CONN 5

/* ================================================================= */
#if DEVICE_ROLE == DEVICE_ROLE_MASTER
/* ========================= 主机端 ================================= */

static const char *TAG = "HOST";
static int g_sta_count = 0;
#define MAC2STR(a) (a)[0],(a)[1],(a)[2],(a)[3],(a)[4],(a)[5]
#define MACSTR    "%02x:%02x:%02x:%02x:%02x:%02x"

/* ---- WiFi 事件 ---- */
static void wifi_event_handler(void *arg, esp_event_base_t eb,
                               int32_t eid, void *ed)
{
    if (eid == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)ed;
        g_sta_count++;
        ESP_LOGI(TAG, "STA " MACSTR " join (total:%d)", MAC2STR(e->mac), g_sta_count);
    } else if (eid == WIFI_EVENT_AP_STADISCONNECTED) {
        if (g_sta_count > 0) g_sta_count--;
        ESP_LOGI(TAG, "STA leave (total:%d)", g_sta_count);
        if (g_sta_count == 0) web_server_clear_slave();
    }
}

/* ---- WiFi AP ---- */
static esp_netif_t *wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap = esp_netif_create_default_wifi_ap();

    esp_netif_ip_info_t ip = {
        .ip={.addr=ESP_IP4TOADDR(192,168,3,1)},
        .gw={.addr=ESP_IP4TOADDR(192,168,3,1)},
        .netmask={.addr=ESP_IP4TOADDR(255,255,255,0)},
    };
    esp_netif_dhcps_stop(ap); esp_netif_set_ip_info(ap, &ip); esp_netif_dhcps_start(ap);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));

    char ssid[32]=AP_SSID, pass[64]=AP_PASS;
    nvs_handle_t h;
    if (nvs_open("wifi_config", NVS_READONLY, &h) == ESP_OK) {
        size_t len=sizeof(ssid); nvs_get_str(h,"ssid",ssid,&len);
        len=sizeof(pass);       nvs_get_str(h,"password",pass,&len);
        nvs_close(h);
    }
    wifi_config_t wc = {
        .ap={.ssid_len=strlen(ssid), .max_connection=AP_MAX_CONN,
             .authmode=strlen(pass)?WIFI_AUTH_WPA_WPA2_PSK:WIFI_AUTH_OPEN},
    };
    memcpy(wc.ap.ssid, ssid, sizeof(wc.ap.ssid));
    memcpy(wc.ap.password, pass, sizeof(wc.ap.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    printf("[HOST] AP '%s' @ 192.168.3.1\n", ssid);
    return ap;
}

/* ---- 主机主循环 ---- */
static void host_run(sdmmc_card_t *card)
{
    /* SD 直接挂载 (无USB) */
    sd_card_mount_fatfs(card, "/SD", 8);
    printf("[HOST] SD mounted. Creating test file...\n");

    /* 创建一个测试文件, 验证SD可写 */
    FILE *tf = fopen("/SD/_HOST_READY_.txt", "w");
    if (tf) { fprintf(tf, "HOST OK\n"); fclose(tf); printf("[HOST] Test file written OK\n"); }
    else printf("[HOST] WARN: cannot write test file\n");

    file_monitor_init();
    web_server_start();
    printf("[HOST] Web http://192.168.3.1\n");

    char sip[16]={0}; bool reg=false;
    int tick=0;

    printf("[HOST] === LOOP START ===\n\n");
    while (1) {
        led_pattern_tick(); tick++;

        if (!reg) { reg=web_server_get_slave_ip(sip,sizeof(sip)); if(reg) printf("\n[HOST] ***** SLAVE REGISTERED: %s *****\n\n",sip); }

        if (reg && tick%25==0) {  /* 每5秒扫描 */
            int n = file_monitor_scan("/SD");
            if (n) printf("[HOST] +%d new file(s)\n", n);

            file_entry_t *e;
            while ((e = file_monitor_next_pending()) != NULL) {
                printf("[HOST] >>> PUSH: %s -> %s\n", e->path, sip);
                led_set_pattern(LED_PAT_FAST_BLINK);
                esp_err_t ret = file_transfer_push(sip, e->path);
                if (ret == ESP_OK) {
                    printf("[HOST] <<< OK! Deleting: %s\n", e->path);
                    remove(e->path); file_monitor_mark_done(e->path);
                    led_set_pattern(LED_PAT_FLASH_3);
                } else {
                    printf("[HOST] <<< FAIL (%s), will retry\n", esp_err_to_name(ret));
                    file_monitor_mark_retry(e->path);
                }
                free(e);
            }
        }

        if (g_sta_count==0 && reg) {
            printf("[HOST] ***** SLAVE DISCONNECTED *****\n");
            web_server_clear_slave(); sip[0]=0; reg=false; file_monitor_reset();
        }

        if (tick%50==0)
            printf("[HOST] t=%d STA=%d SLV=%s PEND=%d\n", tick, g_sta_count, reg?sip:"none", file_monitor_pending_count());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#elif DEVICE_ROLE == DEVICE_ROLE_SLAVE
/* ========================= 从机端 ================================= */

static void slave_run(sdmmc_card_t *card)
{
    /* SD 直接挂载 */
    sd_card_mount_fatfs(card, "/SD", 5);
    printf("[SLAVE] SD mounted\n");

    /* 验证可写 */
    FILE *tf = fopen("/SD/_SLAVE_READY_.txt", "w");
    if (tf) { fprintf(tf, "SLAVE OK\n"); fclose(tf); printf("[SLAVE] Test file written OK\n"); }
    else printf("[SLAVE] WARN: cannot write test file\n");

    printf("[SLAVE] === LOOP START ===\n\n");
    int t=0;
    while (1) {
        led_pattern_tick(); slave_loop_tick(); t++;
        if (t%25==0)
            printf("[SLAVE] t=%d W=%s F=%d\n", t, slave_is_connected()?"OK":"NO", slave_get_files_received());
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
#endif

/* ====================================================================== */

void app_main(void)
{
    /* NVS */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }
    led_init(); led_set_pattern(LED_PAT_SOLID_ON);
    setvbuf(stdout, NULL, _IONBF, 0);

#if DEVICE_ROLE == DEVICE_ROLE_MASTER
    printf("\n");
    printf("========================================\n");
    printf("  WiFi_USB_SD [HOST] v1.8 NO-USB\n");
    printf("  WiFi AP + File Monitor + HTTP Push\n");
    printf("========================================\n\n");

    /* 1. WiFi AP */
    wifi_init_softap();
    vTaskDelay(pdMS_TO_TICKS(6000));

    /* 2. SD卡 */
    sdmmc_card_t *card = NULL;
    r = sd_card_init(&card);
    if (r != ESP_OK || card == NULL) {
        printf("[HOST] FATAL: SD init FAILED: %s\n", esp_err_to_name(r));
        led_set_pattern(LED_PAT_VERY_FAST_BLINK);
        while(1) { led_pattern_tick(); vTaskDelay(pdMS_TO_TICKS(100)); }
    }
    printf("[HOST] SD init OK\n");

    /* 3. 进入主循环 */
    host_run(card);

#elif DEVICE_ROLE == DEVICE_ROLE_SLAVE
    printf("\n");
    printf("========================================\n");
    printf("  WiFi_USB_SD [SLAVE] v1.8 NO-USB\n");
    printf("  WiFi STA + HTTP Receive Server\n");
    printf("========================================\n\n");

    /* 步骤1: SD卡 (先挂载, 后面WiFi连接) */
    sdmmc_card_t *card = NULL;
    r = sd_card_init(&card);
    if (r != ESP_OK || card == NULL) {
        printf("[SLAVE] FATAL: SD init FAILED: %s\n", esp_err_to_name(r));
        led_set_pattern(LED_PAT_VERY_FAST_BLINK);
        while(1) { led_pattern_tick(); vTaskDelay(pdMS_TO_TICKS(100)); }
    }
    printf("[SLAVE] SD init OK\n");

    /* ★ 步骤1.5: 等待主机AP就绪 (主机已上电, AP已在广播) ★ */
    printf("[SLAVE] Waiting for host AP to become ready (8s)...\n");
    vTaskDelay(pdMS_TO_TICKS(8000));

    /* 步骤2: WiFi STA (SD挂载后, 等待主机AP就绪) */
    esp_err_t sl_ret = slave_init();
    printf("[SLAVE] slave_init: %s\n", sl_ret==ESP_OK?"OK":"PARTIAL(OK)");

    /* 步骤3: 进入主循环 */
    slave_run(card);

#endif
}
