/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ONE 
 * @version     V1.0
 * @date        2026-06-11
 * @brief       USB MSC(大容量存储)驱动代码
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * @note        
 *
 ****************************************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "led.h"
#include "sd_card.h"
#include "usb_msc.h"


static const char *TAG = "AP";
#define EXAMPLE_ESP_WIFI_SSID   "123"
#define EXAMPLE_ESP_WIFI_PASS   "123456789"
#define EXAMPLE_MAX_STA_CONN    5
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"


/**
 * @brief       WIFI事件处理函数
 * @param       arg:传入网卡控制块
 * @param       event_base:WIFI事件
 * @param       event_id:事件ID
 * @param       event_data:事件数据
 * @retval      无
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    /* 设备连接 */
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    /* 设备断开 */
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/**
 * @brief       WIFI初始化
 * @param       无
 * @retval      无
 */
static void wifi_init_softap(void)
{
    /* 初始化网卡 */
    ESP_ERROR_CHECK(esp_netif_init());

    /* 创建新的事件循环 */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    /* 使用默认配置初始化包括netif的Wi-Fi */
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    /* 配置WIFI */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info);

    ESP_LOGI(TAG, "Set up softAP with IP: " IPSTR, IP2STR(&ip_info.ip));

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}

/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;

    /* 先初始化NVS和LED (不需要串口) */
    ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    led_init();
    LED(0);  /* 点亮LED表示上电 */

    /* 等待USB Serial/JTAG枚举完成 (LED常亮约3秒) */
    vTaskDelay(pdMS_TO_TICKS(3000));

    /* USB已就绪, 以下printf均可见 */
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("\n\n========== WiFi_USB_SD Starting ==========\n");
    printf("Chip: ESP32-S3 | SD: SDMMC 1-bit(CLK=36,CMD=35,D0=37)\n\n");

    /* ---- SD卡初始化 ---- */
    printf("[MAIN] Step 1/3: SD card init...\n");
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);

    if (sd_ret == ESP_OK && sd_card != NULL)
    {
        printf("[MAIN] Step 2/3: USB MSC init...\n");
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        usb_msc_mount("/sd");
        printf("[MAIN] USB MSC ready! Plug USB OTG cable to PC.\n");
    }
    else
    {
        printf("[MAIN] SD card FAILED: %s (code %d)\n", esp_err_to_name(sd_ret), sd_ret);
    }

    /* ---- WiFi AP 初始化 ---- */
    printf("[MAIN] Step 3/3: WiFi AP init...\n");
    wifi_init_softap();
    printf("[MAIN] WiFi AP: SSID='123' PASS='123456789'\n");

    /* ---- 主循环 ---- */
    bool sd_ok = (sd_ret == ESP_OK);
    int loop_count = 0;
    printf("[MAIN] Loop start. LED %s\n\n", sd_ok ? "slow(SD_OK)" : "fast(no_SD)");
    LED(1);  /* 熄灭LED, 进入闪烁模式 */

    while (1)
    {
        LED_TOGGLE();

        if (++loop_count >= 25)
        {
            loop_count = 0;
            printf("[LOOP] LED=%s SD=%s(%d)\n",
                   sd_ok ? "slow" : "fast",
                   sd_ok ? "OK" : esp_err_to_name(sd_ret), sd_ret);
        }

        vTaskDelay(sd_ok ? pdMS_TO_TICKS(200) : pdMS_TO_TICKS(100));
    }
}
