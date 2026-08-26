/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ONE
 * @version     V1.1
 * @date        2026-06-11
 * @brief       主程序入口 - USB MSC + WiFi AP + Web文件服务器 综合应用
 *              功能: ①初始化SD卡(SDMMC 4-bit模式) → ②注册USB MSC设备 →
 *                    ③启动WiFi AP热点 → ④启动Web文件服务器(端口80) →
 *                    ⑤主循环LED状态指示
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * 硬件连接: SD_CLK=GPIO36, SD_CMD=GPIO35, SD_D0~D3=GPIO37/38/33/34
 *           USB_OTG: D-=GPIO19, D+=GPIO20
 *           LED: GPIO1
 * 工作流程: 上电 → LED亮 → SD初始化 → USB MSC注册 → WiFi AP启动 →
 *           Web服务器启动 → 主循环(模式切换防抖 + LED闪烁: 慢闪=SD正常, 快闪=SD失败)
 * @note
 *
 ****************************************************************************************************
 */

/* ======================== 模式开关 ======================== */
/* 设为1启用USB MSC(大容量存储), 但仅当板子有独立OTG USB口时可用    */
/* 设为0禁用USB MSC, 直接挂载FATFS, 仅通过WiFi Web访问SD卡         */
#define USE_USB_MSC  1   /* 1=USB+WiFi双模式(弹出U盘即切到WiFi) */

/* ======================== 头文件包含 ======================== */
#include <stdio.h>                          /* 标准I/O: printf, setvbuf */
#include <stdlib.h>                         /* 标准库: malloc, free (间接使用) */
#include <string.h>                         /* 字符串操作: strlen */
#include "freertos/FreeRTOS.h"              /* FreeRTOS内核: 任务调度、延时 */
#include "freertos/task.h"                  /* FreeRTOS任务API: vTaskDelay */
#include "freertos/event_groups.h"          /* FreeRTOS事件组: 任务同步(保留) */
#include "esp_system.h"                     /* ESP32系统API: 芯片信息 */
#include "esp_log.h"                        /* ESP-IDF日志系统: ESP_LOGI/ESP_LOGE */
#include "nvs_flash.h"                      /* NVS(非易失性存储) Flash: WiFi配置存储 */
#include "nvs.h"                            /* NVS读写API: nvs_open/nvs_get_str/nvs_set_str */
#include "esp_wifi.h"                       /* ESP32 WiFi驱动: AP/STA模式 */
#include "esp_event.h"                      /* ESP-IDF事件循环: WiFi事件处理 */
#include "led.h"                            /* BSP-LED驱动: LED初始化/控制宏 */
#include "sd_card.h"                        /* BSP-SD卡驱动: SDMMC初始化/FATFS挂载 */
#if USE_USB_MSC
#include "usb_msc.h"                        /* BSP-USB MSC驱动: USB大容量存储设备 */
#include "tusb.h"                           /* TinyUSB: tud_connect/disconnect */
#include "tusb_msc_storage.h"               /* TinyUSB MSC: mount/unmount状态 */
#endif
#include "web_server.h"                     /* Web文件服务器: HTTP文件管理API */


/* ======================== WiFi AP 配置 ======================== */
static const char *TAG = "AP";              /* 日志标签: 用于ESP_LOGI/ESP_LOGE输出前缀 */
#define EXAMPLE_ESP_WIFI_SSID   "BOSSCOM_USB_AP"    /* WiFi AP热点名称(SSID) */
#define EXAMPLE_ESP_WIFI_PASS   "012345678"         /* WiFi AP密码(至少8位) */
#define EXAMPLE_MAX_STA_CONN    1                    /* AP最大同时连接客户端数 */
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]  /* MAC地址拆分为6字节(用于printf) */
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"                      /* MAC地址格式化字符串 */


/* ======================== USB/WiFi 动态切换 ======================== */
static volatile int g_sta_count = 0;    /* 当前WiFi客户端数(volatile: WiFi事件任务写/主循环读) */
static bool g_wifi_mode       = false;   /* true=WiFi网页模式, false=USB U盘模式 */
static int  g_stable_cnt      = 0;       /* 防抖计数器(可为负: 挂载失败退避) */
#define DEBOUNCE_WIFI  15                /* 连WiFi后等3秒切到网页模式 (15×200ms) */
#define DEBOUNCE_USB   20                /* 断WiFi后等4秒切回U盘模式 (20×200ms) */

/**
 * @brief       切换到WiFi网页模式: 断开USB MSC, 挂载FATFS到本地
 * @retval      true: 切换成功(FATFS已挂载本地)
 * @retval      false: 挂载失败(调用方应退避重试)
 */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    printf("[MODE] → WiFi mode (USB disconnect + local mount)\n");
    tud_disconnect();                               /* PC看到U盘移除 */
    vTaskDelay(pdMS_TO_TICKS(800));                 /* 等PC处理断开 */
    /* 先卸载再挂载, 确保VFS/FATFS状态干净 (TinyUSB MSC层可能已部分卸载但is_fat_mounted仍为true) */
    tinyusb_msc_storage_unmount();
    esp_err_t ret = tinyusb_msc_storage_mount("/sd");
    if (ret == ESP_OK) {
        g_wifi_mode = true;
        printf("[MODE] WiFi mode OK - web server can access SD\n");
        return true;
    }
    printf("[MODE] WiFi mode mount FAILED: %s\n", esp_err_to_name(ret));
    return false;
#else
    return true;
#endif
}

/**
 * @brief       切换到USB U盘模式: 卸载本地FATFS, 连接USB MSC
 */
static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    printf("[MODE] → USB mode (local unmount + USB connect)\n");
    tinyusb_msc_storage_unmount();
    g_wifi_mode = false;
    vTaskDelay(pdMS_TO_TICKS(300));
    tud_connect();                                  /* PC看到U盘插入 */
    printf("[MODE] USB mode OK - PC can access SD as USB drive\n");
#endif
}

/**
 * @brief       查询当前是否为WiFi网页模式(供web_server判断SD卡当前归属)
 * @retval      true: WiFi网页模式(FATFS挂在ESP32本地, Web可访问)
 * @retval      false: USB U盘模式(SD卡由PC通过USB MSC访问)
 */
bool app_in_wifi_mode(void)
{
    return g_wifi_mode;
}

/**
 * @brief       WiFi事件处理回调: 仅统计客户端数量, 切换在main循环中做防抖
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)event_data;
        g_sta_count++;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d (total:%d)",
                 MAC2STR(e->mac), e->aid, g_sta_count);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)event_data;
        if (g_sta_count > 0) g_sta_count--;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d (total:%d)",
                 MAC2STR(e->mac), e->aid, g_sta_count);
    }
}

/**
 * @brief       WiFi SoftAP(软接入点)初始化函数
 *              配置ESP32-S3为WiFi热点模式, 允许其他设备连接
 *              步骤: ①初始化TCP/IP协议栈 → ②创建事件循环 → ③创建默认AP网卡 →
 *                    ④初始化WiFi驱动 → ⑤注册事件回调 → ⑥配置SSID/密码 →
 *                    ⑦设置为AP模式 → ⑧启动WiFi
 * @param       无
 * @retval      无
 */
static void wifi_init_softap(void)
{
    /* 步骤1: 初始化TCP/IP网络协议栈(netif层) */
    ESP_ERROR_CHECK(esp_netif_init());                                  /* 初始化LwIP网络接口, 必须最先调用 */

    /* 步骤2: 创建系统默认事件循环(用于WiFi/IP事件分发) */
    ESP_ERROR_CHECK(esp_event_loop_create_default());                   /* 创建默认事件循环, WiFi事件通过此循环投递到回调 */

    /* 步骤3: 使用默认配置创建WiFi AP网络接口 */
    esp_netif_create_default_wifi_ap();                                 /* 创建并注册默认的WiFi AP netif实例(WIFI_AP_DEF) */

    /* 步骤4: 初始化WiFi驱动(分配资源、配置PHY) */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();                /* 获取WiFi初始化的默认配置(含操作系统相关参数) */
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                               /* 用默认配置初始化WiFi硬件驱动层 */

    /* 步骤5: 注册WiFi事件处理回调(监听所有WiFi事件) */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL)); /* 注册回调: 匹配所有WIFI_EVENT */

    /* ===== 从NVS读取WiFi配置(已保存的用户设置), 读不到则用默认值 ===== */
    char nvs_ssid[32] = {0};                                             /* NVS读取的SSID缓冲区(max 31 + null) */
    char nvs_pass[64] = {0};                                             /* NVS读取的密码缓冲区(max 63 + null) */
    nvs_handle_t nvs_h;
    if (nvs_open("wifi_config", NVS_READONLY, &nvs_h) == ESP_OK) {
        size_t len;
        /* 读取SSID */
        len = sizeof(nvs_ssid);
        if (nvs_get_str(nvs_h, "ssid", nvs_ssid, &len) != ESP_OK) {
            strcpy(nvs_ssid, EXAMPLE_ESP_WIFI_SSID);                    /* NVS无数据: 用默认值 */
        }
        /* 读取密码 */
        len = sizeof(nvs_pass);
        if (nvs_get_str(nvs_h, "password", nvs_pass, &len) != ESP_OK) {
            strcpy(nvs_pass, EXAMPLE_ESP_WIFI_PASS);                    /* NVS无数据: 用默认值 */
        }
        nvs_close(nvs_h);
        printf("[AP] NVS config loaded: SSID='%s'\n", nvs_ssid);
    } else {
        /* NVS namespace不存在(首次开机): 使用默认值 */
        strcpy(nvs_ssid, EXAMPLE_ESP_WIFI_SSID);
        strcpy(nvs_pass, EXAMPLE_ESP_WIFI_PASS);
        printf("[AP] NVS not found, using defaults: SSID='%s'\n", nvs_ssid);
    }

    /* 步骤6: 配置AP参数(SSID、密码、加密方式、最大连接数) */
    wifi_config_t wifi_config = {                                       /* 定义WiFi配置结构体(使用C99指定初始化器) */
        .ap = {                                                         /* AP模式专用配置字段 */
            .ssid_len = strlen(nvs_ssid),                               /* 设置SSID字符串长度 */
            .max_connection = EXAMPLE_MAX_STA_CONN,                     /* 设置最大同时连接数(硬件限制通常为4~10) */
            .authmode = WIFI_AUTH_WPA_WPA2_PSK                         /* 设置加密认证模式: WPA/WPA2-PSK混合 */
        },
    };
    /* 将NVS读取的SSID和密码复制到wifi_config结构体 */
    memcpy(wifi_config.ap.ssid, nvs_ssid, sizeof(wifi_config.ap.ssid));
    memcpy(wifi_config.ap.password, nvs_pass, sizeof(wifi_config.ap.password));

    /* 步骤7: 如果密码为空, 则设置为开放网络(无加密) */
    if (strlen(nvs_pass) == 0)                                          /* 判断密码长度是否为0 */
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;                       /* 设为开放模式(Open), 不需要密码即可连接 */
    }

    /* 步骤8: 设置WiFi工作模式为AP并应用配置 */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));                   /* 设置WiFi模式为纯AP(接入点)模式 */
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config)); /* 将SSID/密码等配置应用到AP接口 */
    ESP_ERROR_CHECK(esp_wifi_start());                                  /* 启动WiFi硬件, AP开始广播Beacon帧 */

    /* 步骤9: 获取并打印AP的IP地址信息 */
    esp_netif_ip_info_t ip_info;                                        /* 定义IP信息结构体(含IP/网关/掩码) */
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_DEF"), &ip_info); /* 通过接口键名获取netif句柄, 再读取IP信息 */

    ESP_LOGI(TAG, "Set up softAP with IP: " IPSTR, IP2STR(&ip_info.ip)); /* 打印AP的IPv4地址(默认192.168.4.1) */

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:'%s' password:'%s'", /* 打印WiFi AP配置完成信息 */
             nvs_ssid, nvs_pass);                                         /* 输出实际使用的SSID和密码供调试确认 */
}

/**
 * @brief       应用程序主入口(app_main)
 *              ESP-IDF自动调用, 等价于C标准main()函数
 *              初始化顺序: NVS → LED → 等待USB枚举 → SD卡 → USB MSC →
 *                          WiFi AP → Web服务器 → 主循环
 * @param       无
 * @retval      无(FreeRTOS任务中永不返回)
 */
void app_main(void)
{
    esp_err_t ret;                                                      /* ESP-IDF错误码变量(ESP_OK=0表示成功) */

    /* ===== 阶段0: NVS Flash初始化(存储WiFi配置等非易失数据) ===== */
    ret = nvs_flash_init();                                             /* 初始化NVS(非易失性存储)分区 */

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) /* 如果NVS分区已满或版本不兼容 */
    {
        ESP_ERROR_CHECK(nvs_flash_erase());                             /* 擦除整个NVS分区(清除旧数据) */
        ret = nvs_flash_init();                                         /* 重新初始化NVS(此时应为空) */
    }

    /* ===== 阶段0.5: LED初始化(最早的视觉反馈) ===== */
    led_init();                                                         /* 初始化GPIO1为LED控制引脚(输入输出模式+上拉) */
    LED(1);                                                             /* 点亮LED(高电平), 表示系统已上电启动 */

    /* ===== 阶段0.7: 等待USB Serial/JTAG枚举完成 ===== */
    //vTaskDelay(pdMS_TO_TICKS(2000));                                    /* 已禁用以加快U盘出现速度; 若串口日志丢失可恢复此延时(2000ms) */

    /* ===== 阶段0.8: 重定向标准输出 ===== */
    setvbuf(stdout, NULL, _IONBF, 0);                                   /* 设置stdout为无缓冲模式(printf立即输出, 不掉数据) */

    /* 打印启动横幅 */
    printf("\n\n========== WiFi_USB_SD Starting ==========\n");         /* 输出启动分隔线和项目名称 */
    printf("Chip: ESP32-S3 | SD: SDMMC 4-bit(CLK=36, CMD=35, D0~D3=37/38/33/34)\n\n"); /* 输出芯片型号和SD卡引脚映射信息 */

    /* ===== 阶段1: SD卡初始化(SDMMC 4-bit模式, 40MHz) ===== */
    printf("[MAIN] Step 1/3: SD card init...\n");                       /* 输出进度: 第1步SD卡初始化 */
    sdmmc_card_t *sd_card = NULL;                                       /* 定义SD卡控制块指针(初始为空) */
    esp_err_t sd_ret = sd_card_init(&sd_card);                          /* 调用BSP驱动初始化SD卡, 获取操作结果和卡控制块 */

    if (sd_ret == ESP_OK && sd_card != NULL)
    {
#if USE_USB_MSC
        printf("[MAIN] Step 2/3: USB MSC init + FATFS mount...\n");
        ESP_ERROR_CHECK(usb_msc_init(sd_card));
        ESP_ERROR_CHECK(usb_msc_mount("/sd"));
        printf("[MAIN] USB MSC ready. PC→U盘 | 弹出U盘→WiFi访问\n");
#else
        printf("[MAIN] Step 2/3: FATFS mount (direct)...\n");
        esp_err_t mnt_ret = sd_card_mount_fatfs(sd_card, "/sd", 5);
        if (mnt_ret != ESP_OK) {
            printf("[MAIN] FATFS mount FAILED: %s\n", esp_err_to_name(mnt_ret));
            sd_ret = mnt_ret;
        }
#endif
    }
    else
    {
        printf("[MAIN] SD card FAILED: %s (code %d)\n", esp_err_to_name(sd_ret), sd_ret);
        printf("[MAIN] File access will NOT be available\n");
    }

    /* ===== 阶段2: WiFi AP 初始化 ===== */
    printf("[MAIN] Step 3/3: WiFi AP init...\n");                       /* 输出进度: 第3步WiFi AP初始化 */
    wifi_init_softap();                                                 /* 启动WiFi SoftAP: SSID='BOSSCOM_USB_AP' */
    printf("[MAIN] WiFi AP: SSID='BOSSCOM_USB_AP' PASS='012345678'\n"); /* 输出WiFi AP的SSID和密码供用户连接 */

    /* ===== 阶段2.5: Web文件服务器启动 (端口80, http://192.168.4.1) ===== */
    {
        printf("[MAIN] Step 2.5/3: Web server start...\n");
        esp_err_t ws_ret = web_server_start();
        if (ws_ret == ESP_OK) {
            printf("[MAIN] Web server ready: http://192.168.4.1\n");
        } else {
            printf("[MAIN] Web server FAILED: %s\n", esp_err_to_name(ws_ret));
        }
    }

    /* ===== 阶段3: 主循环(LED + 模式切换防抖 + 周期日志) ===== */
    bool sd_ok = (sd_ret == ESP_OK);
    int loop_count = 0;
#if USE_USB_MSC
    g_wifi_mode = false;  /* 初始: USB U盘模式 (PC可看到) */
#endif
    printf("[MAIN] Loop start. LED %s | USB+WiFi auto-switch\n\n", sd_ok ? "slow" : "fast");
    LED(0);

    while (1)
    {
        LED_TOGGLE();

#if USE_USB_MSC
        /* ---- 防抖切换逻辑 ---- */
        if (sd_ok) {
            if (g_sta_count > 0 && !g_wifi_mode) {
                /* 有人连WiFi → 想切到网页模式 */
                g_stable_cnt++;
                if (g_stable_cnt >= DEBOUNCE_WIFI) {
                    if (switch_to_wifi_mode()) {
                        g_stable_cnt = 0;
                    } else {
                        /* 挂载失败: 置负值退避, 再等一个完整防抖周期后重试, 避免高频刷屏 */
                        g_stable_cnt = -DEBOUNCE_WIFI;
                    }
                }
            } else if (g_sta_count == 0 && g_wifi_mode) {
                /* 没人连WiFi → 想切回U盘模式 */
                g_stable_cnt++;
                if (g_stable_cnt >= DEBOUNCE_USB) {
                    switch_to_usb_mode();
                    g_stable_cnt = 0;
                }
            } else {
                g_stable_cnt = 0;  /* 状态稳定, 重置计数器 */
            }
        }
#endif

        if (++loop_count >= 25) {
            loop_count = 0;
#if USE_USB_MSC
            printf("[LOOP] LED=%s MODE=%s STA=%d\n",
                   sd_ok ? "slow" : "fast",
                   g_wifi_mode ? "WiFi" : "USB",
                   g_sta_count);
#else
            printf("[LOOP] LED=%s SD=%s(%d)\n",
                   sd_ok ? "slow" : "fast",
                   sd_ok ? "OK" : esp_err_to_name(sd_ret), sd_ret);
#endif
        }

        vTaskDelay(sd_ok ? pdMS_TO_TICKS(200) : pdMS_TO_TICKS(100));
    }
}
