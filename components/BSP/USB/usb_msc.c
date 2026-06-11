/**
 ****************************************************************************************************
 * @file        usb_msc.c
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

#include "usb_msc.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"

static const char *TAG = "USB_MSC";

static bool g_msc_initialized = false;      /* USB MSC是否已初始化 */


/* -------------------------------------------------------------------------- */
/*                          USB描述符定义                                     */
/* -------------------------------------------------------------------------- */

#define EPNUM_MSC           1
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,
    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

/* USB设备描述符 */
static tusb_desc_device_t descriptor_config = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,           /* Espressif VID */
    .idProduct          = 0x4002,
    .bcdDevice          = 0x100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

/* MSC配置描述符(全速) */
static uint8_t const msc_fs_configuration_desc[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

/* USB字符串描述符 */
static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },          /* 0: 支持语言英语(0x0409) */
    "Espressif",                            /* 1: 制造商 */
    "ESP32-S3 MSC",                         /* 2: 产品 */
    "123456",                               /* 3: 序列号 */
    "SD Card MSC",                          /* 4: MSC接口 */
};


/* -------------------------------------------------------------------------- */
/*                    TinyUSB MSC 事件回调                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief       FATFS挂载/卸载状态变化回调
 * @param       event: 事件数据
 * @retval      无
 */
static void storage_mount_changed_cb(tinyusb_msc_event_t *event)
{
    ESP_LOGI(TAG, "Storage mounted to application: %s",
             event->mount_changed_data.is_mounted ? "Yes" : "No");
}


/* -------------------------------------------------------------------------- */
/*                           USB MSC 公开API                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief       初始化USB MSC设备, 绑定SD卡
 * @param       card: 已初始化的SD卡控制块(由sd_card_init返回)
 * @retval      ESP_OK: 成功; 其他: 失败
 */
esp_err_t usb_msc_init(sdmmc_card_t *card)
{
    printf("[USB_MSC] Initializing USB MSC...\n");

    if (card == NULL)
    {
        printf("[USB_MSC] ERROR: Invalid card pointer (NULL)\n");
        ESP_LOGE(TAG, "Invalid card pointer (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. 初始化TinyUSB MSC存储层(内部注册FATFS diskio + MSC回调) */
    printf("[USB_MSC] Initializing MSC storage layer...\n");
    const tinyusb_msc_sdmmc_config_t config_sdmmc = {
        .card = card,
        .callback_mount_changed = storage_mount_changed_cb,
        .mount_config.max_files = 5,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));
    printf("[USB_MSC] MSC storage layer OK\n");

    /* 也可用独立API注册回调(会覆盖之前的) */
    ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED,
                                                   storage_mount_changed_cb));

    /* 2. 安装TinyUSB驱动(USB描述符) */
    printf("[USB_MSC] Installing TinyUSB driver (VID:0x%04X PID:0x%04X)...\n",
           descriptor_config.idVendor, descriptor_config.idProduct);
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,                              /* 使用ESP32-S3内部PHY */
        .configuration_descriptor = msc_fs_configuration_desc,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    g_msc_initialized = true;
    printf("[USB_MSC] Init done! Connect USB OTG cable (GPIO19=D-, GPIO20=D+) to PC.\n");
    ESP_LOGI(TAG, "USB MSC initialization done - waiting for USB host connection...");
    return ESP_OK;
}

/**
 * @brief       挂载FATFS供ESP32本地访问
 * @param       base_path: 挂载点(如"/sd")
 * @retval      ESP_OK: 成功; 其他: 失败
 * @note        调用后ESP32可通过标准C文件API访问SD卡文件.
 *              PC通过USB MSC访问时, 需先调用 usb_msc_unmount() 卸载.
 */
esp_err_t usb_msc_mount(const char *base_path)
{
    if (!g_msc_initialized)
    {
        ESP_LOGE(TAG, "USB MSC not initialized, cannot mount");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Mounting storage at %s...", base_path);
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(base_path));
    ESP_LOGI(TAG, "Storage mounted at %s", base_path);

    return ESP_OK;
}

/**
 * @brief       卸载FATFS, 将SD卡控制权交给PC(USB MSC)
 * @param       无
 * @retval      ESP_OK: 成功; 其他: 失败
 */
esp_err_t usb_msc_unmount(void)
{
    if (!g_msc_initialized)
    {
        ESP_LOGE(TAG, "USB MSC not initialized, cannot unmount");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Unmounting storage...");
    ESP_ERROR_CHECK(tinyusb_msc_storage_unmount());
    ESP_LOGI(TAG, "Storage unmounted, ready for USB host access");

    return ESP_OK;
}
