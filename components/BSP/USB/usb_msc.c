/**
 ****************************************************************************************************
 * @file        usb_msc.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-11
 * @brief       USB MSC(大容量存储)驱动代码 - TinyUSB设备栈封装
 *              功能: ①注册USB设备描述符(VID/PID/字符串) → ②配置MSC接口描述符 →
 *                    ③初始化TinyUSB MSC存储层(绑定SD卡) → ④安装TinyUSB驱动 →
 *                    ⑤提供FATFS挂载/卸载API
 *              协议: USB 2.0 Full-Speed (12Mbps), MSC Bulk-Only Transport (BOT)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * USB OTG引脚: D-=GPIO19, D+=GPIO20 (ESP32-S3内置Full-Speed PHY)
 * VID=0x303A (Espressif), PID=0x4002
 * 设备类: MSC (Mass Storage Class), 使用Bulk-Only Transport协议
 * @note        文件系统使用FATFS(ESP-IDF内置), 挂载点"/sd"
 *              PC访问SD卡前需确保ESP32已调用usb_msc_unmount()卸载
 *
 ****************************************************************************************************
 */

#include "usb_msc.h"                        /* USB MSC驱动头文件: 函数声明 */
#include "esp_log.h"                        /* ESP-IDF日志系统: ESP_LOGI/ESP_LOGE */
#include "tinyusb.h"                        /* TinyUSB驱动核心: tinyusb_config_t, tinyusb_driver_install() */
#include "tusb_msc_storage.h"               /* TinyUSB MSC存储抽象层: tinyusb_msc_storage_init_sdmmc/mount/unmount */

static const char *TAG = "USB_MSC";         /* 日志标签: 用于ESP_LOGI/ESP_LOGE输出前缀 */

static bool g_msc_initialized = false;      /* USB MSC初始化状态标志: true=已初始化, false=未初始化 */


/* ========================================================================== */
/*                          USB描述符定义                                      */
/* 所有USB设备必须提供描述符, 主机通过描述符识别设备类型/厂商/产品             */
/* ========================================================================== */

#define EPNUM_MSC           1               /* MSC端点编号: 使用端点1(端点0固定为控制传输) */
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN) /* 配置描述符总长度: 配置描述符+MSC接口描述符 */

enum {                                      /* 接口编号枚举 */
    ITF_NUM_MSC = 0,                        /* MSC接口编号: 0(第一个也是唯一的接口) */
    ITF_NUM_TOTAL                           /* 接口总数: 自动计算为1 */
};

enum {                                      /* 端点地址枚举(含方向位) */
    EDPT_CTRL_OUT = 0x00,                   /* 控制端点输出地址: 0x00 (端点0, 方向: 主机→设备) */
    EDPT_CTRL_IN  = 0x80,                   /* 控制端点输入地址: 0x80 (端点0, 方向: 设备→主机, bit7=1=IN) */
    EDPT_MSC_OUT  = 0x01,                   /* MSC Bulk-OUT端点地址: 0x01 (端点1, 方向: 主机→设备, 写数据) */
    EDPT_MSC_IN   = 0x81,                   /* MSC Bulk-IN端点地址: 0x81 (端点1, 方向: 设备→主机, 读数据) */
};

/* ---------- USB设备描述符 ---------- */
/* 描述设备的总体信息: USB版本、设备类、VID/PID、制造商/产品/序列号字符串索引 */
static tusb_desc_device_t descriptor_config = {                         /* 定义USB设备描述符结构体 */
    .bLength            = sizeof(tusb_desc_device_t),                   /* 描述符自身长度: 标准设备描述符=18字节 */
    .bDescriptorType    = TUSB_DESC_DEVICE,                             /* 描述符类型: 设备描述符(=1) */
    .bcdUSB             = 0x0200,                                       /* USB协议版本: 2.0 (BCD编码, 0x0200=2.00) */
    .bDeviceClass       = TUSB_CLASS_MISC,                              /* 设备类: 杂项(使用IAD描述符组合多个接口类) */
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,                         /* 设备子类: 通用(配合IAD使用) */
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,                            /* 设备协议: IAD(Interface Association Descriptor) */
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,                       /* 端点0最大包长: 通常64字节(全速) */
    .idVendor           = 0x303A,                                       /* 厂商ID(VID): Espressif官方VID */
    .idProduct          = 0x4002,                                       /* 产品ID(PID): 自定义, 标识MSC设备 */
    .bcdDevice          = 0x100,                                        /* 设备版本号: 1.00 (BCD编码) */
    .iManufacturer      = 0x01,                                         /* 制造商字符串索引: 指向string_desc_arr[1]="Espressif" */
    .iProduct           = 0x02,                                         /* 产品字符串索引: 指向string_desc_arr[2]="ESP32-S3 MSC" */
    .iSerialNumber      = 0x03,                                         /* 序列号字符串索引: 指向string_desc_arr[3]="123456" */
    .bNumConfigurations = 0x01,                                         /* 配置数量: 1个配置(标准设备通常只有1个) */
};

/* ---------- MSC配置描述符(全速) ---------- */
/* 配置描述符 + 接口描述符 + 端点描述符, 以字节数组形式定义 */
static uint8_t const msc_fs_configuration_desc[] = {                    /* 全速配置描述符数组(存储在Flash中) */
    /* 配置描述符: 描述设备供电、接口数、属性 */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN,    /* 配置1, 接口总数, 配置值0, 描述符总长 */
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),     /* 支持远程唤醒, 总线供电最大电流=100mA×2=200mA */
    /* MSC接口描述符: 包含Bulk-IN和Bulk-OUT两个端点 */
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64), /* 接口号0, 备用设置0, OUT端点1, IN端点0x81, 端点最大包长64字节 */
};

/* ---------- USB字符串描述符 ---------- */
/* 字符串描述符数组: 索引0=语言列表, 1=制造商, 2=产品, 3=序列号, 4=接口名 */
static char const *string_desc_arr[] = {                                /* 字符串描述符指针数组 */
    (const char[]) { 0x09, 0x04 },                                      /* 索引0: 语言ID列表 → 英语(美国) 0x0409 (小端序: 0x09,0x04) */
    "Espressif",                                                        /* 索引1: 制造商字符串 → 设备管理器显示"Espressif" */
    "ESP32-S3 MSC",                                                     /* 索引2: 产品字符串 → 显示"ESP32-S3 MSC" */
    "123456",                                                           /* 索引3: 序列号字符串 → 唯一标识此设备 */
    "SD Card MSC",                                                      /* 索引4: MSC接口功能字符串 → 描述此接口用途 */
};


/* ========================================================================== */
/*                    TinyUSB MSC 事件回调                                      */
/* TinyUSB在存储挂载/卸载状态变化时调用此回调, 通知应用层                       */
/* ========================================================================== */

/**
 * @brief       FATFS挂载/卸载状态变化回调函数
 *              当主机(PC)通过USB MSC连接/断开时, TinyUSB自动调用此回调
 *              通知应用层当前存储是否被主机挂载
 * @param       event: MSC事件数据(含mount_changed_data.is_mounted标志)
 * @retval      无
 */
static void storage_mount_changed_cb(tinyusb_msc_event_t *event)        /* 回调函数: TinyUSB MSC层在挂载状态变化时调用 */
{
    ESP_LOGI(TAG, "Storage mounted to application: %s",                 /* 打印挂载状态: Yes=主机已挂载, No=主机已卸载 */
             event->mount_changed_data.is_mounted ? "Yes" : "No");      /* is_mounted=true→主机通过USB访问, false→ESP32本地可访问 */
}


/* ========================================================================== */
/*                           USB MSC 公开API                                   */
/* 对外提供3个接口: usb_msc_init / usb_msc_mount / usb_msc_unmount            */
/* ========================================================================== */

/**
 * @brief       初始化USB MSC设备, 将SD卡注册为USB大容量存储
 *              完整流程:
 *              ① 参数校验(card非空) → ② 初始化TinyUSB MSC存储层(注册diskio回调) →
 *              ③ 注册挂载状态变化回调 → ④ 安装TinyUSB驱动(注册描述符, 使能USB PHY)
 *              完成后: 用USB OTG线连接电脑 → 电脑识别为U盘 → 可读写SD卡
 * @param       card: 已初始化的SD卡控制块(由sd_card_init()返回, 不可为NULL)
 * @retval      ESP_OK: USB MSC初始化成功
 * @retval      ESP_ERR_INVALID_ARG: card参数为NULL
 * @retval      其他: TinyUSB驱动安装失败(由ESP_ERROR_CHECK捕获)
 */
esp_err_t usb_msc_init(sdmmc_card_t *card)
{
    printf("[USB_MSC] Initializing USB MSC...\n");                       /* 输出: USB MSC初始化开始 */

    /* ---------- 参数校验 ---------- */
    if (card == NULL)                                                    /* 检查SD卡指针有效性 */
    {
        printf("[USB_MSC] ERROR: Invalid card pointer (NULL)\n");        /* 输出错误提示 */
        ESP_LOGE(TAG, "Invalid card pointer (NULL)");                    /* 记录错误日志 */
        return ESP_ERR_INVALID_ARG;                                      /* 返回参数无效错误码 */
    }

    /* ---------- 步骤1: 初始化TinyUSB MSC存储抽象层 ---------- */
    /* 内部: 注册FATFS diskio函数 + 注册MSC SCSI命令处理回调 */
    printf("[USB_MSC] Initializing MSC storage layer...\n");             /* 输出: 正在初始化存储层 */
    const tinyusb_msc_sdmmc_config_t config_sdmmc = {                    /* 定义SDMMC MSC配置结构体 */
        .card = card,                                                    /* 绑定SD卡控制块(MSC读写操作最终操作此SD卡) */
        .callback_mount_changed = storage_mount_changed_cb,              /* 挂载状态变化回调(主机连接/断开时通知) */
        .mount_config.max_files = 5,                                     /* FATFS同时打开文件数上限(过大浪费RAM) */
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));      /* 初始化MSC存储层: 注册FATFS diskio → 注册SCSI处理 → 设置回调 */
    printf("[USB_MSC] MSC storage layer OK\n");                          /* 存储层初始化成功 */

    /* 也可用独立API注册/覆盖回调(此处覆盖之前的设置, 确保回调正确) */
    ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, /* 注册挂载变化事件回调 */
                                                   storage_mount_changed_cb));     /* 回调函数指针 */

    /* ---------- 步骤2: 安装TinyUSB设备驱动(使能USB PHY + 注册描述符) ---------- */
    printf("[USB_MSC] Installing TinyUSB driver (VID:0x%04X PID:0x%04X)...\n", /* 输出VID/PID用于识别 */
           descriptor_config.idVendor, descriptor_config.idProduct);    /* 0x303A:0x4002 */
    const tinyusb_config_t tusb_cfg = {                                  /* 定义TinyUSB驱动安装配置 */
        .device_descriptor = &descriptor_config,                         /* 指向设备描述符(VID/PID/设备类等) */
        .string_descriptor = string_desc_arr,                            /* 指向字符串描述符数组(制造商/产品/序列号) */
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]), /* 字符串描述符元素个数(5个) */
        .external_phy = false,                                           /* 不使用外部USB PHY芯片, 使用ESP32-S3内部全速PHY */
        .configuration_descriptor = msc_fs_configuration_desc,           /* 指向配置描述符数组(含MSC接口和端点) */
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));                  /* 安装TinyUSB驱动: 注册到USB OTG硬件, 拉高D+通知主机 */

    g_msc_initialized = true;                                            /* 标记MSC已初始化(供mount/unmount检查状态) */
    printf("[USB_MSC] Init done! Connect USB OTG cable (GPIO19=D-, GPIO20=D+) to PC.\n"); /* 提示用户可以连接USB线 */
    ESP_LOGI(TAG, "USB MSC initialization done - waiting for USB host connection..."); /* 记录成功日志 */
    return ESP_OK;                                                       /* 返回成功 */
}

/**
 * @brief       挂载FATFS文件系统, 供ESP32本地访问SD卡文件
 *              调用后ESP32可通过fopen/fread/fwrite等标准C文件API操作SD卡
 *              注意: 挂载期间PC通过USB MSC访问SD卡会造成文件系统冲突
 *              解决方案: PC访问前先调用usb_msc_unmount()卸载
 * @param       base_path: FATFS挂载点路径, 如"/sd" (必须以"/"开头)
 * @retval      ESP_OK: 挂载成功, ESP32可读写SD卡文件
 * @retval      ESP_ERR_INVALID_STATE: USB MSC尚未初始化, 无法挂载
 * @retval      其他: FATFS挂载失败(由ESP_ERROR_CHECK捕获)
 */
esp_err_t usb_msc_mount(const char *base_path)
{
    if (!g_msc_initialized)                                              /* 检查MSC是否已初始化 */
    {
        ESP_LOGE(TAG, "USB MSC not initialized, cannot mount");          /* 记录错误: 未初始化 */
        return ESP_ERR_INVALID_STATE;                                    /* 返回状态无效错误码 */
    }

    ESP_LOGI(TAG, "Mounting storage at %s...", base_path);               /* 输出挂载路径 */
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(base_path));               /* 调用TinyUSB MSC挂载函数: 内部调用f_mount() */
    ESP_LOGI(TAG, "Storage mounted at %s", base_path);                   /* 挂载成功 */

    return ESP_OK;                                                       /* 返回成功 */
}



/**
 * @brief       卸载FATFS文件系统, 将SD卡控制权交还给USB主机(PC)
 *              卸载后PC可通过USB MSC独占访问SD卡, 避免文件系统冲突
 *              使用场景: ①ESP32写完日志后卸载 → PC通过USB读取
 *                       ②PC通过USB弹出磁盘 → ESP32重新挂载继续写日志
 * @param       无
 * @retval      ESP_OK: 卸载成功, PC可安全访问SD卡
 * @retval      ESP_ERR_INVALID_STATE: USB MSC尚未初始化, 无法卸载
 * @retval      其他: FATFS卸载失败(由ESP_ERROR_CHECK捕获)
 */
esp_err_t usb_msc_unmount(void)
{
    if (!g_msc_initialized)                                              /* 检查MSC是否已初始化 */
    {
        ESP_LOGE(TAG, "USB MSC not initialized, cannot unmount");        /* 记录错误: 未初始化 */
        return ESP_ERR_INVALID_STATE;                                    /* 返回状态无效错误码 */
    }

    ESP_LOGI(TAG, "Unmounting storage...");                               /* 输出: 正在卸载 */
    ESP_ERROR_CHECK(tinyusb_msc_storage_unmount());                      /* 调用TinyUSB MSC卸载函数: 内部调用f_unmount(), 刷新缓存 */
    ESP_LOGI(TAG, "Storage unmounted, ready for USB host access");       /* 卸载成功, 提示PC可访问 */

    return ESP_OK;                                                       /* 返回成功 */
}
