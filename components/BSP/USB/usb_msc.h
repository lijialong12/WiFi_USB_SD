/**
 ****************************************************************************************************
 * @file        usb_msc.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-11
 * @brief       USB MSC(大容量存储)驱动头文件 - 函数声明
 *              提供: usb_msc_init(绑定SD卡为U盘)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * @note        VID=0x303A, PID=0x4002, USB 2.0 Full-Speed (12Mbps)
 *              使用TinyUSB协议栈 + FATFS文件系统
 *
 ****************************************************************************************************
 */

#ifndef __USB_MSC_H                           /* 头文件保护宏: 防止重复包含 */
#define __USB_MSC_H                           /* 定义保护宏标识 */

#include "esp_err.h"                          /* ESP-IDF错误类型: esp_err_t (ESP_OK=0, ESP_FAIL=-1等) */
#include "sdmmc_cmd.h"                        /* SDMMC协议栈类型: sdmmc_card_t (SD卡控制块) */


/* ======================== 函数声明 ======================== */

/**
 * @brief       初始化USB MSC设备(将SD卡注册为大容量存储)
 * @param       card: 已初始化的SD卡控制块指针(sd_card_init的返回值)
 * @retval      ESP_OK: 成功, USB线连接PC后可识别为U盘
 * @retval      ESP_ERR_INVALID_ARG: card为NULL
 * @retval      其他: TinyUSB驱动安装失败
 */
esp_err_t usb_msc_init(sdmmc_card_t *card);     /* 绑定SD卡 → 注册MSC描述符 → 安装TinyUSB驱动 */

#endif                                           /* __USB_MSC_H 头文件保护结束 */
