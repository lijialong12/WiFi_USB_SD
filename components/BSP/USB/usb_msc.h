/**
 ****************************************************************************************************
 * @file        usb_msc.h
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

#ifndef __USB_MSC_H
#define __USB_MSC_H

#include "esp_err.h"
#include "sdmmc_cmd.h"

/* 函数声明 */
esp_err_t usb_msc_init(sdmmc_card_t *card);     /* 初始化USB MSC设备(绑定SD卡) */
esp_err_t usb_msc_mount(const char *base_path); /* 挂载FATFS供ESP32本地访问   */
esp_err_t usb_msc_unmount(void);                /* 卸载FATFS, 让PC独占访问    */

#endif
