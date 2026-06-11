/**
 ****************************************************************************************************
 * @file        sd_card.h
 * @author      ONE 
 * @version     V1.0
 * @date        2026-06-11
 * @brief       SD卡驱动代码(SDMMC 4-bit高速模式)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * @note        
 *
 ****************************************************************************************************
 */

#ifndef __SD_CARD_H
#define __SD_CARD_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"

/* SD卡 SDMMC 引脚定义 (根据实际原理图)
 *   SD_CLK → GPIO36
 *   SD_CMD → GPIO35
 *   SD_D0  → GPIO37
 *   SD_D1  → GPIO38 (4-bit模式用)
 *   SD_D2  → GPIO33 (4-bit模式用)
 *   SD_D3  → GPIO34 (4-bit模式用)
 */
#define SD_CLK      GPIO_NUM_36
#define SD_CMD      GPIO_NUM_35
#define SD_D0       GPIO_NUM_37
#define SD_D1       GPIO_NUM_38
#define SD_D2       GPIO_NUM_33
#define SD_D3       GPIO_NUM_34

esp_err_t sd_card_init(sdmmc_card_t **out_card);
esp_err_t sd_card_deinit(sdmmc_card_t *card);

#endif
