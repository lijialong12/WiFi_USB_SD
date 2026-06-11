/**
 ****************************************************************************************************
 * @file        sd_card.h
 * @author      ONE 
 * @version     V1.0
 * @date        2026-06-11
 * @brief       SD卡驱动代码(SDMMC 1-bit模式)
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
 *   SD_CLK → GPIO37 (pin 41)
 *   SD_CMD → GPIO36 (pin 40)
 *   SD_D0  → GPIO38 (pin 42)
 *   SD_D1  → GPIO39 (pin 43, 4-bit模式用)
 *   SD_D2  → GPIO34 (pin 38, 4-bit模式用)
 *   SD_D3  → GPIO35 (pin 39, SPI模式CS)
 */
/* SD卡 SDMMC 引脚定义 (根据实际原理图)
 *   GPIO33 → SD_D2  (4-bit模式用)
 *   GPIO34 → SD_D3  (4-bit模式用)
 *   GPIO35 → SD_CMD
 *   GPIO36 → SD_CLK
 *   GPIO37 → SD_D0
 *   GPIO38 → SD_D1  (4-bit模式用)
 */
#define SD_CLK      GPIO_NUM_36
#define SD_CMD      GPIO_NUM_35
#define SD_D0       GPIO_NUM_37

esp_err_t sd_card_init(sdmmc_card_t **out_card);
esp_err_t sd_card_deinit(sdmmc_card_t *card);

#endif
