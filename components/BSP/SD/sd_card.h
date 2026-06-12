/**
 ****************************************************************************************************
 * @file        sd_card.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-11
 * @brief       SD卡驱动头文件 - 引脚定义 + 函数声明
 *              提供: ①SDMMC 6根信号线引脚宏 → ②sd_card_init/sd_card_deinit函数原型
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * 引脚映射(根据实际原理图):
 *   SD_CLK → GPIO36  (时钟线, 由主机驱动, 最高40MHz)
 *   SD_CMD → GPIO35  (命令/响应线, 双向, 开漏)
 *   SD_D0  → GPIO37  (数据线0, 1-bit和4-bit都使用)
 *   SD_D1  → GPIO38  (数据线1, 仅4-bit模式使用)
 *   SD_D2  → GPIO33  (数据线2, 仅4-bit模式使用)
 *   SD_D3  → GPIO34  (数据线3, 仅4-bit模式使用, 兼Card Detect)
 *
 * 引脚选取避开了:
 *   - PSRAM Quad SPI: GPIO26-32
 *   - USB OTG: GPIO19(D-), GPIO20(D+)
 *   - JTAG/USB-Serial: GPIO18, GPIO19
 * @note        所有信号线需外部上拉(10K~100K)至3.3V, 或使用内部上拉
 *
 ****************************************************************************************************
 */

#ifndef __SD_CARD_H                          /* 头文件保护宏: 防止重复包含 */
#define __SD_CARD_H                          /* 定义保护宏标识 */

#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t (ESP_OK=0, ESP_FAIL=-1等) */
#include "driver/gpio.h"                     /* ESP-IDF GPIO驱动: gpio_num_t枚举, GPIO_NUM_xx定义 */
#include "sdmmc_cmd.h"                       /* ESP-IDF SDMMC协议栈: sdmmc_card_t/sdmmc_host_t等 */


/* ======================== SDMMC 引脚定义 ======================== */
/* 根据实际硬件原理图: 所有引脚均为GPIO编号 */
#define SD_CLK      GPIO_NUM_36              /* SD卡时钟线: 主机输出, 频率40MHz, 需要最强驱动能力 */
#define SD_CMD      GPIO_NUM_35              /* SD卡命令/响应线: 双向, 开漏模式, 需上拉电阻 */
#define SD_D0       GPIO_NUM_37              /* SD卡数据线0: 双向, 1-bit和4-bit模式必需 */
#define SD_D1       GPIO_NUM_38              /* SD卡数据线1: 双向, 仅4-bit模式(与D0组成半字节) */
#define SD_D2       GPIO_NUM_33              /* SD卡数据线2: 双向, 仅4-bit模式 */
#define SD_D3       GPIO_NUM_34              /* SD卡数据线3: 双向, 4-bit模式 + 兼做Card Detect检测 */


/* ======================== 函数声明 ======================== */
esp_err_t sd_card_init(sdmmc_card_t **out_card);    /* 初始化SD卡: SDMMC 4-bit 40MHz, 成功时*out_card指向已分配的控制块 */
esp_err_t sd_card_deinit(sdmmc_card_t *card);       /* 反初始化SD卡: 释放卡槽/主机/内存, 参数为sd_card_init返回的指针 */
esp_err_t sd_card_mount_fatfs(sdmmc_card_t *card, const char *base_path, int max_files); /* 直接挂载FATFS(不经过USB MSC), 挂载后可fopen访问 */

#endif                                           /* __SD_CARD_H 头文件保护结束 */
