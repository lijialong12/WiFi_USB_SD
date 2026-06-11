/**
 ****************************************************************************************************
 * @file        sd_card.c
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

#include "sd_card.h"
#include "esp_log.h"
#include "driver/sdmmc_host.h"
#include <stdio.h>

static const char *TAG = "SD_CARD";


esp_err_t sd_card_init(sdmmc_card_t **out_card)
{
    esp_err_t ret;

    printf("[SD] Initializing SD card (SDMMC 4-bit mode)...\n");
    printf("[SD]   CLK: GPIO%d\n", SD_CLK);
    printf("[SD]   CMD: GPIO%d\n", SD_CMD);
    printf("[SD]   D0 : GPIO%d\n", SD_D0);
    printf("[SD]   D1 : GPIO%d\n", SD_D1);
    printf("[SD]   D2 : GPIO%d\n", SD_D2);
    printf("[SD]   D3 : GPIO%d\n", SD_D3);

    /* SDMMC主机配置: 高速40MHz */
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  /* 40MHz */

    /* SDMMC卡槽配置 */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    /* ESP32-S3 GPIO矩阵: 将SDMMC映射到自定义GPIO */
    slot_config.clk = SD_CLK;
    slot_config.cmd = SD_CMD;
    slot_config.d0  = SD_D0;
    slot_config.d1  = SD_D1;
    slot_config.d2  = SD_D2;
    slot_config.d3  = SD_D3;

    /* 4-bit模式 */
    slot_config.width = 4;
    /* 使能内部上拉 */
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    printf("[SD] Initializing SDMMC host...\n");
    ret = (*host.init)();

    if (ret != ESP_OK)
    {
        printf("[SD] Host init FAILED: %s\n", esp_err_to_name(ret));
        ESP_LOGE(TAG, "SDMMC host init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    printf("[SD] Host init OK\n");

    printf("[SD] Initializing SDMMC slot...\n");
    ret = sdmmc_host_init_slot(host.slot, (const sdmmc_slot_config_t *)&slot_config);

    if (ret != ESP_OK)
    {
        printf("[SD] Slot init FAILED: %s\n", esp_err_to_name(ret));
        ESP_LOGE(TAG, "SDMMC slot init failed: %s", esp_err_to_name(ret));
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)
            host.deinit_p(host.slot);
        else
            (*host.deinit)();
        return ret;
    }
    printf("[SD] Slot init OK\n");

    /* 增强GPIO驱动能力 (高速SD卡需要较强的驱动) */
    gpio_set_drive_capability(SD_CLK, GPIO_DRIVE_CAP_3);  /* CLK需要最强驱动 */
    gpio_set_drive_capability(SD_CMD, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(SD_D0,  GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(SD_D1,  GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(SD_D2,  GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(SD_D3,  GPIO_DRIVE_CAP_3);
    printf("[SD] GPIO drive strength set to max\n");

    /* 分配并初始化SD卡 */
    sdmmc_card_t *card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));

    if (card == NULL)
    {
        printf("[SD] malloc FAILED\n");
        sdmmc_host_deinit_slot(host.slot);
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)
            host.deinit_p(host.slot);
        else
            (*host.deinit)();
        return ESP_ERR_NO_MEM;
    }

    printf("[SD] Probing SD card (SDMMC, 40MHz)...\n");
    ret = sdmmc_card_init(&host, card);

    if (ret != ESP_OK)
    {
        printf("[SD] Card init FAILED: %s (code %d)\n", esp_err_to_name(ret), ret);
        ESP_LOGE(TAG, "sdmmc_card_init failed: %s", esp_err_to_name(ret));
        sdmmc_host_deinit_slot(host.slot);
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)
            host.deinit_p(host.slot);
        else
            (*host.deinit)();
        free(card);
        return ret;
    }

    sdmmc_card_print_info(stdout, card);
    *out_card = card;

    printf("[SD] Init DONE. Name=%s, Size=%lluMB\n",
           card->cid.name,
           ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024 * 1024));
    ESP_LOGI(TAG, "SD card init done");

    return ESP_OK;
}

esp_err_t sd_card_deinit(sdmmc_card_t *card)
{
    if (card != NULL) free(card);
    printf("[SD] Deinit done\n");
    return ESP_OK;
}
