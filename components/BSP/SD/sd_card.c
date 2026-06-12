/**
 ****************************************************************************************************
 * @file        sd_card.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-11
 * @brief       SD卡驱动代码 - SDMMC 4-bit高速模式(40MHz)
 *              功能: ①初始化SDMMC主机 → ②配置GPIO矩阵映射 → ③初始化SD卡槽 →
 *                    ④增强GPIO驱动能力 → ⑤探测/初始化SD卡 → ⑥反初始化
 *              硬件: SD_CLK=GPIO36, SD_CMD=GPIO35, SD_D0=GPIO37,
 *                    SD_D1=GPIO38, SD_D2=GPIO33, SD_D3=GPIO34
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * SDMMC 4-bit模式理论带宽: 40MHz×4bit = 160Mbps ≈ 20MB/s
 * 引脚选取原则: 避开PSRAM(Quad SPI)占用的GPIO26-32, 避开USB OTG(GPIO19-20)
 * @note        驱动能力设为GPIO_DRIVE_CAP_3(最大值), 保证高速信号完整性
 *
 ****************************************************************************************************
 */

#include "sd_card.h"                        /* SD卡驱动头文件: 引脚宏定义 + 函数声明 */
#include "esp_log.h"                        /* ESP-IDF日志系统: ESP_LOGI/ESP_LOGE */
#include "driver/sdmmc_host.h"              /* ESP-IDF SDMMC主机驱动: sdmmc_host_t/sdmmc_card_t/sdmmc_card_init */
#include "esp_vfs_fat.h"                    /* ESP-IDF FATFS VFS: esp_vfs_fat_register */
#include "diskio_sdmmc.h"                   /* FATFS SDMMC diskio: ff_diskio_register_sdmmc */
#include "diskio_impl.h"                    /* FATFS diskio: ff_diskio_get_drive, ff_diskio_unregister */
#include "ff.h"                             /* FATFS: f_mount, FATFS, FRESULT */
#include <stdio.h>                          /* 标准I/O: printf */
#include <string.h>                         /* 字符串操作: strcmp */
#include <dirent.h>                         /* POSIX目录: opendir, readdir, closedir */

static const char *TAG = "SD_CARD";         /* 日志标签: 用于ESP_LOGI/ESP_LOGE输出前缀 */


/**
 * @brief       初始化SD卡(SDMMC 4-bit高速模式, 40MHz)
 *              完整初始化流程:
 *              ① 创建SDMMC主机配置(40MHz) → ② 配置卡槽(GPIO映射+4-bit) →
 *              ③ 初始化主机硬件 → ④ 初始化卡槽 → ⑤ 增强所有6根信号线驱动能力 →
 *              ⑥ 动态分配sdmmc_card_t → ⑦ 探测并初始化SD卡 → ⑧ 返回卡控制块
 * @param       out_card: [输出] SD卡控制块指针的指针, 初始化成功后指向已分配的sdmmc_card_t
 *                        调用者负责最终调用sd_card_deinit()释放
 * @retval      ESP_OK: 初始化成功
 * @retval      ESP_FAIL/ESP_ERR_NO_MEM/其他: 初始化失败(主机/卡槽/内存/卡通信错误)
 */
esp_err_t sd_card_init(sdmmc_card_t **out_card)
{
    esp_err_t ret;                                                      /* ESP错误码缓存变量 */

    /* ---------- 打印引脚映射信息(调试用) ---------- */
    printf("[SD] Initializing SD card (SDMMC 4-bit mode)...\n");       /* 输出初始化开始提示 */
    printf("[SD]   CLK: GPIO%d\n", SD_CLK);                             /* 打印SD_CLK引脚号(GPIO36) */
    printf("[SD]   CMD: GPIO%d\n", SD_CMD);                             /* 打印SD_CMD引脚号(GPIO35) */
    printf("[SD]   D0 : GPIO%d\n", SD_D0);                              /* 打印SD_D0引脚号(GPIO37) */
    printf("[SD]   D1 : GPIO%d\n", SD_D1);                              /* 打印SD_D1引脚号(GPIO38) */
    printf("[SD]   D2 : GPIO%d\n", SD_D2);                              /* 打印SD_D2引脚号(GPIO33) */
    printf("[SD]   D3 : GPIO%d\n", SD_D3);                              /* 打印SD_D3引脚号(GPIO34) */

    /* ---------- 步骤1: SDMMC主机配置 ---------- */
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();                           /* 获取SDMMC主机默认配置(SDMMC_HOST_SLOT_1, 3.3V, 20MHz等) */
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;                           /* 覆盖频率为高速模式: 40MHz(默认20MHz→40MHz, 速度翻倍) */

    /* ---------- 步骤2: SDMMC卡槽配置 ---------- */
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();      /* 获取卡槽默认配置(1-bit模式, 默认GPIO, 无上拉) */

    /* 将SDMMC信号线映射到自定义GPIO(ESP32-S3 GPIO矩阵灵活路由) */
    slot_config.clk = SD_CLK;                                           /* 时钟线映射到GPIO36 */
    slot_config.cmd = SD_CMD;                                           /* 命令/响应线映射到GPIO35 */
    slot_config.d0  = SD_D0;                                            /* 数据线0映射到GPIO37(1-bit和4-bit都必需) */
    slot_config.d1  = SD_D1;                                            /* 数据线1映射到GPIO38(4-bit模式用) */
    slot_config.d2  = SD_D2;                                            /* 数据线2映射到GPIO33(4-bit模式用) */
    slot_config.d3  = SD_D3;                                            /* 数据线3映射到GPIO34(4-bit模式用) */

    slot_config.width = 4;                                              /* 设置总线宽度为4-bit模式(4根数据线并行传输) */
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;               /* 使能内部上拉: CMD和D0-D3需要上拉(Pull-Up)以保持总线空闲高电平 */

    /* ---------- 步骤3: 初始化SDMMC主机硬件 ---------- */
    printf("[SD] Initializing SDMMC host...\n");                        /* 输出: 正在初始化主机 */
    ret = (*host.init)();                                               /* 调用主机初始化函数指针(初始化SDMMC外设时钟/复位) */

    if (ret != ESP_OK)                                                  /* 主机初始化失败 */
    {
        printf("[SD] Host init FAILED: %s\n", esp_err_to_name(ret));    /* 输出错误名称(如"ESP_ERR_INVALID_STATE") */
        ESP_LOGE(TAG, "SDMMC host init failed: %s", esp_err_to_name(ret)); /* 同时记录到ESP-IDF日志系统 */
        return ret;                                                     /* 提前返回错误码, 不继续后续步骤 */
    }

    printf("[SD] Host init OK\n");                                      /* 主机初始化成功 */
    printf("[SD] Initializing SDMMC slot...\n");                        /* 输出: 正在初始化卡槽 */

    /* ---------- 步骤4: 初始化SDMMC卡槽(配置GPIO/上拉/宽度) ---------- */
    ret = sdmmc_host_init_slot(host.slot, (const sdmmc_slot_config_t *)&slot_config); /* 将卡槽配置写入硬件(GPIO矩阵+上拉寄存器) */

    if (ret != ESP_OK)                                                  /* 卡槽初始化失败 */
    {
        printf("[SD] Slot init FAILED: %s\n", esp_err_to_name(ret));    /* 输出错误名称 */
        ESP_LOGE(TAG, "SDMMC slot init failed: %s", esp_err_to_name(ret)); /* 记录错误日志 */
        /* 清理: 根据主机标志位选择正确的反初始化方式 */
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)                    /* 如果主机驱动需要传slot参数反初始化 */
            host.deinit_p(host.slot);                                   /* 调用带参数的反初始化函数指针 */
        else                                                            /* 否则 */
            (*host.deinit)();                                           /* 调用无参数的反初始化函数指针 */
        return ret;                                                     /* 返回错误码 */
    }
    printf("[SD] Slot init OK\n");                                      /* 卡槽初始化成功 */

    /* ---------- 步骤5: 增强GPIO驱动能力 ---------- */
    /* 高速SD卡(40MHz)需要较强的GPIO驱动能力, 以保证信号上升/下降时间满足时序要求 */
    gpio_set_drive_capability(SD_CLK, GPIO_DRIVE_CAP_3);                /* CLK时钟线需要最强驱动: 频率最高, 负载最大 */
    gpio_set_drive_capability(SD_CMD, GPIO_DRIVE_CAP_3);                /* CMD命令线: 双向信号, 需强驱动克服总线电容 */
    gpio_set_drive_capability(SD_D0,  GPIO_DRIVE_CAP_3);                /* D0数据线: 双向数据 */
    gpio_set_drive_capability(SD_D1,  GPIO_DRIVE_CAP_3);                /* D1数据线 */
    gpio_set_drive_capability(SD_D2,  GPIO_DRIVE_CAP_3);                /* D2数据线 */
    gpio_set_drive_capability(SD_D3,  GPIO_DRIVE_CAP_3);                /* D3数据线 */
    printf("[SD] GPIO drive strength set to max\n");                    /* 驱动能力设置完成 */

    /* ---------- 步骤6: 动态分配SD卡控制块内存 ---------- */
    sdmmc_card_t *card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));  /* 从堆上分配sdmmc_card_t结构体(约200+字节) */

    if (card == NULL)                                                   /* 内存分配失败(堆空间不足) */
    {
        printf("[SD] malloc FAILED\n");                                 /* 输出内存不足错误 */
        sdmmc_host_deinit_slot(host.slot);                              /* 反初始化卡槽(释放GPIO资源) */
        /* 清理主机: 根据标志位选择反初始化方式 */
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)                    /* 需要传slot参数 */
            host.deinit_p(host.slot);                                   /* 调用带参数反初始化 */
        else                                                            /* 不需要参数 */
            (*host.deinit)();                                           /* 调用无参数反初始化 */
        return ESP_ERR_NO_MEM;                                          /* 返回内存不足错误码 */
    }

    /* ---------- 步骤7: 探测并初始化SD卡 ---------- */
    printf("[SD] Probing SD card (SDMMC, 40MHz)...\n");                 /* 输出: 正在探测SD卡(发送CMD0→ACMD41→CMD2→CMD3等) */
    ret = sdmmc_card_init(&host, card);                                 /* 执行SD卡初始化序列: 复位→电压验证→OCR→CID→RCA→CSD */

    if (ret != ESP_OK)                                                  /* SD卡初始化/探测失败(可能: 无卡/卡损坏/接触不良) */
    {
        printf("[SD] Card init FAILED: %s (code %d)\n", esp_err_to_name(ret), ret); /* 输出错误名和错误码 */
        ESP_LOGE(TAG, "sdmmc_card_init failed: %s", esp_err_to_name(ret)); /* 记录错误到日志系统 */
        sdmmc_host_deinit_slot(host.slot);                              /* 反初始化卡槽 */
        /* 清理主机 */
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG)                    /* 需要传slot参数 */
            host.deinit_p(host.slot);                                   /* 带参数反初始化 */
        else                                                            /* 不需要参数 */
            (*host.deinit)();                                           /* 无参数反初始化 */
        free(card);                                                     /* 释放之前分配的card内存(避免泄漏) */
        return ret;                                                     /* 返回错误码 */
    }

    /* ---------- 步骤8: 输出SD卡信息并返回 ---------- */
    sdmmc_card_print_info(stdout, card);                                /* 打印SD卡详细信息: CID/CSD/容量/速度等级等 */
    *out_card = card;                                                   /* 通过输出参数返回card指针给调用者(main.c) */

    printf("[SD] Init DONE. Name=%s, Size=%lluMB\n",                    /* 输出初始化完成: 卡名称和总容量 */
           card->cid.name,                                              /* CID寄存器中的卡名称(如"SD02G") */
           ((uint64_t)card->csd.capacity * card->csd.sector_size) / (1024 * 1024)); /* 计算总容量(MB): 扇区数×扇区大小÷1MB */
    ESP_LOGI(TAG, "SD card init done");                                 /* 记录成功日志 */

    return ESP_OK;                                                      /* 返回成功 */
}

/**
 * @brief       反初始化SD卡, 释放所有资源
 *              释放顺序: 卡槽 → 主机 → 卡控制块内存 (与初始化顺序相反)
 * @param       card: 指向要释放的SD卡控制块(由sd_card_init分配)
 * @retval      ESP_OK: 反初始化成功
 */
esp_err_t sd_card_deinit(sdmmc_card_t *card)
{
    if (card != NULL) {                                                 /* 仅当card指针有效时才执行清理(防御性编程) */
        /* 逆序释放: 卡槽 → 主机 → 结构体 */
        sdmmc_host_deinit_slot(SDMMC_HOST_SLOT_1);                      /* 第一步: 反初始化卡槽(GPIO释放, 时钟停止) */
        sdmmc_host_deinit();                                            /* 第二步: 反初始化主机(关闭SDMMC外设时钟) */
        free(card);                                                     /* 第三步: 释放动态分配的卡控制块内存 */
    }
    printf("[SD] Deinit done\n");                                       /* 反初始化完成 */
    return ESP_OK;                                                      /* 返回成功 */
}

/**
 * @brief       直接挂载FATFS到SD卡 (不经过USB MSC)
 *              绕过TinyUSB, 直接通过ESP-IDF VFS挂载FATFS文件系统。
 *              适用于只需本地/WiFi访问SD卡、不需要USB大容量存储的场景。
 *              步骤: ①注册SDMMC为FATFS磁盘驱动 → ②注册VFS路径 →
 *                    ③挂载FATFS → ④返回FATFS句柄
 * @param       card: 已初始化的SD卡控制块(由sd_card_init返回)
 * @param       base_path: 挂载点路径, 如"/sd"
 * @param       max_files: FATFS同时打开文件数上限(建议5)
 * @retval      ESP_OK: 挂载成功, 可通过base_path访问文件
 * @retval      其他: 挂载失败
 */
esp_err_t sd_card_mount_fatfs(sdmmc_card_t *card, const char *base_path, int max_files)
{
    if (card == NULL || base_path == NULL) {
        printf("[SD] FATFS mount: invalid args\n");
        return ESP_ERR_INVALID_ARG;
    }

    printf("[SD] Mounting FATFS directly at %s (max_files=%d)...\n", base_path, max_files);

    /* ---- 步骤1: 获取空闲物理驱动器号 ---- */
    BYTE pdrv = 0xFF;
    esp_err_t ret = ff_diskio_get_drive(&pdrv);
    if (ret != ESP_OK) {
        printf("[SD] FATFS mount: get_drive FAILED %s\n", esp_err_to_name(ret));
        return ret;
    }

    /* ---- 步骤2: 注册SDMMC卡为FATFS磁盘I/O驱动 ---- */
    ff_diskio_register_sdmmc(pdrv, card);

    /* ---- 步骤3: 注册VFS路径 (让标准C API如fopen可访问) ---- */
    char drv[3] = {(char)('0' + pdrv), ':', 0};
    FATFS *fs = NULL;
    ret = esp_vfs_fat_register(base_path, drv, max_files, &fs);
    if (ret == ESP_ERR_INVALID_STATE) {
        printf("[SD] FATFS mount: already registered at %s (OK)\n", base_path);
    } else if (ret != ESP_OK) {
        printf("[SD] FATFS mount: vfs_register FAILED %s\n", esp_err_to_name(ret));
        ff_diskio_unregister(pdrv);
        return ret;
    }

    /* ---- 步骤4: 挂载FATFS ---- */
    FRESULT fresult = f_mount(fs, drv, 1);
    if (fresult != FR_OK) {
        printf("[SD] FATFS mount: f_mount FAILED (FR=%d)\n", (int)fresult);
        esp_vfs_fat_unregister_path(base_path);
        ff_diskio_unregister(pdrv);
        return ESP_FAIL;
    }

    printf("[SD] FATFS mounted at %s (pdrv=%d)\n", base_path, (int)pdrv);

    /* 验证挂载: 列出根目录 */
    DIR *test = opendir(base_path);
    if (test != NULL) {
        int cnt = 0;
        struct dirent *d;
        while ((d = readdir(test)) != NULL) {
            if (strcmp(d->d_name, ".") != 0 && strcmp(d->d_name, "..") != 0) cnt++;
        }
        closedir(test);
        printf("[SD] FATFS verify OK: %d entries at %s\n", cnt, base_path);
    } else {
        printf("[SD] WARNING: opendir(%s) failed after mount\n", base_path);
    }

    return ESP_OK;
}
