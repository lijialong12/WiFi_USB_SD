/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "diskio_wl.h"
#include "wear_levelling.h"
#include "esp_partition.h"
#include "esp_memory_utils.h"
#include "sdkconfig.h"
#include "vfs_fat_internal.h"
#include "tinyusb.h"
#include "device/usbd_pvt.h"
#include "class/msc/msc_device.h"
#include "tusb_msc_storage.h"
#if SOC_SDMMC_HOST_SUPPORTED
#include "diskio_sdmmc.h"
#endif

static const char *TAG = "tinyusb_msc_storage";

#define MSC_STORAGE_MEM_ALIGN 4
#define MSC_STORAGE_BUFFER_SIZE CONFIG_TINYUSB_MSC_BUFSIZE /*!< Size of the buffer, configured via menuconfig (MSC FIFO size) */

#if ((MSC_STORAGE_BUFFER_SIZE) % MSC_STORAGE_MEM_ALIGN != 0)
#error "CONFIG_TINYUSB_MSC_BUFSIZE must be divisible by MSC_STORAGE_MEM_ALIGN. Adjust your configuration (MSC FIFO size) in menuconfig."
#endif

/**
 * @brief Structure representing a write buffer for MSC operations.
 *
 * Two of these are allocated in the storage handle for double-buffering:
 * while one buffer is being written to storage, the other can receive
 * the next USB transfer, resulting in smooth linear transfer progress.
 */
typedef struct {
    uint8_t data_buffer[MSC_STORAGE_BUFFER_SIZE]; /*!< Buffer to store write data. The size is defined by MSC_STORAGE_BUFFER_SIZE. */
    uint32_t lba;                          /*!< Logical Block Address for the current WRITE10 operation. */
    uint32_t offset;                       /*!< Offset within the specified LBA for the current write operation. */
    uint32_t bufsize;                      /*!< Number of bytes to be written in this operation. */
    volatile bool busy;                    /*!< True while this buffer's data is being written to storage. */
} msc_storage_buffer_t;

#define MSC_STORAGE_BUF_COUNT 4            /*!< Number of write buffers for pipelined writes */

/**
 * @brief Handle for TinyUSB MSC storage interface.
 *
 * This structure holds metadata and function pointers required to
 * manage the underlying storage medium (SPI flash, SDMMC).
 */
typedef struct {
    msc_storage_buffer_t storage_buffer[MSC_STORAGE_BUF_COUNT]; /*!< Double-buffered write buffers (ping-pong). */
    uint8_t cur_buf_idx;                  /*!< Index of the next buffer to use (0 or 1), round-robin. */
    bool is_fat_mounted;                  /*!< Indicates if the FAT filesystem is currently mounted. */
    bool is_usb_mounted;                  /*!< Indicates if the USB host has claimed the storage. */
    const char *base_path;                /*!< Base path where the filesystem is mounted. */
    union {
        wl_handle_t wl_handle;            /*!< Handle for wear leveling on SPI flash. */
#if SOC_SDMMC_HOST_SUPPORTED
        sdmmc_card_t *card;               /*!< Handle for SDMMC card. */
#endif
    };
    esp_err_t (*mount)(BYTE pdrv);        /*!< Pointer to the mount function. */
    esp_err_t (*unmount)(void);           /*!< Pointer to the unmount function. */
    uint32_t sector_count;                /*!< Total number of sectors in the storage medium. */
    uint32_t sector_size;                 /*!< Size of a single sector in bytes. */
    esp_err_t (*read)(size_t sector_size, /*!< Function pointer for reading data. */
                      uint32_t lba, uint32_t offset, size_t size, void *dest);
    esp_err_t (*write)(size_t sector_size, /*!< Function pointer for writing data. */
                       size_t addr, uint32_t lba, uint32_t offset, size_t size, const void *src);
    tusb_msc_callback_t callback_mount_changed; /*!< Callback for mount state change. */
    tusb_msc_callback_t callback_premount_changed; /*!< Callback for pre-mount state change. */
    int max_files;                          /*!< Maximum number of files that can be open simultaneously. */
} tinyusb_msc_storage_handle_s;

/* handle of tinyusb driver connected to application */
static tinyusb_msc_storage_handle_s *s_storage_handle;

/* Forward declaration — defined later in the file (after the storage read/write helpers) */
static esp_err_t _msc_storage_write_sector(uint32_t lba, uint32_t offset, size_t size, const void *src);

/* Dedicated writer task: decouples SD card writes from the USB device task.
 * USB receives data into buffers; this task flushes them to storage in parallel. */
static TaskHandle_t s_writer_task = NULL;
static QueueHandle_t s_write_queue = NULL;
#define MSC_WRITE_QUEUE_DEPTH  8
#define MSC_WRITER_TASK_PRIO   3
#define MSC_WRITER_STACK_SIZE  2048

/**
 * @brief Dedicated writer task — drains the write queue, flushing buffers to storage.
 *
 * Runs on a separate FreeRTOS task so that SD card writes never block the
 * TinyUSB device task. This keeps USB transfers flowing at full speed
 * regardless of SD card write latency.
 */
static void msc_writer_task(void *param)
{
    (void) param;
    while (1) {
        msc_storage_buffer_t *buf = NULL;
        if (xQueueReceive(s_write_queue, &buf, portMAX_DELAY) == pdTRUE && buf != NULL) {
            esp_err_t err = _msc_storage_write_sector(
                                buf->lba, buf->offset, buf->bufsize,
                                (const void *)buf->data_buffer
                            );
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Write failed, error=0x%x", err);
            }
            buf->busy = false;  /* Buffer is now free for the next WRITE10 */
        }
    }
}

/**
 * @brief Create the writer task and queue (called once during init).
 */
static esp_err_t _create_writer_resources(void)
{
    if (s_write_queue == NULL) {
        s_write_queue = xQueueCreate(MSC_WRITE_QUEUE_DEPTH, sizeof(msc_storage_buffer_t *));
        if (s_write_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create write queue");
            return ESP_ERR_NO_MEM;
        }
    }
    if (s_writer_task == NULL) {
        BaseType_t ret = xTaskCreate(msc_writer_task, "msc_writer", MSC_WRITER_STACK_SIZE,
                                     NULL, MSC_WRITER_TASK_PRIO, &s_writer_task);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create writer task");
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static esp_err_t _mount_spiflash(BYTE pdrv)
{
    return ff_diskio_register_wl_partition(pdrv, s_storage_handle->wl_handle);
}

static esp_err_t _unmount_spiflash(void)
{
    BYTE pdrv;
    pdrv = ff_diskio_get_pdrv_wl(s_storage_handle->wl_handle);
    if (pdrv == 0xff) {
        ESP_LOGE(TAG, "Invalid state");
        return ESP_ERR_INVALID_STATE;
    }
    ff_diskio_clear_pdrv_wl(s_storage_handle->wl_handle);

    char drv[3] = {(char)('0' + pdrv), ':', 0};
    f_mount(0, drv, 0);
    ff_diskio_unregister(pdrv);

    return ESP_OK;
}

static uint32_t _get_sector_count_spiflash(void)
{
    uint32_t result = 0;
    assert(s_storage_handle->wl_handle != WL_INVALID_HANDLE);
    size_t size = wl_sector_size(s_storage_handle->wl_handle);
    if (size == 0) {
        ESP_LOGW(TAG, "WL Sector size is zero !!!");
        result = 0;
    } else {
        result = (uint32_t)(wl_size(s_storage_handle->wl_handle) / size);
    }
    return result;
}

static uint32_t _get_sector_size_spiflash(void)
{
    assert(s_storage_handle->wl_handle != WL_INVALID_HANDLE);
    return (uint32_t)wl_sector_size(s_storage_handle->wl_handle);
}

static esp_err_t _read_sector_spiflash(size_t sector_size,
                                       uint32_t lba,
                                       uint32_t offset,
                                       size_t size,
                                       void *dest)
{
    size_t temp = 0;
    size_t addr = 0; // Address of the data to be read, relative to the beginning of the partition.
    ESP_RETURN_ON_FALSE(!__builtin_umul_overflow(lba, sector_size, &temp), ESP_ERR_INVALID_SIZE, TAG, "overflow lba %lu sector_size %u", lba, sector_size);
    ESP_RETURN_ON_FALSE(!__builtin_uadd_overflow(temp, offset, &addr), ESP_ERR_INVALID_SIZE, TAG, "overflow addr %u offset %lu", temp, offset);
    return wl_read(s_storage_handle->wl_handle, addr, dest, size);
}

static esp_err_t _write_sector_spiflash(size_t sector_size,
                                        size_t addr,
                                        uint32_t lba,
                                        uint32_t offset,
                                        size_t size,
                                        const void *src)
{
    (void) addr; // addr argument is not used in this function, we calculate it based on lba and offset.
    size_t temp = 0;
    size_t src_addr = 0; // Address of the data to be write, relative to the beginning of the partition.
    ESP_RETURN_ON_FALSE(!__builtin_umul_overflow(lba, sector_size, &temp), ESP_ERR_INVALID_SIZE, TAG, "overflow lba %lu sector_size %u", lba, sector_size);
    ESP_RETURN_ON_FALSE(!__builtin_uadd_overflow(temp, offset, &src_addr), ESP_ERR_INVALID_SIZE, TAG, "overflow addr %u offset %lu", temp, offset);
    ESP_RETURN_ON_ERROR(wl_erase_range(s_storage_handle->wl_handle, src_addr, size), TAG, "Failed to erase");
    return wl_write(s_storage_handle->wl_handle, src_addr, src, size);
}

#if SOC_SDMMC_HOST_SUPPORTED
static esp_err_t _mount_sdmmc(BYTE pdrv)
{
    ff_diskio_register_sdmmc(pdrv, s_storage_handle->card);
    ff_sdmmc_set_disk_status_check(pdrv, false);
    return ESP_OK;
}

static esp_err_t _unmount_sdmmc(void)
{
    BYTE pdrv;
    pdrv = ff_diskio_get_pdrv_card(s_storage_handle->card);
    if (pdrv == 0xff) {
        ESP_LOGE(TAG, "Invalid state");
        return ESP_ERR_INVALID_STATE;
    }

    char drv[3] = {(char)('0' + pdrv), ':', 0};
    f_mount(0, drv, 0);
    ff_diskio_unregister(pdrv);

    return ESP_OK;
}

static uint32_t _get_sector_count_sdmmc(void)
{
    assert(s_storage_handle->card);
    return (uint32_t)s_storage_handle->card->csd.capacity;
}

static uint32_t _get_sector_size_sdmmc(void)
{
    assert(s_storage_handle->card);
    return (uint32_t)s_storage_handle->card->csd.sector_size;
}

static esp_err_t _read_sector_sdmmc(size_t sector_size,
                                    uint32_t lba,
                                    uint32_t offset,
                                    size_t size,
                                    void *dest)
{
    return sdmmc_read_sectors(s_storage_handle->card, dest, lba, size / sector_size);
}

static esp_err_t _write_sector_sdmmc(size_t sector_size,
                                     size_t addr,
                                     uint32_t lba,
                                     uint32_t offset,
                                     size_t size,
                                     const void *src)
{
    (void) addr; // addr argument is not used in this function, we use lba directly
    return sdmmc_write_sectors(s_storage_handle->card, src, lba, size / sector_size);
}
#endif

static esp_err_t _msc_storage_read_sector(uint32_t lba,
        uint32_t offset,
        size_t size,
        void *dest)
{
    assert(s_storage_handle);
    size_t sector_size = tinyusb_msc_storage_get_sector_size();
    return (s_storage_handle->read)(sector_size, lba, offset, size, dest);
}

static esp_err_t _msc_storage_write_sector(uint32_t lba,
        uint32_t offset,
        size_t size,
        const void *src)
{
    assert(s_storage_handle);
    if (s_storage_handle->is_fat_mounted) {
        ESP_LOGE(TAG, "can't write, FAT mounted");
        return ESP_ERR_INVALID_STATE;
    }
    size_t sector_size = tinyusb_msc_storage_get_sector_size();

    if (size % sector_size != 0) {
        ESP_LOGE(TAG, "Invalid Argument lba(%lu) offset(%lu) size(%u) sector_size(%u)", lba, offset, size, sector_size);
        return ESP_ERR_INVALID_ARG;
    }
    return (s_storage_handle->write)(sector_size, 0 /* not used */, lba, offset, size, src);
}

static esp_err_t _mount(char *drv, FATFS *fs)
{
    void *workbuf = NULL;
    const size_t workbuf_size = 4096;
    esp_err_t ret;
    // Try to mount partition
    FRESULT fresult = f_mount(fs, drv, 1);
    if (fresult != FR_OK) {
        ESP_LOGW(TAG, "f_mount failed (%d)", fresult);
        if (!((fresult == FR_NO_FILESYSTEM || fresult == FR_INT_ERR))) {
            ret = ESP_FAIL;
            goto fail;
        }
        workbuf = ff_memalloc(workbuf_size);
        if (workbuf == NULL) {
            ret = ESP_ERR_NO_MEM;
            goto fail;
        }
        size_t alloc_unit_size = esp_vfs_fat_get_allocation_unit_size(
                                     CONFIG_WL_SECTOR_SIZE,
                                     4096);
        ESP_LOGW(TAG, "formatting card, allocation unit size=%d", alloc_unit_size);

        BYTE format_flags;
#if defined(CONFIG_TINYUSB_FAT_FORMAT_ANY)
        format_flags = FM_ANY;

#elif defined(CONFIG_TINYUSB_FAT_FORMAT_FAT)
        format_flags = FM_FAT;

#elif defined(CONFIG_TINYUSB_FAT_FORMAT_FAT32)
        format_flags = FM_FAT32;

#elif defined(CONFIG_TINYUSB_FAT_FORMAT_EXFAT)
        format_flags = FM_EXFAT;
#else

#error "No FAT format type selected"

#endif

#ifdef CONFIG_TINYUSB_FAT_FORMAT_SFD
        format_flags |= FM_SFD;
#endif
        const MKFS_PARM opt = {format_flags, 0, 0, 0, alloc_unit_size};
        fresult = f_mkfs("", &opt, workbuf, workbuf_size); // Use default volume
        if (fresult != FR_OK) {
            ret = ESP_FAIL;
            ESP_LOGE(TAG, "f_mkfs failed (%d)", fresult);
            goto fail;
        }
        free(workbuf);
        workbuf = NULL;
        fresult = f_mount(fs, drv, 0);
        if (fresult != FR_OK) {
            ret = ESP_FAIL;
            ESP_LOGE(TAG, "f_mount failed after formatting (%d)", fresult);
            goto fail;
        }
    }
    return ESP_OK;
fail:
    if (workbuf) {
        free(workbuf);
    }
    return ret;
}

esp_err_t tinyusb_msc_storage_mount(const char *base_path)
{
    esp_err_t ret = ESP_OK;
    assert(s_storage_handle);

    if (s_storage_handle->is_fat_mounted) {
        return ESP_OK;
    }

    tusb_msc_callback_t cb = s_storage_handle->callback_premount_changed;
    if (cb) {
        tinyusb_msc_event_t event = {
            .type = TINYUSB_MSC_EVENT_PREMOUNT_CHANGED,
            .mount_changed_data = {
                .is_mounted = s_storage_handle->is_fat_mounted
            }
        };
        cb(&event);
    }

    if (!base_path) {
        base_path = CONFIG_TINYUSB_MSC_MOUNT_PATH;
    }

    // connect driver to FATFS
    BYTE pdrv = 0xFF;
    ESP_RETURN_ON_ERROR(ff_diskio_get_drive(&pdrv), TAG,
                        "The maximum count of volumes is already mounted");
    char drv[3] = {(char)('0' + pdrv), ':', 0};

    ESP_GOTO_ON_ERROR((s_storage_handle->mount)(pdrv), fail, TAG, "Failed pdrv=%d", pdrv);

    FATFS *fs = NULL;
    ret = esp_vfs_fat_register(base_path, drv, s_storage_handle->max_files, &fs);
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "it's okay, already registered with VFS");
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_fat_register failed (0x%x)", ret);
        goto fail;
    }

    ESP_GOTO_ON_ERROR(_mount(drv, fs), fail, TAG, "Failed _mount");

    s_storage_handle->is_fat_mounted = true;
    s_storage_handle->base_path = base_path;

    cb = s_storage_handle->callback_mount_changed;
    if (cb) {
        tinyusb_msc_event_t event = {
            .type = TINYUSB_MSC_EVENT_MOUNT_CHANGED,
            .mount_changed_data = {
                .is_mounted = s_storage_handle->is_fat_mounted
            }
        };
        cb(&event);
    }

    return ret;

fail:
    if (fs) {
        esp_vfs_fat_unregister_path(base_path);
    }
    ff_diskio_unregister(pdrv);
    s_storage_handle->is_fat_mounted = false;
    ESP_LOGW(TAG, "Failed to mount storage (0x%x)", ret);
    return ret;
}

esp_err_t tinyusb_msc_storage_unmount(void)
{
    if (!s_storage_handle) {
        return ESP_FAIL;
    }

    if (!s_storage_handle->is_fat_mounted) {
        return ESP_OK;
    }

    tusb_msc_callback_t cb = s_storage_handle->callback_premount_changed;
    if (cb) {
        tinyusb_msc_event_t event = {
            .type = TINYUSB_MSC_EVENT_PREMOUNT_CHANGED,
            .mount_changed_data = {
                .is_mounted = s_storage_handle->is_fat_mounted
            }
        };
        cb(&event);
    }

    esp_err_t err = (s_storage_handle->unmount)();
    if (err) {
        return err;
    }
    err = esp_vfs_fat_unregister_path(s_storage_handle->base_path);
    s_storage_handle->base_path = NULL;
    s_storage_handle->is_fat_mounted = false;

    cb = s_storage_handle->callback_mount_changed;
    if (cb) {
        tinyusb_msc_event_t event = {
            .type = TINYUSB_MSC_EVENT_MOUNT_CHANGED,
            .mount_changed_data = {
                .is_mounted = s_storage_handle->is_fat_mounted
            }
        };
        cb(&event);
    }

    return err;
}

uint32_t tinyusb_msc_storage_get_sector_count(void)
{
    assert(s_storage_handle);
    return (s_storage_handle->sector_count);
}

uint32_t tinyusb_msc_storage_get_sector_size(void)
{
    assert(s_storage_handle);
    return (s_storage_handle->sector_size);
}

esp_err_t tinyusb_msc_storage_init_spiflash(const tinyusb_msc_spiflash_config_t *config)
{
    assert(!s_storage_handle);
    ESP_RETURN_ON_FALSE(CONFIG_TINYUSB_MSC_BUFSIZE >= CONFIG_WL_SECTOR_SIZE,
                        ESP_ERR_NOT_SUPPORTED, TAG,
                        "CONFIG_TINYUSB_MSC_BUFSIZE (%d) must be at least the size of CONFIG_WL_SECTOR_SIZE (%d)", (int)(CONFIG_TINYUSB_MSC_BUFSIZE), (int)(CONFIG_WL_SECTOR_SIZE));
    s_storage_handle = (tinyusb_msc_storage_handle_s *)heap_caps_aligned_alloc(MSC_STORAGE_MEM_ALIGN, sizeof(tinyusb_msc_storage_handle_s), MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_storage_handle, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for storage handle");
    s_storage_handle->mount = &_mount_spiflash;
    s_storage_handle->unmount = &_unmount_spiflash;
    s_storage_handle->wl_handle = config->wl_handle;
    s_storage_handle->sector_count = _get_sector_count_spiflash();
    s_storage_handle->sector_size = _get_sector_size_spiflash();
    s_storage_handle->read = &_read_sector_spiflash;
    s_storage_handle->write = &_write_sector_spiflash;
    s_storage_handle->cur_buf_idx = 0;
    s_storage_handle->is_fat_mounted = false;
    s_storage_handle->is_usb_mounted = false;
    s_storage_handle->base_path = NULL;
    // In case the user does not set mount_config.max_files
    // and for backward compatibility with versions <1.4.2
    // max_files is set to 2
    const int max_files = config->mount_config.max_files;
    s_storage_handle->max_files = max_files > 0 ? max_files : 2;

    /* Mark all write buffers as free */
    for (int i = 0; i < MSC_STORAGE_BUF_COUNT; i++) {
        s_storage_handle->storage_buffer[i].busy = false;
    }

    /* Create dedicated writer task + queue for parallel SD writes */
    ESP_RETURN_ON_ERROR(_create_writer_resources(), TAG, "Failed to create writer resources");

    /* Callbacks setting up*/
    if (config->callback_mount_changed) {
        tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, config->callback_mount_changed);
    } else {
        tinyusb_msc_unregister_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED);
    }
    if (config->callback_premount_changed) {
        tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_PREMOUNT_CHANGED, config->callback_premount_changed);
    } else {
        tinyusb_msc_unregister_callback(TINYUSB_MSC_EVENT_PREMOUNT_CHANGED);
    }

    if (!esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[0].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[1].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[2].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[3].data_buffer)) {
        ESP_LOGW(TAG, "storage buffer is not DMA capable");
    }

    return ESP_OK;
}

#if SOC_SDMMC_HOST_SUPPORTED
esp_err_t tinyusb_msc_storage_init_sdmmc(const tinyusb_msc_sdmmc_config_t *config)
{
    assert(!s_storage_handle);
    s_storage_handle = (tinyusb_msc_storage_handle_s *)heap_caps_aligned_alloc(MSC_STORAGE_MEM_ALIGN, sizeof(tinyusb_msc_storage_handle_s), MALLOC_CAP_DMA);
    ESP_RETURN_ON_FALSE(s_storage_handle, ESP_ERR_NO_MEM, TAG, "Failed to allocate memory for storage handle");
    s_storage_handle->mount = &_mount_sdmmc;
    s_storage_handle->unmount = &_unmount_sdmmc;
    s_storage_handle->card = config->card;
    s_storage_handle->sector_count = _get_sector_count_sdmmc();
    s_storage_handle->sector_size = _get_sector_size_sdmmc();
    s_storage_handle->read = &_read_sector_sdmmc;
    s_storage_handle->write = &_write_sector_sdmmc;
    s_storage_handle->cur_buf_idx = 0;
    s_storage_handle->is_fat_mounted = false;
    s_storage_handle->is_usb_mounted = false;
    s_storage_handle->base_path = NULL;
    // In case the user does not set mount_config.max_files
    // and for backward compatibility with versions <1.4.2
    // max_files is set to 2
    const int max_files = config->mount_config.max_files;
    s_storage_handle->max_files = max_files > 0 ? max_files : 2;

    /* Mark all write buffers as free */
    for (int i = 0; i < MSC_STORAGE_BUF_COUNT; i++) {
        s_storage_handle->storage_buffer[i].busy = false;
    }

    /* Create dedicated writer task + queue for parallel SD writes */
    ESP_RETURN_ON_ERROR(_create_writer_resources(), TAG, "Failed to create writer resources");

    /* Callbacks setting up*/
    if (config->callback_mount_changed) {
        tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, config->callback_mount_changed);
    } else {
        tinyusb_msc_unregister_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED);
    }
    if (config->callback_premount_changed) {
        tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_PREMOUNT_CHANGED, config->callback_premount_changed);
    } else {
        tinyusb_msc_unregister_callback(TINYUSB_MSC_EVENT_PREMOUNT_CHANGED);
    }

    if (!esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[0].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[1].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[2].data_buffer) ||
        !esp_ptr_dma_capable((const void *)s_storage_handle->storage_buffer[3].data_buffer)) {
        ESP_LOGW(TAG, "storage buffer is not DMA capable");
    }

    return ESP_OK;
}
#endif

void tinyusb_msc_storage_deinit(void)
{
    /* Stop writer task first so no more writes are processed */
    if (s_writer_task != NULL) {
        vTaskDelete(s_writer_task);
        s_writer_task = NULL;
    }
    if (s_write_queue != NULL) {
        vQueueDelete(s_write_queue);
        s_write_queue = NULL;
    }
    if (s_storage_handle) {
        heap_caps_free(s_storage_handle);
        s_storage_handle = NULL;
    }
}

esp_err_t tinyusb_msc_register_callback(tinyusb_msc_event_type_t event_type,
                                        tusb_msc_callback_t callback)
{
    assert(s_storage_handle);
    switch (event_type) {
    case TINYUSB_MSC_EVENT_MOUNT_CHANGED:
        s_storage_handle->callback_mount_changed = callback;
        return ESP_OK;
    case TINYUSB_MSC_EVENT_PREMOUNT_CHANGED:
        s_storage_handle->callback_premount_changed = callback;
        return ESP_OK;
    default:
        ESP_LOGE(TAG, "Wrong event type");
        return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t tinyusb_msc_unregister_callback(tinyusb_msc_event_type_t event_type)
{
    assert(s_storage_handle);
    switch (event_type) {
    case TINYUSB_MSC_EVENT_MOUNT_CHANGED:
        s_storage_handle->callback_mount_changed = NULL;
        return ESP_OK;
    case TINYUSB_MSC_EVENT_PREMOUNT_CHANGED:
        s_storage_handle->callback_premount_changed = NULL;
        return ESP_OK;
    default:
        ESP_LOGE(TAG, "Wrong event type");
        return ESP_ERR_INVALID_ARG;
    }
}

bool tinyusb_msc_storage_in_use_by_usb_host(void)
{
    assert(s_storage_handle);
    return s_storage_handle->is_usb_mounted;
}


/* TinyUSB MSC callbacks
   ********************************************************************* */

/** SCSI ASC/ASCQ codes. **/
/** User can add and use more codes as per the need of the application **/
#define SCSI_CODE_ASC_MEDIUM_NOT_PRESENT 0x3A /** SCSI ASC code for 'MEDIUM NOT PRESENT' **/
#define SCSI_CODE_ASC_INVALID_COMMAND_OPERATION_CODE 0x20 /** SCSI ASC code for 'INVALID COMMAND OPERATION CODE' **/
#define SCSI_CODE_ASCQ 0x00

// Invoked when received SCSI_CMD_INQUIRY
// Application fill vendor id, product id and revision with string up to 8, 16, 4 characters respectively
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;
    const char vid[] = "TinyUSB";
    const char pid[] = "Flash Storage";
    const char rev[] = "0.2";

    memcpy(vendor_id, vid, strlen(vid));
    memcpy(product_id, pid, strlen(pid));
    memcpy(product_rev, rev, strlen(rev));
}

// Invoked when received Test Unit Ready command.
// return true allowing host to read/write this LUN e.g SD card inserted
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void) lun;

    if (s_storage_handle->is_fat_mounted) {
        // Locally mounted: auto-unmount to give USB host access, then report ready
        if (tinyusb_msc_storage_unmount() != ESP_OK) {
            ESP_LOGW(TAG, "tud_msc_test_unit_ready_cb() unmount Fails");
            return false;
        }
        s_storage_handle->is_usb_mounted = true;
    }
    return true;
}

// Invoked when received SCSI_CMD_READ_CAPACITY_10 and SCSI_CMD_READ_FORMAT_CAPACITY to determine the disk size
// Application update block count and block size
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void) lun;

    uint32_t sec_count = tinyusb_msc_storage_get_sector_count();
    uint32_t sec_size = tinyusb_msc_storage_get_sector_size();
    *block_count = sec_count;
    *block_size  = (uint16_t)sec_size;
}

// Invoked when received Start Stop Unit command
// - Start = 0 : stopped power mode, if load_eject = 1 : unload disk storage
// - Start = 1 : active mode, if load_eject = 1 : load disk storage
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void) lun;
    (void) power_condition;

    if (load_eject && !start) {
        if (tinyusb_msc_storage_mount(s_storage_handle->base_path) != ESP_OK) {
            ESP_LOGW(TAG, "tud_msc_start_stop_cb() mount Fails");
        }
    }
    return true;
}

// Invoked when received SCSI READ10 command
// - Address = lba * BLOCK_SIZE + offset
// - Application fill the buffer (up to bufsize) with address contents and return number of read byte.
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    esp_err_t err = _msc_storage_read_sector(lba, offset, bufsize, buffer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "msc_storage_read_sector failed: 0x%x", err);
        return 0;
    }
    return bufsize;
}

// Invoked when received SCSI WRITE10 command
// - Address = lba * BLOCK_SIZE + offset
// - Application write data from buffer to address contents (up to bufsize) and return number of written byte.
//
// Uses double-buffering: while one buffer is being written to storage (deferred),
// the next WRITE10 can fill the other buffer immediately. This produces smooth,
// linear transfer progress instead of stop-and-go.
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    assert(bufsize <= MSC_STORAGE_BUFFER_SIZE);

    /* Find a free buffer (round-robin starting from current index) */
    msc_storage_buffer_t *buf = NULL;
    for (int i = 0; i < MSC_STORAGE_BUF_COUNT; i++) {
        uint8_t idx = (s_storage_handle->cur_buf_idx + i) % MSC_STORAGE_BUF_COUNT;
        if (!s_storage_handle->storage_buffer[idx].busy) {
            buf = &s_storage_handle->storage_buffer[idx];
            s_storage_handle->cur_buf_idx = (idx + 1) % MSC_STORAGE_BUF_COUNT;
            break;
        }
    }

    if (buf == NULL) {
        /* All buffers are still being written — should be rare with 4×4KB buffers.
         * Stall the endpoint so the host retries later. */
        ESP_LOGW(TAG, "All write buffers busy, stalling");
        return 0;
    }

    /* Copy data from USB FIFO into our buffer */
    memcpy((void *)buf->data_buffer, buffer, bufsize);
    buf->lba = lba;
    buf->offset = offset;
    buf->bufsize = bufsize;
    buf->busy = true;

    /* Send buffer to the dedicated writer task via FreeRTOS queue.
     * The writer task runs on a separate thread, so the USB task returns
     * immediately and is ready for the next WRITE10. SD writes happen
     * in parallel, completely decoupled from USB transfers. */
    if (xQueueSend(s_write_queue, &buf, 0) != pdTRUE) {
        /* Queue full — writer is backlogged. Stall the endpoint so the
         * host retries; this avoids dropping data. */
        buf->busy = false;
        ESP_LOGW(TAG, "Write queue full, stalling");
        return 0;
    }

    /* Return immediately — the host can send the next WRITE10 while
     * the writer task flushes this buffer to storage in parallel. */
    return bufsize;
}

/**
 * Invoked when received an SCSI command not in built-in list below.
 * - READ_CAPACITY10, READ_FORMAT_CAPACITY, INQUIRY, TEST_UNIT_READY, START_STOP_UNIT, MODE_SENSE6, REQUEST_SENSE
 * - READ10 and WRITE10 has their own callbacks
 *
 * \param[in]   lun         Logical unit number
 * \param[in]   scsi_cmd    SCSI command contents which application must examine to response accordingly
 * \param[out]  buffer      Buffer for SCSI Data Stage.
 *                            - For INPUT: application must fill this with response.
 *                            - For OUTPUT it holds the Data from host
 * \param[in]   bufsize     Buffer's length.
 *
 * \return      Actual bytes processed, can be zero for no-data command.
 * \retval      negative    Indicate error e.g unsupported command, tinyusb will \b STALL the corresponding
 *                          endpoint and return failed status in command status wrapper phase.
 */
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    int32_t ret;

    switch (scsi_cmd[0]) {
    case SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL:
        /* SCSI_CMD_PREVENT_ALLOW_MEDIUM_REMOVAL is the Prevent/Allow Medium Removal
        command (1Eh) that requests the library to enable or disable user access to
        the storage media/partition. */
        ret = 0;
        break;
    default:
        ESP_LOGW(TAG, "tud_msc_scsi_cb() invoked: %d", scsi_cmd[0]);
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, SCSI_CODE_ASC_INVALID_COMMAND_OPERATION_CODE, SCSI_CODE_ASCQ);
        ret = -1;
        break;
    }
    return ret;
}

// Invoked when device is unmounted (USB disconnected)
void tud_umount_cb(void)
{
    if (tinyusb_msc_storage_mount(s_storage_handle->base_path) != ESP_OK) {
        ESP_LOGW(TAG, "tud_umount_cb() mount Fails");
    }
    s_storage_handle->is_usb_mounted = false;
}

// Invoked when device is mounted (configured by USB host)
void tud_mount_cb(void)
{
    tinyusb_msc_storage_unmount();
    s_storage_handle->is_usb_mounted = true;
}
/*********************************************************************** TinyUSB MSC callbacks*/
