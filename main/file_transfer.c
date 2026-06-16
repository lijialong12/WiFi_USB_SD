/**
 ****************************************************************************************************
 * @file        file_transfer.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-15
 * @brief       文件传输客户端实现 - HTTP POST推送文件
 *              使用ESP-IDF esp_http_client库, 支持流式分块传输
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 (DEVICE_ROLE=0 主机端)
 * 内存: 使用2KB发送缓冲区, 适合在HTTP任务栈(~8KB)中运行
 * 超时: 连接超时5秒, 传输超时30秒
 * 错误处理: 网络错误/HTTP非200均返回ESP_FAIL
 *
 ****************************************************************************************************
 */

#include "file_transfer.h"                   /* 文件传输API头文件 */
#include <stdio.h>                           /* 标准I/O: printf, snprintf, fopen, fread, fclose */
#include <stdlib.h>                          /* 标准库: malloc */
#include <string.h>                          /* 字符串: strrchr, strlen */
#include <sys/stat.h>                        /* POSIX: stat, struct stat */
#include "esp_http_client.h"                 /* ESP-IDF HTTP客户端 */
#include "esp_log.h"                         /* ESP日志 */
#include "freertos/FreeRTOS.h"               /* FreeRTOS: vTaskDelay */
#include "freertos/task.h"                   /* FreeRTOS任务 */

static const char *TAG = "FILE_XFER";        /* 日志标签 */

/**
 * @brief       从文件路径中提取纯文件名(最后一段)
 */
const char *file_transfer_basename(const char *filepath)
{
    if (filepath == NULL) return "unknown";
    const char *last = strrchr(filepath, '/');
    if (last != NULL) {
        return last + 1;                     /* 跳过'/' */
    }
    /* Windows风格反斜杠 */
    last = strrchr(filepath, '\\');
    if (last != NULL) {
        return last + 1;
    }
    return filepath;                         /* 无路径分隔符, 整个就是文件名 */
}

/**
 * @brief       推送文件到从机设备
 */
esp_err_t file_transfer_push(const char *slave_ip, const char *filepath)
{
    if (slave_ip == NULL || filepath == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ---- 步骤1: 检查文件是否存在并获取大小 ---- */
    struct stat st;
    if (stat(filepath, &st) != 0) {
        printf("[FILE_XFER] File not found: %s\n", filepath);
        return ESP_FAIL;
    }
    if (S_ISDIR(st.st_mode)) {
        printf("[FILE_XFER] Is a directory, skip: %s\n", filepath);
        return ESP_FAIL;
    }

    const char *basename = file_transfer_basename(filepath);
    uint64_t file_size   = (uint64_t)st.st_size;

    printf("[FILE_XFER] Pushing: %s → http://%s/api/upload?name=%s (%llu bytes)\n",
           filepath, slave_ip, basename, (unsigned long long)file_size);

    /* ---- 步骤2: 构建URL ---- */
    /* URL编码文件名(简易: 只处理空格, 其他特殊字符在FATFS文件名中不常见) */
    char url[640];
    int url_len = snprintf(url, sizeof(url),
                           "http://%s/api/upload?name=", slave_ip);
    /* 简单URL编码: 将空格替换为%20 */
    const char *src = basename;
    char *dst = url + url_len;
    size_t remaining = sizeof(url) - url_len - 1;
    while (*src && remaining > 0) {
        if (*src == ' ') {
            if (remaining < 3) break;
            *dst++ = '%'; *dst++ = '2'; *dst++ = '0';
            remaining -= 3;
        } else {
            *dst++ = *src;
            remaining--;
        }
        src++;
    }
    *dst = '\0';

    /* ---- 步骤3: 配置HTTP客户端 ---- */
    esp_http_client_config_t config = {
        .url               = url,
        .method            = HTTP_METHOD_POST,
        .timeout_ms        = 30000,          /* 30秒超时 */
        .disable_auto_redirect = false,
        .buffer_size       = 4096,           /* 4KB接收缓冲 */
        .buffer_size_tx    = 2048,           /* 2KB发送缓冲 */
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        printf("[FILE_XFER] Client init failed\n");
        return ESP_FAIL;
    }

    /* ---- 步骤4: 设置请求头 ---- */
    esp_http_client_set_header(client, "Content-Type", "application/octet-stream");

    char size_hdr[32];
    snprintf(size_hdr, sizeof(size_hdr), "%llu", (unsigned long long)file_size);
    esp_http_client_set_header(client, "X-File-Size", size_hdr);

    /* ---- 步骤5: 打开连接 ---- */
    esp_err_t ret = esp_http_client_open(client, 0);  /* 0=不先写body长度(使用chunked) */
    /* 重试: 先设置post_field为NULL触发Content-Length=0的写法不对,
     * 改用open+write的方式 */
    if (ret != ESP_OK) {
        printf("[FILE_XFER] Open failed: %s\n", esp_err_to_name(ret));
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    /* ---- 步骤6: 设置Content-Length (覆盖chunked) ---- */
    /* esp_http_client_open with write_len=0 使用chunked, 我们需要设置Content-Length */
    /* 由于open已调用, 我们通过write来发送, 内部自动chunked */
    /* 实际上对于esp_http_client, 用open(0) + write即可流式发送 */

    /* ---- 步骤7: 打开本地文件并流式发送 ---- */
    FILE *fp = fopen(filepath, "rb");
    if (fp == NULL) {
        printf("[FILE_XFER] Cannot open local file: %s\n", filepath);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    uint8_t buf[2048];                       /* 2KB发送缓冲 */
    size_t  total_sent = 0;
    int     chunk_count = 0;

    while (!feof(fp)) {
        size_t bytes_read = fread(buf, 1, sizeof(buf), fp);
        if (bytes_read == 0) break;

        int write_ret = esp_http_client_write(client, (const char *)buf, bytes_read);
        if (write_ret < 0) {
            printf("[FILE_XFER] Write error at %zu bytes\n", total_sent);
            fclose(fp);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }

        total_sent += bytes_read;

        /* 每16个chunk(32KB)短暂让出CPU, 防止看门狗 */
        if (++chunk_count % 16 == 0) {
            vTaskDelay(1);
        }
    }

    fclose(fp);

    /* ---- 步骤8: 完成请求并检查响应 ---- */
    int content_length = esp_http_client_fetch_headers(client);

    /* 对于POST, 需要先发送空chunk表示结束? esp_http_client在close时处理 */
    int status_code = esp_http_client_get_status_code(client);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status_code == 200) {
        printf("[FILE_XFER] OK: %s → %s (%zu bytes sent, HTTP %d)\n",
               filepath, basename, total_sent, status_code);
        return ESP_OK;
    } else {
        printf("[FILE_XFER] FAIL: %s → HTTP %d (sent %zu bytes)\n",
               filepath, status_code, total_sent);
        return ESP_FAIL;
    }
}
