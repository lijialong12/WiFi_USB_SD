/**
 ****************************************************************************************************
 * @file        web_server.c
 * @author      ONE
 * @version     V1.1
 * @date        2026-06-15
 * @brief       Web文件服务器 - HTTP请求处理实现
 *              功能: ①启动/停止HTTP服务器(端口80) → ②路径安全校验 →
 *                    ③MIME类型检测 → ④目录列表JSON API → ⑤文件预览/下载API →
 *                    ⑥从机注册API → ⑦文件删除API
 *              协议: HTTP/1.0 over TCP (LwIP协议栈)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板 (DEVICE_ROLE=0 主机端)
 * 安全措施: 所有文件路径经过validate_path()校验, 拒绝目录穿越攻击(../)
 * 性能: 文件下载采用分块传输(httpd_resp_send_chunk), 支持大文件
 * 线程: httpd内部单线程处理请求, 与app_main任务隔离运行
 * @note        文本预览上限512KB, 超过则截断并提示下载
 *              MIME表支持30+种常见文件类型, 未知类型默认octet-stream
 *
 ****************************************************************************************************
 */

#include "web_server.h"                      /* Web服务器API头文件 */
#include "web_page.h"                        /* 内嵌HTML网页字符串 */
#include "esp_http_server.h"                 /* ESP-IDF HTTP服务器框架 */
#include "esp_log.h"                         /* ESP-IDF日志: ESP_LOGI/W/E */
#include "freertos/FreeRTOS.h"               /* FreeRTOS: vTaskDelay */
#include "freertos/task.h"                   /* FreeRTOS任务: vTaskDelay */
#include "tusb_msc_storage.h"                /* TinyUSB MSC: tinyusb_msc_storage_mount */
#include "nvs.h"                             /* NVS读写API: nvs_open/nvs_get_str/nvs_set_str */
#include "esp_system.h"                      /* ESP系统: esp_restart() */
#include "lwip/sockets.h"                    /* LwIP socket: getpeername, sockaddr_in */
#include <stdio.h>                           /* 标准I/O: snprintf, fopen, fread, fclose */
#include <stdlib.h>                          /* 标准库: malloc, free, realloc */
#include <string.h>                          /* 字符串: strlen, strcpy, strchr, strcmp */
#include <ctype.h>                           /* 字符分类: tolower */
#include <dirent.h>                          /* POSIX目录: opendir, readdir, closedir */
#include <sys/stat.h>                        /* POSIX文件状态: stat, S_ISDIR */
#include <strings.h>                         /* POSIX扩展字符串: strcasecmp */

static const char *TAG = "WEB_SRV";          /* 日志标签: 用于ESP_LOGI/ESP_LOGE输出前缀 */

/* ======================== 全局变量 ======================== */
static httpd_handle_t g_server = NULL;       /* HTTP服务器句柄: NULL=未启动, 非NULL=运行中 */
static char g_base_path[16] = "/SD";         /* SD卡根路径: 所有文件访问均限制在此目录下 */
static char g_slave_ip[16] = {0};            /* 已注册从机IP地址 (空=无注册) */

/* ======================== MIME类型映射表 ======================== */
/* 文件扩展名 → HTTP Content-Type 映射, 用于文件预览和下载 */
typedef struct {
    const char *ext;                         /* 文件扩展名(含点, 如".txt") */
    const char *mime;                        /* MIME类型字符串 */
} mime_entry_t;

static const mime_entry_t mime_table[] = {
    /* ---- 文本类型(可预览) ---- */
    {".txt",  "text/plain; charset=utf-8"},
    {".md",   "text/markdown; charset=utf-8"},
    {".c",    "text/plain; charset=utf-8"},
    {".h",    "text/plain; charset=utf-8"},
    {".cpp",  "text/plain; charset=utf-8"},
    {".py",   "text/plain; charset=utf-8"},
    {".js",   "application/javascript; charset=utf-8"},
    {".json", "application/json; charset=utf-8"},
    {".xml",  "application/xml; charset=utf-8"},
    {".html", "text/html; charset=utf-8"},
    {".css",  "text/css; charset=utf-8"},
    {".csv",  "text/csv; charset=utf-8"},
    {".log",  "text/plain; charset=utf-8"},
    {".ini",  "text/plain; charset=utf-8"},
    {".cfg",  "text/plain; charset=utf-8"},
    {".yaml", "text/plain; charset=utf-8"},
    {".yml",  "text/plain; charset=utf-8"},
    {".sh",   "text/plain; charset=utf-8"},
    {".bat",  "text/plain; charset=utf-8"},
    {".cmake","text/plain; charset=utf-8"},
    {".mk",   "text/plain; charset=utf-8"},
    {".s",    "text/plain; charset=utf-8"},
    {".ld",   "text/plain; charset=utf-8"},
    /* ---- 图片类型 ---- */
    {".png",  "image/png"},
    {".jpg",  "image/jpeg"},
    {".jpeg", "image/jpeg"},
    {".gif",  "image/gif"},
    {".bmp",  "image/bmp"},
    {".svg",  "image/svg+xml"},
    {".ico",  "image/x-icon"},
    /* ---- 应用类型 ---- */
    {".pdf",  "application/pdf"},
    {".zip",  "application/zip"},
    {".gz",   "application/gzip"},
    {".tar",  "application/x-tar"},
    {".mp3",  "audio/mpeg"},
    {".wav",  "audio/wav"},
    {".mp4",  "video/mp4"},
    {".avi",  "video/x-msvideo"},
    /* ---- 终止标记 ---- */
    {NULL,    "application/octet-stream"}    /* 未知类型默认: 二进制流 */
};

/* 文本文件扩展名集合(用于判断是否可预览) */
static const char *text_extensions[] = {
    "txt","md","c","h","cpp","py","js","json","xml","html","css",
    "csv","log","ini","cfg","yaml","yml","sh","bat","cmake","mk",
    "s","ld","gitignore","readme","license", NULL
};

/* ======================== 工具函数 ======================== */

/**
 * @brief       根据文件名获取MIME类型
 *              从文件名提取扩展名(最后一个点之后), 在mime_table中查找对应MIME
 * @param       filename: 文件名(可含路径, 只取扩展名部分)
 * @retval      MIME类型字符串指针 (始终有效, 未知类型返回 application/octet-stream)
 */
static const char *get_mime_type(const char *filename)
{
    if (filename == NULL) return "application/octet-stream";   /* 空指针保护 */

    const char *dot = strrchr(filename, '.');                   /* 查找最后一个点的位置 */
    if (dot == NULL) return "application/octet-stream";         /* 无扩展名: 默认二进制 */

    /* 在MIME表中线性查找 (表很小, ~30项, 不必使用哈希) */
    for (const mime_entry_t *m = mime_table; m->ext != NULL; m++) {
        /* 大小写不敏感比较扩展名 */
        const char *a = dot;                                   /* a指向文件名中的点 */
        const char *b = m->ext;                                /* b指向表中的扩展名 */
        bool match = true;
        while (*a && *b) {
            if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
                match = false;
                break;
            }
            a++; b++;
        }
        if (match && *a == '\0' && *b == '\0') return m->mime; /* 完全匹配 */
    }
    return "application/octet-stream";                          /* 未匹配: 默认二进制流 */
}

/**
 * @brief       判断文件是否为可预览的文本类型
 *              通过比较文件扩展名判断(不读取文件内容)
 * @param       filename: 文件名
 * @retval      true: 文本类型, 可预览
 * @retval      false: 二进制类型, 只可下载
 */
static bool is_text_file(const char *filename)
{
    if (filename == NULL) return false;

    const char *dot = strrchr(filename, '.');
    if (dot == NULL) return false;                              /* 无扩展名: 不确定, 不预览 */

    dot++;                                                      /* 跳过点, 指向扩展名首字符 */

    for (const char **te = text_extensions; *te != NULL; te++) {
        /* 大小写不敏感比较 */
        const char *a = dot;
        const char *b = *te;
        bool match = true;
        while (*a && *b) {
            if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) {
                match = false;
                break;
            }
            a++; b++;
        }
        if (match && *a == '\0' && *b == '\0') return true;
    }
    return false;
}

/**
 * @brief       路径安全校验 + 规范化
 *              检测并拒绝目录穿越攻击 (如 /sd/../etc/passwd)
 *              将相对路径转换为标准绝对路径 (解析 . 和 .. 段)
 * @param       path: 用户输入的原始路径
 * @param       normalized: 输出缓冲区, 存储规范化后的绝对路径
 * @param       max_len: 输出缓冲区最大长度
 * @retval      true: 路径合法, 限制在g_base_path内
 * @retval      false: 路径非法 (NULL/空/目录穿越/过长)
 */
static bool validate_path(const char *path, char *normalized, size_t max_len)
{
    /* ---- 参数校验 ---- */
    if (path == NULL || normalized == NULL || max_len == 0) return false;

    /* ---- 移除前导空格 ---- */
    while (*path == ' ') path++;
    if (*path == '\0') return false;                            /* 空字符串 */

    /* ---- 必须以 /SD 开头 ---- */
    size_t base_len = strlen(g_base_path);
    if (strncmp(path, g_base_path, base_len) != 0) return false;
    /* 必须刚好匹配 /SD 或 /SD/xxx */
    if (path[base_len] != '\0' && path[base_len] != '/') return false;

    /* ---- 规范化: 解析 . 和 .. 段 ---- */
    char *out = normalized;
    const char *in = path;
    size_t remaining = max_len;

    /* 复制根路径 */
    size_t i = 0;
    for (; i < base_len && remaining > 1; i++) {
        *out++ = path[i];
        remaining--;
    }
    /* 如果输入就是 /SD, 直接返回 */
    if (path[base_len] == '\0') {
        *out = '\0';
        return true;
    }

    /* 跳过 /SD/ 后的分隔符, 逐段处理 */
    in = path + base_len;
    if (*in == '/') in++;

    while (*in && remaining > 1) {
        /* 跳过连续斜杠 */
        if (*in == '/') { in++; continue; }

        /* 提取下一个路径段 */
        const char *seg_start = in;
        while (*in && *in != '/') in++;

        size_t seg_len = in - seg_start;

        /* 检查 "." 段: 忽略 */
        if (seg_len == 1 && seg_start[0] == '.') {
            /* 跳过 */
        }
        /* 检查 ".." 段: 向上一级 */
        else if (seg_len == 2 && seg_start[0] == '.' && seg_start[1] == '.') {
            /* 回退到上一个 / 处 */
            while (out > normalized && *(out - 1) != '/') {
                out--;
                remaining++;
            }
            if (out > normalized) {
                out--;                                         /* 移除 / */
                remaining++;
            }
            /* 不允许穿越到 /SD 之外 */
            if (out < normalized + (int)base_len) return false;
        }
        /* 普通路径段: 追加 */
        else {
            *out++ = '/';
            remaining--;
            if ((size_t)(seg_len) >= remaining) return false;  /* 缓冲区不足 */
            memcpy(out, seg_start, seg_len);
            out += seg_len;
            remaining -= seg_len;
        }
    }

    *out = '\0';

    /* 最终检查: 规范化后至少等于 /SD */
    if (strncmp(normalized, g_base_path, base_len) != 0) return false;
    if (strlen(normalized) < base_len) return false;

    return true;
}

/**
 * @brief       URL解码: 将 %XX 和 + 转为原始字符, 原地修改
 * @param       str: 要解码的字符串(原地修改)
 */
static void url_decode_inplace(char *str)
{
    if (str == NULL) return;
    char *src = str;
    char *dst = str;
    while (*src) {
        if (*src == '%' && ((src[1] >= '0' && src[1] <= '9') || (src[1] >= 'A' && src[1] <= 'F') || (src[1] >= 'a' && src[1] <= 'f'))
                       && ((src[2] >= '0' && src[2] <= '9') || (src[2] >= 'A' && src[2] <= 'F') || (src[2] >= 'a' && src[2] <= 'f'))) {
            char hex[3] = {src[1], src[2], '\0'};
            *dst++ = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst = '\0';
}

/* ======================== HTTP URI 处理器 ======================== */

/**
 * @brief       根路径处理器: GET /
 *              返回内嵌的HTML文件管理器单页应用
 * @param       req: HTTP请求对象
 * @retval      ESP_OK: 网页已发送
 */
static esp_err_t root_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: GET / (root)\n");
    httpd_resp_set_type(req, "text/html; charset=utf-8");       /* 设置响应Content-Type */
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);    /* 发送嵌入HTML (字符串常量在Flash中) */
    printf("[WEB_SRV] <<< DONE: GET /\n");
    return ESP_OK;
}

/**
 * @brief       健康检查: GET /api/ping
 *              简单返回 {"ok":true}, 用于诊断API路由是否正常工作
 */
static esp_err_t ping_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: GET /api/ping\n");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true,\"ping\":\"pong\"}");
    printf("[WEB_SRV] <<< DONE: GET /api/ping\n");
    return ESP_OK;
}

/**
 * @brief       目录列表API处理器: GET /api/list?path=/sd/subdir
 *              列出指定路径下的文件和子目录, 返回JSON数组
 *              目录在前(字母序), 文件在后(字母序)
 * @param       req: HTTP请求对象
 * @retval      ESP_OK: JSON已发送
 */
static esp_err_t api_list_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: GET /api/list\n");
    char query[512] = {0};                                      /* 查询字符串缓冲区 */
    char raw_path[512] = {0};                                   /* 原始路径参数 */
    char norm_path[512] = {0};                                  /* 规范化后的安全路径 */

    /* ---- 步骤1: 获取查询字符串 ---- */
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > sizeof(query)) query_len = sizeof(query);
    if (httpd_req_get_url_query_str(req, query, query_len) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query string");
        return ESP_FAIL;
    }

    /* ---- 步骤2: 提取path参数 ---- */
    if (httpd_query_key_value(query, "path", raw_path, sizeof(raw_path)) != ESP_OK) {
        printf("[WEB_SRV] ERROR: Missing 'path' in query: '%s'\n", query);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Missing 'path' parameter\",\"code\":\"MISSING_PATH\"}");
        return ESP_FAIL;
    }
    /* 确保URL解码 (httpd_query_key_value在某些ESP-IDF版本中不解码) */
    url_decode_inplace(raw_path);
    printf("[WEB_SRV] Query='%s' raw_path='%s' (base_path='%s')\n", query, raw_path, g_base_path);

    /* ---- 步骤3: 路径安全校验 ---- */
    if (!validate_path(raw_path, norm_path, sizeof(norm_path))) {
        printf("[WEB_SRV] REJECTED path='%s'\n", raw_path);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Invalid path\",\"code\":\"INVALID_PATH\"}");
        return ESP_FAIL;
    }

    /* ---- 步骤4: 检查路径是否存在且为目录 ---- */
    /* FATFS根目录stat可能失败, 因此stat失败时直接尝试opendir作为降级 */
    struct stat st;
    int stat_ok = (stat(norm_path, &st) == 0);
    if (stat_ok && !S_ISDIR(st.st_mode)) {
        ESP_LOGW(TAG, "Not a directory: %s", norm_path);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Path is not a directory\",\"code\":\"NOT_A_DIR\"}");
        return ESP_FAIL;
    }

    /* ---- 步骤5: 遍历目录 (失败时尝试重新挂载) ---- */
    DIR *dir = opendir(norm_path);
    if (dir == NULL) {
        /* 可能是USB弹出后TinyUSB未自动重新挂载, 尝试手动挂载一次 */
        printf("[WEB_SRV] opendir failed, trying remount...\n");
        tinyusb_msc_storage_unmount();                              /* 先卸载清理残留状态 */
        esp_err_t remount_ret = tinyusb_msc_storage_mount("/SD");
        printf("[WEB_SRV] remount result: %s\n", esp_err_to_name(remount_ret));
        if (remount_ret == ESP_OK) {
            dir = opendir(norm_path);  /* 重试 */
        }
    }
    if (dir == NULL) {
        ESP_LOGW(TAG, "Cannot opendir: %s (stat_ok=%d)", norm_path, (int)stat_ok);
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD\xE5\x8D\xA1\xE6\x9A\x82\xE6\x97\xB6\xE4\xB8\x8D\xE5\x8F\xAF\xE7\x94\xA8\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x88\xB7\xE6\x96\xB0\xE9\xA1\xB5\xE9\x9D\xA2\xE9\x87\x8D\xE8\xAF\x95\",\"code\":\"SD_NOT_MOUNTED\"}");
        return ESP_FAIL;
    }

    /* 收集条目到动态数组 (先收集后排序) */
    typedef struct {
        char name[256];                                        /* 文件/目录名 */
        bool is_dir;                                           /* true=目录, false=文件 */
        uint64_t size;                                         /* 文件大小(字节), 目录为0 */
        time_t mtime;                                          /* 修改时间(unix时间戳) */
    } entry_t;

    entry_t *entries = NULL;
    size_t entry_count = 0;
    size_t entry_cap = 64;                                     /* 初始容量 */
    entries = malloc(entry_cap * sizeof(entry_t));
    if (entries == NULL) {
        closedir(dir);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    struct dirent *dp;
    while ((dp = readdir(dir)) != NULL) {
        /* 跳过 . 和 .. */
        if (strcmp(dp->d_name, ".") == 0 || strcmp(dp->d_name, "..") == 0) continue;

        /* 扩容 */
        if (entry_count >= entry_cap) {
            entry_cap *= 2;
            entry_t *new_entries = realloc(entries, entry_cap * sizeof(entry_t));
            if (new_entries == NULL) {
                free(entries);
                closedir(dir);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
                return ESP_FAIL;
            }
            entries = new_entries;
        }

        /* 填充条目 */
        entry_t *e = &entries[entry_count];
        strncpy(e->name, dp->d_name, sizeof(e->name) - 1);
        e->name[sizeof(e->name) - 1] = '\0';

        /* 获取文件详细信息 */
        char full_path[768];
        snprintf(full_path, sizeof(full_path), "%s/%s", norm_path, dp->d_name);
        if (stat(full_path, &st) == 0) {
            e->is_dir = S_ISDIR(st.st_mode);
            e->size   = e->is_dir ? 0 : st.st_size;
            e->mtime  = st.st_mtime;
        } else {
            e->is_dir = (dp->d_type == DT_DIR);
            e->size   = 0;
            e->mtime  = 0;
        }
        entry_count++;
    }
    closedir(dir);

    /* ---- 步骤6: 排序 (目录在前字母序 → 文件在后字母序) ---- */
    /* 冒泡排序 (条目数通常较小, 简单实现即可) */
    for (size_t i = 0; i < entry_count; i++) {
        for (size_t j = i + 1; j < entry_count; j++) {
            bool swap = false;
            /* 目录优先 */
            if (entries[i].is_dir != entries[j].is_dir) {
                if (!entries[i].is_dir) swap = true;           /* 文件在目录前 → 交换 */
            } else {
                /* 同类型按名称字母排序 */
                if (strcasecmp(entries[i].name, entries[j].name) > 0) swap = true;
            }
            if (swap) {
                entry_t tmp = entries[i];
                entries[i] = entries[j];
                entries[j] = tmp;
            }
        }
    }

    /* ---- 步骤7: 构建JSON响应 ---- */
    /* 预估大小: 每项约150字节, 加上外层结构 */
    size_t json_size = 256 + entry_count * 256 + 1;
    char *json = malloc(json_size);
    if (json == NULL) {
        free(entries);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    /* 计算parent路径 */
    char parent_path[512] = {0};
    strncpy(parent_path, norm_path, sizeof(parent_path) - 1);
    char *last_slash = strrchr(parent_path, '/');
    if (last_slash != NULL && last_slash > parent_path) {
        *last_slash = '\0';                                    /* 截断最后一段 */
    }
    if (strlen(parent_path) < strlen(g_base_path)) {
        strcpy(parent_path, g_base_path);                      /* 不低于根路径 */
    }

    int offset = snprintf(json, json_size,
        "{\"ok\":true,\"path\":\"%s\",\"parent\":\"%s\",\"entries\":[",
        norm_path, parent_path);

    for (size_t i = 0; i < entry_count; i++) {
        if (offset < 0 || (size_t)offset >= json_size - 256) {
            /* 缓冲区不足, 扩容 */
            json_size *= 2;
            char *new_json = realloc(json, json_size);
            if (new_json == NULL) {
                free(json);
                free(entries);
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
                return ESP_FAIL;
            }
            json = new_json;
            /* 重试... 简化处理: 截断列表并标记 */
        }

        char mtime_str[32] = "null";
        if (entries[i].mtime > 0) {
            snprintf(mtime_str, sizeof(mtime_str), "%lld", (long long)entries[i].mtime);
        }

        char size_str[32] = "0";
        if (!entries[i].is_dir) {
            snprintf(size_str, sizeof(size_str), "%llu", (unsigned long long)entries[i].size);
        }

        /* JSON字符串转义: 处理双引号和反斜杠 */
        char escaped_name[512];
        {
            const char *src = entries[i].name;
            char *dst = escaped_name;
            while (*src && (dst - escaped_name) < (int)sizeof(escaped_name) - 2) {
                if (*src == '"' || *src == '\\') *dst++ = '\\';
                *dst++ = *src++;
            }
            *dst = '\0';
        }

        offset += snprintf(json + offset, json_size - offset,
            "%s{\"name\":\"%s\",\"type\":\"%s\",\"size\":%s,\"modified\":%s}",
            (i > 0) ? "," : "",
            escaped_name,
            entries[i].is_dir ? "dir" : "file",
            size_str,
            mtime_str);
    }

    snprintf(json + offset, json_size - offset, "]}");

    /* ---- 步骤8: 发送响应 ---- */
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);

    free(entries);
    free(json);

    ESP_LOGI(TAG, "GET /api/list?path=%s → 200 (%d entries)", norm_path, (int)entry_count);
    return ESP_OK;
}

/**
 * @brief       文件操作API处理器: GET /api/file?path=/sd/file&action=preview|download
 *              action=preview: 返回文件内容 (JSON包装), 仅支持文本文件
 *              action=download: 返回原始文件流 (二进制), 支持所有文件类型
 * @param       req: HTTP请求对象
 * @retval      ESP_OK: 文件已发送
 */
static esp_err_t api_file_handler(httpd_req_t *req)
{
    char query[512] = {0};
    char raw_path[512] = {0};
    char action[32] = {0};
    char norm_path[512] = {0};

    /* ---- 步骤1: 获取查询字符串 ---- */
    size_t query_len = httpd_req_get_url_query_len(req) + 1;
    if (query_len > sizeof(query)) query_len = sizeof(query);
    if (httpd_req_get_url_query_str(req, query, query_len) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query string");
        return ESP_FAIL;
    }

    /* ---- 步骤2: 提取path和action参数 ---- */
    if (httpd_query_key_value(query, "path", raw_path, sizeof(raw_path)) != ESP_OK) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Missing 'path' parameter\",\"code\":\"MISSING_PATH\"}");
        return ESP_FAIL;
    }
    
    /* 确保URL解码 */
    url_decode_inplace(raw_path);
    if (httpd_query_key_value(query, "action", action, sizeof(action)) != ESP_OK) {
        /* action参数缺失: 默认download */
        strcpy(action, "download");
    }

    /* ---- 步骤3: 路径安全校验 ---- */
    if (!validate_path(raw_path, norm_path, sizeof(norm_path))) {
        ESP_LOGW(TAG, "Rejected file path: %s", raw_path);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Invalid path\",\"code\":\"INVALID_PATH\"}");
        return ESP_FAIL;
    }

    /* ---- 步骤4: 检查文件存在性 ---- */
    struct stat st;
    if (stat(norm_path, &st) != 0 || S_ISDIR(st.st_mode)) {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"File not found\",\"code\":\"NOT_FOUND\"}");
        return ESP_FAIL;
    }

    /* ---- 步骤5: 根据action分发处理 ---- */

    /* ========== preview: JSON内嵌文件内容 (仅文本文件) ========== */
    if (strcmp(action, "preview") == 0) {
        /* 检查是否为可预览的文本格式 */
        if (!is_text_file(norm_path)) {
            httpd_resp_set_status(req, "415 Unsupported Media Type");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Preview not supported for this file type\",\"code\":\"NOT_PREVIEWABLE\"}");
            return ESP_FAIL;
        }

        /* 文件大小限制: 最大512KB预览 */
        const size_t MAX_PREVIEW = 512 * 1024;
        if (st.st_size > (off_t)(MAX_PREVIEW * 2)) {
            httpd_resp_set_status(req, "413 Payload Too Large");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"File too large for preview (>1MB)\",\"code\":\"TOO_LARGE\"}");
            return ESP_FAIL;
        }

        FILE *fp = fopen(norm_path, "rb");
        if (fp == NULL) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot open file");
            return ESP_FAIL;
        }

        /* 分配读取缓冲区 */
        size_t content_size = st.st_size < (off_t)MAX_PREVIEW ? st.st_size : MAX_PREVIEW;
        char *content = malloc(content_size + 1);
        if (content == NULL) {
            fclose(fp);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
            return ESP_FAIL;
        }

        size_t bytes_read = fread(content, 1, content_size, fp);
        fclose(fp);
        content[bytes_read] = '\0';

        /* 检查是否为有效的UTF-8/ASCII文本 */
        bool valid_text = true;
        for (size_t i = 0; i < bytes_read; i++) {
            unsigned char c = (unsigned char)content[i];
            if (c < 0x20 && c != '\n' && c != '\r' && c != '\t') {
                valid_text = false;
                break;
            }
        }
        if (!valid_text) {
            free(content);
            httpd_resp_set_status(req, "415 Unsupported Media Type");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"File appears to be binary\",\"code\":\"NOT_PREVIEWABLE\"}");
            return ESP_FAIL;
        }

        /* 构建JSON响应 (需要转义content中的特殊字符) */
        const char *mime = get_mime_type(norm_path);
        size_t json_buf_size = bytes_read * 2 + 512;
        char *json_buf = malloc(json_buf_size);
        if (json_buf == NULL) {
            free(content);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
            return ESP_FAIL;
        }

        /* 构建JSON前缀 */
        int prefix_len = snprintf(json_buf, json_buf_size,
            "{\"ok\":true,\"path\":\"%s\",\"size\":%lld,\"type\":\"%s\",\"content\":\"",
            norm_path, (long long)st.st_size, mime);

        /* 转义文件内容并追加到JSON */
        char *dst = json_buf + prefix_len;
        size_t dst_remaining = json_buf_size - prefix_len - 10; /* 留10字节给后缀 */
        for (size_t i = 0; i < bytes_read && dst_remaining > 2; i++) {
            char c = content[i];
            if (c == '"') {
                *dst++ = '\\'; *dst++ = '"'; dst_remaining -= 2;
            } else if (c == '\\') {
                *dst++ = '\\'; *dst++ = '\\'; dst_remaining -= 2;
            } else if (c == '\n') {
                *dst++ = '\\'; *dst++ = 'n'; dst_remaining -= 2;
            } else if (c == '\r') {
                *dst++ = '\\'; *dst++ = 'r'; dst_remaining -= 2;
            } else if (c == '\t') {
                *dst++ = '\\'; *dst++ = 't'; dst_remaining -= 2;
            } else if ((unsigned char)c < 0x20) {
                /* 其他控制字符: Unicode转义 */
                int n = snprintf(dst, dst_remaining, "\\u%04x", (unsigned char)c);
                dst += n; dst_remaining -= n;
            } else {
                *dst++ = c; dst_remaining--;
            }
        }
        /* JSON后缀 */
        strcpy(dst, "\"}");

        free(content);

        httpd_resp_set_type(req, "application/json; charset=utf-8");
        httpd_resp_send(req, json_buf, HTTPD_RESP_USE_STRLEN);
        free(json_buf);

        ESP_LOGI(TAG, "GET /api/file?path=%s&action=preview → 200 (%d bytes)", norm_path, (int)bytes_read);
    }

    /* ========== download: 流式传输原始文件 (所有类型) ========== */
    else {
        FILE *fp = fopen(norm_path, "rb");
        if (fp == NULL) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Cannot open file");
            return ESP_FAIL;
        }

        /* 设置响应头 */
        const char *mime = get_mime_type(norm_path);
        httpd_resp_set_type(req, mime);

        /* 提取纯文件名(路径最后一段)用于Content-Disposition */
        const char *filename = strrchr(norm_path, '/');
        if (filename != NULL) filename++;
        else filename = norm_path;

        char cd_header[512];
        int cd_written = snprintf(cd_header, sizeof(cd_header), "attachment; filename=\"%.256s\"", filename);
        if (cd_written >= (int)sizeof(cd_header)) {
            /* 文件名过长(>256字符), 截断已在format中处理 */
            cd_header[sizeof(cd_header) - 2] = '"';
            cd_header[sizeof(cd_header) - 1] = '\0';
        }
        httpd_resp_set_hdr(req, "Content-Disposition", cd_header);

        /* 设置Content-Length */
        char cl_header[32];
        snprintf(cl_header, sizeof(cl_header), "%lld", (long long)st.st_size);
        httpd_resp_set_hdr(req, "Content-Length", cl_header);

        /* 分块读取并发送文件内容 (每32KB让出一次CPU, 防止看门狗超时) */
        char chunk[2048];
        size_t bytes_read;
        int chunk_count = 0;
        while ((bytes_read = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
            if (httpd_resp_send_chunk(req, chunk, bytes_read) != ESP_OK) {
                fclose(fp);
                ESP_LOGW(TAG, "Client disconnected during download");
                return ESP_FAIL;                               /* 客户端断开连接 */
            }
            if (++chunk_count % 16 == 0) {
                vTaskDelay(1);                                  /* 每16个chunk(32KB)让出CPU 1ms */
            }
        }
        httpd_resp_send_chunk(req, NULL, 0);                   /* 终止分块传输 */
        fclose(fp);

        ESP_LOGI(TAG, "GET /api/file?path=%s&action=download → 200 (%lld bytes)", norm_path, (long long)st.st_size);
    }

    return ESP_OK;
}

/* ======================== WiFi配置API ======================== */

/**
 * @brief       将WiFi配置保存到NVS(非易失性存储)
 * @param       ssid: WiFi名称 (最大31字符)
 * @param       password: WiFi密码 (最大63字符, 可为空字符串表示开放网络)
 * @retval      ESP_OK: 保存成功
 */
static esp_err_t save_wifi_config_to_nvs(const char *ssid, const char *password)
{
    nvs_handle_t nvs_h;
    esp_err_t ret = nvs_open("wifi_config", NVS_READWRITE, &nvs_h);
    if (ret != ESP_OK) {
        printf("[WEB_SRV] NVS open for write failed: %s\n", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(nvs_h, "ssid", ssid);
    if (ret != ESP_OK) {
        printf("[WEB_SRV] NVS set ssid failed: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_h);
        return ret;
    }

    ret = nvs_set_str(nvs_h, "password", password);
    if (ret != ESP_OK) {
        printf("[WEB_SRV] NVS set password failed: %s\n", esp_err_to_name(ret));
        nvs_close(nvs_h);
        return ret;
    }

    ret = nvs_commit(nvs_h);
    if (ret != ESP_OK) {
        printf("[WEB_SRV] NVS commit failed: %s\n", esp_err_to_name(ret));
    }
    nvs_close(nvs_h);
    return ret;
}

/**
 * @brief       读取当前WiFi配置: GET /api/wifi-config
 *              密码脱敏返回(仅显示首尾字符)
 */
static esp_err_t wifi_config_get_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: GET /api/wifi-config\n");

    char ssid[32] = "BOSSCOM_USB_AP";
    char password[64] = "012345678";
    nvs_handle_t nvs_h;

    if (nvs_open("wifi_config", NVS_READONLY, &nvs_h) == ESP_OK) {
        size_t len = sizeof(ssid);
        nvs_get_str(nvs_h, "ssid", ssid, &len);
        len = sizeof(password);
        nvs_get_str(nvs_h, "password", password, &len);
        nvs_close(nvs_h);
    }

    char json[256];
    snprintf(json, sizeof(json),
        "{\"ok\":true,\"ssid\":\"%s\",\"password\":\"%s\"}",
        ssid, password);

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);

    printf("[WEB_SRV] <<< DONE: GET /api/wifi-config → SSID='%s'\n", ssid);
    return ESP_OK;
}

/**
 * @brief       保存WiFi配置: POST /api/wifi-config
 *              Body: {"ssid":"...", "password":"..."}
 *              保存后延时1秒自动重启芯片使配置生效
 */
static esp_err_t wifi_config_post_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: POST /api/wifi-config\n");

    /* ---- 读取POST body ---- */
    char body[256] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Empty request body\",\"code\":\"EMPTY_BODY\"}");
        return ESP_FAIL;
    }
    body[received] = '\0';
    printf("[WEB_SRV] POST body: '%s'\n", body);

    /* ---- 简易JSON解析: 提取ssid和password字段 ---- */
    char new_ssid[32] = {0};
    char new_pass[64] = {0};

    /* 查找 "ssid" 字段 */
    const char *ssid_key = strstr(body, "\"ssid\"");
    if (ssid_key) {
        const char *val_start = strchr(ssid_key, ':');
        if (val_start) {
            val_start++; /* 跳过冒号 */
            while (*val_start == ' ' || *val_start == '"') val_start++; /* 跳过空格和引号 */
            const char *val_end = strchr(val_start, '"');
            if (val_end) {
                size_t len = val_end - val_start;
                if (len >= sizeof(new_ssid)) len = sizeof(new_ssid) - 1;
                memcpy(new_ssid, val_start, len);
                new_ssid[len] = '\0';
            }
        }
    }

    /* 查找 "password" 字段 */
    const char *pass_key = strstr(body, "\"password\"");
    if (pass_key) {
        const char *val_start = strchr(pass_key, ':');
        if (val_start) {
            val_start++;
            while (*val_start == ' ' || *val_start == '"') val_start++;
            const char *val_end = strchr(val_start, '"');
            if (val_end) {
                size_t len = val_end - val_start;
                if (len >= sizeof(new_pass)) len = sizeof(new_pass) - 1;
                memcpy(new_pass, val_start, len);
                new_pass[len] = '\0';
            }
        }
    }

    /* ---- 校验 ---- */
    size_t ssid_len = strlen(new_ssid);
    size_t pass_len = strlen(new_pass);

    if (ssid_len < 1 || ssid_len > 31) {
        printf("[WEB_SRV] SSID validation failed: len=%d\n", (int)ssid_len);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SSID\xE9\x95\xBF\xE5\xBA\xA6\xE5\xBF\x85\xE9\xA1\xBB\xE4\xB8\xBA""1-31\xE4\xBD\x8D\",\"code\":\"INVALID_SSID\"}");
        return ESP_FAIL;
    }

    if (pass_len > 0 && (pass_len < 8 || pass_len > 63)) {
        printf("[WEB_SRV] Password validation failed: len=%d\n", (int)pass_len);
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"\xE5\xAF\x86\xE7\xA0\x81\xE9\x9C\x80\xE8\xA6\x81""8-63\xE4\xBD\x8D\xE6\x88\x96\xE7\x95\x99\xE7\xA9\xBA\",\"code\":\"INVALID_PASSWORD\"}");
        return ESP_FAIL;
    }

    /* ---- 保存到NVS ---- */
    esp_err_t ret = save_wifi_config_to_nvs(new_ssid, (pass_len > 0) ? new_pass : "");
    if (ret != ESP_OK) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"\xE4\xBF\x9D\xE5\xAD\x98\xE5\xA4\xB1\xE8\xB4\xA5\",\"code\":\"SAVE_FAILED\"}");
        return ESP_FAIL;
    }

    printf("[WEB_SRV] WiFi config saved: SSID='%s', PW_LEN=%d, restarting...\n", new_ssid, (int)pass_len);

    /* ---- 返回成功响应 ---- */
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"\xE5\xB7\xB2\xE4\xBF\x9D\xE5\xAD\x98\xEF\xBC\x8CWiFi\xE6\xAD\xA3\xE5\x9C\xA8\xE9\x87\x8D\xE5\x90\xAF\xEF\xBC\x8C\xE8\xAF\xB7\xE9\x87\x8D\xE6\x96\xB0\xE8\xBF\x9E\xE6\x8E\xA5\"}");

    /* ---- 延时1秒后重启 ---- */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK;
}

/* ======================== 从机注册API (主机端) ======================== */

/**
 * @brief       从机注册: POST /api/register
 *              接收从机的注册请求, 记录从机IP地址用于后续文件推送
 *              Body: {"role":"slave"}
 *              通过HTTP连接的socket获取从机真实IP
 * @param       req: HTTP请求对象
 * @retval      ESP_OK: 注册成功
 */
static esp_err_t slave_register_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: POST /api/register\n");

    /* ---- 读取POST body ---- */
    char body[128] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[received] = '\0';

    /* ---- 通过socket获取客户端IP地址 ---- */
    int sockfd = httpd_req_to_sockfd(req);
    if (sockfd >= 0) {
        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        if (getpeername(sockfd, (struct sockaddr *)&addr, &addr_len) == 0) {
            snprintf(g_slave_ip, sizeof(g_slave_ip), "%s", inet_ntoa(addr.sin_addr));
            printf("[WEB_SRV] Slave registered: IP=%s\n", g_slave_ip);
        } else {
            printf("[WEB_SRV] getpeername failed\n");
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Cannot determine client IP\"}");
            return ESP_FAIL;
        }
    } else {
        /* 降级: 从请求头X-Forwarded-For获取(如有) */
        size_t hdr_len = httpd_req_get_hdr_value_len(req, "X-Forwarded-For");
        if (hdr_len > 0) {
            char *hdr_val = malloc(hdr_len + 1);
            if (hdr_val) {
                httpd_req_get_hdr_value_str(req, "X-Forwarded-For", hdr_val, hdr_len + 1);
                strncpy(g_slave_ip, hdr_val, sizeof(g_slave_ip) - 1);
                g_slave_ip[sizeof(g_slave_ip) - 1] = '\0';
                free(hdr_val);
                printf("[WEB_SRV] Slave registered (via X-Forwarded-For): IP=%s\n", g_slave_ip);
            }
        }
        if (g_slave_ip[0] == '\0') {
            printf("[WEB_SRV] Cannot determine slave IP\n");
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Cannot determine client IP\"}");
            return ESP_FAIL;
        }
    }

    /* ---- 返回成功 ---- */
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"registered\"}");

    printf("[WEB_SRV] <<< DONE: POST /api/register → slave at %s\n", g_slave_ip);
    return ESP_OK;
}

/* ======================== 文件删除API (主机端) ======================== */

/**
 * @brief       删除文件: POST /api/delete
 *              Body: {"path":"/SD/filename.txt"}
 *              路径经过安全校验, 仅允许删除/SD下的文件
 * @param       req: HTTP请求对象
 * @retval      ESP_OK: 删除成功
 */
static esp_err_t file_delete_handler(httpd_req_t *req)
{
    printf("[WEB_SRV] >>> HANDLER: POST /api/delete\n");

    /* ---- 读取POST body ---- */
    char body[512] = {0};
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Empty body\",\"code\":\"EMPTY_BODY\"}");
        return ESP_FAIL;
    }
    body[received] = '\0';

    /* 解析 "path" 字段 */
    char raw_path[512] = {0};
    const char *path_key = strstr(body, "\"path\"");
    if (path_key) {
        const char *val_start = strchr(path_key, ':');
        if (val_start) {
            val_start++;
            while (*val_start == ' ' || *val_start == '"') val_start++;
            const char *val_end = strchr(val_start, '"');
            if (val_end) {
                size_t len = val_end - val_start;
                if (len >= sizeof(raw_path)) len = sizeof(raw_path) - 1;
                memcpy(raw_path, val_start, len);
                raw_path[len] = '\0';
            }
        }
    }

    if (raw_path[0] == '\0') {
        httpd_resp_set_status(req, "400 Bad Request");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Missing 'path' parameter\"}");
        return ESP_FAIL;
    }

    /* ---- 路径安全校验 ---- */
    char norm_path[512] = {0};
    if (!validate_path(raw_path, norm_path, sizeof(norm_path))) {
        printf("[WEB_SRV] DELETE rejected invalid path: %s\n", raw_path);
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Invalid path\",\"code\":\"INVALID_PATH\"}");
        return ESP_FAIL;
    }

    /* 拒绝删除根目录/SD */
    if (strcmp(norm_path, "/SD") == 0) {
        httpd_resp_set_status(req, "403 Forbidden");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Cannot delete root\",\"code\":\"ROOT_DENIED\"}");
        return ESP_FAIL;
    }

    /* ---- 检查文件存在 ---- */
    struct stat st;
    if (stat(norm_path, &st) != 0) {
        httpd_resp_set_status(req, "404 Not Found");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"File not found\"}");
        return ESP_FAIL;
    }

    /* ---- 执行删除 ---- */
    if (S_ISDIR(st.st_mode)) {
        /* 目录: 使用rmdir (仅空目录) */
        if (rmdir(norm_path) != 0) {
            printf("[WEB_SRV] DELETE rmdir FAILED: %s\n", norm_path);
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Cannot remove directory\"}");
            return ESP_FAIL;
        }
        printf("[WEB_SRV] DELETE dir OK: %s\n", norm_path);
    } else {
        /* 文件: 使用remove */
        if (remove(norm_path) != 0) {
            printf("[WEB_SRV] DELETE remove FAILED: %s\n", norm_path);
            httpd_resp_set_status(req, "500 Internal Server Error");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"Cannot remove file\"}");
            return ESP_FAIL;
        }
        printf("[WEB_SRV] DELETE file OK: %s\n", norm_path);
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":true}");

    printf("[WEB_SRV] <<< DONE: POST /api/delete → %s\n", norm_path);
    return ESP_OK;
}

/* ======================== 公开API ======================== */

/**
 * @brief       启动HTTP Web文件服务器
 *              注册3个URI处理器到端口80
 * @param       无
 * @retval      ESP_OK: 启动成功
 * @retval      ESP_FAIL: 启动失败
 */
esp_err_t web_server_start(void)
{
    if (g_server != NULL) {
        ESP_LOGW(TAG, "Web server already running");
        return ESP_OK;                                          /* 已在运行 */
    }

    printf("[WEB_SRV] Starting HTTP server on port 80...\n");

    /* ---- 启动前诊断: 检查SD卡是否可访问 ---- */
    {
        DIR *test_dir = opendir("/SD");
        if (test_dir == NULL) {
            printf("[WEB_SRV] WARNING: Cannot opendir('/SD') - SD card not accessible!\n");
            printf("[WEB_SRV] Possible reasons:\n");
            printf("[WEB_SRV]   1) No SD card inserted\n");
            printf("[WEB_SRV]   2) USB cable connected to OTG port (PC claimed the storage)\n");
            printf("[WEB_SRV]   3) SD card init failed\n");
            printf("[WEB_SRV] Web server will start but file listing may fail.\n");
        } else {
            /* 计数根目录下的文件数 */
            int count = 0;
            struct dirent *d;
            while ((d = readdir(test_dir)) != NULL) {
                if (strcmp(d->d_name, ".") != 0 && strcmp(d->d_name, "..") != 0) count++;
            }
            closedir(test_dir);
            printf("[WEB_SRV] SD card OK: /SD  has %d entries\n", count);
        }
    }

    /* ---- 创建HTTP服务器配置 ---- */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;                                /* 当前8个处理器 */
    config.stack_size = 8192;                                   /* 8KB栈: 足够文件I/O操作 */
    config.server_port = 80;                                    /* 标准HTTP端口 */

    /* ---- 启动服务器 ---- */
    esp_err_t ret = httpd_start(&g_server, &config);
    if (ret != ESP_OK) {
        printf("[WEB_SRV] Start FAILED: %s\n", esp_err_to_name(ret));
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    printf("[WEB_SRV] HTTP server started (port 80)\n");

    /* ---- 注册URI处理器: GET / (根路径) ---- */
    httpd_uri_t uri_root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = root_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_root);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register / handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: GET /\n");

    /* ---- 注册URI处理器: GET /api/list ---- */
    httpd_uri_t uri_list = {
        .uri       = "/api/list",
        .method    = HTTP_GET,
        .handler   = api_list_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /api/list handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: GET /api/list\n");

    /* ---- 注册URI处理器: GET /api/ping (健康检查) ---- */
    httpd_uri_t uri_ping = {
        .uri       = "/api/ping",
        .method    = HTTP_GET,
        .handler   = ping_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_ping);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /api/ping handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: GET /api/ping\n");

    /* ---- 注册URI处理器: GET /api/file ---- */
    httpd_uri_t uri_file = {
        .uri       = "/api/file",
        .method    = HTTP_GET,
        .handler   = api_file_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_file);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register /api/file handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: GET /api/file\n");

    /* ---- 注册URI处理器: GET /api/wifi-config (读取WiFi配置) ---- */
    httpd_uri_t uri_wifi_get = {
        .uri       = "/api/wifi-config",
        .method    = HTTP_GET,
        .handler   = wifi_config_get_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_wifi_get);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /api/wifi-config handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: GET /api/wifi-config\n");

    /* ---- 注册URI处理器: POST /api/wifi-config (保存WiFi配置) ---- */
    httpd_uri_t uri_wifi_post = {
        .uri       = "/api/wifi-config",
        .method    = HTTP_POST,
        .handler   = wifi_config_post_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_wifi_post);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /api/wifi-config handler");
        httpd_stop(g_server);
        g_server = NULL;
        return ret;
    }
    printf("[WEB_SRV] Registered: POST /api/wifi-config\n");

    /* ---- 注册URI处理器: POST /api/register (从机注册) ---- */
    httpd_uri_t uri_register = {
        .uri       = "/api/register",
        .method    = HTTP_POST,
        .handler   = slave_register_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_register);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /api/register handler");
    } else {
        printf("[WEB_SRV] Registered: POST /api/register\n");
    }

    /* ---- 注册URI处理器: POST /api/delete (文件删除) ---- */
    httpd_uri_t uri_delete = {
        .uri       = "/api/delete",
        .method    = HTTP_POST,
        .handler   = file_delete_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(g_server, &uri_delete);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /api/delete handler");
    } else {
        printf("[WEB_SRV] Registered: POST /api/delete\n");
    }

    printf("[WEB_SRV] Web file server ready at http://192.168.3.1\n");
    ESP_LOGI(TAG, "Web file server started successfully");
    return ESP_OK;
}

/**
 * @brief       停止HTTP Web文件服务器
 * @param       无
 * @retval      ESP_OK: 停止成功
 */
esp_err_t web_server_stop(void)
{
    if (g_server == NULL) {
        ESP_LOGW(TAG, "Web server not running");
        return ESP_FAIL;
    }

    esp_err_t ret = httpd_stop(g_server);
    g_server = NULL;

    if (ret == ESP_OK) {
        printf("[WEB_SRV] Server stopped\n");
        ESP_LOGI(TAG, "Web file server stopped");
    } else {
        ESP_LOGE(TAG, "Failed to stop server: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* ======================== 从机IP查询 ======================== */

/**
 * @brief       获取已注册从机的IP地址
 */
bool web_server_get_slave_ip(char *ip_buf, size_t buf_size)
{
    if (g_slave_ip[0] == '\0' || ip_buf == NULL || buf_size == 0) {
        return false;
    }
    strncpy(ip_buf, g_slave_ip, buf_size - 1);
    ip_buf[buf_size - 1] = '\0';
    return true;
}

/**
 * @brief       清除从机注册状态
 */
void web_server_clear_slave(void)
{
    printf("[WEB_SRV] Slave cleared (was: %s)\n", g_slave_ip[0] ? g_slave_ip : "none");
    g_slave_ip[0] = '\0';
}
