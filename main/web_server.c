/**
 ****************************************************************************************************
 * @file        web_server.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-12
 * @brief       Web文件服务器 - HTTP请求处理实现
 *              功能: ①启动/停止HTTP服务器(端口80) → ②路径安全校验 →
 *                    ③MIME类型检测 → ④目录列表JSON API → ⑤文件预览/下载API
 *              协议: HTTP/1.0 over TCP (LwIP协议栈)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
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
#include <stdio.h>                           /* 标准I/O: snprintf, fopen, fread, fclose */
#include <stdlib.h>                          /* 标准库: malloc, free, realloc */
#include <string.h>                          /* 字符串: strlen, strcpy, strchr, strcmp */
#include <ctype.h>                           /* 字符分类: tolower */
#include <dirent.h>                          /* POSIX目录: opendir, readdir, closedir */
#include <sys/stat.h>                        /* POSIX文件状态: stat, S_ISDIR */
#include <strings.h>                         /* POSIX扩展字符串: strcasecmp */

static const char *TAG = "WEB_SRV";          /* 日志标签: 用于ESP_LOGI/ESP_LOGE输出前缀 */

/* 来自main.c的模式查询: true=WiFi网页模式(FATFS在ESP32本地), false=USB U盘模式(SD归PC独占) */
extern bool app_in_wifi_mode(void);

/* ======================== 全局变量 ======================== */
static httpd_handle_t g_server = NULL;       /* HTTP服务器句柄: NULL=未启动, 非NULL=运行中 */
static char g_base_path[16] = "/sd";         /* SD卡根路径: 所有文件访问均限制在此目录下 */

/* 目录条目结构(供api_list_handler和排序比较器使用) */
typedef struct {
    char name[256];                                        /* 文件/目录名 */
    bool is_dir;                                           /* true=目录, false=文件 */
    uint64_t size;                                         /* 文件大小(字节), 目录为0 */
    time_t mtime;                                          /* 修改时间(unix时间戳) */
} entry_t;

/**
 * @brief       目录条目排序比较器(qsort用)
 *              规则: 目录在前; 同类型按名称字母序(大小写不敏感)
 */
static int entry_cmp(const void *a, const void *b)
{
    const entry_t *ea = (const entry_t *)a;
    const entry_t *eb = (const entry_t *)b;
    if (ea->is_dir != eb->is_dir) return ea->is_dir ? -1 : 1;  /* 目录优先 */
    return strcasecmp(ea->name, eb->name);                     /* 同类型按名称 */
}

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

    /* ---- 必须以 /sd 开头 ---- */
    size_t base_len = strlen(g_base_path);
    if (strncmp(path, g_base_path, base_len) != 0) return false;
    /* 必须刚好匹配 /sd 或 /sd/xxx */
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
    /* 如果输入就是 /sd, 直接返回 */
    if (path[base_len] == '\0') {
        *out = '\0';
        return true;
    }

    /* 跳过 /sd/ 后的分隔符, 逐段处理 */
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
            /* 不允许穿越到 /sd 之外 */
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

    /* 最终检查: 规范化后至少等于 /sd */
    if (strncmp(normalized, g_base_path, base_len) != 0) return false;
    if (strlen(normalized) < base_len) return false;

    return true;
}

/**
 * @brief       URL解码: 将 %XX 转为原始字符, 原地修改
 * @note        不转换 '+': 本函数仅用于路径参数, 路径中的'+'是合法文件名字符
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

    /* ---- 步骤5: 遍历目录 (仅WiFi模式允许失败时重新挂载恢复) ---- */
    DIR *dir = opendir(norm_path);
    if (dir == NULL) {
        /* USB U盘模式下SD卡由PC独占, 此处强行本地挂载会导致PC端U盘
         * 报"介质不存在"(test_unit_ready_cb在is_fat_mounted时返回NOT READY),
         * 且主循环认为状态稳定不会纠正, 因此必须拒绝重挂载 */
        if (!app_in_wifi_mode()) {
            printf("[WEB_SRV] opendir failed & USB mode active - refuse remount\n");
            httpd_resp_set_status(req, "503 Service Unavailable");
            httpd_resp_set_type(req, "application/json");
            httpd_resp_sendstr(req, "{\"ok\":false,\"error\":\"SD\xE5\x8D\xA1\xE6\x9A\x82\xE6\x97\xB6\xE4\xB8\x8D\xE5\x8F\xAF\xE7\x94\xA8\xEF\xBC\x8C\xE8\xAF\xB7\xE5\x88\xB7\xE6\x96\xB0\xE9\xA1\xB5\xE9\x9D\xA2\xE9\x87\x8D\xE8\xAF\x95\",\"code\":\"SD_NOT_MOUNTED\"}");
            return ESP_FAIL;
        }
        /* WiFi模式下可能是USB弹出后TinyUSB未自动重新挂载, 尝试手动挂载一次 */
        printf("[WEB_SRV] opendir failed, trying remount...\n");
        tinyusb_msc_storage_unmount();                              /* 先卸载清理残留状态 */
        esp_err_t remount_ret = tinyusb_msc_storage_mount("/sd");
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
            /* stat失败时不用d_type判断(FATFS VFS下常为DT_UNKNOWN), 改用opendir探测 */
            DIR *sub = opendir(full_path);
            if (sub != NULL) {
                e->is_dir = true;
                closedir(sub);
            } else {
                e->is_dir = false;
            }
            e->size   = 0;
            e->mtime  = 0;
        }
        entry_count++;
    }
    closedir(dir);

    /* ---- 步骤6: 排序 (目录在前字母序 → 文件在后字母序, O(n·logn)) ---- */
    qsort(entries, entry_count, sizeof(entry_t), entry_cmp);

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

        /* 构建JSON前缀 (truncated: 文件超过预览上限被截断时为true, 前端可提示下载查看全文) */
        int prefix_len = snprintf(json_buf, json_buf_size,
            "{\"ok\":true,\"path\":\"%s\",\"size\":%lld,\"type\":\"%s\",\"truncated\":%s,\"content\":\"",
            norm_path, (long long)st.st_size, mime,
            ((off_t)st.st_size > (off_t)MAX_PREVIEW) ? "true" : "false");

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

        /* 注: 分块传输(httpd_resp_send_chunk)自动使用Transfer-Encoding: chunked,
         * RFC 7230禁止与Content-Length同时出现, 因此不再手动设置Content-Length */

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

/* ======================== 简易JSON字符串提取 ======================== */

/**
 * @brief       定位JSON中指定键的字符串值(返回值的首个引号位置)
 *              逐个扫描完整字符串token: 键名必须整体匹配, 值内的转义引号不会干扰扫描,
 *              避免了strstr方式"键名出现在其他键的值中导致误匹配"的问题
 * @param       body: JSON文本
 * @param       key: 键名(不含引号)
 * @retval      指向值的起始引号; 未找到返回NULL
 */
static const char *json_find_string_value(const char *body, const char *key)
{
    const size_t klen = strlen(key);
    const char *p = body;

    while ((p = strchr(p, '"')) != NULL) {
        p++;                                                    /* 跳过起始引号, p指向内容首字符 */
        if (strncmp(p, key, klen) == 0 && p[klen] == '"') {     /* 键名整体匹配(如"password"不匹配"password2") */
            const char *v = p + klen + 1;                       /* 跳过键名结束引号 */
            while (*v == ' ' || *v == ':') v++;                 /* 跳过冒号和空白 */
            return v;
        }
        /* 不是目标键: 跳过整个字符串(处理\"转义), 继续找下一个token */
        while (*p && *p != '"') {
            if (*p == '\\' && p[1] != '\0') p++;
            p++;
        }
    }
    return NULL;
}

/**
 * @brief       从JSON中提取字符串值并解码转义序列
 *              支持: \" \\ \/ \n \r \t \b \f \uXXXX(BMP基本多文种平面, 代理对丢弃)
 * @param       body: JSON文本
 * @param       key: 键名
 * @param       out: 输出缓冲区(始终以'\0'结尾)
 * @param       out_size: 缓冲区总大小
 * @retval      true: 提取成功(含空串); false: 键不存在或值不是字符串
 */
static bool json_extract_string(const char *body, const char *key, char *out, size_t out_size)
{
    const char *v = json_find_string_value(body, key);
    size_t o = 0;

    if (v == NULL || *v != '"') return false;                   /* 值缺失或非字符串类型 */
    v++;                                                        /* 跳过值的起始引号 */

    while (*v && *v != '"' && o + 1 < out_size) {
        char c = *v;
        if (c == '\\' && v[1] != '\0') {
            v++;                                                /* 指向转义字母 */
            switch (*v) {
            case '"':  c = '"';  break;
            case '\\': c = '\\'; break;
            case '/':  c = '/';  break;
            case 'n':  c = '\n'; break;
            case 'r':  c = '\r'; break;
            case 't':  c = '\t'; break;
            case 'b':  c = '\b'; break;
            case 'f':  c = '\f'; break;
            case 'u': {                                             /* \uXXXX → UTF-8 */
                char hex[5] = {0};
                unsigned long cp;
                if (v[1] == '\0' || v[2] == '\0' || v[3] == '\0' || v[4] == '\0') return false;
                memcpy(hex, v + 1, 4);                              /* 固定取4位hex, 防止strtol越界吞字符 */
                cp = strtoul(hex, NULL, 16);
                v += 4;                                             /* 移到最后一位hex上(循环尾部再统一++) */
                if (cp < 0x80) {                                    /* 1字节 ASCII */
                    if (o + 1 < out_size) out[o++] = (char)cp;
                } else if (cp < 0x800) {                            /* 2字节 */
                    if (o + 2 < out_size) {
                        out[o++] = (char)(0xC0 | (cp >> 6));
                        out[o++] = (char)(0x80 | (cp & 0x3F));
                    }
                } else if (cp <= 0xFFFF && !(cp >= 0xD800 && cp <= 0xDFFF)) { /* 3字节(代理对区间丢弃) */
                    if (o + 3 < out_size) {
                        out[o++] = (char)(0xE0 | (cp >> 12));
                        out[o++] = (char)(0x80 | ((cp >> 6) & 0x3F));
                        out[o++] = (char)(0x80 | (cp & 0x3F));
                    }
                }
                v++;
                continue;                                           /* 已自行写入并前进, 跳过下方通用追加 */
            }
            default: c = *v; break;                                 /* 未知转义: 保留原字符 */
            }
        }
        out[o++] = c;
        v++;
    }
    out[o] = '\0';
    return true;
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
 *              密码明文返回(网页设置面板回显需要, 设备仅通过AP局域网访问)
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

    /* ---- JSON解析: 提取ssid和password字段(支持转义字符, 无跨键误匹配) ---- */
    char new_ssid[32] = {0};
    char new_pass[64] = {0};

    json_extract_string(body, "ssid", new_ssid, sizeof(new_ssid));      /* 缺失时为空串, 由下方校验拦截 */
    json_extract_string(body, "password", new_pass, sizeof(new_pass));  /* 可选字段: 空串=开放网络 */

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
        DIR *test_dir = opendir("/sd");
        if (test_dir == NULL) {
            printf("[WEB_SRV] WARNING: Cannot opendir('/sd') - SD card not accessible!\n");
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
            printf("[WEB_SRV] SD card OK: /sd has %d entries\n", count);
        }
    }

    /* ---- 创建HTTP服务器配置 ---- */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 10;                                /* 当前6个处理器(含2个wifi-config) */
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

    printf("[WEB_SRV] Web file server ready at http://192.168.4.1\n");
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
