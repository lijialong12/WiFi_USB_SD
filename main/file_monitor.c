/**
 ****************************************************************************************************
 * @file        file_monitor.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-15
 * @brief       文件变化检测模块实现 - FATFS周期性扫描 + 已知集对比
 *              实现: ①递归扫描SD卡 → ②对比已知集 → ③维护传输队列 →
 *                    ④重试/完成标记
 *              FATFS不支持文件系统事件通知, 采用周期polling方式
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 (DEVICE_ROLE=0 主机端)
 * 内存限制: 已知集最多128条, 传输队列最多64条
 * 线程安全: 仅在app_main任务中调用, 无需加锁
 *
 ****************************************************************************************************
 */

#include "file_monitor.h"                    /* 文件监控API头文件 */
#include <stdio.h>                           /* 标准I/O: printf, snprintf, remove */
#include <stdlib.h>                          /* 标准库: malloc, free, realloc */
#include <string.h>                          /* 字符串: strcmp, strncpy, strlen */
#include <dirent.h>                          /* POSIX目录: opendir, readdir, closedir */
#include <sys/stat.h>                        /* POSIX文件状态: stat, S_ISDIR */
#include "esp_log.h"                         /* ESP-IDF日志: ESP_LOGI/W/E */

static const char *TAG = "FILE_MON";         /* 日志标签 */

/* ======================== 内部数据结构 ======================== */

/* 已知集链表节点: 记录已扫描到的文件(用于判断新文件) */
typedef struct known_node_t {
    char     path[512];                      /* 文件完整路径 */
    uint64_t size;                           /* 上次记录的文件大小 */
    time_t   mtime;                          /* 上次记录的修改时间 */
    struct known_node_t *next;               /* 链表下一节点 */
} known_node_t;

/* 传输队列链表节点: 待传输的文件 */
typedef struct pending_node_t {
    file_entry_t entry;                      /* 文件条目(含路径/大小/重试次数等) */
    struct pending_node_t *next;             /* 链表下一节点 */
} pending_node_t;

/* ======================== 全局状态 ======================== */
static known_node_t  *g_known_head   = NULL; /* 已知集链表头 */
static int            g_known_count  = 0;     /* 已知集条目数 */
static pending_node_t *g_pending_head = NULL; /* 传输队列链表头 */
static pending_node_t *g_pending_tail = NULL; /* 传输队列链表尾(优化追加) */
static int            g_pending_count = 0;    /* 传输队列条目数 */

#define MAX_KNOWN   128                       /* 已知集最大条目数 */
#define MAX_PENDING 64                        /* 传输队列最大条目数 */
#define MAX_SCAN_DEPTH 8                      /* 最大扫描深度(目录嵌套层数) */

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief       在已知集中查找指定路径的文件
 * @param       path: 文件路径
 * @retval      找到的节点指针, NULL=未找到
 */
static known_node_t *known_find(const char *path)
{
    for (known_node_t *n = g_known_head; n != NULL; n = n->next) {
        if (strcmp(n->path, path) == 0) {
            return n;
        }
    }
    return NULL;
}

/**
 * @brief       将文件路径添加到已知集 (或更新已有记录)
 * @param       path: 文件路径
 * @param       size: 文件大小
 * @param       mtime: 修改时间
 */
static void known_add_or_update(const char *path, uint64_t size, time_t mtime)
{
    known_node_t *existing = known_find(path);
    if (existing != NULL) {
        /* 更新已有记录 */
        existing->size  = size;
        existing->mtime = mtime;
        return;
    }

    /* 检查容量上限 */
    if (g_known_count >= MAX_KNOWN) {
        /* 移除最旧的节点(链表头)来腾出空间 */
        known_node_t *old = g_known_head;
        if (old != NULL) {
            g_known_head = old->next;
            free(old);
            g_known_count--;
        }
    }

    /* 分配新节点 */
    known_node_t *node = malloc(sizeof(known_node_t));
    if (node == NULL) {
        ESP_LOGE(TAG, "known_add: malloc failed");
        return;
    }

    strncpy(node->path, path, sizeof(node->path) - 1);
    node->path[sizeof(node->path) - 1] = '\0';
    node->size  = size;
    node->mtime = mtime;
    node->next  = g_known_head;              /* 插入到链表头 */
    g_known_head = node;
    g_known_count++;
}

/**
 * @brief       从已知集中移除文件记录
 * @param       path: 文件路径
 */
static void known_remove(const char *path)
{
    known_node_t *prev = NULL;
    known_node_t *curr = g_known_head;

    while (curr != NULL) {
        if (strcmp(curr->path, path) == 0) {
            if (prev == NULL) {
                g_known_head = curr->next;
            } else {
                prev->next = curr->next;
            }
            free(curr);
            g_known_count--;
            return;
        }
        prev = curr;
        curr = curr->next;
    }
}

/**
 * @brief       检查文件是否已在传输队列中
 * @param       path: 文件路径
 * @retval      true=已在队列
 */
static bool pending_contains(const char *path)
{
    for (pending_node_t *n = g_pending_head; n != NULL; n = n->next) {
        if (strcmp(n->entry.path, path) == 0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief       将文件加入传输队列
 * @param       path: 文件完整路径
 * @param       size: 文件大小(字节)
 * @param       mtime: 修改时间
 */
static void pending_add(const char *path, uint64_t size, time_t mtime)
{
    /* 防止重复添加 */
    if (pending_contains(path)) {
        return;
    }

    /* 检查容量 */
    if (g_pending_count >= MAX_PENDING) {
        printf("[FILE_MON] Queue full (%d), dropping: %s\n", MAX_PENDING, path);
        return;
    }

    pending_node_t *node = malloc(sizeof(pending_node_t));
    if (node == NULL) {
        ESP_LOGE(TAG, "pending_add: malloc failed");
        return;
    }

    strncpy(node->entry.path, path, sizeof(node->entry.path) - 1);
    node->entry.path[sizeof(node->entry.path) - 1] = '\0';
    node->entry.size        = size;
    node->entry.mtime       = mtime;
    node->entry.transferred = false;
    node->entry.retry_count = 0;
    node->next              = NULL;

    /* 追加到队列尾部 */
    if (g_pending_tail == NULL) {
        g_pending_head = node;
        g_pending_tail = node;
    } else {
        g_pending_tail->next = node;
        g_pending_tail = node;
    }
    g_pending_count++;

    printf("[FILE_MON] New file queued: %s (%llu bytes)\n", path, (unsigned long long)size);
}

/**
 * @brief       递归扫描目录, 检测新文件和已修改文件
 * @param       dir_path: 当前目录路径
 * @param       depth: 当前递归深度
 * @param       new_count: [输出] 新发现文件计数器
 */
static void scan_dir(const char *dir_path, int depth, int *new_count)
{
    if (depth > MAX_SCAN_DEPTH) {
        return;  /* 防止深层递归 */
    }

    DIR *dir = opendir(dir_path);
    if (dir == NULL) {
        /* 目录无法打开(权限/不存在), 静默跳过 */
        return;
    }

    struct dirent *dp;
    while ((dp = readdir(dir)) != NULL) {
        /* 跳过 . 和 .. */
        if (strcmp(dp->d_name, ".") == 0 || strcmp(dp->d_name, "..") == 0) {
            continue;
        }

        /* 构建完整路径 */
        char full_path[768];
        int written = snprintf(full_path, sizeof(full_path), "%s/%s", dir_path, dp->d_name);
        if (written < 0 || (size_t)written >= sizeof(full_path)) {
            continue;  /* 路径过长, 跳过 */
        }

        /* 获取文件详细信息 */
        struct stat st;
        if (stat(full_path, &st) != 0) {
            continue;  /* stat失败, 跳过 */
        }

        if (S_ISDIR(st.st_mode)) {
            /* 递归进入子目录 */
            scan_dir(full_path, depth + 1, new_count);
        } else {
            /* 是文件: 与已知集对比 */
            known_node_t *known = known_find(full_path);

            bool is_new = false;
            if (known == NULL) {
                /* 未知文件 → 新文件 */
                is_new = true;
            } else if (known->size != (uint64_t)st.st_size ||
                       known->mtime != st.st_mtime) {
                /* 已知文件但大小或修改时间变化 → 视为新文件 */
                is_new = true;
                /* 重新加入队列前, 检查是否已在队列中 */
                if (pending_contains(full_path)) {
                    is_new = false;  /* 已在队列, 不用重复加 */
                }
            }

            if (is_new) {
                pending_add(full_path, st.st_size, st.st_mtime);
                known_add_or_update(full_path, st.st_size, st.st_mtime);
                (*new_count)++;
            }
        }
    }

    closedir(dir);
}

/* ======================== 公开API实现 ======================== */

/**
 * @brief       初始化文件监控模块
 */
void file_monitor_init(void)
{
    file_monitor_reset();                    /* 清空所有状态 */
    printf("[FILE_MON] Initialized (max known=%d, max pending=%d)\n", MAX_KNOWN, MAX_PENDING);
}

/**
 * @brief       重置文件监控状态
 */
void file_monitor_reset(void)
{
    /* 释放已知集 */
    known_node_t *kn = g_known_head;
    while (kn != NULL) {
        known_node_t *next = kn->next;
        free(kn);
        kn = next;
    }
    g_known_head  = NULL;
    g_known_count = 0;

    /* 释放传输队列 */
    pending_node_t *pn = g_pending_head;
    while (pn != NULL) {
        pending_node_t *next = pn->next;
        free(pn);
        pn = next;
    }
    g_pending_head  = NULL;
    g_pending_tail  = NULL;
    g_pending_count = 0;

    printf("[FILE_MON] Reset (known and pending cleared)\n");
}

/**
 * @brief       扫描SD卡, 检测新文件
 * @param       base_path: SD卡根路径
 * @retval      新发现文件数量
 */
int file_monitor_scan(const char *base_path)
{
    int new_count = 0;

    if (base_path == NULL || base_path[0] == '\0') {
        return 0;
    }

    /* 递归扫描根目录 */
    scan_dir(base_path, 0, &new_count);

    if (new_count > 0) {
        ESP_LOGI(TAG, "scan: %d new/changed file(s), queue=%d",
                 new_count, g_pending_count);
    }

    return new_count;
}

/**
 * @brief       从传输队列取出下一个待传输文件
 * @retval      文件条目指针(调用者free), NULL=队列为空
 */
file_entry_t *file_monitor_next_pending(void)
{
    if (g_pending_head == NULL) {
        return NULL;
    }

    /* 取出头节点 */
    pending_node_t *node = g_pending_head;
    g_pending_head = node->next;
    if (g_pending_head == NULL) {
        g_pending_tail = NULL;  /* 队列已空 */
    }
    g_pending_count--;

    /* 拷贝条目数据到独立内存块 */
    file_entry_t *entry = malloc(sizeof(file_entry_t));
    if (entry == NULL) {
        free(node);
        return NULL;
    }

    memcpy(entry, &node->entry, sizeof(file_entry_t));
    free(node);

    return entry;
}

/**
 * @brief       标记文件已传输完成
 */
void file_monitor_mark_done(const char *path)
{
    if (path == NULL) return;
    /* 从已知集移除(因为文件即将被删除) */
    known_remove(path);
    printf("[FILE_MON] Done: %s\n", path);
}

/**
 * @brief       标记文件传输失败并增加重试次数
 */
void file_monitor_mark_retry(const char *path)
{
    if (path == NULL) return;

    /* 在队列中查找并更新重试计数, 或者重新加入 */
    /* 简化处理: 如果未达重试上限, 重新加入队列 */
    /* 注意: next_pending已经将条目取出, 此函数重新加入 */

    /* 检查已知集中是否有此条目的记录(获取重试计数) */
    /* 由于我们从队列中取出了条目, 需要重建一个 */
    for (pending_node_t *n = g_pending_head; n != NULL; n = n->next) {
        if (strcmp(n->entry.path, path) == 0) {
            n->entry.retry_count++;
            if (n->entry.retry_count >= FILE_MONITOR_MAX_RETRY) {
                /* 达到上限: 从队列移除, 记录放弃 */
                printf("[FILE_MON] GAVE UP: %s (retries=%d)\n", path, n->entry.retry_count);
                /* 简单标记: 留在已知集, 不再重试 */
                /* 此处不做物理删除, 依靠已知集来防止重新扫描 */
            }
            return;
        }
    }

    /* 未在队列中找到 — 需要重建条目并重新加入 */
    /* 获取文件当前状态 */
    struct stat st;
    if (stat(path, &st) != 0) {
        /* 文件已不存在(可能被删除了), 从已知集移除 */
        known_remove(path);
        return;
    }

    if (S_ISDIR(st.st_mode)) {
        return;  /* 跳过目录 */
    }

    /* 重建条目并重新加入队列 */
    pending_node_t *node = malloc(sizeof(pending_node_t));
    if (node == NULL) return;

    strncpy(node->entry.path, path, sizeof(node->entry.path) - 1);
    node->entry.path[sizeof(node->entry.path) - 1] = '\0';
    node->entry.size        = st.st_size;
    node->entry.mtime       = st.st_mtime;
    node->entry.transferred = false;
    node->entry.retry_count = 1;  /* 第一次重试 */

    if (node->entry.retry_count >= FILE_MONITOR_MAX_RETRY) {
        printf("[FILE_MON] GAVE UP: %s (retries=%d)\n", path, FILE_MONITOR_MAX_RETRY);
        free(node);
        known_remove(path);
        return;
    }

    node->next = NULL;

    /* 追加到队列尾部 */
    if (g_pending_tail == NULL) {
        g_pending_head = node;
        g_pending_tail = node;
    } else {
        g_pending_tail->next = node;
        g_pending_tail = node;
    }
    g_pending_count++;

    printf("[FILE_MON] Retry queued: %s (#%d)\n", path, node->entry.retry_count);
}

/**
 * @brief       获取待传输文件数
 */
int file_monitor_pending_count(void)
{
    return g_pending_count;
}
