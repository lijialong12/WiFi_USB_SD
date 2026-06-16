/**
 ****************************************************************************************************
 * @file        file_monitor.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-15
 * @brief       文件变化检测模块 - 主机端文件监控API
 *              功能: ①递归扫描SD卡目录 → ②检测新增/修改的文件 →
 *                    ③维护传输队列 → ④标记完成/重试
 *              FATFS不支持inotify, 采用周期性stat对比实现
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板 (DEVICE_ROLE=0 主机端)
 * 使用场景: PC通过USB MSC写入文件后, 主机在WiFi模式下检测并推送
 * 内存: 已知集最多保存128个文件记录, 传输队列最多64个待传文件
 *
 ****************************************************************************************************
 */

#ifndef __FILE_MONITOR_H_
#define __FILE_MONITOR_H_

#include <stdint.h>                          /* 标准整数类型: uint64_t */
#include <stdbool.h>                         /* bool类型 */
#include <time.h>                            /* time_t */
#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t */

/* ======================== 文件条目结构体 ======================== */
/**
 * @brief 文件条目信息 (用于已知集和传输队列)
 */
typedef struct {
    char     path[512];        /* 文件完整路径 (如 /SD/subdir/file.txt) */
    uint64_t size;             /* 文件大小(字节), 目录为0 */
    time_t   mtime;            /* 修改时间(unix时间戳) */
    bool     transferred;      /* true=已传输完成, false=待传输 */
    int      retry_count;      /* 已重试次数 (0=未重试, 达到MAX_RETRY后跳过) */
} file_entry_t;

#define FILE_MONITOR_MAX_RETRY  3     /* 最大重试次数 */

/* ======================== 函数声明 ======================== */

/**
 * @brief       初始化文件监控模块
 *              清空已知集和传输队列, 分配内存
 */
void file_monitor_init(void);

/**
 * @brief       重置文件监控状态
 *              清空已知集和传输队列, 通常在从WiFi模式切回USB模式时调用
 */
void file_monitor_reset(void);

/**
 * @brief       扫描SD卡根目录(递归), 检测新文件和修改过的文件
 *              对比已知集: 如果文件是新出现的, 加入传输队列
 *              如果文件大小/mtime变化, 重新加入传输队列
 * @param       base_path: SD卡根路径, 如 "/SD"
 * @retval      本次扫描发现的新文件数量 (0=无变化)
 * @note        建议每2秒调用一次, 在WiFi模式下且从机已注册时
 */
int file_monitor_scan(const char *base_path);

/**
 * @brief       从传输队列取出下一个待传输文件
 *              取出后从队列中移除, 调用者负责释放返回的指针
 * @retval      文件条目指针 (需调用者free), NULL=队列为空
 */
file_entry_t *file_monitor_next_pending(void);

/**
 * @brief       标记文件已传输完成
 *              从已知集中移除(因为文件会被删除), 确保不会再次被检测到
 * @param       path: 文件完整路径
 */
void file_monitor_mark_done(const char *path);

/**
 * @brief       标记文件传输失败并增加重试次数
 *              如果重试次数达上限(MAX_RETRY), 从队列中移除并记录警告
 *              否则放回队列等待下次重试
 * @param       path: 文件完整路径
 */
void file_monitor_mark_retry(const char *path);

/**
 * @brief       获取传输队列中待处理文件数量
 * @retval      待传输文件数
 */
int file_monitor_pending_count(void);

#endif /* __FILE_MONITOR_H_ */
