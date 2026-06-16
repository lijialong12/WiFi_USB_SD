/**
 ****************************************************************************************************
 * @file        web_server.h
 * @author      ONE
 * @version     V1.1
 * @date        2026-06-15
 * @brief       Web文件服务器 - 对外API头文件
 *              提供: ①web_server_start() 启动HTTP服务器 →
 *                    ②web_server_stop()  停止HTTP服务器 →
 *                    ③web_server_get_slave_ip() 获取已注册从机IP
 *              依赖: esp_http_server (ESP-IDF内置, 已自动链接)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板 (DEVICE_ROLE=0 主机端)
 * 功能: 在WiFi AP启动后调用, 提供以下HTTP接口:
 *       GET /              → 返回嵌入式文件管理器网页 (SPA)
 *       GET /api/list      → 列出SD卡目录内容 (JSON)
 *       GET /api/file      → 预览/下载 SD卡文件
 *       GET /api/ping      → 健康检查
 *       GET/POST /api/wifi-config → WiFi配置
 *       POST /api/register → 从机注册 (新增)
 *       POST /api/delete   → 删除文件 (新增)
 *
 * 使用约束:
 *       - 必须在SD卡已挂载(/SD)且WiFi AP已启动后调用
 *       - 网页文件管理器与USB MSC互斥: 两者同时访问SD卡会损坏文件系统
 *
 * @note        路径验证: 所有文件访问限制在 /SD 目录下, 防止目录穿越攻击
 *
 ****************************************************************************************************
 */

#ifndef __WEB_SERVER_H_
#define __WEB_SERVER_H_

#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t */
#include <stdbool.h>                         /* bool类型 */

/* ======================== 函数声明 ======================== */

/**
 * @brief       启动HTTP Web文件服务器
 *              创建httpd实例(端口80), 注册7个URI处理器
 * @param       无
 * @retval      ESP_OK: 服务器启动成功
 * @retval      ESP_FAIL: 启动失败
 */
esp_err_t web_server_start(void);

/**
 * @brief       停止HTTP Web文件服务器
 * @param       无
 * @retval      ESP_OK: 成功停止
 * @retval      ESP_FAIL: 服务器未在运行
 */
esp_err_t web_server_stop(void);

/**
 * @brief       获取已注册从机的IP地址
 * @param       ip_buf: 输出缓冲区(至少16字节)
 * @param       buf_size: 缓冲区大小
 * @retval      true: 有从机已注册, ip_buf包含IP字符串
 * @retval      false: 无从机注册
 */
bool web_server_get_slave_ip(char *ip_buf, size_t buf_size);

/**
 * @brief       清除从机注册状态
 *              当从机断开WiFi时调用, 清除记录的IP
 */
void web_server_clear_slave(void);

#endif /* __WEB_SERVER_H_ */
