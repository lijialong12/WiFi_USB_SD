/**
 ****************************************************************************************************
 * @file        web_server.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-12
 * @brief       Web文件服务器 - 对外API头文件
 *              提供: ①web_server_start() 启动HTTP服务器 →
 *                    ②web_server_stop()  停止HTTP服务器
 *              依赖: esp_http_server (ESP-IDF内置, 已自动链接)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * 功能: 在WiFi AP启动后调用, 提供3个HTTP接口:
 *       GET /         → 返回嵌入式文件管理器网页 (SPA)
 *       GET /api/list → 列出SD卡目录内容 (JSON)
 *       GET /api/file → 预览(YON)或下载(原始流) SD卡文件
 *
 * 使用约束:
 *       - 必须在SD卡已挂载(/sd)且WiFi AP已启动后调用
 *       - HTTP处理在esp_http_server内部任务中运行, 不阻塞调用者
 *       - 网页文件管理器与USB MSC互斥: 两者同时访问SD卡会损坏文件系统
 *       - 端口80, IP地址为AP默认地址 192.168.4.1
 *
 * @note        esp_http_server 已随ESP-IDF自动编译链接, 无需修改CMake依赖
 *              路径验证: 所有文件访问限制在 /sd 目录下, 防止目录穿越攻击
 *
 ****************************************************************************************************
 */

#ifndef __WEB_SERVER_H_
#define __WEB_SERVER_H_

#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t */

/* ======================== 函数声明 ======================== */

/**
 * @brief       启动HTTP Web文件服务器
 *              创建httpd实例(端口80), 注册3个URI处理器:
 *                GET /         根路径 → 返回嵌入HTML SPA网页
 *                GET /api/list 带path参数 → JSON目录列表
 *                GET /api/file 带path+action参数 → 文件预览/下载
 * @param       无
 * @retval      ESP_OK: 服务器启动成功, 可访问 http://192.168.4.1
 * @retval      ESP_FAIL: 启动失败 (端口被占用 / 内存不足)
 * @note        内部创建独立的FreeRTOS任务处理HTTP请求, 不阻塞调用者
 *              调用前确保: SD卡已初始化 + FATFS已挂载到/sd + WiFi AP已启动
 */
esp_err_t web_server_start(void);

/**
 * @brief       停止HTTP Web文件服务器
 *              释放httpd实例资源, 关闭监听端口
 * @param       无
 * @retval      ESP_OK: 成功停止
 * @retval      ESP_FAIL: 服务器未在运行
 * @note        调用后 http://192.168.4.1 不可访问
 *              如需深度睡眠或重启WiFi, 应先停止服务器
 */
esp_err_t web_server_stop(void);

#endif /* __WEB_SERVER_H_ */
