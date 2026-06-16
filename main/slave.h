/**
 ****************************************************************************************************
 * @file        slave.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-15
 * @brief       从机模块 - WiFi STA连接 + 文件上传HTTP服务器API
 *              提供: ①从机WiFi STA初始化/连接 → ②文件接收HTTP服务器 →
 *                    ③主机注册 → ④连接状态监控
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 (DEVICE_ROLE=1 从机端)
 * 功能: 连接主机WiFi AP → 启动HTTP接收服务器 → 向主机注册 →
 *       接收文件并保存到本地SD卡
 * WiFi: STA模式, 连接 SSID=BOSSCOM_USB_AP, 密码=012345678
 *
 ****************************************************************************************************
 */

#ifndef __SLAVE_H_
#define __SLAVE_H_

#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t */
#include <stdbool.h>                         /* bool类型 */

/* ======================== 函数声明 ======================== */

/**
 * @brief       初始化从机(WiFi STA + HTTP服务器)
 *              步骤: ①创建STA netif → ②初始化WiFi驱动 →
 *                    ③配置STA SSID/密码 → ④启动WiFi连接 →
 *                    ⑤等待获取IP → ⑥启动文件接收HTTP服务器
 * @param       无
 * @retval      ESP_OK: 初始化成功, 已连接到主机AP
 * @retval      ESP_FAIL: 初始化失败
 */
esp_err_t slave_init(void);

/**
 * @brief       从机主循环tick (在app_main循环中调用)
 *              功能: ①检查WiFi连接状态 → ②断线自动重连 →
 *                    ③向主机重新注册 → ④LED模式管理
 * @param       无
 * @retval      无
 */
void slave_loop_tick(void);

/**
 * @brief       获取从机当前WiFi连接状态
 * @retval      true=已连接, false=未连接
 */
bool slave_is_connected(void);

/**
 * @brief       获取已接收文件计数
 * @retval      文件数量
 */
int slave_get_files_received(void);

#endif /* __SLAVE_H_ */
