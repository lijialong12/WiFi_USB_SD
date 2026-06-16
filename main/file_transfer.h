/**
 ****************************************************************************************************
 * @file        file_transfer.h
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-15
 * @brief       文件传输客户端 - 主机端HTTP推送文件到从机API
 *              使用esp_http_client以POST方式推送原始文件内容到从机
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 (DEVICE_ROLE=0 主机端)
 * 依赖: esp_http_client (ESP-IDF框架自带, 已链接)
 * 协议: POST http://<slave_ip>/api/upload?name=<filename>
 *       Headers: Content-Type=application/octet-stream, X-File-Size=<bytes>
 *       Body: 原始文件内容(分块发送)
 *
 ****************************************************************************************************
 */

#ifndef __FILE_TRANSFER_H_
#define __FILE_TRANSFER_H_

#include "esp_err.h"                         /* ESP-IDF错误类型: esp_err_t */

/**
 * @brief       推送文件到从机设备
 *              打开本地文件, 通过HTTP POST发送到从机, 流式传输支持大文件
 * @param       slave_ip: 从机IP地址(如 "192.168.3.2")
 * @param       filepath: 本地文件完整路径(如 "/SD/test.txt")
 * @retval      ESP_OK: 传输成功, 从机已保存文件
 * @retval      ESP_FAIL: 传输失败(网络错误/从机拒绝/超时)
 * @note        内部使用esp_http_client, 阻塞直到传输完成或超时
 *              每2KB发送一个chunk并短暂让出CPU, 防止看门狗超时
 */
esp_err_t file_transfer_push(const char *slave_ip, const char *filepath);

/**
 * @brief       从文件路径中提取纯文件名(最后一段)
 * @param       filepath: 文件完整路径
 * @retval      文件名指针(指向路径中最后一个'/'之后的部分)
 * @note        返回的指针指向filepath内部, 不需要释放
 */
const char *file_transfer_basename(const char *filepath);

#endif /* __FILE_TRANSFER_H_ */
