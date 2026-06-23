/**
 ****************************************************************************************************
 * @file        main.c  (从端)
 * @author      ONE
 * @version     V3.6
 * @date        2026-06-22
 * @brief       从端程序 - 后台持续扫描WiFi，扫描期间不影响USB挂载，
 *              待TCP握手成功后才拔出USB进行文件传输
 *              【新增】 LED状态指示灯光系统，配合错误码闪灯，调试一目了然
 *          💡 [慢速呼吸]（1.2秒闪烁一次）

 *              代表状态：从机已上电，正常待机。

 *              正在干嘛：WiFi 正在后台扫描主端热点，USB 正常挂载在电脑上。

 *              💡 [中速闪烁]（0.2秒交替一次）

 *              代表状态：TCP 端口轮询中。

 *              正在干嘛：从机已连上主端 WiFi 热点，正在尝试 TCP 连接主端的 3333 端口。

 *              💡 [快闪 3 次]（0.1秒交替一次）

 *              代表状态：TCP 连接成功。

 *              正在干嘛：已经连上主端 TCP，正在等待主端发送 MASTER_SEND 握手信号。

 *              💡 [快闪 5 次]（0.1秒交替一次）

 *              代表状态：拔插 USB / 挂载内部 SD 卡。

 *              正在干嘛：握手成功，从端正在物理拔出电脑的 U 盘，并在内部挂载 SD 卡准备写文件。传输结束后，也会快闪 5 次插回 U 盘。

 *              💡 [长亮不灭]

 *                  代表状态：正在写文件。

 *               正在干嘛：正在通过 WiFi 接收主端发来的文件数据，并直接写入 SD 卡。

 *                  💡 [快闪 10 次]（常亮/熄灭/常亮极快）

 *               代表状态：传输全部成功。

 *               正在干嘛：led_success_blink() 触发，表示主端发来的文件已经全部完美写入从端 SD 卡。

 *               💡 [闪 N 下 + 停顿]（如 3闪，4闪...）

 *               代表状态：具体错误码报错。

 *               正在干嘛：led_error_blink(N) 触发。根据你原代码的定义，1闪：SD卡初始化失败；3闪：TCP握手失败；4闪~8闪 分别对应不同的文件传输故障。
 * 
 * 
 ****************************************************************************************************
 */

#define USE_USB_MSC  1  // 宏定义：启用 USB 大容量存储（Mass Storage Class）功能

// ======================== 标准库与 ESP-IDF 头文件 ========================
#include <stdio.h>      // 标准输入输出头文件
#include <stdlib.h>     // 标准库头文件，用于 malloc, free, exit 等
#include <string.h>     // 字符串处理头文件，用于 strncpy, strlen, memset 等
#include <stdbool.h>    // 标准布尔类型支持，用于 true/false 判断
#include "freertos/FreeRTOS.h"       // FreeRTOS 实时操作系统核心头文件
#include "freertos/task.h"           // FreeRTOS 任务管理头文件，用于 vTaskDelay 等延时函数
#include "freertos/event_groups.h"   // FreeRTOS 事件组头文件，用于 WiFi 连接状态的多位同步
#include "esp_system.h"              // ESP 系统基础功能头文件
#include "esp_log.h"                 // ESP 日志系统头文件
#include "esp_wifi.h"                // ESP WiFi 驱动头文件
#include "esp_event.h"               // ESP 事件循环头文件
#include "nvs_flash.h"               // 非易失性闪存存储（NVS）头文件
#include "nvs.h"                     // NVS 底层操作头文件
#include "led.h"                     // 用户自定义的 LED 控制头文件（包含 LED(1), LED(0), LED_TOGGLE 等）
#include "sd_card.h"                 // 用户自定义的 SD 卡 SPI/SDMMC 驱动初始化头文件
#if USE_USB_MSC
#include "usb_msc.h"                 // 用户自定义的 TinyUSB MSC 类配置头文件
#include "tusb.h"                    // TinyUSB 协议栈核心头文件，用于 tud_connect / tud_disconnect
#include "tusb_msc_storage.h"        // TinyUSB MSC 存储组件，用于 tinyusb_msc_storage_mount
#endif
#include "lwip/sockets.h"            // lwIP TCP/IP 协议栈的 Socket API
#include "lwip/inet.h"               // 网络字节序转换和 IP 地址转换
#include "sys/stat.h"                // 文件状态头文件，用于获取文件属性（未用，但已习惯包含）
#include "unistd.h"                  // POSIX 操作系统 API，用于 unlink 删除文件

// ======================== 字节序转换宏 ========================
#ifndef be64toh
#define be64toh(x)  __builtin_bswap64(x) // 将 64 位大端序转为主机字节序的宏，用于接收主端发来的文件大小
#endif

// ======================== 日志 TAG 定义 ========================
static const char *TAG = "SLAVE";   // 定义串口日志的过滤标签，终端打印时显示为 [SLAVE]

// ======================== 核心参数配置宏定义 ========================
#define MASTER_SSID             "BOSSCOM_USB_AP" // 要连接的主端 WiFi 热点名称
#define MASTER_PASS             "012345678"      // 主端 WiFi 的密码
#define MASTER_IP               "192.168.4.1"    // 主端开启 AP 时的固定 IP (默认 lwIP 分配的第一个)
#define MASTER_PORT             3333             // 双方约定的 TCP 文件传输端口

#define WIFI_RETRY_INTERVAL_MS  3000             // WiFi 连接失败后的重试间隔（3秒）
#define WIFI_GOT_IP_TIMEOUT_MS  15000            // 发起 WiFi 连接后，等待获取 IP 地址的超时时间（15秒）
#define TCP_POLL_INTERVAL_MS    5000             // 连接上 WiFi 后，若 TCP 连不上主端，等待 5 秒再重试
#define TCP_CONNECT_TIMEOUT_S   5                // TCP connect 连接函数的单次超时时间（秒）
#define TCP_RECV_TIMEOUT_S      30               // 文件接收过程中，单次 recv 接收数据的超时时间（秒）

#define SD_MOUNT_POINT          "/sd"            // SD 卡在 ESP32 内部文件系统中的挂载路径
#define FILE_RECV_BUF_SIZE      4096             // 接收文件数据的缓冲区大小（4KB，与主端匹配）

// ======================== 事件组位标志定义 ========================
#define WIFI_GOT_IP_BIT         BIT0  // 事件组第 0 位：表示 WiFi 已成功连接并获取到了 IP 地址
#define WIFI_DISCONNECTED_BIT   BIT1  // 事件组第 1 位：表示 WiFi 连接已断开

// ======================== 全局状态变量 ========================
static bool               g_sd_ok          = false; // 全局标志：SD 卡是否初始化成功
static bool               g_wifi_mode      = false; // 全局标志：当前是否处于 WiFi 文件传输模式（true：USB已拔出，正在写文件）
static bool               g_wifi_init_done = false; // 全局标志：WiFi 底层网络环境是否已初始化一次
static EventGroupHandle_t g_wifi_eg        = NULL;  // 用于 WiFi 事件同步的 FreeRTOS 事件组句柄

// ======================== 本地函数声明 ========================
static void  switch_to_wifi_mode(void);            // 从 USB 切换为 WiFi 接收模式（拔出 USB，挂载 SD）
static void  switch_to_usb_mode(void);             // 从 WiFi 切换回 USB 模式（卸载 SD，插回 USB）
static int   recv_all(int sock, void *buf, size_t len); // 保证完整接收一定长度 TCP 数据的函数
static bool  do_file_receive_after_handshake(int sock); // 核心接收逻辑：解析协议，写文件，回复确认
static void  wifi_init_sta(void);                  // 初始化 WiFi Station 模式（作为从机）
static bool  wifi_try_connect(void);               // 触发一次 WiFi 连接，并等待连接结果


/* ======================== LED 灯语辅助与错误码系统 ========================
 *
 *  错误码含义（从端专用）：
 *   1闪 —— SD卡初始化失败 / USB MSC初始化失败
 *   3闪 —— TCP握手失败（收到的不是 MASTER_SEND）
 *   4闪 —— 接收文件总数失败
 *   5闪 —— 接收文件名或文件大小失败
 *   6闪 —— recv_all 数据接收中断
 *   7闪 —— fwrite 写SD卡失败
 *   8闪 —— 未收到 DONE 包
 *   快闪10下 —— 传输全部成功
 * =========================================================== */

/* 🔴【灯语辅助宏】：专门用于控制 LED 快速闪烁的简易宏。
 * 原理：执行 times 次，每次把 LED 翻转一次，并延时 period_ms 毫秒。
 * 不会影响任何核心数据逻辑。 */
#define LED_BLINK(times, period_ms) for(int _i=0; _i<(times); _i++){ LED_TOGGLE(); vTaskDelay(pdMS_TO_TICKS((period_ms))); }

/**
 * @brief  LED 错误码闪烁循环提示。
 * @param code: 要报错闪烁的次数（1~8）。
 * 闪烁模式：连续闪 code 次（亮200ms，灭200ms），停顿1000ms，循环5轮。
 * 这样肉眼可以非常明显地区分出是闪了多少次。
 */
static void led_error_blink(int code)
{
    for (int round = 0; round < 5; round++) {              // 循环 5 轮，确保用户能看清
        for (int i = 0; i < code; i++) {                   // 每一轮中，闪烁 code 次
            LED(1); vTaskDelay(pdMS_TO_TICKS(200));        // 亮 200ms
            LED(0); vTaskDelay(pdMS_TO_TICKS(200));        // 灭 200ms
        }
        vTaskDelay(pdMS_TO_TICKS(1000));                   // 每轮闪烁结束后，等待 1 秒
    }
}

/**
 * @brief  LED 传输成功闪烁提示。
 * 节奏：极快的频闪（亮80ms，灭80ms），重复10次。
 */
static void led_success_blink(void)
{
    for (int i = 0; i < 10; i++) {                         // 闪 10 下
        LED(1); vTaskDelay(pdMS_TO_TICKS(80));             // 亮 80ms
        LED(0); vTaskDelay(pdMS_TO_TICKS(80));             // 灭 80ms
    }
}


/* ======================== WiFi 事件处理函数 ======================== */

/**
 * @brief  WiFi 和 IP 事件回调函数。通过操作事件组与主循环通信。
 * @param event_base: 事件基类 (WIFI_EVENT 或 IP_EVENT)。
 * @param event_id: 具体的事件 ID。
 * @param event_data: 携带的事件数据指针。
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {                         // 如果是 Wi-Fi 相关事件
        if (event_id == WIFI_EVENT_STA_START) {              // WiFi 驱动启动完成
            esp_wifi_connect();                             // 触发一次连接（不用等待结果，非阻塞）
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) { // WiFi 连接断开
            xEventGroupClearBits(g_wifi_eg, WIFI_GOT_IP_BIT); // 清除“已获IP”标志位
            xEventGroupSetBits(g_wifi_eg, WIFI_DISCONNECTED_BIT); // 设置“连接断开”标志位，通知主循环
        }
    } else if (event_base == IP_EVENT) {                    // 如果是 IP 地址相关事件
        if (event_id == IP_EVENT_STA_GOT_IP) {               // 成功从路由器（主端 AP）获取到了 IP 地址
            xEventGroupClearBits(g_wifi_eg, WIFI_DISCONNECTED_BIT); // 清除“连接断开”标志位
            xEventGroupSetBits(g_wifi_eg, WIFI_GOT_IP_BIT);       // 设置“已获IP”标志位，通知主循环
        }
    }
}


/* ======================== WiFi STA 初始化 ======================== */

/**
 * @brief  整个生命周期只执行一次的基础 WiFi 网络环境初始化。
 * 注意：如果初始化多次，会导致 esp_netif 内存泄漏。
 */
static void wifi_init_sta(void)
{
    if (g_wifi_init_done) return;                          // 如果已经初始化过，直接跳过
    g_wifi_eg = xEventGroupCreate();                       // 创建 WiFi 事件同步用的 FreeRTOS 事件组
    configASSERT(g_wifi_eg);                               // 断言：如果事件组创建失败则卡死报错

    ESP_ERROR_CHECK(esp_netif_init());                     // 初始化网络协议栈底层
    ESP_ERROR_CHECK(esp_event_loop_create_default());      // 创建默认的事件循环（用于处理 WiFi 事件）
    esp_netif_create_default_wifi_sta();                   // 创建并初始化默认的 Wi-Fi 网卡接口（Station 模式）

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();   // 获取 WiFi 驱动的默认初始化配置参数
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                  // 使用这些参数初始化 Wi-Fi 驱动

    // 注册我们上面写的 wifi_event_handler 回调函数，处理 WIFI 和 IP 事件
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // 配置从机要连接的目标 AP（主端热点）
    wifi_config_t sta_cfg = {0};                           // 清零结构体，避免垃圾值
    strncpy((char *)sta_cfg.sta.ssid, MASTER_SSID, sizeof(sta_cfg.sta.ssid)); // 填入目标 SSID
    strncpy((char *)sta_cfg.sta.password, MASTER_PASS, sizeof(sta_cfg.sta.password)); // 填入目标密码
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK; // 设定只连接 WPA/WPA2 加密的热点
    sta_cfg.sta.failure_retry_cnt  = 1;                   // 设定驱动层的重试计数为 1（失败后主循环控制重试间隔）

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));     // 将 WiFi 芯片设置为 Station 模式
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &sta_cfg)); // 将上面的 AP 配置应用到 Station 接口
    g_wifi_init_done = true;                              // 标记为已初始化
}

/**
 * @brief  执行一次 WiFi 连接尝试，并阻塞等待连接结果。
 * @return true 表示成功获取 IP，false 表示超时或连接断开。
 */
static bool wifi_try_connect(void)
{
    // 清除旧的标志位（包括“已获IP”和“断开”位），准备进行新一轮的连接
    xEventGroupClearBits(g_wifi_eg, WIFI_GOT_IP_BIT | WIFI_DISCONNECTED_BIT);
    esp_wifi_connect(); // 触发 WiFi 连接（非阻塞，结果靠事件组通知）

    // 等待事件组通知：要么 WIFI_GOT_IP_BIT 置位（成功），要么 WIFI_DISCONNECTED_BIT 置位（失败）
    EventBits_t bits = xEventGroupWaitBits(g_wifi_eg, WIFI_GOT_IP_BIT | WIFI_DISCONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(WIFI_GOT_IP_TIMEOUT_MS));
    return (bits & WIFI_GOT_IP_BIT); // 如果 bits 里包含 WIFI_GOT_IP_BIT，说明连上了，返回 true
}

/**
 * @brief  快速检查当前 WiFi 网络是否有效（已连上且有 IP）。
 * @return true 表示连接正常，false 表示断开。
 */
static bool wifi_is_connected(void)
{
    EventBits_t bits = xEventGroupGetBits(g_wifi_eg); // 获取事件组当前的位状态
    // 必须同时满足：有 IP 标志位，且没有断开标志位
    return (bits & WIFI_GOT_IP_BIT) && !(bits & WIFI_DISCONNECTED_BIT);
}


/* ======================== 模式切换 ======================== */

/**
 * @brief  进入文件接收模式：断开 USB，并将 SD 卡的控制权转交给内部 FatFS 文件系统。
 */
static void switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return;                             // 如果已经在 WiFi 模式下，直接跳过
    printf("[模式] 握手成功，拔出USB，挂载SD到FatFS...\n");

    /* 🔴【灯语逻辑】：拔掉USB，快闪5次提示操作 */
    LED_BLINK(5, 100);

    tud_disconnect();                                    // 物理断开 USB，此时电脑会听到“U盘拔出”
    vTaskDelay(pdMS_TO_TICKS(500));                      // 等待 500ms，让 PC 彻底感知 USB 已移除

    tinyusb_msc_storage_unmount();                       // 从 TinyUSB 的 MSC 存储设备中卸载 SD 卡
    vTaskDelay(pdMS_TO_TICKS(100));                      // 等待卸载完成

    // 将 SD 卡在 ESP32 内部重新挂载为普通的 FatFS 文件系统（供 fopen、fwrite 使用）
    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {                                 // 如果内部挂载失败
        printf("[模式] FatFS挂载失败: %s\n", esp_err_to_name(ret));
        tud_connect();                                   // 失败则重新连接 USB，防止死机
        return;
    }

    g_wifi_mode = true;                                  // 标记当前处于 WiFi 文件接收模式
    printf("[模式] SD卡已挂载到FatFS，可以写文件\n");
#endif
}

/**
 * @brief  退出文件接收模式：卸载内部 FatFS，插回 USB 让电脑恢复识别 U 盘。
 */
static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    if (!g_wifi_mode) return;                            // 如果不在 WiFi 模式下，直接跳过
    printf("[模式] 传输完毕，把SD卡还给TinyUSB...\n");

    tinyusb_msc_storage_unmount();                       // 从内部 FatFS 文件系统中卸载 SD 卡
    vTaskDelay(pdMS_TO_TICKS(100));                      // 等待卸载完成

    g_wifi_mode = false;                                 // 清除 WiFi 模式标志
    tud_connect();                                       // 物理连接 USB，电脑会听到“U盘插入”
    vTaskDelay(pdMS_TO_TICKS(1500));                     // 等待 1.5 秒，让 Windows 完成驱动枚举

    /* 🔴【灯语逻辑】：插回USB，快闪5次提示 */
    LED_BLINK(5, 100);

    printf("[模式] USB已恢复连接\n");
#endif
}


/* ======================== TCP 数据辅助函数 ======================== */

/**
 * @brief  循环读取 Socket，直到读到指定长度的数据或连接断开。
 * @param sock: 已连接的 TCP Socket 句柄。
 * @param buf: 接收数据的内存缓存区。
 * @param len: 要求读取的数据长度（字节）。
 * @return 成功返回 0，失败或断开返回 -1。
 */
static int recv_all(int sock, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;                         // 将缓冲区指针强转为字节指针，方便按字节偏移
    while (len > 0) {                                    // 只要还有数据没读满
        int r = recv(sock, p, len, 0);                   // 调用 lwIP recv 接收数据，r 为实际接收字节数
        if (r <= 0) return -1;                           // 如果 r<=0 说明对端断开了或者超时
        p += r;                                          // 接收缓冲区指针前移 r 字节
        len -= r;                                        // 剩余需要的字节数减少 r
    }
    return 0;                                            // 成功读完指定长度
}


/* ======================== 文件接收核心（握手后） ======================== */

/**
 * @brief  接收主端发来的文件数据，并按照协议解析写入 SD 卡。
 * @param sock: 已经建立连接的 TCP Socket。
 * @return 成功返回 true，失败返回 false。
 */
static bool do_file_receive_after_handshake(int sock)
{
    char *recv_buf = NULL;                               // 用于接收文件数据的内存缓冲区指针
    FILE *fp       = NULL;                               // 用于写入 SD 卡的文件指针句柄
    int   err_code = 0;                                 // 错误代码，0 表示成功

    /* 🔴【灯语逻辑】：正式开始核心数据写入SD卡，LED长亮代表正在往SD卡写文件 */
    LED(1);

    /* --- 1. 接收文件总数量 --- */
    uint32_t file_count = 0;
    {
        uint32_t count_net = 0;                          // 存放网络字节序的计数器
        if (recv_all(sock, &count_net, sizeof(count_net)) != 0) { // 接收 4 字节
            printf("[传输] 接收文件总数失败\n");
            err_code = 4; goto recv_exit;               // 报错：4闪
        }
        file_count = ntohl(count_net);                  // 将网络字节序转换为主机字节序
        printf("[传输] 期望接收 %lu 个文件\n", (unsigned long)file_count);
        if (file_count == 0) { goto recv_exit; }        // 如果提示 0 个文件，直接正常退出
    }

    /* --- 2. 申请数据缓冲区 --- */
    recv_buf = (char *)malloc(FILE_RECV_BUF_SIZE);       // 在堆内存中申请 4KB 的内存空间
    if (!recv_buf) { printf("[传输] malloc失败\n"); err_code = 4; goto recv_exit; }

    /* --- 3. 循环接收每一个文件 --- */
    for (uint32_t i = 0; i < file_count; i++) {

        /* 接收文件名长度（2字节，大端序） */
        uint16_t name_len_net = 0;
        if (recv_all(sock, &name_len_net, sizeof(name_len_net)) != 0) {
            printf("[传输] 接收文件名长度失败 [%lu]\n", (unsigned long)i);
            err_code = 5; goto recv_exit;               // 报错：5闪
        }
        uint16_t name_len = ntohs(name_len_net);         // 转主机字节序
        if (name_len == 0 || name_len > 255) {           // 文件名长度合法性校验
            printf("[传输] 文件名长度非法=%u\n", name_len);
            err_code = 5; goto recv_exit;               // 报错：5闪
        }

        /* 接收文件名（变长字节） */
        char file_name[256] = {0};                       // 声明 256 字节数组存放文件名，全初始化为 0
        if (recv_all(sock, file_name, name_len) != 0) {
            printf("[传输] 接收文件名失败\n");
            err_code = 5; goto recv_exit;               // 报错：5闪
        }
        file_name[name_len] = '\0';                     // 手动添加字符串结束符

        /* 接收文件大小（8字节，大端序） */
        uint64_t file_size_be = 0;
        if (recv_all(sock, &file_size_be, sizeof(file_size_be)) != 0) {
            printf("[传输] 接收文件大小失败: %s\n", file_name);
            err_code = 5; goto recv_exit;               // 报错：5闪
        }
        uint64_t file_size = be64toh(file_size_be);      // 转主机字节序（大端转小端）
        printf("[传输] [%lu/%lu] %s (%llu字节)\n",
               (unsigned long)(i+1), (unsigned long)file_count,
               file_name, (unsigned long long)file_size);

        /* 创建/覆盖本地目标文件 */
        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, file_name); // 拼接 SD 卡绝对路径
        unlink(path);                                     // 如果该文件存在，先删除（避免同名冲突）

        fp = fopen(path, "wb");                           // 以“只写二进制”模式打开文件
        if (!fp) {                                       // 如果打开/创建失败（可能是 SD 卡已满）
            /* 无法创建：必须丢弃主端发过来的剩余数据，否则会导致 TCP 协议错乱 */
            printf("[传输] 无法创建 %s，丢弃数据\n", path);
            uint64_t discard = file_size;
            while (discard > 0) {                         // 循环读取并直接丢弃，直到读完规定的尺寸
                size_t chunk = (discard > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)discard;
                if (recv_all(sock, recv_buf, chunk) != 0) { err_code = 6; goto recv_exit; }
                discard -= chunk;
            }
            continue;                                    // 处理下一个文件
        }

        /* 接收并写入文件内容 */
        uint64_t remain = file_size;
        while (remain > 0) {
            size_t chunk = (remain > FILE_RECV_BUF_SIZE) ? FILE_RECV_BUF_SIZE : (size_t)remain;
            if (recv_all(sock, recv_buf, chunk) != 0) {   // 接收 4KB 数据到缓冲区
                printf("[传输] 接收数据中断: %s\n", file_name);
                fclose(fp); fp = NULL;
                err_code = 6; goto recv_exit;            // 报错：6闪
            }
            if (fwrite(recv_buf, 1, chunk, fp) != chunk) { // 将缓冲区数据写入 SD 卡
                printf("[传输] 写SD失败: %s\n", file_name);
                fclose(fp); fp = NULL;
                err_code = 7; goto recv_exit;            // 报错：7闪
            }
            remain -= chunk;
        }
        fclose(fp); fp = NULL;                           // 一个文件写完，关闭文件句柄
        printf("[传输] 已保存: %s\n", path);
    }

    /* 🔴【灯语逻辑】：所有文件数据写完，熄灭长亮状态 */
    LED(0);

    /* --- 4. 等待主端发送结束指令 "DONE" --- */
    {
        char done_buf[8] = {0};
        if (recv_all(sock, done_buf, 4) != 0) {          // 接收 4 字节
            printf("[传输] 接收DONE失败\n");
            err_code = 8; goto recv_exit;               // 报错：8闪
        }
        done_buf[4] = '\0';
        if (strncmp(done_buf, "DONE", 4) != 0) {         // 校验字符串是否为 "DONE"
            printf("[传输] DONE包内容错误: %s\n", done_buf);
            err_code = 8; goto recv_exit;               // 报错：8闪
        }
        send(sock, "OK", 2, 0);                         // 向主端回复 "OK"，表示接收成功
        printf("[传输] 完成，OK已发送\n");
        char drain[16]; recv(sock, drain, sizeof(drain), 0); // 读取主端发送的最后一个字节，等待主端关闭连接
    }

recv_exit:                                               // 异常退出和清理标签
    if (fp)       fclose(fp);                           // 如果有文件句柄没关，强制关闭
    if (recv_buf) free(recv_buf);                       // 释放动态申请的 4KB 缓冲区内存

    if (err_code == 0) {
        led_success_blink();                            // 错误码为0，说明完全成功，快闪10下庆祝
        return true;
    } else {
        LED(0);                                         // 发生错误时，强制熄灭 LED 以防误导
        led_error_blink(err_code);                      // 闪烁 error_code 对应的次数报错
        return false;
    }
}


/* ======================== 主函数入口 ======================== */
void app_main(void)
{
    // ========== 1. NVS 闪存分区初始化 ==========
    esp_err_t ret = nvs_flash_init();                    // 初始化非易失性存储分区（用于保存 WiFi 配置等）
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();             // 如果分区损坏或版本不对，则擦除并重建 NVS
    }

    led_init();                                          // 初始化板载 LED 引脚
    LED(1);                                              // 短暂亮起，证明程序开始运行

    // ========== 2. SD 卡与 USB MSC 初始化 ==========
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card);           // 初始化 SD 卡驱动并挂载检测
    g_sd_ok = (sd_ret == ESP_OK && sd_card != NULL);     // 记录 SD 卡是否初始化成功

    if (g_sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        esp_err_t msc_ret = usb_msc_init(sd_card);       // 将 SD 卡注册到 TinyUSB 的 MSC 存储类中
        if (msc_ret != ESP_OK) {
            printf("[主函数] USB MSC 初始化失败!\n");
            led_error_blink(1);                          /* 1闪 = 初始化失败 */
        } else {
            tud_connect();                               // 初始状态：USB 连接（电脑能看到 U 盘）
            vTaskDelay(pdMS_TO_TICKS(1000));             // 等待 1 秒让 Windows 加载驱动
            printf("[主函数] USB MSC连接正常，U盘已挂载\n");
        }
#endif
    } else {
        printf("[主函数] SD卡失败\n");
        led_error_blink(1);                              /* 1闪 = SD卡失败 */
    }

    // ========== 3. WiFi 初始化与启动 ==========
    wifi_init_sta();                                     // 注册事件回调、配置参数（不发起连接）
    esp_wifi_start();                                    // 启动 WiFi 驱动（触发 WIFI_EVENT_STA_START 事件，进而自动发起连接）
    vTaskDelay(pdMS_TO_TICKS(500));                      // 延时 500ms 让 WiFi 系统初始化稳定
    setvbuf(stdout, NULL, _IONBF, 0);                    // 关闭 printf 的缓冲区，让串口日志立刻输出（对于调试至关重要）
    LED(0);                                              // 初始完毕后熄灭 LED，准备进入待机闪烁状态

    printf("\n========== 从机 V3.6 (扫描不挂载模式) ==========\n");
    printf("USB始终保持连接，后台持续扫描主端热点...\n");

    int trans_cnt = 0;                                   // 成功传输次数的统计
    int log_cnt   = 0;                                   // 用于控制日志打印频率的计数器

    // ========== 4. 主循环状态机 ==========
    while (1) {
        /* 🔴【灯语逻辑】：基础待机状态，慢速呼吸（1.2秒交替一次）。当后面状态发生时，会替换此闪烁 */
        LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(600));                  // 配合上一步，达成 1.2 秒一个循环的呼吸频率

        /* --- 阶段1：WiFi 后台自动扫描与连接 ---
           如果 WiFi 没连上，代码不会往下走，防止 TCP 没网络盲连 */
        if (!wifi_is_connected()) {
            if (++log_cnt >= 3) { log_cnt = 0; printf("[WiFi] 扫描中，U盘仍挂载...\n"); } // 每3秒打印一次，避免刷屏
            wifi_try_connect();                           // 尝试发起一次 WiFi 连接（内部非阻塞，带超时）
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS)); // 无论连接是否成功，都等待 3 秒再重试
            continue;                                    // 直接跳入下一次循环
        }
        log_cnt = 0;                                     // 一旦连接上了，重置打印计数器

        /* --- 阶段2：TCP 端口轮询 ---
           WiFi 连上了，就开始尝试 TCP 连主端 3333 端口 */
        int sock = socket(AF_INET, SOCK_STREAM, 0);       // 创建一个 TCP Socket
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS)); continue; }

        struct timeval tv = { .tv_sec = TCP_CONNECT_TIMEOUT_S, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); // 设置接收超时
        setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)); // 设置发送超时

        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;                        // IPv4 协议族
        addr.sin_port   = htons(MASTER_PORT);             // 目标端口 3333
        inet_pton(AF_INET, MASTER_IP, &addr.sin_addr);    // 将字符串 IP "192.168.4.1" 转为网络字节序

        // 尝试连接主端 TCP 服务端
        if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
            close(sock);                                  // 连接失败，关闭 socket
            /* 🔴【灯语逻辑】：TCP 端口轮询中，中速闪烁 10 次（代表正在尝试建立连接） */
            for(int i=0; i<10; i++) { LED_TOGGLE(); vTaskDelay(pdMS_TO_TICKS(200)); }
            vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS)); // 等待 5 秒再试
            continue;
        }

        printf("[TCP] 已连接到主机！准备握手...\n");
        /* 🔴【灯语逻辑】：TCP连接成功，快闪3次提示，准备握手 */
        LED_BLINK(3, 100);

        /* --- 阶段3：握手协议 --- */
        {
            struct timeval htv = { .tv_sec = 10, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &htv, sizeof(htv)); // 握手阶段单独设置 10 秒超时

            char greeting[16] = {0};
            // 主端会发来 11 字节的 "MASTER_SEND"
            int n = recv(sock, greeting, 11, MSG_WAITALL);
            if (n != 11 || strncmp(greeting, "MASTER_SEND", 11) != 0) { // 校验握手内容
                printf("[握手] 握手失败，U盘保持连接\n");
                led_error_blink(3);                        /* 3闪 = 握手失败 */
                close(sock);
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS));
                continue;
            }
            printf("[握手] 收到 MASTER_SEND，立刻准备拔出U盘！\n");

            switch_to_wifi_mode();                         // 拔出 USB，挂载内部 FatFS

            struct timeval ttv = { .tv_sec = TCP_RECV_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &ttv, sizeof(ttv)); // 恢复文件接收超时
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &ttv, sizeof(ttv));

            if (send(sock, "READY", 5, 0) != 5) {          // 告诉主端：我已经拔好 USB 准备好接收了
                printf("[传输] 发READY失败\n");
                close(sock);
                switch_to_usb_mode();                      // 如果发送失败，紧急恢复 USB
                vTaskDelay(pdMS_TO_TICKS(TCP_POLL_INTERVAL_MS));
                continue;
            }
        }

        /* --- 阶段4：核心文件接收 --- 
           此函数内部包含了长亮写文件的灯语逻辑 */
        bool ok = do_file_receive_after_handshake(sock);
        close(sock);                                       // 关闭 TCP 连接

        switch_to_usb_mode();                              // 接收完毕后，插回 USB

        if (ok) {
            printf("[传输] 文件接收成功，已完成%d次\n", ++trans_cnt);
            vTaskDelay(pdMS_TO_TICKS(3000));               // 成功后，等待 3 秒再扫描下一次
        } else {
            printf("[传输] 传输失败，冷却10秒...\n");
            vTaskDelay(pdMS_TO_TICKS(10000));              // 失败后，冷却 10 秒防止电脑鬼畜反复弹窗
        }
    }
}