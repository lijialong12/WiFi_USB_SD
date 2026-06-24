/**
 ****************************************************************************************************
 * @file        main.c  (主端)
 * @author      ONE
 * @version     V3.6
 * @date        2026-06-22
 * @brief       主端程序 - USB MSC + GPIO0按键触发 + WiFi AP按需开启
 *
 *              核心逻辑说明：
 *              1. 上电后处于USB模式，电脑能识别到U盘（TinyUSB挂载SD卡）。
 *              2. 用户按下 GPIO0 按键，主端物理断开USB（拔出U盘），并在内部挂载SD卡。
 *              3. 检查SD卡内是否有文件，如果有则开启WiFi热点。
 *              4. 等待从机通过WiFi连接，建立TCP连接后，将SD卡文件通过WiFi传输给从机。
 *              5. 传输完毕且收到从机ACK确认，主端删除SD卡原文件，重新插回USB（恢复电脑识别）。
 *              【新增】加入完整的LED状态指示系统，方便USB断开时通过灯语得知运行状态。
 * 
*                * 💡 [慢速呼吸]（1.2秒闪烁一次）
* 
*               代表状态：系统正常运行，处于待机状态。
* 
*               正在干嘛：U盘已正常连接电脑，主端正在等待你按下 GPIO0 按键。
* 
*                💡 [快闪 3 次]（0.1秒亮、0.1秒灭）
* 
*               代表状态：按键已被系统识别。
* 
*                正在干嘛：你按下了按键，主端正在检查 SD 卡里面是否存有文件，准备开始传输流程。
* 
*               💡 [快闪 5 次]（0.1秒亮、0.1秒灭）
* 
*               代表状态：正在断开 USB（拔出U盘）。
* 
*               正在干嘛：即将从电脑上拔出U盘，并在主端单片机内部重新挂载 SD 卡。
* 
*               💡 [长亮不灭]
* 
*                代表状态：核心数据传输中。
* 
*               正在干嘛：这是最主要的状态！从机已经连接成功，主端正在从 SD 卡读取数据，并通过 WiFi 发送给从机。这个灯亮的时间越长，说明传输的文件越大。
* 
*               💡 [灯熄灭]
* 
*                代表状态：数据发送完毕。
* 
*               正在干嘛：所有的文件都已经通过 WiFi 成功发送给了从机，系统准备进入收尾阶段。
* 
*                 💡 [快闪 5 次]（0.1秒亮、0.1秒灭） (出现第二次)

*                代表状态：正在删除旧文件。

*               正在干嘛：收到了从机“接收成功”的确认，现在主端正在物理删除 SD 卡上的原文件，腾出空间。

*               💡 [快闪 5 次]（0.1秒亮、0.1秒灭） (出现第三次)

*                代表状态：正在恢复 USB 连接。

*               正在干嘛：关闭了 WiFi 热点，正重新把 U 盘插回给电脑。

*               💡 [长灭 3 秒]（完全熄灭 3 秒）

*                代表状态：严重错误报警。

*               正在干嘛：SD 卡挂载失败、SD 卡里面没有文件、或者本地读取文件损坏导致终止。

*                💡 [恢复慢速呼吸]

*                代表状态：一轮传输闭环完成。

*                正在干嘛：回到最初的待机状态，U盘已重新插好，你可以再次按下按键，进行下一次传输。
 ****************************************************************************************************
 */

// ======================== 头文件包含 ========================
#define USE_USB_MSC  1 // 宏定义：启用 USB 大容量存储（Mass Storage Class）功能

#include <stdio.h>     // 标准输入输出头文件，用于 printf 等
#include <stdlib.h>    // 标准库头文件，用于 malloc, free 等
#include <string.h>    // 字符串处理头文件，用于 strncpy, strlen, memset 等
#include <stdbool.h>   // 标准布尔类型头文件，支持 true/false
#include "freertos/FreeRTOS.h"       // ESP-IDF 的 FreeRTOS 实时操作系统核心头文件
#include "freertos/task.h"           // FreeRTOS 任务管理头文件，用于 vTaskDelay 等
#include "nvs_flash.h"               // 非易失性存储（NVS）头文件，保存 WiFi 配置等
#include "nvs.h"                     // NVS 操作的具体函数
#include "esp_wifi.h"                // ESP 的 WiFi 驱动头文件，用于开启热点等
#include "esp_event.h"               // ESP 的事件循环机制头文件，用于处理 Wi-Fi 事件
#include "driver/gpio.h"             // GPIO 驱动头文件，用于控制按键引脚
#include "led.h"                     // 用户自定义的 LED 控制头文件（包含 LED(1), LED_TOGGLE 等）
#include "sd_card.h"                 // 用户自定义的 SD 卡初始化头文件
#if USE_USB_MSC
#include "usb_msc.h"                 // 用户自定义的 USB MSC 初始化头文件
#include "tusb.h"                    // TinyUSB 协议栈头文件，用于拔插 USB 设备
#include "tusb_msc_storage.h"        // TinyUSB 存储组件头文件，用于挂载/卸载 SD
#endif
#include "lwip/sockets.h"            // lwIP 轻量级 TCP/IP 协议栈的 Socket API，用于 TCP 网络通讯
#include "lwip/inet.h"               // lwIP 的网络字节序和 IP 地址转换功能
#include "dirent.h"                  // 目录操作头文件，用于遍历 SD 卡中的文件
#include "sys/stat.h"                // 文件状态头文件，用于获取文件大小等属性
#include "unistd.h"                  // 标准符号常量及定义，用于 unlink（删除文件）

// ======================== 字节序转换宏 ========================
#ifndef htobe64
#define htobe64(x)  __builtin_bswap64(x) // 定义将 64 位主机字节序转为网络字节序（大端）的宏，用于传输文件大小时网络对齐
#endif

// ======================== 日志 TAG 定义 ========================

// ======================== 核心参数配置宏定义 ========================
#define WIFI_SSID       "BOSSCOM_USB_AP" // 主端开启的热点 SSID（名字），从机必须以此名称连接
#define WIFI_PASS       "012345678"      // 热点密码，设定为 9 位数字
#define MAX_STA_CONN    5               // 设置当前热点允许同时连接的从机最大数量

#define TRANSFER_PORT           3333    // 文件传输使用的 TCP 端口号，主从机需一致
#define ACCEPT_TIMEOUT_S        60      // 【关键】主端开启 TCP 监听后，等待从机连接的超时时间（秒）
#define TRANSFER_ACK_TIMEOUT    60      // 【关键】TCP 发送（send）和接收（recv）的单次超时时间（秒），防止大文件传输卡死
#define SD_MOUNT_POINT          "/sd"   // SD 卡在 ESP32 内部文件系统中的挂载路径
#define MAX_FILES               100     // 单次传输限制的最大文件数量，防止数组越界
#define FILE_SEND_BUF_SIZE      4096    // 文件读取和发送的内存块缓冲区大小（4KB）

#define BUTTON_GPIO             GPIO_NUM_0  // 触发传输启动的物理按键引脚（GPIO0，通常是开发板上的 BOOT 键）
#define BUTTON_DEBOUNCE_MS      200         // 按键消抖延时（毫秒），防止机械按键抖动导致多次触发

// 🔴【灯语辅助宏】：专门用于控制 LED 快速闪烁的宏。
// 参数 times: 闪烁次数, period_ms: 单次亮/灭的持续时间(毫秒)。
// 本质就是在一个 for 循环里持续调用 LED 翻转和任务延时。
#define LED_BLINK(times, period_ms) for(int _i=0; _i<(times); _i++){ LED_TOGGLE(); vTaskDelay(pdMS_TO_TICKS((period_ms))); }

// ======================== 全局状态变量 ========================
static bool g_wifi_mode = false;          // 当前系统模式标志位：true 表示已在 WiFi 模式下（USB已拔出），false 表示 USB 模式
static volatile bool g_button_pressed = false; // 按键中断触发标志位：volatile 防止编译器优化，主循环检测此变量执行操作

// ======================== 函数声明 ========================
static bool switch_to_wifi_mode(void);      // 切换到 WiFi 发送模式（断开 USB，挂载 SD）
static void switch_to_usb_mode(void);       // 恢复到 USB 模式（插回 USB）
static int  send_all(int sock, const void *buf, size_t len); // 安全的 TCP 发送函数，确保完整发送指定长度的数据
static void handle_transfer(int client_sock); // 核心文件传输处理函数，负责握手、发文件名、发数据、等 ACK、删文件
static void wifi_ap_start(void);            // 启动 WiFi 热点 AP
static void wifi_ap_stop(void);             // 关闭 WiFi 热点 AP
static void IRAM_ATTR button_isr_handler(void *arg); // 按键中断服务函数（ISR），必须放在 IRAM 中快速响应

// 文件信息结构体，用于在内存中暂存 SD 卡内的文件名称和文件大小（字节）
typedef struct { 
    char name[256];     // 文件名缓存，最长支持 255 字节
    uint64_t size;      // 64位无符号整型，记录文件的大小（字节）
} file_info_t;


// ======================== 核心业务函数实现 ========================

/**
 * @brief  使用 Socket 循环阻塞发送，直到发送完指定长度的数据，或者连接断开。
 * @param sock: 已连接的 TCP 套接字句柄。
 * @param buf: 需要发送的数据内存指针。
 * @param len: 需要发送的总字节长度。
 * @return 成功返回 0，失败（连接断开或错误）返回 -1。
 */
static int send_all(int sock, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf; // 将数据指针强转为以字节为单位的指针，方便按字节偏移
    while (len > 0) {                       // 只要还需要发送的数据长度大于 0，就持续循环
        int r = send(sock, p, len, 0);      // 调用 lwIP 的 send 函数实际发送。r 代表本次实际发送成功的字节数
        if (r <= 0) return -1;              // 如果 r <= 0，说明发送出错或者对方断开了 TCP 连接
        p += r;                             // 内存指针向前移动 r 字节（跳过已发送的数据）
        len -= r;                           // 剩余待发送长度减去 r
    }
    return 0;                               // 循环结束说明全发完了，返回成功
}

/**
 * @brief  从 USB 模式切换为 WiFi 数据传输模式。
 * @return true 表示切换成功，false 表示内部挂载 SD 卡失败。
 */
static bool switch_to_wifi_mode(void)
{
#if USE_USB_MSC
    if (g_wifi_mode) return true;           // 如果当前已经是 WiFi 模式，不重复执行，直接返回成功
    printf("[模式] → WiFi模式 (断开USB + 本地挂载SD)\n");
    
    /* 🔴【灯语逻辑】：准备拔出 USB，LED 快闪 5 次，提示用户 PC 侧即将失去 U 盘识别 */
    LED_BLINK(5, 100);
    
    tud_disconnect();                       // 调用 TinyUSB 库，物理断开 USB 连接，此时电脑会发出“U盘已拔出”的声音
    vTaskDelay(pdMS_TO_TICKS(800));         // 延时 800ms，确保 Windows 系统的 USB 总线彻底完成断开枚举
    
    tinyusb_msc_storage_unmount();          // 将 TinyUSB 的 MSC 存储设备卸载，释放 FATFS 资源
    
    // 在 ESP32 内部文件系统中重新挂载 SD 卡（底层是 FatFS），此时电脑看不到，但 ESP 自己可以 fopen 读写
    esp_err_t ret = tinyusb_msc_storage_mount(SD_MOUNT_POINT);
    if (ret != ESP_OK) {                    // 如果内部挂载失败
        printf("[模式] 挂载失败: %s\n", esp_err_to_name(ret));
        /* 🔴【灯语逻辑】：内部挂载 SD 卡失败，长灭 3 秒作为严重错误提示，然后尝试重新连接 USB 防止假死 */
        LED(0); vTaskDelay(pdMS_TO_TICKS(3000));
        tud_connect();                      // 挂载失败，强制连回 USB 让电脑恢复 U 盘
        return false;
    }
    g_wifi_mode = true;                     // 标记当前已处于 WiFi 模式
    printf("[模式] WiFi模式就绪\n");
    return true;
#else
    g_wifi_mode = true;
    return true;
#endif
}

/**
 * @brief  从 WiFi 模式切换回 USB 模式（插回 U 盘）。
 */
static void switch_to_usb_mode(void)
{
#if USE_USB_MSC
    if (!g_wifi_mode) return;               // 如果本来就不是 WiFi 模式，没必要切换
    printf("[模式] → USB模式\n");
    
    tinyusb_msc_storage_unmount();          // 先卸载 ESP32 内部文件系统挂载的 SD 卡
    g_wifi_mode = false;                    // 清除 WiFi 模式标志
    vTaskDelay(pdMS_TO_TICKS(500));         // 延时 500ms，让文件系统完全从内存中退栈
    tud_connect();                          // 调用 TinyUSB 物理重新连接 USB，此时电脑会重新识别到 U 盘
    
    /* 🔴【灯语逻辑】：插回 USB，LED 快闪 5 次，提示用户 PC 侧恢复 U 盘识别 */
    LED_BLINK(5, 100);
    
    vTaskDelay(pdMS_TO_TICKS(500));         // 再延时 500ms 让电脑完成 USB 枚举和驱动加载
    printf("[模式] USB就绪\n");
#endif
}

/**
 * @brief  按键中断服务函数 (ISR)
 * @note   由于是在中断环境运行，必须加上 IRAM_ATTR 标志，且不能调用 printf 或 vTaskDelay！
 */
static void IRAM_ATTR button_isr_handler(void *arg)
{
    g_button_pressed = true; // 仅仅将全局标志位设为 true，具体工作交给 app_main 的主循环处理
}

/**
 * @brief  GPIO 按键初始化配置
 */
static void button_init(void)
{
    // 配置 GPIO0 (BOOT键)
    gpio_config_t io = {
        .intr_type    = GPIO_INTR_NEGEDGE,   // 配置中断触发方式：下降沿触发（按键按下时电平从高变低）
        .mode         = GPIO_MODE_INPUT,     // 配置为输入模式
        .pin_bit_mask = (1ULL << BUTTON_GPIO), // 选中 GPIO0
        .pull_up_en   = GPIO_PULLUP_ENABLE,   // 开启内部上拉电阻（默认高电平，按键按下接地拉低）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,// 关闭下拉电阻
    };
    gpio_config(&io);                         // 应用上述 GPIO 配置
    
    gpio_install_isr_service(0);              // 安装 ESP32 的 GPIO 中断服务系统
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL); // 为指定引脚注册中断回调函数
}


// ======================== WiFi AP 按需启停逻辑 ========================
static bool g_wifi_init_once = false;        // 全局网络环境（netif 和 event_loop）的“仅初始化一次”标志
static esp_netif_t *g_ap_netif = NULL;       // 保存 AP 网络接口句柄，用于后续获取 IP 地址

/**
 * @brief 整个程序生命周期中，只执行一次的网络基础环境初始化。
 * 如果放在 wifi_ap_start 中反复执行会导致崩溃或内存泄漏。
 */
static void wifi_init_once(void)
{
    if (g_wifi_init_once) return;           // 如果已经执行过一次，直接跳过
    ESP_ERROR_CHECK(esp_netif_init());        // 初始化 ESP32 的网络协议栈底层
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 创建系统默认的事件循环（用于处理 WiFi 连接/断开等事件）
    g_ap_netif = esp_netif_create_default_wifi_ap();  // 创建 WiFi AP 的默认网络接口句柄
    g_wifi_init_once = true;                // 标记为已初始化
}

/**
 * @brief  动态按需启动 WiFi 热点 AP
 * @note   彻底去除了开头的 esp_wifi_stop()，采用“初始化->配置->启动”的干净流程
 */
static void wifi_ap_start(void)
{
    wifi_init_once();                       // 确保底层网络环境和 netif 已准备好
    
    // 1. 初始化 ESP WiFi 驱动层（分配 WiFi 所需的内存和资源）
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 2. 从 NVS (非易失性闪存存储) 中读取用户之前保存的 SSID 和 密码，如果没有则使用代码里写的默认值
    char ssid[32] = WIFI_SSID;
    char pass[64] = WIFI_PASS;
    nvs_handle_t nvs;
    if (nvs_open("wifi_cfg", NVS_READONLY, &nvs) == ESP_OK) { // 尝试打开名为 "wifi_cfg" 的 NVS 分区
        size_t len = sizeof(ssid);
        nvs_get_str(nvs, "ssid", ssid, &len); // 读取键值为 "ssid" 的字符串
        len = sizeof(pass);
        nvs_get_str(nvs, "pass", pass, &len); // 读取键值为 "pass" 的字符串
        nvs_close(nvs);                       // 读取完毕，关闭 NVS 句柄
    }

    // 3. 构建 Wi-Fi AP 的配置结构体
    wifi_config_t wifi_cfg = {
        .ap = { 
            .ssid_len = strlen(ssid),         // 计算 SSID 的长度
            .max_connection = MAX_STA_CONN,   // 最大允许连接的从机数量
            .authmode = WIFI_AUTH_WPA_WPA2_PSK // 设置 WiFi 安全认证模式
        },
    };
    memcpy(wifi_cfg.ap.ssid, ssid, sizeof(wifi_cfg.ap.ssid));     // 将 SSID 复制到配置结构中
    memcpy(wifi_cfg.ap.password, pass, sizeof(wifi_cfg.ap.password)); // 将密码复制到配置结构中
    if (strlen(pass) == 0) wifi_cfg.ap.authmode = WIFI_AUTH_OPEN; // 如果读取到的密码为空，则强制将 WiFi 设为“无密码开放模式”

    // 4. 应用配置并启动 Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));             // 将 WiFi 芯片模式设置为 AP（热点）模式
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_cfg)); // 将刚才的配置参数应用给 AP 接口
    ESP_ERROR_CHECK(esp_wifi_start());                            // 正式启动 WiFi 芯片

    // 5. 获取并打印 AP 分配给主端自己的 IP 地址
    esp_netif_ip_info_t ip;
    if (g_ap_netif) {
        esp_netif_get_ip_info(g_ap_netif, &ip);                   // 从网络接口句柄中取出 IP 信息
        printf("[WiFi] AP已启动: SSID=%s, IP=" IPSTR "\n", ssid, IP2STR(&ip.ip)); // 打印日志
    }
}

/**
 * @brief  停止当前开启的 WiFi 热点
 */
static void wifi_ap_stop(void)
{
    // 调用 WiFi 停止和反初始化，释放 WiFi 驱动占用的资源
    if (esp_wifi_stop() == ESP_OK) {       // 如果 WiFi 停止成功
        esp_wifi_deinit();                  // 反初始化 WiFi 驱动层
        printf("[WiFi] AP已停止\n");
    }
}


// ======================== 核心文件传输处理逻辑 ========================
static void handle_transfer(int client_sock)
{
    file_info_t *files = NULL;   // 用于存放待传输文件信息的动态数组指针
    char *send_buf = NULL;       // 用于读取和发送文件数据的缓冲区指针

    // ============ 【1. 检查SD卡是否有文件】 ============
    {
        DIR *dir = opendir(SD_MOUNT_POINT); // 打开 SD 卡的根目录路径
        if (!dir) { printf("[传输] 无法打开SD\n"); return; }
        bool has_file = false;              // 文件是否存在标志位
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL) {       // 遍历 SD 卡中的每一个条目
            if (entry->d_type == DT_REG) {             // DT_REG 表示这是一个常规文件（而非文件夹或隐藏文件）
                has_file = true; break;                // 只要找到一个文件，就退出遍历
            }
        }
        closedir(dir);                      // 遍历完毕，关闭目录句柄
        if (!has_file) { printf("[传输] SD无文件\n"); return; } // 如果整个目录没有任何文件，直接退出
    }

    // ============ 【2. 设置 Socket 超时】 ============
    struct timeval tv = { .tv_sec = TRANSFER_ACK_TIMEOUT, .tv_usec = 0 }; // 构造 60 秒的超时结构体
    // 为接收设置超时：如果 60 秒内没收到对端发来的一个字节，recv 会自动返回 -1 并报错
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    // 为发送设置超时：如果 60 秒内对端没取走数据，send 会自动返回 -1
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    // ============ 【3. 握手协议 (MASTER_SEND)】 ============
    // 主端先发 11 个字节的字符串 "MASTER_SEND" 给从机，声明：我要发文件了
    if (send_all(client_sock, "MASTER_SEND", 11) != 0) { printf("[传输] 握手失败\n"); return; }
    char ack[16] = {0};                     // 用于接收从机回复的缓冲区
    // 等待接收从机的回复，期望收到 5 个字节的字符串 "READY"
    if (recv(client_sock, ack, 15, 0) <= 0 || strncmp(ack, "READY", 5) != 0)
    { printf("[传输] 从机未就绪\n"); return; }
    printf("[传输] 从机就绪\n");           // 从机已经准备好接收文件

    // ============ 【4. 收集 SD 卡中所有文件的信息】 ============
    files = malloc(sizeof(file_info_t) * MAX_FILES); // 动态申请 100 个结构体的内存空间
    if (!files) return;
    int file_cnt = 0;                               // 统计文件数量
    {
        DIR *dir = opendir(SD_MOUNT_POINT);         // 再次打开 SD 卡根目录
        if (!dir) { 
            uint32_t z = 0; 
            send_all(client_sock, &z, 4);           // 如果打开失败，发送数量 0 给从机
            free(files); return; 
        }
        struct dirent *entry;
        while ((entry = readdir(dir)) != NULL && file_cnt < MAX_FILES) {
            if (entry->d_type != DT_REG) continue;  // 只要读取到的不是常规文件，直接跳过
            char path[300];
            snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name); // 拼接文件的绝对路径
            struct stat st;
            if (stat(path, &st) == 0) {            // 调用 stat 函数获取该文件的元数据（如大小）
                strncpy(files[file_cnt].name, entry->d_name, 255); // 拷贝文件名
                files[file_cnt].name[255] = 0;                     // 确保名字以 \0 结尾
                files[file_cnt].size = (uint64_t)st.st_size;       // 记录文件大小
                file_cnt++;                                       // 文件计数 +1
            }
        }
        closedir(dir);                           // 遍历完毕，关闭目录句柄
    }

    // ============ 【5. 发送文件总数量给从机】 ============
    uint32_t cnt_net = htonl((uint32_t)file_cnt); // 把文件数量从主机字节序转为网络字节序（大端）
    send_all(client_sock, &cnt_net, 4);           // 通过 WiFi 发送 4 个字节给从机
    printf("[传输] %d个文件\n", file_cnt);

    // ============ 【6. 逐个发送文件】 ============
    send_buf = malloc(FILE_SEND_BUF_SIZE);        // 申请 4KB 的发送内存缓冲区
    if (!send_buf) { free(files); return; }

    /* 🔴【灯语逻辑】：正式开始核心数据发送，将 LED 长亮，代表文件正在通过 WiFi 往外吐数据，串口断开时可通过亮灯确认 */
    LED(1);

    for (int i = 0; i < file_cnt; i++) {
        // 6.1 发送文件名长度 (2字节)
        uint16_t nl = htons((uint16_t)strlen(files[i].name)); 
        send_all(client_sock, &nl, 2);
        // 6.2 发送文件名 (n字节)
        send_all(client_sock, files[i].name, strlen(files[i].name));

        // 6.3 发送文件大小 (8字节，大端序)
        uint64_t sb = htobe64(files[i].size);
        send_all(client_sock, &sb, 8);

        // 6.4 打开本地文件并发送内容
        char path[300];
        snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, files[i].name);
        FILE *fp = fopen(path, "rb"); // 以“只读二进制”模式打开 SD 卡上的文件
        if (!fp) { // 如果找不到文件（比如 SD 卡被意外拔出）
            memset(send_buf, 0, FILE_SEND_BUF_SIZE); // 将缓冲区清 0
            uint64_t r = files[i].size;
            while (r > 0) { // 即便打不开，为了保证 TCP 协议不断连，发送对应长度的全 0 数据“糊弄”过去
                size_t c = (r > FILE_SEND_BUF_SIZE) ? FILE_SEND_BUF_SIZE : (size_t)r;
                send_all(client_sock, send_buf, c);
                r -= c;
            }
            continue;
        }
        uint64_t remain = files[i].size; // 还剩多少字节没发完
        while (remain > 0) {
            size_t c = (remain > FILE_SEND_BUF_SIZE) ? FILE_SEND_BUF_SIZE : (size_t)remain; // 本轮最多读 4KB
            size_t n = fread(send_buf, 1, c, fp); // 从 SD 卡读取实际的数据到 send_buf
            
            /* 🔴【重要修改】读取到 0 字节时认为是错误，说明 SD 卡因速度过快读不出来了。立刻用 0 填充剩余数据，保证协议不断开 */
            if (n == 0) { 
                fclose(fp); 
                printf("[传输] 本地读取文件 %s 失败\n", files[i].name);
                /* 🔴【灯语逻辑】：读取故障，关闭长亮状态。 */
                LED(0);
                goto cleanup; 
            }
            
            // 将读到的 n 字节通过网络发送出去
            if (send_all(client_sock, send_buf, n) != 0) { 
                fclose(fp); 
                /* 🔴【灯语逻辑】：网络发送故障，关闭长亮状态。 */
                LED(0);
                goto cleanup; 
            }
            remain -= n; // 剩余未发送的字节减少
        }
        fclose(fp); // 一个文件发完了，关闭句柄释放内存
        printf("[传输] %s (%llu字节)\n", files[i].name, files[i].size);
    }

    /* 🔴【灯语逻辑】：所有文件发送完毕，熄灭长亮，结束传输状态 */
    LED(0);

    // ============ 【7. 发送完成指令 DONE，等待从机回复 OK】 ============
    send_all(client_sock, "DONE", 4);       // 告诉从机：主端全部发完了
    bool should_delete = false;             // 是否要删除源文件（取决于从机是否发回 OK）
    {
        char a[16] = {0};
        int n = recv(client_sock, a, 15, 0); // 等待从机发回确认信号
        if (n > 0) { a[n] = 0; printf("[传输] ACK:%s\n", a); }
        if (strncmp(a, "OK", 2) == 0) should_delete = true; // 从机回 ACK "OK"，说明收到了完整文件
    }

    // ============ 【8. 删除 SD 卡源文件】 ============
    if (should_delete) { // 如果从机发回 OK，主端就可以把文件删掉了
        printf("[传输] 删除SD文件...\n");
        /* 🔴【灯语逻辑】：正在删除旧文件，快闪 5 次提示文件正在被清理 */
        LED_BLINK(5, 100);

        DIR *dir = opendir(SD_MOUNT_POINT);
        if (dir) {
            struct dirent *entry;
            while ((entry = readdir(dir)) != NULL) {
                if (entry->d_type == DT_REG) { // 只删常规文件，不删目录
                    char path[300];
                    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, entry->d_name);
                    unlink(path);             // 删除该文件
                }
            }
            closedir(dir);
        }
        vTaskDelay(pdMS_TO_TICKS(300));       // 延时 300ms 保证磁盘 I/O 完成
    }

cleanup: // 错误清理的跳转标签
    free(send_buf); // 释放 4KB 发送缓冲区内存
    free(files);    // 释放文件信息结构体数组内存
    printf("[传输] 完成\n");
}


// ======================== 主函数入口 ========================
void app_main(void)
{
    // ======= 1. 基础系统与存储初始化 =======
    esp_err_t ret = nvs_flash_init(); // 初始化 NVS 闪存分区（用于保存 Wi-Fi 设置等）
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); // 如果 NVS 分区损坏或版本不对，擦除重建
        nvs_flash_init();  // 重新初始化
    }
    
    led_init();            // 初始化开发板上的 LED 灯引脚
    LED(1);                // 点亮 LED (短暂亮起代表 CPU 已进入运行状态)

    // ======= 2. SD 卡与 USB MSC 初始化 =======
    sdmmc_card_t *sd_card = NULL;
    esp_err_t sd_ret = sd_card_init(&sd_card); // 初始化 SPI/SDMMC 总线驱动并挂载 SD 卡
    bool sd_ok = (sd_ret == ESP_OK && sd_card != NULL); // 判断 SD 卡是否初始化成功

    if (sd_ok) {
        printf("[主函数] SD卡就绪\n");
#if USE_USB_MSC
        ESP_ERROR_CHECK(usb_msc_init(sd_card)); // 将 SD 卡注册到 TinyUSB 的 MSC 存储设备上
        tud_connect();                         // 初始状态：USB 为连接状态（电脑能识别 U 盘）
        vTaskDelay(pdMS_TO_TICKS(1000));       // 等待 1 秒，让 Windows 完成驱动加载
        printf("[主函数] USB MSC 就绪，PC可访问SD卡\n");
#endif
    } else {
        printf("[主函数] SD卡失败\n");
    }

    // ======= 3. 按键与日志初始化 =======
    button_init();                           // 配置 GPIO0 按键及其中断功能
    
    vTaskDelay(pdMS_TO_TICKS(500));          // 延时 500ms 让系统稳定
    setvbuf(stdout, NULL, _IONBF, 0);        // 禁用 printf 的缓冲区，让串口日志实时打印（对调试极其重要）
    LED(0);                                  // 初始化完成后，熄灭 LED，准备进入待机闪烁状态

    printf("\n========== 主机 V3.5 (按键触发) ==========\n");
    printf("U盘已就绪，按下 GPIO0 按键开始传输\n");

    int trans_cnt = 0; // 成功传输的次数统计

    // ======= 4. 主循环 (事件轮询) =======
    while (1) {
        /* 🔴【灯语逻辑】：正常待机状态，1.2秒一个周期慢速翻转（缓慢呼吸），代表设备正常运行等待按键 */
        LED_TOGGLE();
        vTaskDelay(pdMS_TO_TICKS(800)); 

        // 检测按键是否被按下（在中断中被置位）
        if (g_button_pressed) {
            g_button_pressed = false; // 清除按键标志位（防止重复触发）
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)); // 延时消抖 200ms
            printf("[按键] 用户按下，检查SD卡...\n");
            
            /* 🔴【灯语逻辑】：成功检测到有效按键按下，LED 快闪 3 下提示用户系统响应了按键 */
            LED_BLINK(3, 100);

            // 4.1 切换为 WiFi 模式 (断开 USB，ESP32 独占 SD 卡)
            if (!switch_to_wifi_mode()) {
                printf("[按键] 切模式失败\n");
                switch_to_usb_mode(); // 失败则紧急恢复 USB
                continue;
            }

            // 4.2 再次检查 SD 卡里面是否有文件
            bool has_file = false;
            DIR *dir = opendir(SD_MOUNT_POINT);
            if (dir) {
                struct dirent *entry;
                while ((entry = readdir(dir)) != NULL) {
                    if (entry->d_type == DT_REG) { 
                        has_file = true; break; 
                    }
                }
                closedir(dir);
            }
            if (!has_file) { // 如果没有找到任何常规文件，直接切回 USB 模式
                printf("[按键] SD无文件\n");
                switch_to_usb_mode();
                continue;
            }

            // 4.3 有文件，启动 WiFi 热点
            printf("[按键] 发现文件，启动WiFi热点...\n");
            wifi_ap_start(); // 开启 AP 热点

            // 4.4 建立 TCP Socket 监听服务
            int lsock = socket(AF_INET, SOCK_STREAM, 0); // 创建一个 TCP 监听 Socket
            if (lsock < 0) { wifi_ap_stop(); switch_to_usb_mode(); continue; }

            int opt = 1;
            setsockopt(lsock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)); // 设置端口复用，防止内存泄漏

            struct sockaddr_in addr = {0};
            addr.sin_family = AF_INET;                     // IPV4 协议族
            addr.sin_addr.s_addr = htonl(INADDR_ANY);      // 监听本机所有 IP 网卡
            addr.sin_port = htons(TRANSFER_PORT);          // 绑定 3333 端口
            if (bind(lsock, (struct sockaddr *)&addr, sizeof(addr)) != 0) { // 将 Socket 绑定到端口
                close(lsock); wifi_ap_stop(); switch_to_usb_mode(); continue;
            }
            listen(lsock, 1); // 开始监听，最多排队 1 个连接

            // 监听 Socket 的超时设置
            struct timeval tv = { .tv_sec = ACCEPT_TIMEOUT_S, .tv_usec = 0 };
            setsockopt(lsock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)); // 让 accept 在 60 秒后自动超时

            printf("[按键] 等待从机连接(超时%ds)...\n", ACCEPT_TIMEOUT_S);

            // 4.5 等待从机的 TCP 连接
            struct sockaddr_in cli;
            socklen_t cl = sizeof(cli);
            // accept 是一个阻塞函数，此处会阻塞主循环 60 秒，直到有从机连过来或超时
            int csock = accept(lsock, (struct sockaddr *)&cli, &cl); 

            if (csock < 0) {
                printf("[按键] 从机连接超时\n");
            } else {
                printf("[按键] 从机已连接: %s\n", inet_ntoa(cli.sin_addr));
                handle_transfer(csock); // 调用核心传输函数进行文件投递
                close(csock);           // 传输完毕，关闭客户端 Socket
                trans_cnt++;
            }

            // 4.6 收尾操作
            close(lsock);             // 关闭监听 Socket
            wifi_ap_stop();           // 关闭 WiFi 热点
            switch_to_usb_mode();     // 切换回 USB 模式（电脑重新识别到 U 盘）

            printf("[按键] 完成(共%d次传输)，可再按按键\n", trans_cnt);
        }
    }
}