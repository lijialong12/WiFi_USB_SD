# WiFi_USB_SD — ESP32-S3 WiFi 智能U盘

ESP32-S3 综合应用：SD 卡同时作为 **电脑U盘（USB MSC）** 和 **手机网页文件管理器（WiFi AP + HTTP服务器）**，两者根据 WiFi 客户端数量自动互斥切换，全程无需拔插线缆。

- 手机连上热点 → 约3秒后电脑上的U盘自动弹出，切换为网页模式
- 手机断开热点 → 约4秒后U盘自动重新出现在电脑上

## 功能特性

| 模块 | 说明 |
|---|---|
| USB MSC | TinyUSB 全速设备（VID `0x303A` / PID `0x4002`），插入电脑即识别为U盘 |
| WiFi AP | 默认 SSID `BOSSCOM_USB_AP` / 密码 `012345678`；用户自定义配置保存在 NVS，启动时优先加载 |
| Web文件管理器 | `http://192.168.4.1`：目录浏览、图片预览、文本预览(≤512KB)、文件下载、WiFi名称/密码修改 |
| 自动切换 | 主循环200ms防抖：客户端连入15次(3s)切网页模式，全部断开20次(4s)切回U盘模式 |
| LED指示 | GPIO1：慢闪(200ms)=SD正常，快闪(100ms)=SD异常 |

## 硬件连接

| 外设 | 引脚 |
|---|---|
| SD_CLK / SD_CMD | GPIO36 / GPIO35 |
| SD_D0 ~ SD_D3 | GPIO37 / GPIO38 / GPIO33 / GPIO34 |
| USB OTG D- / D+ | GPIO19 / GPIO20（内置全速PHY） |
| LED | GPIO1（高电平点亮） |

SDMMC 4-bit @ 40MHz，GPIO驱动能力 CAP_3；USB 2.0 Full-Speed (12Mbps)。

## 编译烧录

基于 ESP-IDF v5.4.1：

```
idf.py set-target esp32s3
idf.py build flash monitor
```

## 目录结构

```
main/
  main.c          入口：初始化流程、USB/WiFi模式切换状态机、WiFi AP
  web_server.c/h  HTTP文件服务器（/api/list、/api/file、/api/wifi-config等）
  web_page.h      内嵌单页前端（文件管理器 + WiFi设置面板）
components/BSP/
  LED/            GPIO1 LED驱动
  SD/             SDMMC卡驱动 + 直接FATFS挂载(USE_USB_MSC=0时使用)
  USB/            TinyUSB MSC封装
```

## 注意事项

- SD卡的**本地FATFS访问与PC的USB MSC访问互斥**，切换由主循环统一调度，
  切换瞬间由 TinyUSB 回调（`tud_mount_cb`/`tud_umount_cb`）与显式 mount/unmount 配合完成。
- Web服务器的自动重挂载仅在 WiFi 网页模式下生效，避免破坏PC端U盘的可用性。
