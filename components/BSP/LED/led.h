/**
 ****************************************************************************************************
 * @file        led.h
 * @author      ONE
 * @version     V1.1
 * @date        2026-06-15
 * @brief       LED驱动头文件 - GPIO1引脚定义与控制宏 + LED闪烁模式API
 *              提供: ①引脚宏定义 → ②电平状态枚举 → ③LED开关/翻转宏 →
 *                    ④LED闪烁模式枚举 → ⑤模式设置/tick函数声明
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * @note        LED(x): x=1 点亮(高电平), x=0 熄灭(低电平)
 *              LED_TOGGLE(): 读取当前电平并取反, 实现闪烁
 *              led_set_pattern(): 设置闪烁模式(非阻塞, 需在主循环调用tick)
 *
 ****************************************************************************************************
 */

#ifndef __LED_H_                           /* 头文件保护宏: 防止重复包含 */
#define __LED_H_                           /* 定义保护宏标识 */

#include "driver/gpio.h"                   /* ESP-IDF GPIO驱动: gpio_set_level/gpio_get_level函数 */
#include <stdint.h>                        /* 标准整数类型: uint32_t */


/* ======================== 引脚定义 ======================== */
#define LED_GPIO_PIN    GPIO_NUM_1         /* LED连接的GPIO端口: GPIO1, 板上LED */

/* ======================== 电平状态枚举 ======================== */
/* 引脚的输出的电平状态 */
enum GPIO_OUTPUT_STATE                     /* 定义GPIO输出电平枚举类型 */
{
    PIN_RESET,                             /* 低电平(0) = LED熄灭 */
    PIN_SET                                /* 高电平(1) = LED点亮 */
};

/* ======================== LED控制宏 ======================== */
/*
 * LED端口定义: 三目运算符实现开关控制
 * 使用 do-while(0) 惯用法: 确保宏在任何 if-else 上下文中安全展开(消除悬挂问题)
 * LED(1) → 输出高电平 → LED点亮
 * LED(0) → 输出低电平 → LED熄灭
 * 注意: 反斜杠 \ 必须是行尾最后一个字符, 后面不能有注释
 */
#define LED(x)          do { x ?                                      \
                             gpio_set_level(LED_GPIO_PIN, PIN_SET) :  \
                             gpio_set_level(LED_GPIO_PIN, PIN_RESET); \
                        } while(0)

/* LED取反定义: 读取当前电平 → 取反 → 输出, 实现翻转 */
#define LED_TOGGLE()    do { gpio_set_level(LED_GPIO_PIN, !gpio_get_level(LED_GPIO_PIN)); } while(0)  /* 读取GPIO1电平→逻辑取反→写回, 实现LED亮灭切换 */

/* ======================== LED闪烁模式枚举 ======================== */
/**
 * @brief LED闪烁模式枚举 (非阻塞, 需在主循环调用 led_pattern_tick)
 */
typedef enum {
    LED_PAT_SOLID_ON,        /* 常亮 */
    LED_PAT_SLOW_BLINK,      /* 慢闪: 400ms亮/400ms灭 (待机/等待连接) */
    LED_PAT_MED_BLINK,       /* 中速闪: 200ms亮/200ms灭 (WiFi模式空闲) */
    LED_PAT_FAST_BLINK,      /* 快闪: 100ms亮/100ms灭 (传输中) */
    LED_PAT_VERY_FAST_BLINK, /* 超快闪: 50ms亮/50ms灭 (错误) */
    LED_PAT_FLASH_3,         /* 3次快速闪烁后常亮 (传输成功) */
    LED_PAT_FLASH_2,         /* 2次快速闪烁后常亮 (传输成功-从机) */
    LED_PAT_OFF,             /* 常灭 */
} led_pattern_t;

/* ======================== 函数声明 ======================== */
void led_init(void);                       /* 初始化LED: 配置GPIO1为推挽输出, 默认点亮 */

/**
 * @brief       设置LED闪烁模式 (即时生效)
 * @param       pattern: 闪烁模式枚举值
 */
void led_set_pattern(led_pattern_t pattern);

/**
 * @brief       LED模式tick (非阻塞驱动, 需每50~200ms调用一次)
 *              根据当前模式和内部定时器自动切换LED状态
 */
void led_pattern_tick(void);

#endif                                     /* __LED_H_ 头文件保护结束 */
