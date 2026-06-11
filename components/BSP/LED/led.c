/**
 ****************************************************************************************************
 * @file        led.c
 * @author      ONE
 * @version     V1.0
 * @date        2026-06-11
 * @brief       LED驱动代码 - GPIO1控制指示灯
 *              功能: ①初始化GPIO1为推挽输出 → ②提供LED开关/翻转宏供上层调用
 *              硬件: GPIO1 连接LED(高电平亮, 低电平灭)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * LED电路: GPIO1 → 限流电阻 → LED → GND (高电平点亮)
 *          内部上拉使能, 上电默认高电平=LED亮
 * @note        LED(x)宏定义在led.h中: x=1点亮, x=0熄灭
 *
 ****************************************************************************************************
 */

#include "led.h"                            /* LED驱动头文件: 引脚定义 + 控制宏 */


/**
 * @brief       初始化LED控制GPIO
 *              配置GPIO1为输入输出模式(推挽), 使能上拉, 默认输出高电平(LED亮)
 *              注意: LED(1) = 输出高电平 = GPIO置位, 此处LED(1)为点亮
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    gpio_config_t gpio_init_struct = {0};                           /* 定义GPIO配置结构体, 全部字段初始化为0(安全) */

    gpio_init_struct.intr_type = GPIO_INTR_DISABLE;                 /* 失能引脚中断: LED不需要中断功能 */
    gpio_init_struct.mode = GPIO_MODE_INPUT_OUTPUT;                 /* 设置为输入输出模式: 可读回当前电平状态(用于LED_TOGGLE) */
    gpio_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;               /* 使能内部上拉电阻: 防止浮空时LED误亮, 默认拉高→LED亮 */
    gpio_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;          /* 失能内部下拉电阻: 不需要下拉 */
    gpio_init_struct.pin_bit_mask = 1ull << LED_GPIO_PIN;           /* 设置引脚位掩码: 1<<GPIO1 = 0x02, 仅配置GPIO1 */
    gpio_config(&gpio_init_struct);                                 /* 调用ESP-IDF GPIO配置API, 将上述设置写入硬件寄存器 */

    LED(1);                                                         /* 点亮LED: 输出高电平, 表示系统已上电(视觉确认) */
}
