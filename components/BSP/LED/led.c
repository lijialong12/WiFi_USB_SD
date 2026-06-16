/**
 ****************************************************************************************************
 * @file        led.c
 * @author      ONE
 * @version     V1.1
 * @date        2026-06-15
 * @brief       LED驱动代码 - GPIO1控制指示灯 + 非阻塞闪烁模式
 *              功能: ①初始化GPIO1为推挽输出 → ②提供LED开关/翻转宏 →
 *                    ③非阻塞LED闪烁模式(用于状态指示)
 *              硬件: GPIO1 连接LED(高电平亮, 低电平灭)
 * @license     重庆博士康科技有限公司版权所有
 ****************************************************************************************************
 * @attention
 *
 * 实验平台: ESP32-S3 WIFI USB SD卡 开发板
 * LED电路: GPIO1 → 限流电阻 → LED → GND (高电平点亮)
 *          内部上拉使能, 上电默认高电平=LED亮
 * @note        LED(x)宏定义在led.h中: x=1点亮, x=0熄灭
 *              led_pattern_tick() 需要在主循环中周期性调用(建议每100~200ms)
 *              内部使用esp_timer_get_time()实现精确定时, 不受调用间隔影响
 *
 ****************************************************************************************************
 */

#include "led.h"                            /* LED驱动头文件: 引脚定义 + 控制宏 + 模式枚举 */
#include "driver/gpio.h"                    /* ESP-IDF GPIO驱动: gpio_set_level/gpio_get_level */
#include "esp_timer.h"                      /* ESP-IDF高精度定时器: esp_timer_get_time (微秒) */

/* ======================== 模式驱动内部状态 ======================== */
static led_pattern_t g_pattern     = LED_PAT_SOLID_ON; /* 当前LED模式 */
static int64_t       g_last_toggle = 0;               /* 上次翻转时间(微秒) */
static bool          g_led_state   = true;             /* 当前LED实际电平(true=亮) */
static bool          g_in_flash    = false;            /* 是否在执行一次性flash序列 */
static int           g_flash_count = 0;                /* flash序列剩余闪烁次数 */
static int           g_flash_phase = 0;                /* flash序列中的子阶段(0=灭,1=亮,2=灭...) */

/**
 * @brief       blink模式的一个半周期长度(毫秒)
 * @param pat: 模式枚举
 * @retval      半周期毫秒数
 */
static uint32_t get_half_period(led_pattern_t pat)
{
    switch (pat) {
        case LED_PAT_SLOW_BLINK:      return 400;   /* 400ms亮 + 400ms灭 = 0.8s周期 */
        case LED_PAT_MED_BLINK:       return 200;   /* 200ms亮 + 200ms灭 = 0.4s周期 */
        case LED_PAT_FAST_BLINK:      return 100;   /* 100ms亮 + 100ms灭 = 0.2s周期 */
        case LED_PAT_VERY_FAST_BLINK: return 50;    /* 50ms亮 + 50ms灭 = 0.1s周期 */
        case LED_PAT_FLASH_3:         return 80;    /* 快速flash: 80ms半周期 */
        case LED_PAT_FLASH_2:         return 80;    /* 快速flash: 80ms半周期 */
        default:                      return 500;   /* 默认 */
    }
}

/**
 * @brief       初始化LED控制GPIO
 *              配置GPIO1为输入输出模式(推挽), 使能上拉, 默认输出高电平(LED亮)
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

    /* 初始化模式驱动状态 */
    g_pattern     = LED_PAT_SOLID_ON;
    g_last_toggle = esp_timer_get_time();                           /* 记录当前时刻作为起始 */
    g_led_state   = true;
    g_in_flash    = false;
    LED(1);                                                         /* 点亮LED: 输出高电平, 表示系统已上电(视觉确认) */
}

/**
 * @brief       设置LED闪烁模式 (即时生效)
 * @param       pattern: 闪烁模式枚举值
 */
void led_set_pattern(led_pattern_t pattern)
{
    if (pattern == g_pattern && !g_in_flash) {
        return;  /* 相同模式且不在flash中, 忽略 */
    }

    g_pattern     = pattern;
    g_last_toggle = esp_timer_get_time();                           /* 重置定时基准 */

    /* 处理常亮/常灭模式 */
    if (pattern == LED_PAT_SOLID_ON) {
        g_in_flash  = false;
        g_led_state = true;
        LED(1);
        return;
    }
    if (pattern == LED_PAT_OFF) {
        g_in_flash  = false;
        g_led_state = false;
        LED(0);
        return;
    }

    /* 处理一次性flash模式 */
    if (pattern == LED_PAT_FLASH_3 || pattern == LED_PAT_FLASH_2) {
        g_in_flash    = true;
        g_flash_count = (pattern == LED_PAT_FLASH_3) ? 3 : 2;
        g_flash_phase = 0;               /* 从灭开始 */
        g_led_state   = false;
        LED(0);
        return;
    }

    /* blink模式: 从亮开始 */
    g_in_flash  = false;
    g_led_state = true;
    LED(1);
}

/**
 * @brief       LED模式tick (非阻塞驱动, 需在主循环中周期性调用)
 *              内部使用esp_timer_get_time()微秒定时器, 不受调用间隔影响
 *              建议调用周期: 50~200ms
 * @param       无
 * @retval      无
 */
void led_pattern_tick(void)
{
    int64_t now  = esp_timer_get_time();                            /* 微秒级时间戳 */
    int64_t elapsed_ms = (now - g_last_toggle) / 1000;              /* 距离上次翻转的毫秒数 */

    /* ===== 一次性flash序列处理 ===== */
    if (g_in_flash) {
        uint32_t half = get_half_period(g_pattern);  /* 80ms */
        if (elapsed_ms >= (int64_t)half) {
            g_last_toggle = now;
            g_flash_phase++;

            if (g_flash_phase % 2 == 1) {
                /* 奇数阶段: 亮 */
                g_led_state = true;
                LED(1);
            } else {
                /* 偶数阶段: 灭 */
                g_led_state = false;
                LED(0);
                if (g_flash_phase / 2 >= g_flash_count) {
                    /* flash序列结束, 回到常亮 */
                    g_in_flash  = false;
                    g_pattern   = LED_PAT_SOLID_ON;
                    g_led_state = true;
                    LED(1);
                }
            }
        }
        return;
    }

    /* ===== 常规blink模式 ===== */
    if (g_pattern == LED_PAT_SOLID_ON || g_pattern == LED_PAT_OFF) {
        return;  /* 静态模式不需要tick */
    }

    uint32_t half = get_half_period(g_pattern);
    if (elapsed_ms >= (int64_t)half) {
        /* 重置基准(减去余数以补偿累积误差, 保持节奏均匀) */
        g_last_toggle = now - (elapsed_ms % (int64_t)half) * 1000;
        g_led_state   = !g_led_state;
        LED(g_led_state ? 1 : 0);
    }
}
