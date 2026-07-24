#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ==================== 系统时间 ==================== */

#define APP_CPU_CLOCK_HZ              (32000000UL)
#define APP_CONTROL_PERIOD_MS         (5UL)

/* OLED menu buttons are sampled every control period. */
#define APP_BUTTON_DEBOUNCE_MS        (50UL)
#define APP_OLED_STATUS_PERIOD_MS     (250UL)

#define APP_CONTROL_DELAY_CYCLES      \
    ((APP_CPU_CLOCK_HZ / 1000UL) * APP_CONTROL_PERIOD_MS)

/* ==================== PWM ==================== */

#define APP_PWM_PERIOD_TICKS          (1600U)
#define APP_MOTOR_COMMAND_MAX         (1000)

/* ==================== 机械参数 ==================== */

/* 驱动轮直径 */
#define APP_WHEEL_DIAMETER_MM         (65L)

/* 左右后轮中心线距离 */
#define APP_WHEEL_TRACK_MM            (114L)

/* A/B 相四倍频后，车轮一圈的计数 */
#define APP_ENCODER_COUNTS_PER_REV    (1468L)

/* π × 10000 */
#define APP_PI_X10000                 (31416L)

/* ==================== 位置控制参数 ==================== */

/*
 * 克服电机死区所需的最低命令。
 * 首次测试建议从 90~120 之间调整。
 */
#define APP_POSITION_MIN_PWM          (100)

/* 默认最大命令 */
#define APP_POSITION_DEFAULT_MAX_PWM  (300)

/*
 * 剩余计数转换为附加 PWM：
 *
 * PWM = MIN_PWM + remaining / DIV
 */
#define APP_POSITION_DECEL_COUNTS     (400L)

/* 允许的最终位置误差 */
#define APP_POSITION_TOLERANCE_COUNT  (8L)

/* 连续满足目标多少个周期后，确认运动完成 */
#define APP_SYNC_KP                   (2L)
#define APP_SYNC_KI_DIV               (32L)
#define APP_SYNC_KD                   (3L)
#define APP_SYNC_INTEGRAL_LIMIT       (1000L)
#define APP_SYNC_CORRECTION_LIMIT     (80L)

/* TB6612 short-brake duration: 8 x 5 ms = 40 ms. */
#define APP_ACTIVE_BRAKE_TICKS        (8U)

/* 5 ms × 4000 = 20 s 超时 */
#define APP_MOTION_TIMEOUT_TICKS      (4000U)

/* ==================== 编码器滤波 ==================== */

/* 32 MHz 下 1600 cycle = 50 us */
#define APP_ENCODER_DEGLITCH_CYCLES   (1600UL)

#endif
