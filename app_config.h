#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ==================== System timing ==================== */
#define APP_CPU_CLOCK_HZ                 (32000000UL)
#define APP_CONTROL_PERIOD_MS            (5UL)
#define APP_CONTROL_TICK_RELOAD          \
    (((APP_CPU_CLOCK_HZ / 1000UL) * APP_CONTROL_PERIOD_MS) - 1UL)

#define APP_BUTTON_DEBOUNCE_MS           (50UL)
#define APP_OLED_STATUS_PERIOD_MS        (250UL)
#define APP_BUTTON_DEBOUNCE_TICKS        \
    (APP_BUTTON_DEBOUNCE_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_STATUS_PERIOD_TICKS     \
    (APP_OLED_STATUS_PERIOD_MS / APP_CONTROL_PERIOD_MS)

/* ==================== OLED reliability ==================== */
#define APP_OLED_I2C_DELAY_CYCLES        (32UL)
#define APP_OLED_POWER_UP_DELAY_CYCLES   (1600000UL)
#define APP_OLED_TRANSACTION_RETRIES     (2U)
#define APP_OLED_RETRY_PERIOD_MS         (1000UL)
#define APP_OLED_RETRY_PERIOD_TICKS      \
    (APP_OLED_RETRY_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_PAGES_PER_SERVICE       (1U)


/* ==================== Serial telemetry ==================== */
#define APP_SERIAL_BAUD_RATE              (115200UL)
#define APP_SERIAL_REPORT_PERIOD_MS       (250UL)
#define APP_SERIAL_REPORT_PERIOD_TICKS    \
    (APP_SERIAL_REPORT_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_SERIAL_TX_BUFFER_SIZE         (1024U)

/* ==================== PWM ==================== */
#define APP_PWM_PERIOD_TICKS             (1600U)
#define APP_MOTOR_COMMAND_MAX            (1000)

/* ==================== Mechanical parameters ==================== */
#define APP_WHEEL_DIAMETER_MM            (65L)
#define APP_WHEEL_TRACK_MM               (114L)
#define APP_ENCODER_COUNTS_PER_REV       (1468L)
#define APP_PI_X10000                    (31416L)

/* ==================== Position control ==================== */
#define APP_POSITION_DEFAULT_MAX_PWM     (300)

/* Normal low-speed command after the motor has started moving. */
#define APP_POSITION_MIN_PWM             (120)

/* Final approach command. Tune in the range 95 to 120. */
#define APP_POSITION_APPROACH_PWM        (105)

/* Begin normal deceleration about 700 counts before the target. */
#define APP_POSITION_DECEL_COUNTS        (700L)

/* Enter the slow final approach zone about 100 counts before target. */
#define APP_POSITION_APPROACH_COUNTS     (100L)

#define APP_POSITION_TOLERANCE_COUNT     (8L)

/* PWM slew limits per 5 ms control tick. */
#define APP_MOTOR_ACCEL_STEP             (4)
#define APP_MOTOR_DECEL_STEP             (12)

/* Left/right travel synchronization controller. */
#define APP_SYNC_KP                      (2L)
#define APP_SYNC_KI_DIV                  (64L)
#define APP_SYNC_KD                      (2L)
#define APP_SYNC_INTEGRAL_LIMIT          (800L)
#define APP_SYNC_CORRECTION_LIMIT        (70L)

/* TB6612 short-brake duration: 8 x 5 ms = 40 ms. */
#define APP_ACTIVE_BRAKE_TICKS           (8U)

/* 5 ms x 4000 = 20 s timeout. */
#define APP_MOTION_TIMEOUT_TICKS         (4000U)

#endif
