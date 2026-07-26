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
#define APP_SERIAL_BAUD_RATE             (115200UL)
#define APP_SERIAL_REPORT_PERIOD_MS      (250UL)
#define APP_SERIAL_REPORT_PERIOD_TICKS   \
    (APP_SERIAL_REPORT_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_SERIAL_TX_BUFFER_SIZE        (1024U)

/* ==================== PWM ==================== */
#define APP_PWM_PERIOD_TICKS             (1600U)
#define APP_MOTOR_COMMAND_MAX            (1000)

/* ==================== Mechanical parameters ==================== */
#define APP_WHEEL_DIAMETER_MM            (66L)
#define APP_WHEEL_TRACK_MM               (114L)
#define APP_ENCODER_COUNTS_PER_REV       (1469L)
#define APP_PI_X10000                    (31416L)

/* ==================== Position profile ==================== */
#define APP_POSITION_DEFAULT_MAX_PWM     (300)
#define APP_POSITION_MIN_PWM             (120)
#define APP_POSITION_APPROACH_PWM        (105)
#define APP_POSITION_DECEL_COUNTS        (700L)
#define APP_POSITION_APPROACH_COUNTS     (100L)
#define APP_POSITION_TOLERANCE_COUNT     (8L)

/* Nominal feed-forward command slew per 5 ms tick. */
#define APP_MOTOR_ACCEL_STEP             (4)
#define APP_MOTOR_DECEL_STEP             (12)

/* ==================== Wheel-speed PI inner loop ==================== */
/*
 * Encoder speed is represented in counts/control-tick x 16.
 * 590 / 16 = 36.875 counts per 5 ms at PWM command 1000.
 * This initial value is based on the measured operating point:
 * about 8.48 counts/tick at PWM 230.
 */
#define APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16 (590L)

/* First-order IIR: filtered += (sample - filtered) / DIV. */
#define APP_SPEED_FILTER_DIV             (4L)

/* PI output is a PWM correction added to speed feed-forward. */
#define APP_SPEED_PI_KP_NUM              (3L)
#define APP_SPEED_PI_KP_DIV              (4L)
#define APP_SPEED_PI_KI_DIV              (32L)
#define APP_SPEED_PI_INTEGRAL_LIMIT      (6400L)
#define APP_SPEED_PI_CORRECTION_LIMIT    (250L)

/* ==================== Left/right synchronization ==================== */
/*
 * Position-difference controller output is target-speed correction in
 * counts/tick x 16. A positive error means the left wheel is ahead.
 */
#define APP_SYNC_SPEED_KP_X16            (2L)
#define APP_SYNC_SPEED_KI_DIV            (128L)
#define APP_SYNC_SPEED_KD_X16            (3L)
#define APP_SYNC_SPEED_INTEGRAL_LIMIT    (4096L)
#define APP_SYNC_SPEED_CORRECTION_LIMIT_X16 (64L)

/* ==================== Encoder-fault protection ==================== */
#define APP_ENCODER_FAULT_MIN_PWM        (150)
#define APP_ENCODER_FAULT_MIN_TARGET_SPEED_X16 (80L)
#define APP_ENCODER_FAULT_WINDOW_TICKS   (40U)  /* 200 ms */
#define APP_ENCODER_FAULT_MIN_PROGRESS_COUNT (8L)

#define APP_ENCODER_FAULT_LEFT           (1U << 0)
#define APP_ENCODER_FAULT_RIGHT          (1U << 1)

/* TB6612 short-brake duration: 8 x 5 ms = 40 ms. */
#define APP_ACTIVE_BRAKE_TICKS           (8U)

/* 5 ms x 12000 = 60 s timeout for long-distance tests. */
#define APP_MOTION_TIMEOUT_TICKS         (12000U)

#endif
