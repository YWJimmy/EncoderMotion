#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ==================== System timing ==================== */
#define APP_CPU_CLOCK_HZ                 (32000000UL)
#define APP_CONTROL_PERIOD_MS            (5UL)
#define APP_CONTROL_TICK_RELOAD          \
    (((APP_CPU_CLOCK_HZ / 1000UL) * APP_CONTROL_PERIOD_MS) - 1UL)
#define APP_BUTTON_DEBOUNCE_MS           (50UL)
#define APP_BUTTON_DEBOUNCE_TICKS        \
    (APP_BUTTON_DEBOUNCE_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_STATUS_PERIOD_MS        (250UL)
#define APP_OLED_STATUS_PERIOD_TICKS     \
    (APP_OLED_STATUS_PERIOD_MS / APP_CONTROL_PERIOD_MS)

/* ==================== OLED ==================== */
#define APP_OLED_I2C_DELAY_CYCLES        (32UL)
#define APP_OLED_POWER_UP_DELAY_CYCLES   (1600000UL)
#define APP_OLED_TRANSACTION_RETRIES     (2U)
#define APP_OLED_RETRY_PERIOD_MS         (1000UL)
#define APP_OLED_RETRY_PERIOD_TICKS      \
    (APP_OLED_RETRY_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_PAGES_PER_SERVICE       (1U)

/* ==================== UART telemetry ==================== */
#define APP_SERIAL_BAUD_RATE             (115200UL)
#define APP_SERIAL_REPORT_PERIOD_MS      (250UL)
#define APP_SERIAL_REPORT_PERIOD_TICKS   \
    (APP_SERIAL_REPORT_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_SERIAL_TX_BUFFER_SIZE        (1024U)

/* ==================== Motor/PWM ==================== */
#define APP_PWM_PERIOD_TICKS             (1600U)
#define APP_MOTOR_COMMAND_MAX            (1000)

/* ==================== Mechanical calibration ==================== */
#define APP_WHEEL_DIAMETER_MM            (66L)
#define APP_ENCODER_COUNTS_PER_REV       (1469L)
#define APP_TURN_EFFECTIVE_TRACK_MM      (114L)
#define APP_PI_X10000                    (31416L)

/* ==================== Straight motion ==================== */
#define APP_STRAIGHT_CONTROL_MAX_PWM     (400)
#define APP_STRAIGHT_MIN_PWM             (140)
#define APP_STRAIGHT_APPROACH_PWM        (125)
#define APP_STRAIGHT_DECEL_COUNTS        (900L)
#define APP_STRAIGHT_APPROACH_COUNTS     (140L)
#define APP_STRAIGHT_TOLERANCE_COUNTS    (8L)
#define APP_STRAIGHT_SYNC_DEADBAND       (2L)
#define APP_STRAIGHT_SYNC_KP_DIV         (4L)
#define APP_STRAIGHT_SYNC_LIMIT          (35)

/* ==================== In-place turn ==================== */
#define APP_TURN_CONTROL_MAX_PWM         (300)
#define APP_TURN_MIN_PWM                 (155)
#define APP_TURN_APPROACH_PWM            (140)
#define APP_TURN_DECEL_COUNTS            (320L)
#define APP_TURN_APPROACH_COUNTS         (80L)
#define APP_TURN_TOLERANCE_COUNTS        (6L)
#define APP_TURN_SYNC_DEADBAND           (2L)
#define APP_TURN_SYNC_KP_DIV             (4L)
#define APP_TURN_SYNC_LIMIT              (40)

/* Optional mechanical feed-forward trims, in PWM command units. */
#define APP_LEFT_PWM_TRIM                (0)
#define APP_RIGHT_PWM_TRIM               (0)

/* Command slew per real 5 ms tick. */
#define APP_MOTOR_ACCEL_STEP             (4)
#define APP_MOTOR_DECEL_STEP             (12)

/* ==================== Encoder fault protection ==================== */
#define APP_ENCODER_FAULT_MIN_PWM        (180)
#define APP_ENCODER_FAULT_STARTUP_TICKS  (200U) /* 1 second */
#define APP_ENCODER_FAULT_WINDOW_TICKS   (200U) /* 1 second */
#define APP_ENCODER_FAULT_MIN_PROGRESS   (16L)
#define APP_ENCODER_FAULT_LEFT           (1U << 0)
#define APP_ENCODER_FAULT_RIGHT          (1U << 1)

#define APP_ACTIVE_BRAKE_TICKS           (8U)   /* 40 ms */
#define APP_MOTION_TIMEOUT_TICKS         (12000U) /* 60 seconds */
#define APP_CONTROL_ELAPSED_LIMIT_TICKS  (20U)

#endif
