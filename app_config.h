#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ==================== System timing ==================== */
#define APP_CPU_CLOCK_HZ                      (32000000UL)
#define APP_CONTROL_PERIOD_MS                 (5UL)
#define APP_CONTROL_TICK_RELOAD               \
    (((APP_CPU_CLOCK_HZ / 1000UL) * APP_CONTROL_PERIOD_MS) - 1UL)

#define APP_BUTTON_DEBOUNCE_MS                (50UL)
#define APP_BUTTON_DEBOUNCE_TICKS             \
    (APP_BUTTON_DEBOUNCE_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_STATUS_PERIOD_MS             (250UL)
#define APP_OLED_STATUS_PERIOD_TICKS          \
    (APP_OLED_STATUS_PERIOD_MS / APP_CONTROL_PERIOD_MS)

/* ==================== OLED reliability ==================== */
#define APP_OLED_I2C_DELAY_CYCLES             (32UL)
#define APP_OLED_POWER_UP_DELAY_CYCLES        (1600000UL)
#define APP_OLED_TRANSACTION_RETRIES          (2U)
#define APP_OLED_RETRY_PERIOD_MS              (1000UL)
#define APP_OLED_RETRY_PERIOD_TICKS           \
    (APP_OLED_RETRY_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_OLED_PAGES_PER_SERVICE            (1U)

/* ==================== Serial telemetry ==================== */
#define APP_SERIAL_BAUD_RATE                  (115200UL)
#define APP_SERIAL_REPORT_PERIOD_MS           (250UL)
#define APP_SERIAL_REPORT_PERIOD_TICKS        \
    (APP_SERIAL_REPORT_PERIOD_MS / APP_CONTROL_PERIOD_MS)
#define APP_SERIAL_TX_BUFFER_SIZE             (1024U)

/* ==================== PWM ==================== */
#define APP_PWM_PERIOD_TICKS                  (1600U)
#define APP_MOTOR_COMMAND_MAX                 (1000)

/* ==================== Mechanical calibration ==================== */
/* 50 turns: left 73460 counts -> 1469.2 counts/rev.
 * 58 turns: right 85193 counts -> 1468.84 counts/rev. */
#define APP_ENCODER_COUNTS_PER_REV            (1469L)
#define APP_WHEEL_DIAMETER_MM                 (66L)
#define APP_PI_X10000                         (31416L)

/* This is an effective turning track, not necessarily the ruler-measured
 * wheel-centre spacing. Calibrate it with repeated 360-degree turns. */
#define APP_TURN_EFFECTIVE_TRACK_MM           (114L)

/* Optional open-loop trims. Keep both at 1000 until a straight run is
 * repeatable. If the car consistently turns right, increase RIGHT slightly
 * (for example 1005), or reduce LEFT slightly. */
#define APP_STRAIGHT_LEFT_TRIM_PERMILLE       (1000L)
#define APP_STRAIGHT_RIGHT_TRIM_PERMILLE      (1000L)
#define APP_TURN_LEFT_TRIM_PERMILLE           (1000L)
#define APP_TURN_RIGHT_TRIM_PERMILLE          (1000L)

/* ==================== Motion profiles ==================== */
#define APP_POSITION_DEFAULT_MAX_PWM          (300)
#define APP_STRAIGHT_CONTROL_MAX_PWM          (400)
#define APP_TURN_CONTROL_MAX_PWM              (300)

/* Straight profile: slow and conservative near the target. */
#define APP_STRAIGHT_MIN_PWM                  (125)
#define APP_STRAIGHT_APPROACH_PWM             (115)
#define APP_STRAIGHT_DECEL_COUNTS             (900L)
#define APP_STRAIGHT_APPROACH_COUNTS          (180L)
#define APP_STRAIGHT_TOLERANCE_COUNTS         (8L)

/* Turn profile: separate parameters because tyre scrub is much larger. */
#define APP_TURN_MIN_PWM                      (140)
#define APP_TURN_APPROACH_PWM                 (125)
#define APP_TURN_DECEL_COUNTS                 (420L)
#define APP_TURN_APPROACH_COUNTS              (100L)
#define APP_TURN_TOLERANCE_COUNTS             (6L)

/* Feed-forward PWM slew, expressed per real 5 ms tick. */
#define APP_MOTOR_ACCEL_STEP                  (4)
#define APP_MOTOR_DECEL_STEP                  (14)
#define APP_CONTROL_ELAPSED_LIMIT_TICKS       (20U)

/* ==================== Left/right synchronization ==================== */
/* Stable P-only cross coupling. No I/D terms are used because encoder
 * quantisation and delayed main-loop service made them oscillatory. */
#define APP_STRAIGHT_SYNC_DEADBAND_COUNTS     (2L)
#define APP_STRAIGHT_SYNC_KP_DIV              (4L)
#define APP_STRAIGHT_SYNC_LIMIT_PWM           (35L)

#define APP_TURN_SYNC_DEADBAND_COUNTS         (2L)
#define APP_TURN_SYNC_KP_DIV                  (3L)
#define APP_TURN_SYNC_LIMIT_PWM               (40L)

/* Diagnostic speed filter only; it does not drive PWM. Speed units are
 * counts per 5 ms tick multiplied by 16. */
#define APP_SPEED_FILTER_DIV                  (4L)

/* ==================== Encoder/stall protection ==================== */
#define APP_ENCODER_FAULT_LEFT                (1U << 0)
#define APP_ENCODER_FAULT_RIGHT               (1U << 1)
#define APP_ENCODER_FAULT_MIN_PWM             (180)
#define APP_ENCODER_FAULT_STARTUP_GRACE_TICKS (200U) /* 1.0 s */
#define APP_ENCODER_FAULT_WINDOW_TICKS        (200U) /* 1.0 s */
#define APP_ENCODER_FAULT_MIN_PROGRESS_COUNT  (16L)

/* TB6612 short-brake duration: 8 x 5 ms = 40 ms. */
#define APP_ACTIVE_BRAKE_TICKS                (8U)

/* Absolute safety timeout: 12000 x 5 ms = 60 s. */
#define APP_MOTION_TIMEOUT_TICKS              (12000UL)

#endif
