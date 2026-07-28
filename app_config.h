#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ======================== Clock and scheduling ======================== */
#define APP_CPU_CLOCK_HZ                  (32000000UL)
#define APP_SYSTICK_PERIOD_MS             (1UL)
#define APP_CONTROL_PERIOD_MS             (10UL)
#define APP_SERIAL_PERIOD_MS              (250UL)
#define APP_OLED_STATUS_PERIOD_MS         (250UL)
#define APP_SYSTICK_RELOAD                \
    (((APP_CPU_CLOCK_HZ / 1000UL) * APP_SYSTICK_PERIOD_MS) - 1UL)

/* ======================== Buttons ===================================== */
/*
 * LP-MSPM0G3507 S1: PA18 is connected to VCC when pressed and has an
 * external pull-down, therefore released=0 and pressed=1.
 * External S2: PB21 uses the MCU internal pull-up and is shorted to GND
 * when pressed, therefore released=1 and pressed=0.
 */
#define APP_BUTTON_SELECT_ACTIVE_LEVEL    (1U)
#define APP_BUTTON_CONFIRM_ACTIVE_LEVEL   (0U)
#define APP_BUTTON_PRESS_DEBOUNCE_MS      (25UL)
#define APP_BUTTON_RELEASE_DEBOUNCE_MS    (25UL)
#define APP_BUTTON_STUCK_WARNING_MS       (5000UL)

/* ======================== Mechanical calibration ===================== */
#define APP_ENCODER_COUNTS_PER_REV        (1469L)
#define APP_WHEEL_DIAMETER_MM             (66L)
#define APP_WHEEL_TRACK_MM                (114L)
#define APP_PI_X10000                     (31416L)
/* Independent count signs. Change only after a manual direction test. */
#define APP_LEFT_ENCODER_SIGN             (1)
#define APP_RIGHT_ENCODER_SIGN            (-1)

/* ======================== Motor and PWM =============================== */
#define APP_PWM_PERIOD_TICKS              (1600U)
#define APP_MOTOR_COMMAND_MAX             (1000)
#define APP_STRAIGHT_PWM_LIMIT            (400)
#define APP_TURN_PWM_LIMIT                (300)
/*
 * Physical motor installation polarity.
 * A positive logical command must make both wheels move the vehicle forward.
 * The right motor is mirror-mounted, so its TB6612 direction is inverted.
 */
#define APP_LEFT_MOTOR_DIRECTION_SIGN     (1)
#define APP_RIGHT_MOTOR_DIRECTION_SIGN    (-1)
/* Feed-forward trim in per-mille. Keep 1000/1000 until straight tests. */
#define APP_LEFT_PWM_TRIM_PERMILLE        (1000L)
#define APP_RIGHT_PWM_TRIM_PERMILLE       (1000L)

/* ======================== Position profiles =========================== */
#define APP_POSITION_TOLERANCE_COUNTS     (10L)
#define APP_ACTIVE_BRAKE_MS               (40UL)
#define APP_MOTION_TIMEOUT_MS             (60000UL)
#define APP_STRAIGHT_MIN_PWM              (135)
#define APP_STRAIGHT_APPROACH_PWM         (120)
#define APP_STRAIGHT_DECEL_COUNTS         (900L)
#define APP_STRAIGHT_APPROACH_COUNTS      (140L)
#define APP_STRAIGHT_SYNC_DEADBAND        (3L)
#define APP_STRAIGHT_SYNC_KP_NUM          (1L)
#define APP_STRAIGHT_SYNC_KP_DIV          (3L)
#define APP_STRAIGHT_SYNC_LIMIT           (35L)
#define APP_TURN_MIN_PWM                  (155)
#define APP_TURN_APPROACH_PWM             (140)
#define APP_TURN_DECEL_COUNTS             (260L)
#define APP_TURN_APPROACH_COUNTS          (70L)
#define APP_TURN_SYNC_DEADBAND            (3L)
#define APP_TURN_SYNC_KP_NUM              (1L)
#define APP_TURN_SYNC_KP_DIV              (2L)
#define APP_TURN_SYNC_LIMIT               (40L)
#define APP_PWM_ACCEL_PER_CONTROL         (8)
#define APP_PWM_DECEL_PER_CONTROL         (24)

/* ======================== Encoder health protection ================== */
#define APP_ENCODER_STALL_PROTECTION_ENABLE (1U)
#define APP_ENCODER_STALL_GRACE_MS        (1000UL)
#define APP_ENCODER_STALL_WINDOW_MS       (1000UL)
#define APP_ENCODER_STALL_MIN_PWM         (180)
#define APP_ENCODER_STALL_MIN_COUNTS      (20L)
#define APP_ENCODER_FAULT_LEFT            (1U << 0)
#define APP_ENCODER_FAULT_RIGHT           (1U << 1)

/* ======================== UART ======================================== */
#define APP_UART_TX_BUFFER_SIZE           (1024U)

/* ======================== OLED ======================================== */
#define APP_OLED_ENABLE                   (1U)
#define APP_OLED_I2C_DELAY_CYCLES         (32U)
#define APP_OLED_RETRY_PERIOD_MS          (1000UL)
#define APP_OLED_TRANSACTION_RETRIES      (2U)

#endif
