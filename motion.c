#include "motion.h"

#include <stddef.h>

#include "app_config.h"
#include "encoder.h"
#include "motor.h"

typedef struct {
    MotionState state;
    int32_t start_left;
    int32_t start_right;
    int32_t target_left;
    int32_t target_right;
    int32_t sync_integral;
    int32_t sync_previous_error;
    int16_t max_pwm;
    int16_t current_base_pwm;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    uint16_t brake_ticks;
    uint32_t elapsed_ticks;
    MotionDebugData debug;
} MotionController;

static MotionController gMotion;

static int32_t abs_i32(int32_t value)
{
    return value >= 0 ? value : -value;
}

static int32_t clamp_i32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t min_i16(int16_t a, int16_t b)
{
    return a < b ? a : b;
}

static int32_t distance_mm_to_counts(int32_t distance_mm)
{
    int64_t numerator;
    int64_t denominator;
    int32_t sign = distance_mm >= 0 ? 1 : -1;
    int32_t distance_abs = abs_i32(distance_mm);

    numerator =
        (int64_t)distance_abs *
        APP_ENCODER_COUNTS_PER_REV *
        10000LL;
    denominator =
        (int64_t)APP_WHEEL_DIAMETER_MM * APP_PI_X10000;

    return sign * (int32_t)(
        (numerator + denominator / 2LL) / denominator);
}

static int32_t turn_deg_to_counts(int32_t angle_deg)
{
    int64_t numerator =
        (int64_t)abs_i32(angle_deg) *
        APP_WHEEL_TRACK_MM *
        APP_ENCODER_COUNTS_PER_REV;
    int64_t denominator =
        360LL * APP_WHEEL_DIAMETER_MM;

    return (int32_t)(
        (numerator + denominator / 2LL) / denominator);
}

static int16_t approach_profile(int32_t remaining, int16_t max_pwm)
{
    int32_t magnitude;
    int32_t span;

    if (remaining <= APP_POSITION_TOLERANCE_COUNT) {
        return 0;
    }

    if (remaining <= APP_POSITION_APPROACH_COUNTS) {
        span =
            APP_POSITION_APPROACH_COUNTS -
            APP_POSITION_TOLERANCE_COUNT;
        magnitude = APP_POSITION_APPROACH_PWM;
        if (span > 0) {
            magnitude +=
                ((remaining - APP_POSITION_TOLERANCE_COUNT) *
                 (APP_POSITION_MIN_PWM - APP_POSITION_APPROACH_PWM)) /
                span;
        }
        return (int16_t)clamp_i32(
            magnitude,
            APP_POSITION_APPROACH_PWM,
            APP_POSITION_MIN_PWM);
    }

    if (remaining < APP_POSITION_DECEL_COUNTS) {
        span =
            APP_POSITION_DECEL_COUNTS -
            APP_POSITION_APPROACH_COUNTS;
        magnitude = APP_POSITION_MIN_PWM;
        if (span > 0) {
            magnitude +=
                ((remaining - APP_POSITION_APPROACH_COUNTS) *
                 (max_pwm - APP_POSITION_MIN_PWM)) /
                span;
        }
        return (int16_t)clamp_i32(
            magnitude,
            APP_POSITION_MIN_PWM,
            max_pwm);
    }

    return max_pwm;
}

static int16_t slew_base_pwm(int16_t current, int16_t target)
{
    if (target > current) {
        if (current == 0) {
            return min_i16(target, APP_POSITION_MIN_PWM);
        }
        current = (int16_t)(current + APP_MOTOR_ACCEL_STEP);
        if (current > target) {
            current = target;
        }
    } else if (target < current) {
        current = (int16_t)(current - APP_MOTOR_DECEL_STEP);
        if (current < target) {
            current = target;
        }
    }

    return current;
}

static int32_t synchronization_correction(
    int32_t left_travel,
    int32_t right_travel,
    int32_t remaining)
{
    int32_t error = left_travel - right_travel;
    int32_t derivative = error - gMotion.sync_previous_error;
    int32_t correction;

    gMotion.sync_previous_error = error;

    if (remaining > APP_POSITION_APPROACH_COUNTS) {
        gMotion.sync_integral = clamp_i32(
            gMotion.sync_integral + error,
            -APP_SYNC_INTEGRAL_LIMIT,
            APP_SYNC_INTEGRAL_LIMIT);
    } else {
        gMotion.sync_integral /= 2;
    }

    correction =
        APP_SYNC_KP * error +
        gMotion.sync_integral / APP_SYNC_KI_DIV +
        APP_SYNC_KD * derivative;

    return clamp_i32(
        correction,
        -APP_SYNC_CORRECTION_LIMIT,
        APP_SYNC_CORRECTION_LIMIT);
}

static void begin_braking(void)
{
    motor_brake();
    gMotion.brake_ticks = 0U;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.state = MOTION_BRAKING;
}

static bool start_targets(
    MotionState state,
    int32_t target_left,
    int32_t target_right,
    int16_t max_pwm)
{
    if (motion_is_busy()) {
        return false;
    }

    max_pwm = (int16_t)clamp_i32(
        max_pwm,
        APP_POSITION_MIN_PWM,
        APP_MOTOR_COMMAND_MAX);

    encoder_get_counts(&gMotion.start_left, &gMotion.start_right);
    gMotion.target_left = target_left;
    gMotion.target_right = target_right;
    gMotion.sync_integral = 0;
    gMotion.sync_previous_error = 0;
    gMotion.max_pwm = max_pwm;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.brake_ticks = 0U;
    gMotion.elapsed_ticks = 0U;
    gMotion.state = state;
    motor_enable();
    return true;
}

static void update_normalized_motion(
    int32_t left_progress,
    int32_t right_progress,
    int32_t left_direction,
    int32_t right_direction,
    int32_t target)
{
    int32_t left_travel = left_progress * left_direction;
    int32_t right_travel = right_progress * right_direction;
    int32_t average_travel = (left_travel + right_travel) / 2;
    int32_t remaining = target - average_travel;
    int32_t correction;
    int32_t left_magnitude;
    int32_t right_magnitude;
    int16_t target_base_pwm;

    if (remaining <= APP_POSITION_TOLERANCE_COUNT) {
        begin_braking();
        return;
    }

    target_base_pwm = approach_profile(remaining, gMotion.max_pwm);
    gMotion.current_base_pwm = slew_base_pwm(
        gMotion.current_base_pwm,
        target_base_pwm);

    correction = synchronization_correction(
        left_travel,
        right_travel,
        remaining);

    left_magnitude = clamp_i32(
        (int32_t)gMotion.current_base_pwm - correction,
        0,
        gMotion.max_pwm);
    right_magnitude = clamp_i32(
        (int32_t)gMotion.current_base_pwm + correction,
        0,
        gMotion.max_pwm);

    gMotion.last_left_pwm =
        (int16_t)(left_direction * left_magnitude);
    gMotion.last_right_pwm =
        (int16_t)(right_direction * right_magnitude);

    motor_set(gMotion.last_left_pwm, gMotion.last_right_pwm);

    gMotion.debug.left_progress = left_travel;
    gMotion.debug.right_progress = right_travel;
    gMotion.debug.target_count = target;
    gMotion.debug.remaining_count = remaining;
    gMotion.debug.base_pwm = gMotion.current_base_pwm;
    gMotion.debug.left_pwm = gMotion.last_left_pwm;
    gMotion.debug.right_pwm = gMotion.last_right_pwm;
}

void motion_init(void)
{
    gMotion.state = MOTION_IDLE;
    gMotion.start_left = 0;
    gMotion.start_right = 0;
    gMotion.target_left = 0;
    gMotion.target_right = 0;
    gMotion.sync_integral = 0;
    gMotion.sync_previous_error = 0;
    gMotion.max_pwm = APP_POSITION_DEFAULT_MAX_PWM;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.brake_ticks = 0U;
    gMotion.elapsed_ticks = 0U;
    gMotion.debug = (MotionDebugData){0};
}

bool motion_start_distance_mm(int32_t distance_mm, int16_t max_pwm)
{
    int32_t target_count;

    if (distance_mm == 0) {
        return false;
    }

    target_count = distance_mm_to_counts(distance_mm);
    return start_targets(
        MOTION_RUNNING_DISTANCE,
        target_count,
        target_count,
        max_pwm);
}

bool motion_start_turn_deg(int32_t angle_deg, int16_t max_pwm)
{
    int32_t target_count;

    if (angle_deg == 0) {
        return false;
    }

    target_count = turn_deg_to_counts(angle_deg);
    if (angle_deg > 0) {
        return start_targets(
            MOTION_RUNNING_TURN,
            -target_count,
            target_count,
            max_pwm);
    }

    return start_targets(
        MOTION_RUNNING_TURN,
        target_count,
        -target_count,
        max_pwm);
}

void motion_update(void)
{
    int32_t left_count;
    int32_t right_count;
    int32_t left_progress;
    int32_t right_progress;

    if (!motion_is_busy()) {
        return;
    }

    if (gMotion.state == MOTION_BRAKING) {
        if (gMotion.brake_ticks < APP_ACTIVE_BRAKE_TICKS) {
            gMotion.brake_ticks++;
        } else {
            motor_stop();
            gMotion.state = MOTION_DONE;
        }
        return;
    }

    encoder_get_counts(&left_count, &right_count);
    left_progress = left_count - gMotion.start_left;
    right_progress = right_count - gMotion.start_right;

    if (gMotion.state == MOTION_RUNNING_DISTANCE) {
        int32_t direction = gMotion.target_left >= 0 ? 1 : -1;
        update_normalized_motion(
            left_progress,
            right_progress,
            direction,
            direction,
            abs_i32(gMotion.target_left));
    } else {
        int32_t left_direction = gMotion.target_left >= 0 ? 1 : -1;
        int32_t right_direction = gMotion.target_right >= 0 ? 1 : -1;
        update_normalized_motion(
            left_progress,
            right_progress,
            left_direction,
            right_direction,
            abs_i32(gMotion.target_left));
    }

    gMotion.elapsed_ticks++;
    if ((gMotion.elapsed_ticks > APP_MOTION_TIMEOUT_TICKS) &&
        (gMotion.state != MOTION_BRAKING)) {
        motor_stop();
        gMotion.state = MOTION_TIMEOUT;
    }
}

void motion_abort(void)
{
    motor_stop();
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.state = MOTION_IDLE;
}

bool motion_is_busy(void)
{
    return
        gMotion.state == MOTION_RUNNING_DISTANCE ||
        gMotion.state == MOTION_RUNNING_TURN ||
        gMotion.state == MOTION_BRAKING;
}

MotionState motion_get_state(void)
{
    return gMotion.state;
}

void motion_get_debug(MotionDebugData *debug_data)
{
    if (debug_data != NULL) {
        *debug_data = gMotion.debug;
    }
}
