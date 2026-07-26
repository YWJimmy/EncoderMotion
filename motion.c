#include "motion.h"

#include <stddef.h>

#include "app_config.h"
#include "app_tick.h"
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

    int32_t left_speed_integral;
    int32_t right_speed_integral;
    int32_t left_filtered_speed_x16;
    int32_t right_filtered_speed_x16;

    int32_t left_fault_progress;
    int32_t right_fault_progress;
    uint16_t left_fault_ticks;
    uint16_t right_fault_ticks;
    uint16_t encoder_fault_grace_ticks;
    uint8_t encoder_fault_flags;

    uint32_t last_control_tick;
    uint16_t last_control_elapsed_ticks;

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

static int16_t slew_base_pwm(
    int16_t current,
    int16_t target,
    uint16_t elapsed_ticks)
{
    int32_t step_ticks = elapsed_ticks;
    int32_t updated = current;

    if (step_ticks > (int32_t)APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS) {
        step_ticks = APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS;
    }

    if (target > current) {
        if (current == 0) {
            return min_i16(target, APP_POSITION_MIN_PWM);
        }
        updated += APP_MOTOR_ACCEL_STEP * step_ticks;
        if (updated > target) {
            updated = target;
        }
    } else if (target < current) {
        updated -= APP_MOTOR_DECEL_STEP * step_ticks;
        if (updated < target) {
            updated = target;
        }
    }

    return (int16_t)updated;
}

static int32_t pwm_to_speed_x16(int32_t pwm)
{
    return
        (pwm * APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16 + 500L) /
        1000L;
}

static int32_t speed_x16_to_pwm(int32_t speed_x16)
{
    if (speed_x16 <= 0) {
        return 0;
    }

    return
        (speed_x16 * 1000L +
         APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16 / 2L) /
        APP_SPEED_FULL_SCALE_COUNTS_PER_TICK_X16;
}

static int32_t update_filtered_speed(
    int32_t filtered_speed_x16,
    int32_t normalized_delta,
    uint16_t elapsed_ticks)
{
    int32_t sample_x16;
    int32_t filter_steps;

    if (elapsed_ticks == 0U) {
        return filtered_speed_x16;
    }

    sample_x16 =
        (normalized_delta * 16L) / (int32_t)elapsed_ticks;

    filter_steps = elapsed_ticks;
    if (filter_steps > APP_SPEED_FILTER_DIV) {
        filter_steps = APP_SPEED_FILTER_DIV;
    }

    return
        filtered_speed_x16 +
        ((sample_x16 - filtered_speed_x16) * filter_steps) /
        APP_SPEED_FILTER_DIV;
}

static int32_t synchronization_speed_correction_x16(
    int32_t left_travel,
    int32_t right_travel,
    int32_t remaining,
    uint16_t elapsed_ticks)
{
    int32_t error = left_travel - right_travel;
    int32_t derivative;
    int32_t correction;
    int32_t gain_ticks = elapsed_ticks;

    if (gain_ticks <= 0) {
        gain_ticks = 1;
    } else if (gain_ticks > (int32_t)APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS) {
        gain_ticks = APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS;
    }

    derivative =
        (error - gMotion.sync_previous_error) /
        (int32_t)elapsed_ticks;
    gMotion.sync_previous_error = error;

    if (remaining > APP_POSITION_APPROACH_COUNTS) {
        gMotion.sync_integral = clamp_i32(
            gMotion.sync_integral + error * gain_ticks,
            -APP_SYNC_SPEED_INTEGRAL_LIMIT,
            APP_SYNC_SPEED_INTEGRAL_LIMIT);
    } else {
        gMotion.sync_integral /= 2;
    }

    correction =
        APP_SYNC_SPEED_KP_X16 * error +
        gMotion.sync_integral / APP_SYNC_SPEED_KI_DIV +
        APP_SYNC_SPEED_KD_X16 * derivative;

    return clamp_i32(
        correction,
        -APP_SYNC_SPEED_CORRECTION_LIMIT_X16,
        APP_SYNC_SPEED_CORRECTION_LIMIT_X16);
}

static int32_t calculate_speed_pi_correction(
    int32_t target_speed_x16,
    int32_t measured_speed_x16,
    int32_t *integral,
    int32_t feedforward_pwm,
    int16_t max_pwm,
    uint16_t elapsed_ticks)
{
    int32_t error;
    int32_t old_integral;
    int32_t candidate_integral;
    int32_t correction;
    int32_t unclamped_output;

    if ((integral == NULL) || (target_speed_x16 <= 0)) {
        if (integral != NULL) {
            *integral = 0;
        }
        return 0;
    }

    error = target_speed_x16 - measured_speed_x16;
    old_integral = *integral;
    {
        int32_t gain_ticks = elapsed_ticks;
        if (gain_ticks > (int32_t)APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS) {
            gain_ticks = APP_CONTROL_GAIN_ELAPSED_LIMIT_TICKS;
        }
        candidate_integral = clamp_i32(
        old_integral + error * gain_ticks,
        -APP_SPEED_PI_INTEGRAL_LIMIT,
        APP_SPEED_PI_INTEGRAL_LIMIT);
    }

    correction =
        error * APP_SPEED_PI_KP_NUM / APP_SPEED_PI_KP_DIV +
        candidate_integral / APP_SPEED_PI_KI_DIV;
    correction = clamp_i32(
        correction,
        -APP_SPEED_PI_CORRECTION_LIMIT,
        APP_SPEED_PI_CORRECTION_LIMIT);

    unclamped_output = feedforward_pwm + correction;

    /* Conditional integration prevents windup at either actuator limit. */
    if (((unclamped_output > max_pwm) && (error > 0)) ||
        ((unclamped_output < 0) && (error < 0))) {
        candidate_integral = old_integral;
        correction =
            error * APP_SPEED_PI_KP_NUM / APP_SPEED_PI_KP_DIV +
            candidate_integral / APP_SPEED_PI_KI_DIV;
        correction = clamp_i32(
            correction,
            -APP_SPEED_PI_CORRECTION_LIMIT,
            APP_SPEED_PI_CORRECTION_LIMIT);
    }

    *integral = candidate_integral;
    return correction;
}

static int16_t wheel_speed_controller(
    int32_t target_speed_x16,
    int32_t measured_speed_x16,
    int32_t *integral,
    int16_t max_pwm,
    uint16_t elapsed_ticks,
    int16_t *pi_correction_out)
{
    int32_t feedforward_pwm;
    int32_t correction;
    int32_t output;

    feedforward_pwm = speed_x16_to_pwm(target_speed_x16);
    correction = calculate_speed_pi_correction(
        target_speed_x16,
        measured_speed_x16,
        integral,
        feedforward_pwm,
        max_pwm,
        elapsed_ticks);

    output = clamp_i32(
        feedforward_pwm + correction,
        0,
        max_pwm);

    if (pi_correction_out != NULL) {
        *pi_correction_out = (int16_t)correction;
    }

    return (int16_t)output;
}

static bool update_one_encoder_fault_window(
    int32_t normalized_delta,
    int16_t output_pwm,
    int32_t target_speed_x16,
    int32_t *progress,
    uint16_t *ticks,
    uint16_t elapsed_ticks)
{
    bool active;
    uint32_t updated_ticks;

    active =
        (output_pwm >= APP_ENCODER_FAULT_MIN_PWM) &&
        (target_speed_x16 >= APP_ENCODER_FAULT_MIN_TARGET_SPEED_X16);

    if (!active) {
        *progress = 0;
        *ticks = 0U;
        return false;
    }

    *progress += normalized_delta;
    updated_ticks = (uint32_t)(*ticks) + elapsed_ticks;
    if (updated_ticks > UINT16_MAX) {
        updated_ticks = UINT16_MAX;
    }
    *ticks = (uint16_t)updated_ticks;

    if (*ticks < APP_ENCODER_FAULT_WINDOW_TICKS) {
        return false;
    }

    active = *progress < APP_ENCODER_FAULT_MIN_PROGRESS_COUNT;
    *progress = 0;
    *ticks = 0U;
    return active;
}

static bool update_encoder_fault_detection(
    int32_t left_delta,
    int32_t right_delta,
    int32_t left_target_speed_x16,
    int32_t right_target_speed_x16,
    int16_t left_pwm,
    int16_t right_pwm,
    uint16_t elapsed_ticks)
{
    if (gMotion.encoder_fault_grace_ticks != 0U) {
        if (elapsed_ticks >= gMotion.encoder_fault_grace_ticks) {
            gMotion.encoder_fault_grace_ticks = 0U;
        } else {
            gMotion.encoder_fault_grace_ticks =
                (uint16_t)(gMotion.encoder_fault_grace_ticks - elapsed_ticks);
        }

        gMotion.left_fault_progress = 0;
        gMotion.right_fault_progress = 0;
        gMotion.left_fault_ticks = 0U;
        gMotion.right_fault_ticks = 0U;
        return false;
    }

    if (update_one_encoder_fault_window(
            left_delta,
            (int16_t)abs_i32(left_pwm),
            left_target_speed_x16,
            &gMotion.left_fault_progress,
            &gMotion.left_fault_ticks,
            elapsed_ticks)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_LEFT;
    }

    if (update_one_encoder_fault_window(
            right_delta,
            (int16_t)abs_i32(right_pwm),
            right_target_speed_x16,
            &gMotion.right_fault_progress,
            &gMotion.right_fault_ticks,
            elapsed_ticks)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_RIGHT;
    }

    if (gMotion.encoder_fault_flags != 0U) {
        motor_stop();
        gMotion.last_left_pwm = 0;
        gMotion.last_right_pwm = 0;
        gMotion.debug.left_pwm = 0;
        gMotion.debug.right_pwm = 0;
        gMotion.state = MOTION_ENCODER_FAULT;
        return true;
    }

    return false;
}

static void begin_braking(void)
{
    motor_brake();
    gMotion.brake_ticks = 0U;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.left_speed_integral = 0;
    gMotion.right_speed_integral = 0;
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
    (void)encoder_take_left_delta();
    (void)encoder_take_right_delta();

    gMotion.target_left = target_left;
    gMotion.target_right = target_right;
    gMotion.sync_integral = 0;
    gMotion.sync_previous_error = 0;
    gMotion.left_speed_integral = 0;
    gMotion.right_speed_integral = 0;
    gMotion.left_filtered_speed_x16 = 0;
    gMotion.right_filtered_speed_x16 = 0;
    gMotion.left_fault_progress = 0;
    gMotion.right_fault_progress = 0;
    gMotion.left_fault_ticks = 0U;
    gMotion.right_fault_ticks = 0U;
    gMotion.encoder_fault_grace_ticks =
        APP_ENCODER_FAULT_STARTUP_GRACE_TICKS;
    gMotion.encoder_fault_flags = 0U;
    gMotion.last_control_tick = app_tick_now();
    gMotion.last_control_elapsed_ticks = 0U;
    gMotion.max_pwm = max_pwm;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.brake_ticks = 0U;
    gMotion.elapsed_ticks = 0U;
    gMotion.debug = (MotionDebugData){0};
    gMotion.state = state;
    motor_enable();
    return true;
}

static void update_normalized_motion(
    int32_t left_progress,
    int32_t right_progress,
    int32_t left_delta,
    int32_t right_delta,
    int32_t left_direction,
    int32_t right_direction,
    int32_t target,
    uint16_t elapsed_ticks)
{
    int32_t left_travel = left_progress * left_direction;
    int32_t right_travel = right_progress * right_direction;
    int32_t normalized_left_delta = left_delta * left_direction;
    int32_t normalized_right_delta = right_delta * right_direction;
    int32_t average_travel = (left_travel + right_travel) / 2;
    int32_t remaining = target - average_travel;
    int32_t base_target_speed_x16;
    int32_t max_target_speed_x16;
    int32_t sync_speed_x16;
    int32_t left_target_speed_x16;
    int32_t right_target_speed_x16;
    int16_t target_base_pwm;
    int16_t left_magnitude;
    int16_t right_magnitude;
    int16_t left_pi_pwm;
    int16_t right_pi_pwm;

    if (remaining <= APP_POSITION_TOLERANCE_COUNT) {
        begin_braking();
        return;
    }

    target_base_pwm = approach_profile(remaining, gMotion.max_pwm);
    gMotion.current_base_pwm = slew_base_pwm(
        gMotion.current_base_pwm,
        target_base_pwm,
        elapsed_ticks);

    gMotion.left_filtered_speed_x16 = update_filtered_speed(
        gMotion.left_filtered_speed_x16,
        normalized_left_delta,
        elapsed_ticks);
    gMotion.right_filtered_speed_x16 = update_filtered_speed(
        gMotion.right_filtered_speed_x16,
        normalized_right_delta,
        elapsed_ticks);

    base_target_speed_x16 = pwm_to_speed_x16(gMotion.current_base_pwm);
    max_target_speed_x16 = pwm_to_speed_x16(gMotion.max_pwm);
    sync_speed_x16 = synchronization_speed_correction_x16(
        left_travel,
        right_travel,
        remaining,
        elapsed_ticks);

    left_target_speed_x16 = clamp_i32(
        base_target_speed_x16 - sync_speed_x16,
        0,
        max_target_speed_x16);
    right_target_speed_x16 = clamp_i32(
        base_target_speed_x16 + sync_speed_x16,
        0,
        max_target_speed_x16);

    left_magnitude = wheel_speed_controller(
        left_target_speed_x16,
        gMotion.left_filtered_speed_x16,
        &gMotion.left_speed_integral,
        gMotion.max_pwm,
        elapsed_ticks,
        &left_pi_pwm);
    right_magnitude = wheel_speed_controller(
        right_target_speed_x16,
        gMotion.right_filtered_speed_x16,
        &gMotion.right_speed_integral,
        gMotion.max_pwm,
        elapsed_ticks,
        &right_pi_pwm);

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
    gMotion.debug.left_target_speed_x16 = left_target_speed_x16;
    gMotion.debug.right_target_speed_x16 = right_target_speed_x16;
    gMotion.debug.left_measured_speed_x16 =
        gMotion.left_filtered_speed_x16;
    gMotion.debug.right_measured_speed_x16 =
        gMotion.right_filtered_speed_x16;
    gMotion.debug.synchronization_speed_x16 = sync_speed_x16;
    gMotion.debug.left_speed_pi_pwm = left_pi_pwm;
    gMotion.debug.right_speed_pi_pwm = right_pi_pwm;
    gMotion.debug.encoder_fault_flags = gMotion.encoder_fault_flags;
    gMotion.debug.control_elapsed_ticks = elapsed_ticks;

    (void)update_encoder_fault_detection(
        normalized_left_delta,
        normalized_right_delta,
        left_target_speed_x16,
        right_target_speed_x16,
        left_magnitude,
        right_magnitude,
        elapsed_ticks);

    gMotion.debug.encoder_fault_flags = gMotion.encoder_fault_flags;
    gMotion.debug.encoder_fault_grace_ticks =
        gMotion.encoder_fault_grace_ticks;
}

void motion_init(void)
{
    gMotion = (MotionController){0};
    gMotion.state = MOTION_IDLE;
    gMotion.max_pwm = APP_POSITION_DEFAULT_MAX_PWM;
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
    uint32_t now_tick;
    uint32_t elapsed_ticks_u32;
    uint16_t elapsed_ticks;
    int32_t left_count;
    int32_t right_count;
    int32_t left_progress;
    int32_t right_progress;
    int32_t left_delta;
    int32_t right_delta;

    if (!motion_is_busy()) {
        return;
    }

    now_tick = app_tick_now();
    elapsed_ticks_u32 = now_tick - gMotion.last_control_tick;

    /* A queued/catch-up call at the same real SysTick must not advance the
     * speed PI, fault detector, brake timer or timeout. */
    if (elapsed_ticks_u32 == 0U) {
        return;
    }

    gMotion.last_control_tick = now_tick;
    if (elapsed_ticks_u32 > UINT16_MAX) {
        elapsed_ticks = UINT16_MAX;
    } else {
        elapsed_ticks = (uint16_t)elapsed_ticks_u32;
    }
    gMotion.last_control_elapsed_ticks = elapsed_ticks;
    gMotion.debug.control_elapsed_ticks = elapsed_ticks;

    if (gMotion.state == MOTION_BRAKING) {
        uint32_t updated_brake_ticks =
            (uint32_t)gMotion.brake_ticks + elapsed_ticks;

        if (updated_brake_ticks < APP_ACTIVE_BRAKE_TICKS) {
            gMotion.brake_ticks = (uint16_t)updated_brake_ticks;
        } else {
            motor_stop();
            gMotion.brake_ticks = APP_ACTIVE_BRAKE_TICKS;
            gMotion.state = MOTION_DONE;
        }
        return;
    }

    encoder_get_counts(&left_count, &right_count);
    left_delta = encoder_take_left_delta();
    right_delta = encoder_take_right_delta();
    left_progress = left_count - gMotion.start_left;
    right_progress = right_count - gMotion.start_right;

    if (gMotion.state == MOTION_RUNNING_DISTANCE) {
        int32_t direction = gMotion.target_left >= 0 ? 1 : -1;
        update_normalized_motion(
            left_progress,
            right_progress,
            left_delta,
            right_delta,
            direction,
            direction,
            abs_i32(gMotion.target_left),
            elapsed_ticks);
    } else if (gMotion.state == MOTION_RUNNING_TURN) {
        int32_t left_direction = gMotion.target_left >= 0 ? 1 : -1;
        int32_t right_direction = gMotion.target_right >= 0 ? 1 : -1;
        update_normalized_motion(
            left_progress,
            right_progress,
            left_delta,
            right_delta,
            left_direction,
            right_direction,
            abs_i32(gMotion.target_left),
            elapsed_ticks);
    }

    if (!motion_is_busy()) {
        return;
    }

    gMotion.elapsed_ticks += elapsed_ticks_u32;
    if ((gMotion.elapsed_ticks > APP_MOTION_TIMEOUT_TICKS) &&
        (gMotion.state != MOTION_BRAKING)) {
        motor_stop();
        gMotion.last_left_pwm = 0;
        gMotion.last_right_pwm = 0;
        gMotion.debug.left_pwm = 0;
        gMotion.debug.right_pwm = 0;
        gMotion.state = MOTION_TIMEOUT;
    }
}

void motion_abort(void)
{
    motor_stop();
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.left_speed_integral = 0;
    gMotion.right_speed_integral = 0;
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
