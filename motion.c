#include "motion.h"

#include <stddef.h>

#include "app_config.h"
#include "app_tick.h"
#include "encoder.h"
#include "motor.h"

typedef enum {
    PROFILE_STRAIGHT = 0,
    PROFILE_TURN
} MotionProfile;

typedef struct {
    MotionState state;
    MotionProfile profile;
    int32_t start_left;
    int32_t start_right;
    int32_t target_left;
    int32_t target_right;
    int16_t requested_max_pwm;
    int16_t current_base_pwm;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    uint32_t last_update_tick;
    uint32_t elapsed_ticks;
    uint32_t brake_ticks;
    uint32_t fault_grace_ticks;
    uint32_t left_fault_ticks;
    uint32_t right_fault_ticks;
    int32_t left_fault_progress;
    int32_t right_fault_progress;
    uint8_t encoder_fault_flags;
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

static int32_t distance_mm_to_counts(int32_t distance_mm)
{
    int32_t sign = distance_mm >= 0 ? 1 : -1;
    int64_t numerator =
        (int64_t)abs_i32(distance_mm) *
        APP_ENCODER_COUNTS_PER_REV * 10000LL;
    int64_t denominator =
        (int64_t)APP_WHEEL_DIAMETER_MM * APP_PI_X10000;

    return sign * (int32_t)((numerator + denominator / 2LL) / denominator);
}

static int32_t turn_deg_to_counts(int32_t angle_deg)
{
    int64_t numerator =
        (int64_t)abs_i32(angle_deg) *
        APP_TURN_EFFECTIVE_TRACK_MM *
        APP_ENCODER_COUNTS_PER_REV;
    int64_t denominator = 360LL * APP_WHEEL_DIAMETER_MM;

    return (int32_t)((numerator + denominator / 2LL) / denominator);
}

static int16_t slew_pwm(
    int16_t current,
    int16_t target,
    uint32_t elapsed_ticks)
{
    int32_t next = current;
    int32_t gain_ticks = (int32_t)clamp_i32(
        (int32_t)elapsed_ticks, 1, APP_CONTROL_ELAPSED_LIMIT_TICKS);

    if (target > current) {
        if (current == 0) {
            next = target;
        } else {
            next += APP_MOTOR_ACCEL_STEP * gain_ticks;
            if (next > target) {
                next = target;
            }
        }
    } else if (target < current) {
        next -= APP_MOTOR_DECEL_STEP * gain_ticks;
        if (next < target) {
            next = target;
        }
    }

    return (int16_t)next;
}

static int16_t profile_pwm(
    int32_t remaining,
    int16_t max_pwm,
    int16_t minimum_pwm,
    int16_t approach_pwm,
    int32_t decel_counts,
    int32_t approach_counts,
    int32_t tolerance)
{
    int32_t pwm;

    if (remaining <= tolerance) {
        return 0;
    }
    if (remaining <= approach_counts) {
        int32_t span = approach_counts - tolerance;
        pwm = approach_pwm;
        if (span > 0) {
            pwm +=
                (remaining - tolerance) *
                (minimum_pwm - approach_pwm) / span;
        }
        return (int16_t)clamp_i32(pwm, approach_pwm, minimum_pwm);
    }
    if (remaining < decel_counts) {
        int32_t span = decel_counts - approach_counts;
        pwm = minimum_pwm;
        if (span > 0) {
            pwm +=
                (remaining - approach_counts) *
                (max_pwm - minimum_pwm) / span;
        }
        return (int16_t)clamp_i32(pwm, minimum_pwm, max_pwm);
    }
    return max_pwm;
}

static int16_t synchronization_pwm(
    int32_t left_travel,
    int32_t right_travel,
    int32_t deadband,
    int32_t kp_div,
    int16_t limit)
{
    int32_t error = left_travel - right_travel;
    int32_t correction;

    if (abs_i32(error) <= deadband) {
        return 0;
    }
    correction = error / kp_div;
    return (int16_t)clamp_i32(correction, -limit, limit);
}

static void begin_braking(void)
{
    motor_brake();
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.brake_ticks = 0U;
    gMotion.state = MOTION_BRAKING;
}

static bool start_targets(
    MotionState state,
    MotionProfile profile,
    int32_t left_target,
    int32_t right_target,
    int16_t max_pwm)
{
    EncoderSnapshot snapshot;
    int16_t control_limit = profile == PROFILE_TURN ?
        APP_TURN_CONTROL_MAX_PWM : APP_STRAIGHT_CONTROL_MAX_PWM;
    int16_t minimum_pwm = profile == PROFILE_TURN ?
        APP_TURN_MIN_PWM : APP_STRAIGHT_MIN_PWM;

    if (motion_is_busy()) {
        return false;
    }

    max_pwm = (int16_t)clamp_i32(max_pwm, minimum_pwm, control_limit);
    encoder_take_snapshot(&snapshot);

    gMotion.state = state;
    gMotion.profile = profile;
    gMotion.start_left = snapshot.left_count;
    gMotion.start_right = snapshot.right_count;
    gMotion.target_left = left_target;
    gMotion.target_right = right_target;
    gMotion.requested_max_pwm = max_pwm;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.last_update_tick = app_tick_now();
    gMotion.elapsed_ticks = 0U;
    gMotion.brake_ticks = 0U;
    gMotion.fault_grace_ticks = APP_ENCODER_FAULT_STARTUP_TICKS;
    gMotion.left_fault_ticks = 0U;
    gMotion.right_fault_ticks = 0U;
    gMotion.left_fault_progress = 0;
    gMotion.right_fault_progress = 0;
    gMotion.encoder_fault_flags = 0U;
    gMotion.debug = (MotionDebugData){0};

    motor_enable();
    return true;
}

static bool update_fault_side(
    int32_t delta,
    int16_t pwm,
    uint32_t elapsed_ticks,
    uint32_t *window_ticks,
    int32_t *window_progress)
{
    if (abs_i32(pwm) < APP_ENCODER_FAULT_MIN_PWM) {
        *window_ticks = 0U;
        *window_progress = 0;
        return false;
    }

    *window_ticks += elapsed_ticks;
    *window_progress += abs_i32(delta);

    if (*window_ticks < APP_ENCODER_FAULT_WINDOW_TICKS) {
        return false;
    }

    {
        bool fault = *window_progress < APP_ENCODER_FAULT_MIN_PROGRESS;
        *window_ticks = 0U;
        *window_progress = 0;
        return fault;
    }
}

static void update_running(
    const EncoderSnapshot *snapshot,
    uint32_t elapsed_ticks)
{
    int32_t left_direction = gMotion.target_left >= 0 ? 1 : -1;
    int32_t right_direction = gMotion.target_right >= 0 ? 1 : -1;
    int32_t left_progress = snapshot->left_count - gMotion.start_left;
    int32_t right_progress = snapshot->right_count - gMotion.start_right;
    int32_t left_travel = left_progress * left_direction;
    int32_t right_travel = right_progress * right_direction;
    int32_t target = abs_i32(gMotion.target_left);
    int32_t left_remaining = target - left_travel;
    int32_t right_remaining = target - right_travel;
    int32_t controlling_remaining =
        left_remaining > right_remaining ? left_remaining : right_remaining;
    int32_t tolerance;
    int16_t target_base_pwm;
    int16_t sync_pwm;
    int16_t left_pwm;
    int16_t right_pwm;

    if (gMotion.profile == PROFILE_TURN) {
        tolerance = APP_TURN_TOLERANCE_COUNTS;
        target_base_pwm = profile_pwm(
            controlling_remaining,
            gMotion.requested_max_pwm,
            APP_TURN_MIN_PWM,
            APP_TURN_APPROACH_PWM,
            APP_TURN_DECEL_COUNTS,
            APP_TURN_APPROACH_COUNTS,
            tolerance);
        sync_pwm = synchronization_pwm(
            left_travel,
            right_travel,
            APP_TURN_SYNC_DEADBAND,
            APP_TURN_SYNC_KP_DIV,
            APP_TURN_SYNC_LIMIT);
    } else {
        tolerance = APP_STRAIGHT_TOLERANCE_COUNTS;
        target_base_pwm = profile_pwm(
            controlling_remaining,
            gMotion.requested_max_pwm,
            APP_STRAIGHT_MIN_PWM,
            APP_STRAIGHT_APPROACH_PWM,
            APP_STRAIGHT_DECEL_COUNTS,
            APP_STRAIGHT_APPROACH_COUNTS,
            tolerance);
        sync_pwm = synchronization_pwm(
            left_travel,
            right_travel,
            APP_STRAIGHT_SYNC_DEADBAND,
            APP_STRAIGHT_SYNC_KP_DIV,
            APP_STRAIGHT_SYNC_LIMIT);
    }

    if ((left_remaining <= tolerance) && (right_remaining <= tolerance)) {
        begin_braking();
        return;
    }

    gMotion.current_base_pwm = slew_pwm(
        gMotion.current_base_pwm,
        target_base_pwm,
        elapsed_ticks);

    left_pwm = (int16_t)(gMotion.current_base_pwm - sync_pwm + APP_LEFT_PWM_TRIM);
    right_pwm = (int16_t)(gMotion.current_base_pwm + sync_pwm + APP_RIGHT_PWM_TRIM);
    left_pwm = (int16_t)clamp_i32(left_pwm, 0, gMotion.requested_max_pwm);
    right_pwm = (int16_t)clamp_i32(right_pwm, 0, gMotion.requested_max_pwm);

    /* Do not drive an already-arrived wheel farther past the target. */
    if (left_remaining <= tolerance) {
        left_pwm = 0;
    }
    if (right_remaining <= tolerance) {
        right_pwm = 0;
    }

    gMotion.last_left_pwm = (int16_t)(left_direction * left_pwm);
    gMotion.last_right_pwm = (int16_t)(right_direction * right_pwm);
    motor_set(gMotion.last_left_pwm, gMotion.last_right_pwm);

    if (gMotion.fault_grace_ticks > elapsed_ticks) {
        gMotion.fault_grace_ticks -= elapsed_ticks;
    } else {
        gMotion.fault_grace_ticks = 0U;
        if (update_fault_side(
                snapshot->left_delta * left_direction,
                gMotion.last_left_pwm,
                elapsed_ticks,
                &gMotion.left_fault_ticks,
                &gMotion.left_fault_progress)) {
            gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_LEFT;
        }
        if (update_fault_side(
                snapshot->right_delta * right_direction,
                gMotion.last_right_pwm,
                elapsed_ticks,
                &gMotion.right_fault_ticks,
                &gMotion.right_fault_progress)) {
            gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_RIGHT;
        }
    }

    if (gMotion.encoder_fault_flags != 0U) {
        motor_stop();
        gMotion.last_left_pwm = 0;
        gMotion.last_right_pwm = 0;
        gMotion.state = MOTION_ENCODER_FAULT;
    }

    gMotion.debug.left_progress = left_travel;
    gMotion.debug.right_progress = right_travel;
    gMotion.debug.target_count = target;
    gMotion.debug.left_remaining = left_remaining;
    gMotion.debug.right_remaining = right_remaining;
    gMotion.debug.base_pwm = gMotion.current_base_pwm;
    gMotion.debug.synchronization_pwm = sync_pwm;
    gMotion.debug.left_pwm = gMotion.last_left_pwm;
    gMotion.debug.right_pwm = gMotion.last_right_pwm;
    gMotion.debug.left_delta = snapshot->left_delta;
    gMotion.debug.right_delta = snapshot->right_delta;
    gMotion.debug.elapsed_control_ticks = elapsed_ticks;
    gMotion.debug.encoder_fault_flags = gMotion.encoder_fault_flags;
}

void motion_init(void)
{
    gMotion = (MotionController){0};
    gMotion.state = MOTION_IDLE;
}

bool motion_start_distance_mm(int32_t distance_mm, int16_t max_pwm)
{
    int32_t target;

    if (distance_mm == 0) {
        return false;
    }
    target = distance_mm_to_counts(distance_mm);
    return start_targets(
        MOTION_RUNNING_DISTANCE,
        PROFILE_STRAIGHT,
        target,
        target,
        max_pwm);
}

bool motion_start_turn_deg(int32_t angle_deg, int16_t max_pwm)
{
    int32_t target;

    if (angle_deg == 0) {
        return false;
    }
    target = turn_deg_to_counts(angle_deg);
    if (angle_deg > 0) {
        return start_targets(
            MOTION_RUNNING_TURN,
            PROFILE_TURN,
            -target,
            target,
            max_pwm);
    }
    return start_targets(
        MOTION_RUNNING_TURN,
        PROFILE_TURN,
        target,
        -target,
        max_pwm);
}

void motion_update(void)
{
    uint32_t now_tick;
    uint32_t elapsed_ticks;
    EncoderSnapshot snapshot;

    if (!motion_is_busy()) {
        return;
    }

    now_tick = app_tick_now();
    elapsed_ticks = now_tick - gMotion.last_update_tick;
    if (elapsed_ticks == 0U) {
        return;
    }
    gMotion.last_update_tick = now_tick;
    gMotion.elapsed_ticks += elapsed_ticks;

    if (gMotion.state == MOTION_BRAKING) {
        gMotion.brake_ticks += elapsed_ticks;
        if (gMotion.brake_ticks >= APP_ACTIVE_BRAKE_TICKS) {
            motor_stop();
            gMotion.state = MOTION_DONE;
        }
        return;
    }

    encoder_take_snapshot(&snapshot);
    update_running(&snapshot, elapsed_ticks);

    if ((gMotion.state == MOTION_RUNNING_DISTANCE) ||
        (gMotion.state == MOTION_RUNNING_TURN)) {
        if (gMotion.elapsed_ticks >= APP_MOTION_TIMEOUT_TICKS) {
            motor_stop();
            gMotion.last_left_pwm = 0;
            gMotion.last_right_pwm = 0;
            gMotion.state = MOTION_TIMEOUT;
        }
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
