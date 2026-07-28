#include "motion.h"

#include <stddef.h>

#include "app_config.h"
#include "encoder.h"
#include "motor.h"

typedef struct {
    MotionState state;
    int32_t start_left;
    int32_t start_right;
    int32_t target_counts;
    int8_t left_direction;
    int8_t right_direction;
    int16_t requested_max_pwm;
    int16_t current_base_pwm;
    int16_t current_left_pwm;
    int16_t current_right_pwm;
    uint32_t start_time_ms;
    uint32_t brake_start_ms;
    uint32_t last_update_ms;
    uint32_t stall_window_start_ms;
    int32_t stall_window_left_start;
    int32_t stall_window_right_start;
    uint8_t encoder_fault_flags;
    MotionDebug debug;
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
    int64_t numerator =
        (int64_t)abs_i32(distance_mm) *
        APP_ENCODER_COUNTS_PER_REV *
        10000LL;
    int64_t denominator =
        (int64_t)APP_WHEEL_DIAMETER_MM * APP_PI_X10000;

    return (int32_t)((numerator + denominator / 2LL) / denominator);
}

static int32_t turn_deg_to_counts(int32_t angle_deg)
{
    int64_t numerator =
        (int64_t)abs_i32(angle_deg) *
        APP_WHEEL_TRACK_MM *
        APP_ENCODER_COUNTS_PER_REV;
    int64_t denominator =
        360LL * APP_WHEEL_DIAMETER_MM;

    return (int32_t)((numerator + denominator / 2LL) / denominator);
}

static int16_t slew_pwm(int16_t current, int16_t target)
{
    if (target > current) {
        current = (int16_t)(current + APP_PWM_ACCEL_PER_CONTROL);
        if (current > target) {
            current = target;
        }
    } else if (target < current) {
        current = (int16_t)(current - APP_PWM_DECEL_PER_CONTROL);
        if (current < target) {
            current = target;
        }
    }
    return current;
}

static int16_t profile_pwm(
    int32_t remaining,
    int16_t max_pwm,
    int16_t minimum_pwm,
    int16_t approach_pwm,
    int32_t decel_counts,
    int32_t approach_counts)
{
    int32_t pwm;

    if (remaining <= APP_POSITION_TOLERANCE_COUNTS) {
        return 0;
    }

    if (remaining <= approach_counts) {
        int32_t span = approach_counts - APP_POSITION_TOLERANCE_COUNTS;
        pwm = approach_pwm;
        if (span > 0) {
            pwm +=
                (remaining - APP_POSITION_TOLERANCE_COUNTS) *
                (minimum_pwm - approach_pwm) /
                span;
        }
        return (int16_t)clamp_i32(pwm, approach_pwm, minimum_pwm);
    }

    if (remaining < decel_counts) {
        int32_t span = decel_counts - approach_counts;
        pwm = minimum_pwm;
        if (span > 0) {
            pwm +=
                (remaining - approach_counts) *
                (max_pwm - minimum_pwm) /
                span;
        }
        return (int16_t)clamp_i32(pwm, minimum_pwm, max_pwm);
    }

    return max_pwm;
}

static int16_t apply_trim(int16_t pwm, int32_t trim_permille)
{
    return (int16_t)clamp_i32(
        ((int32_t)pwm * trim_permille + 500L) / 1000L,
        0,
        APP_MOTOR_COMMAND_MAX);
}

static int16_t synchronization_correction(
    int32_t left_progress,
    int32_t right_progress,
    bool turning)
{
    int32_t error = left_progress - right_progress;
    int32_t deadband = turning ?
        APP_TURN_SYNC_DEADBAND : APP_STRAIGHT_SYNC_DEADBAND;
    int32_t numerator = turning ?
        APP_TURN_SYNC_KP_NUM : APP_STRAIGHT_SYNC_KP_NUM;
    int32_t divisor = turning ?
        APP_TURN_SYNC_KP_DIV : APP_STRAIGHT_SYNC_KP_DIV;
    int32_t limit = turning ?
        APP_TURN_SYNC_LIMIT : APP_STRAIGHT_SYNC_LIMIT;

    if (abs_i32(error) <= deadband) {
        return 0;
    }

    return (int16_t)clamp_i32(
        error * numerator / divisor,
        -limit,
        limit);
}

static void enter_braking(uint32_t now_ms)
{
    motor_brake();
    gMotion.current_base_pwm = 0;
    gMotion.current_left_pwm = 0;
    gMotion.current_right_pwm = 0;
    gMotion.brake_start_ms = now_ms;
    gMotion.state = MOTION_BRAKING;
}

static bool start_motion(
    MotionState state,
    int32_t target_counts,
    int8_t left_direction,
    int8_t right_direction,
    int16_t max_pwm)
{
    int32_t left_count;
    int32_t right_count;
    int16_t hard_limit;

    if (motion_is_busy() || (target_counts <= 0)) {
        return false;
    }

    hard_limit = state == MOTION_TURN ?
        APP_TURN_PWM_LIMIT : APP_STRAIGHT_PWM_LIMIT;
    max_pwm = (int16_t)clamp_i32(max_pwm, 0, hard_limit);
    if (max_pwm == 0) {
        return false;
    }

    encoder_get_counts(&left_count, &right_count);
    gMotion = (MotionController){0};
    gMotion.state = state;
    gMotion.start_left = left_count;
    gMotion.start_right = right_count;
    gMotion.target_counts = target_counts;
    gMotion.left_direction = left_direction;
    gMotion.right_direction = right_direction;
    gMotion.requested_max_pwm = max_pwm;
    gMotion.debug.state = state;
    motor_enable();
    return true;
}

void motion_init(void)
{
    gMotion = (MotionController){0};
    gMotion.state = MOTION_IDLE;
    gMotion.debug.state = MOTION_IDLE;
}

bool motion_start_distance_mm(int32_t distance_mm, int16_t max_pwm)
{
    int8_t direction;

    if (distance_mm == 0) {
        return false;
    }

    direction = distance_mm > 0 ? 1 : -1;
    return start_motion(
        MOTION_DISTANCE,
        distance_mm_to_counts(distance_mm),
        direction,
        direction,
        max_pwm);
}

bool motion_start_turn_deg(int32_t angle_deg, int16_t max_pwm)
{
    if (angle_deg == 0) {
        return false;
    }

    return start_motion(
        MOTION_TURN,
        turn_deg_to_counts(angle_deg),
        angle_deg > 0 ? -1 : 1,
        angle_deg > 0 ? 1 : -1,
        max_pwm);
}

static void update_stall_detection(
    uint32_t now_ms,
    int32_t left_progress,
    int32_t right_progress,
    int16_t left_pwm,
    int16_t right_pwm)
{
#if APP_ENCODER_STALL_PROTECTION_ENABLE
    if ((uint32_t)(now_ms - gMotion.start_time_ms) <
        APP_ENCODER_STALL_GRACE_MS) {
        gMotion.stall_window_start_ms = now_ms;
        gMotion.stall_window_left_start = left_progress;
        gMotion.stall_window_right_start = right_progress;
        return;
    }

    if ((uint32_t)(now_ms - gMotion.stall_window_start_ms) <
        APP_ENCODER_STALL_WINDOW_MS) {
        return;
    }

    if ((abs_i32(left_pwm) >= APP_ENCODER_STALL_MIN_PWM) &&
        ((left_progress - gMotion.stall_window_left_start) <
         APP_ENCODER_STALL_MIN_COUNTS)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_LEFT;
    }

    if ((abs_i32(right_pwm) >= APP_ENCODER_STALL_MIN_PWM) &&
        ((right_progress - gMotion.stall_window_right_start) <
         APP_ENCODER_STALL_MIN_COUNTS)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_RIGHT;
    }

    gMotion.stall_window_start_ms = now_ms;
    gMotion.stall_window_left_start = left_progress;
    gMotion.stall_window_right_start = right_progress;

    if (gMotion.encoder_fault_flags != 0U) {
        motor_stop();
        gMotion.current_left_pwm = 0;
        gMotion.current_right_pwm = 0;
        gMotion.state = MOTION_ENCODER_FAULT;
    }
#else
    (void)now_ms;
    (void)left_progress;
    (void)right_progress;
    (void)left_pwm;
    (void)right_pwm;
#endif
}

void motion_update(uint32_t now_ms)
{
    EncoderSnapshot snapshot;
    int32_t left_progress;
    int32_t right_progress;
    int32_t left_remaining;
    int32_t right_remaining;
    int32_t profile_remaining;
    bool left_reached;
    bool right_reached;
    bool turning;
    int16_t minimum_pwm;
    int16_t approach_pwm;
    int32_t decel_counts;
    int32_t approach_counts;
    int16_t target_base_pwm;
    int16_t correction;
    int16_t target_left_pwm;
    int16_t target_right_pwm;

    if (gMotion.state == MOTION_BRAKING) {
        if ((uint32_t)(now_ms - gMotion.brake_start_ms) >=
            APP_ACTIVE_BRAKE_MS) {
            motor_stop();
            gMotion.state = MOTION_DONE;
        }
        gMotion.debug.state = gMotion.state;
        return;
    }

    if ((gMotion.state != MOTION_DISTANCE) &&
        (gMotion.state != MOTION_TURN)) {
        return;
    }

    if (gMotion.start_time_ms == 0U) {
        gMotion.start_time_ms = now_ms;
        gMotion.last_update_ms = now_ms;
        gMotion.stall_window_start_ms = now_ms;
    } else if ((uint32_t)(now_ms - gMotion.last_update_ms) <
               APP_CONTROL_PERIOD_MS) {
        return;
    } else {
        gMotion.last_update_ms = now_ms;
    }

    encoder_take_snapshot(&snapshot);
    left_progress =
        (snapshot.left_count - gMotion.start_left) *
        gMotion.left_direction;
    right_progress =
        (snapshot.right_count - gMotion.start_right) *
        gMotion.right_direction;

    left_remaining = gMotion.target_counts - left_progress;
    right_remaining = gMotion.target_counts - right_progress;
    left_reached = left_remaining <= APP_POSITION_TOLERANCE_COUNTS;
    right_reached = right_remaining <= APP_POSITION_TOLERANCE_COUNTS;

    if (left_reached && right_reached) {
        enter_braking(now_ms);
        gMotion.debug.state = gMotion.state;
        return;
    }

    if ((uint32_t)(now_ms - gMotion.start_time_ms) >
        APP_MOTION_TIMEOUT_MS) {
        motor_stop();
        gMotion.state = MOTION_TIMEOUT;
        gMotion.debug.state = gMotion.state;
        return;
    }

    turning = gMotion.state == MOTION_TURN;
    minimum_pwm = turning ? APP_TURN_MIN_PWM : APP_STRAIGHT_MIN_PWM;
    approach_pwm = turning ?
        APP_TURN_APPROACH_PWM : APP_STRAIGHT_APPROACH_PWM;
    decel_counts = turning ?
        APP_TURN_DECEL_COUNTS : APP_STRAIGHT_DECEL_COUNTS;
    approach_counts = turning ?
        APP_TURN_APPROACH_COUNTS : APP_STRAIGHT_APPROACH_COUNTS;

    if (!left_reached && !right_reached) {
        profile_remaining = left_remaining < right_remaining ?
            left_remaining : right_remaining;
    } else {
        profile_remaining = left_reached ? right_remaining : left_remaining;
    }

    target_base_pwm = profile_pwm(
        profile_remaining,
        gMotion.requested_max_pwm,
        minimum_pwm,
        approach_pwm,
        decel_counts,
        approach_counts);
    gMotion.current_base_pwm = slew_pwm(
        gMotion.current_base_pwm,
        target_base_pwm);

    correction = synchronization_correction(
        left_progress,
        right_progress,
        turning);

    target_left_pwm = left_reached ? 0 :
        apply_trim(
            (int16_t)clamp_i32(
                gMotion.current_base_pwm - correction,
                0,
                gMotion.requested_max_pwm),
            APP_LEFT_PWM_TRIM_PERMILLE);
    target_right_pwm = right_reached ? 0 :
        apply_trim(
            (int16_t)clamp_i32(
                gMotion.current_base_pwm + correction,
                0,
                gMotion.requested_max_pwm),
            APP_RIGHT_PWM_TRIM_PERMILLE);

    gMotion.current_left_pwm = slew_pwm(
        gMotion.current_left_pwm,
        target_left_pwm);
    gMotion.current_right_pwm = slew_pwm(
        gMotion.current_right_pwm,
        target_right_pwm);

    motor_set(
        (int16_t)(gMotion.left_direction * gMotion.current_left_pwm),
        (int16_t)(gMotion.right_direction * gMotion.current_right_pwm));

    update_stall_detection(
        now_ms,
        left_progress,
        right_progress,
        gMotion.current_left_pwm,
        gMotion.current_right_pwm);

    gMotion.debug.state = gMotion.state;
    gMotion.debug.left_progress = left_progress;
    gMotion.debug.right_progress = right_progress;
    gMotion.debug.target_counts = gMotion.target_counts;
    gMotion.debug.left_remaining = left_remaining;
    gMotion.debug.right_remaining = right_remaining;
    gMotion.debug.base_pwm = gMotion.current_base_pwm;
    gMotion.debug.synchronization_pwm = correction;
    gMotion.debug.left_pwm = motor_get_left_command();
    gMotion.debug.right_pwm = motor_get_right_command();
    gMotion.debug.left_delta = snapshot.left_delta;
    gMotion.debug.right_delta = snapshot.right_delta;
    gMotion.debug.encoder_fault_flags = gMotion.encoder_fault_flags;
}

void motion_abort(void)
{
    motor_stop();
    gMotion.current_base_pwm = 0;
    gMotion.current_left_pwm = 0;
    gMotion.current_right_pwm = 0;
    gMotion.state = MOTION_IDLE;
    gMotion.debug.state = MOTION_IDLE;
}

bool motion_is_busy(void)
{
    return
        (gMotion.state == MOTION_DISTANCE) ||
        (gMotion.state == MOTION_TURN) ||
        (gMotion.state == MOTION_BRAKING);
}

MotionState motion_get_state(void)
{
    return gMotion.state;
}

void motion_get_debug(MotionDebug *debug)
{
    if (debug != NULL) {
        *debug = gMotion.debug;
    }
}
