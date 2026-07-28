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

    int32_t left_filtered_speed_x16;
    int32_t right_filtered_speed_x16;

    int32_t left_fault_progress;
    int32_t right_fault_progress;
    uint16_t left_fault_ticks;
    uint16_t right_fault_ticks;
    uint16_t encoder_fault_grace_ticks;
    uint8_t encoder_fault_flags;

    uint32_t last_control_tick;
    bool control_time_initialized;
    uint32_t elapsed_ticks;
    uint16_t brake_ticks;

    int16_t max_pwm;
    int16_t current_base_pwm;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    MotionDebugData debug;
} MotionController;

typedef struct {
    int16_t control_max_pwm;
    int16_t minimum_pwm;
    int16_t approach_pwm;
    int32_t decel_counts;
    int32_t approach_counts;
    int32_t tolerance_counts;
    int32_t sync_deadband_counts;
    int32_t sync_kp_div;
    int32_t sync_limit_pwm;
    int32_t left_trim_permille;
    int32_t right_trim_permille;
} MotionProfile;

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

static uint16_t clamp_elapsed_ticks(uint32_t elapsed_ticks)
{
    if (elapsed_ticks > APP_CONTROL_ELAPSED_LIMIT_TICKS) {
        return APP_CONTROL_ELAPSED_LIMIT_TICKS;
    }
    return (uint16_t)elapsed_ticks;
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
        APP_TURN_EFFECTIVE_TRACK_MM *
        APP_ENCODER_COUNTS_PER_REV;
    int64_t denominator =
        360LL * APP_WHEEL_DIAMETER_MM;

    return (int32_t)(
        (numerator + denominator / 2LL) / denominator);
}

static MotionProfile get_motion_profile(MotionState state)
{
    MotionProfile profile;

    if (state == MOTION_RUNNING_TURN) {
        profile.control_max_pwm = APP_TURN_CONTROL_MAX_PWM;
        profile.minimum_pwm = APP_TURN_MIN_PWM;
        profile.approach_pwm = APP_TURN_APPROACH_PWM;
        profile.decel_counts = APP_TURN_DECEL_COUNTS;
        profile.approach_counts = APP_TURN_APPROACH_COUNTS;
        profile.tolerance_counts = APP_TURN_TOLERANCE_COUNTS;
        profile.sync_deadband_counts = APP_TURN_SYNC_DEADBAND_COUNTS;
        profile.sync_kp_div = APP_TURN_SYNC_KP_DIV;
        profile.sync_limit_pwm = APP_TURN_SYNC_LIMIT_PWM;
        profile.left_trim_permille = APP_TURN_LEFT_TRIM_PERMILLE;
        profile.right_trim_permille = APP_TURN_RIGHT_TRIM_PERMILLE;
    } else {
        profile.control_max_pwm = APP_STRAIGHT_CONTROL_MAX_PWM;
        profile.minimum_pwm = APP_STRAIGHT_MIN_PWM;
        profile.approach_pwm = APP_STRAIGHT_APPROACH_PWM;
        profile.decel_counts = APP_STRAIGHT_DECEL_COUNTS;
        profile.approach_counts = APP_STRAIGHT_APPROACH_COUNTS;
        profile.tolerance_counts = APP_STRAIGHT_TOLERANCE_COUNTS;
        profile.sync_deadband_counts = APP_STRAIGHT_SYNC_DEADBAND_COUNTS;
        profile.sync_kp_div = APP_STRAIGHT_SYNC_KP_DIV;
        profile.sync_limit_pwm = APP_STRAIGHT_SYNC_LIMIT_PWM;
        profile.left_trim_permille = APP_STRAIGHT_LEFT_TRIM_PERMILLE;
        profile.right_trim_permille = APP_STRAIGHT_RIGHT_TRIM_PERMILLE;
    }

    return profile;
}

static int16_t position_profile_pwm(
    int32_t remaining,
    int16_t max_pwm,
    const MotionProfile *profile)
{
    int32_t magnitude;
    int32_t span;

    if ((profile == NULL) ||
        (remaining <= profile->tolerance_counts)) {
        return 0;
    }

    if (remaining <= profile->approach_counts) {
        span = profile->approach_counts - profile->tolerance_counts;
        magnitude = profile->approach_pwm;
        if (span > 0) {
            magnitude +=
                ((remaining - profile->tolerance_counts) *
                 (profile->minimum_pwm - profile->approach_pwm)) /
                span;
        }
        return (int16_t)clamp_i32(
            magnitude,
            profile->approach_pwm,
            profile->minimum_pwm);
    }

    if (remaining < profile->decel_counts) {
        span = profile->decel_counts - profile->approach_counts;
        magnitude = profile->minimum_pwm;
        if (span > 0) {
            magnitude +=
                ((remaining - profile->approach_counts) *
                 (max_pwm - profile->minimum_pwm)) /
                span;
        }
        return (int16_t)clamp_i32(
            magnitude,
            profile->minimum_pwm,
            max_pwm);
    }

    return max_pwm;
}

static int16_t slew_pwm(
    int16_t current,
    int16_t target,
    uint16_t elapsed_ticks,
    int16_t startup_pwm)
{
    int32_t updated = current;

    if (target > current) {
        if (current == 0) {
            return target < startup_pwm ? target : startup_pwm;
        }
        updated += (int32_t)APP_MOTOR_ACCEL_STEP * elapsed_ticks;
        if (updated > target) {
            updated = target;
        }
    } else if (target < current) {
        updated -= (int32_t)APP_MOTOR_DECEL_STEP * elapsed_ticks;
        if (updated < target) {
            updated = target;
        }
    }

    return (int16_t)updated;
}

static int32_t filtered_speed_x16(
    int32_t previous,
    int32_t normalized_delta,
    uint32_t elapsed_ticks)
{
    int32_t sample;
    int32_t filter_steps;

    if (elapsed_ticks == 0U) {
        return previous;
    }

    sample = (normalized_delta * 16L) / (int32_t)elapsed_ticks;
    filter_steps = (int32_t)elapsed_ticks;
    if (filter_steps > APP_SPEED_FILTER_DIV) {
        filter_steps = APP_SPEED_FILTER_DIV;
    }

    return previous +
        ((sample - previous) * filter_steps) / APP_SPEED_FILTER_DIV;
}

static int32_t synchronization_correction_pwm(
    int32_t error,
    const MotionProfile *profile)
{
    int32_t correction;

    if (profile == NULL) {
        return 0;
    }

    if (abs_i32(error) <= profile->sync_deadband_counts) {
        return 0;
    }

    correction = error / profile->sync_kp_div;
    return clamp_i32(
        correction,
        -profile->sync_limit_pwm,
        profile->sync_limit_pwm);
}

static int16_t apply_trim(int16_t pwm, int32_t trim_permille)
{
    int32_t trimmed =
        ((int32_t)pwm * trim_permille + 500L) / 1000L;

    return (int16_t)clamp_i32(trimmed, 0, APP_MOTOR_COMMAND_MAX);
}

static void begin_braking(void)
{
    motor_brake();
    gMotion.brake_ticks = 0U;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.debug.left_pwm = 0;
    gMotion.debug.right_pwm = 0;
    gMotion.state = MOTION_BRAKING;
}

static bool start_targets(
    MotionState state,
    int32_t target_left,
    int32_t target_right,
    int16_t max_pwm)
{
    EncoderSnapshot snapshot;
    MotionProfile profile;

    if (motion_is_busy()) {
        return false;
    }

    profile = get_motion_profile(state);
    max_pwm = (int16_t)clamp_i32(
        max_pwm,
        profile.minimum_pwm,
        profile.control_max_pwm);

    encoder_take_snapshot(&snapshot);

    gMotion.start_left = snapshot.left_count;
    gMotion.start_right = snapshot.right_count;
    gMotion.target_left = target_left;
    gMotion.target_right = target_right;
    gMotion.left_filtered_speed_x16 = 0;
    gMotion.right_filtered_speed_x16 = 0;
    gMotion.left_fault_progress = 0;
    gMotion.right_fault_progress = 0;
    gMotion.left_fault_ticks = 0U;
    gMotion.right_fault_ticks = 0U;
    gMotion.encoder_fault_grace_ticks =
        APP_ENCODER_FAULT_STARTUP_GRACE_TICKS;
    gMotion.encoder_fault_flags = 0U;
    gMotion.last_control_tick = 0U;
    gMotion.control_time_initialized = false;
    gMotion.elapsed_ticks = 0U;
    gMotion.brake_ticks = 0U;
    gMotion.max_pwm = max_pwm;
    gMotion.current_base_pwm = 0;
    gMotion.last_left_pwm = 0;
    gMotion.last_right_pwm = 0;
    gMotion.debug = (MotionDebugData){0};
    gMotion.state = state;
    motor_enable();
    return true;
}

static bool update_one_fault_window(
    int32_t normalized_delta,
    int16_t magnitude_pwm,
    int32_t *progress,
    uint16_t *ticks,
    uint32_t elapsed_ticks)
{
    uint32_t updated_ticks;

    if ((progress == NULL) || (ticks == NULL)) {
        return false;
    }

    if (magnitude_pwm < APP_ENCODER_FAULT_MIN_PWM) {
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

    {
        bool fault = *progress < APP_ENCODER_FAULT_MIN_PROGRESS_COUNT;
        *progress = 0;
        *ticks = 0U;
        return fault;
    }
}

static bool update_encoder_fault_detection(
    int32_t left_delta,
    int32_t right_delta,
    int16_t left_magnitude,
    int16_t right_magnitude,
    uint32_t elapsed_ticks)
{
    if (gMotion.encoder_fault_grace_ticks != 0U) {
        if (elapsed_ticks >= gMotion.encoder_fault_grace_ticks) {
            gMotion.encoder_fault_grace_ticks = 0U;
        } else {
            gMotion.encoder_fault_grace_ticks = (uint16_t)(
                gMotion.encoder_fault_grace_ticks - elapsed_ticks);
        }
        return false;
    }

    if (update_one_fault_window(
            left_delta,
            left_magnitude,
            &gMotion.left_fault_progress,
            &gMotion.left_fault_ticks,
            elapsed_ticks)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_LEFT;
    }

    if (update_one_fault_window(
            right_delta,
            right_magnitude,
            &gMotion.right_fault_progress,
            &gMotion.right_fault_ticks,
            elapsed_ticks)) {
        gMotion.encoder_fault_flags |= APP_ENCODER_FAULT_RIGHT;
    }

    if (gMotion.encoder_fault_flags != 0U) {
        motor_stop();
        gMotion.current_base_pwm = 0;
        gMotion.last_left_pwm = 0;
        gMotion.last_right_pwm = 0;
        gMotion.debug.left_pwm = 0;
        gMotion.debug.right_pwm = 0;
        gMotion.state = MOTION_ENCODER_FAULT;
        return true;
    }

    return false;
}

static void update_running_motion(
    const EncoderSnapshot *snapshot,
    uint32_t elapsed_ticks)
{
    MotionProfile profile;
    int32_t left_direction;
    int32_t right_direction;
    int32_t left_progress;
    int32_t right_progress;
    int32_t left_travel;
    int32_t right_travel;
    int32_t normalized_left_delta;
    int32_t normalized_right_delta;
    int32_t target;
    int32_t average_travel;
    int32_t remaining;
    int32_t error;
    int32_t correction;
    int32_t left_magnitude;
    int32_t right_magnitude;
    int16_t target_base_pwm;
    uint16_t gain_ticks;

    if (snapshot == NULL) {
        return;
    }

    profile = get_motion_profile(gMotion.state);
    left_direction = gMotion.target_left >= 0 ? 1 : -1;
    right_direction = gMotion.target_right >= 0 ? 1 : -1;
    target = abs_i32(gMotion.target_left);

    left_progress = snapshot->left_count - gMotion.start_left;
    right_progress = snapshot->right_count - gMotion.start_right;
    left_travel = left_progress * left_direction;
    right_travel = right_progress * right_direction;
    normalized_left_delta = snapshot->left_delta * left_direction;
    normalized_right_delta = snapshot->right_delta * right_direction;

    average_travel = (left_travel + right_travel) / 2;
    remaining = target - average_travel;

    gMotion.left_filtered_speed_x16 = filtered_speed_x16(
        gMotion.left_filtered_speed_x16,
        normalized_left_delta,
        elapsed_ticks);
    gMotion.right_filtered_speed_x16 = filtered_speed_x16(
        gMotion.right_filtered_speed_x16,
        normalized_right_delta,
        elapsed_ticks);

    /* Completion requires both wheels to reach their own target. This avoids
     * stopping early when one encoder is ahead of the other. */
    if ((left_travel >= target - profile.tolerance_counts) &&
        (right_travel >= target - profile.tolerance_counts)) {
        begin_braking();
        return;
    }

    target_base_pwm = position_profile_pwm(
        remaining,
        gMotion.max_pwm,
        &profile);
    gain_ticks = clamp_elapsed_ticks(elapsed_ticks);
    gMotion.current_base_pwm = slew_pwm(
        gMotion.current_base_pwm,
        target_base_pwm,
        gain_ticks,
        profile.minimum_pwm);

    error = left_travel - right_travel;
    correction = synchronization_correction_pwm(error, &profile);

    left_magnitude = apply_trim(
        gMotion.current_base_pwm,
        profile.left_trim_permille);
    right_magnitude = apply_trim(
        gMotion.current_base_pwm,
        profile.right_trim_permille);

    /* Positive error means left is ahead: reduce left and increase right. */
    left_magnitude -= correction;
    right_magnitude += correction;

    left_magnitude = clamp_i32(left_magnitude, 0, gMotion.max_pwm);
    right_magnitude = clamp_i32(right_magnitude, 0, gMotion.max_pwm);

    /* A lagging wheel must still receive enough command to overcome static
     * friction, even when the average position is already near the target. */
    if ((left_travel < target - profile.tolerance_counts) &&
        (left_magnitude < profile.approach_pwm)) {
        left_magnitude = profile.approach_pwm;
    }
    if ((right_travel < target - profile.tolerance_counts) &&
        (right_magnitude < profile.approach_pwm)) {
        right_magnitude = profile.approach_pwm;
    }

    /* Never command a wheel farther once it has already crossed the target.
     * The opposite wheel may continue forward to remove residual mismatch;
     * no automatic reverse correction is used. */
    if (left_travel >= target) {
        left_magnitude = 0;
    }
    if (right_travel >= target) {
        right_magnitude = 0;
    }

    gMotion.last_left_pwm =
        (int16_t)(left_direction * left_magnitude);
    gMotion.last_right_pwm =
        (int16_t)(right_direction * right_magnitude);

    motor_set(gMotion.last_left_pwm, gMotion.last_right_pwm);

    gMotion.debug.left_progress = left_travel;
    gMotion.debug.right_progress = right_travel;
    gMotion.debug.target_count = target;
    gMotion.debug.remaining_count = remaining;
    gMotion.debug.synchronization_error = error;
    gMotion.debug.synchronization_pwm = (int16_t)correction;
    gMotion.debug.base_pwm = gMotion.current_base_pwm;
    gMotion.debug.left_pwm = gMotion.last_left_pwm;
    gMotion.debug.right_pwm = gMotion.last_right_pwm;
    gMotion.debug.left_measured_speed_x16 =
        gMotion.left_filtered_speed_x16;
    gMotion.debug.right_measured_speed_x16 =
        gMotion.right_filtered_speed_x16;
    gMotion.debug.encoder_fault_flags = gMotion.encoder_fault_flags;
    gMotion.debug.encoder_fault_grace_ticks =
        gMotion.encoder_fault_grace_ticks;

    (void)update_encoder_fault_detection(
        normalized_left_delta,
        normalized_right_delta,
        (int16_t)left_magnitude,
        (int16_t)right_magnitude,
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

void motion_update(uint32_t now_tick)
{
    EncoderSnapshot snapshot;
    uint32_t elapsed_ticks;

    if (!motion_is_busy()) {
        return;
    }

    if (!gMotion.control_time_initialized) {
        gMotion.last_control_tick = now_tick;
        gMotion.control_time_initialized = true;
        return;
    }

    elapsed_ticks = now_tick - gMotion.last_control_tick;
    if (elapsed_ticks == 0U) {
        return;
    }
    gMotion.last_control_tick = now_tick;

    if (elapsed_ticks > UINT16_MAX) {
        gMotion.debug.control_elapsed_ticks = UINT16_MAX;
    } else {
        gMotion.debug.control_elapsed_ticks = (uint16_t)elapsed_ticks;
    }

    if (gMotion.state == MOTION_BRAKING) {
        uint32_t updated = (uint32_t)gMotion.brake_ticks + elapsed_ticks;

        if (updated >= APP_ACTIVE_BRAKE_TICKS) {
            motor_stop();
            gMotion.brake_ticks = APP_ACTIVE_BRAKE_TICKS;
            gMotion.state = MOTION_DONE;
        } else {
            gMotion.brake_ticks = (uint16_t)updated;
        }
        return;
    }

    encoder_take_snapshot(&snapshot);
    update_running_motion(&snapshot, elapsed_ticks);

    if ((gMotion.state != MOTION_RUNNING_DISTANCE) &&
        (gMotion.state != MOTION_RUNNING_TURN)) {
        return;
    }

    gMotion.elapsed_ticks += elapsed_ticks;
    if (gMotion.elapsed_ticks > APP_MOTION_TIMEOUT_TICKS) {
        motor_stop();
        gMotion.current_base_pwm = 0;
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
    gMotion.debug.left_pwm = 0;
    gMotion.debug.right_pwm = 0;
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
