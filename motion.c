#include "motion.h"

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
    uint16_t brake_ticks;
    uint32_t elapsed_ticks;
} MotionController;

static MotionController gMotion;

static int32_t abs_i32(int32_t value)
{
    return value >= 0 ? value : -value;
}

static int32_t clamp_i32(
    int32_t value,
    int32_t minimum,
    int32_t maximum)
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
    int64_t numerator;
    int64_t denominator;
    int32_t sign;
    int32_t distance_abs;

    sign = distance_mm >= 0 ? 1 : -1;
    distance_abs = abs_i32(distance_mm);

    numerator =
        (int64_t)distance_abs *
        APP_ENCODER_COUNTS_PER_REV *
        10000LL;

    denominator =
        (int64_t)APP_WHEEL_DIAMETER_MM *
        APP_PI_X10000;

    return sign *
        (int32_t)(
            (numerator + denominator / 2LL) /
            denominator);
}

static int32_t turn_deg_to_counts(int32_t angle_deg)
{
    int64_t numerator;
    int64_t denominator;

    numerator =
        (int64_t)abs_i32(angle_deg) *
        APP_WHEEL_TRACK_MM *
        APP_ENCODER_COUNTS_PER_REV;

    denominator =
        360LL *
        APP_WHEEL_DIAMETER_MM;

    return (int32_t)(
        (numerator + denominator / 2LL) /
        denominator);
}

static bool target_reached(
    int32_t progress,
    int32_t target)
{
    if (target >= 0) {
        return progress >=
            target -
            APP_POSITION_TOLERANCE_COUNT;
    }

    return progress <=
        target +
        APP_POSITION_TOLERANCE_COUNT;
}

/*
 * Linear approach curve:
 * far away -> max PWM; near the target -> minimum effective PWM;
 * inside the final tolerance -> zero drive PWM and active braking.
 */
static int16_t decel_command_magnitude(
    int32_t remaining,
    int16_t max_pwm)
{
    int32_t effective_remaining;
    int32_t magnitude;

    if (remaining <= APP_POSITION_TOLERANCE_COUNT) {
        return 0;
    }

    effective_remaining =
        remaining -
        APP_POSITION_TOLERANCE_COUNT;

    if (effective_remaining >=
        APP_POSITION_DECEL_COUNTS) {
        return max_pwm;
    }

    magnitude =
        APP_POSITION_MIN_PWM +
        (effective_remaining *
         (max_pwm - APP_POSITION_MIN_PWM)) /
        APP_POSITION_DECEL_COUNTS;

    return (int16_t)clamp_i32(
        magnitude,
        APP_POSITION_MIN_PWM,
        max_pwm);
}

static int16_t remaining_to_turn_command(
    int32_t remaining,
    int16_t max_pwm)
{
    int16_t magnitude;

    magnitude =
        decel_command_magnitude(
            abs_i32(remaining),
            max_pwm);

    return remaining >= 0 ?
        magnitude :
        (int16_t)-magnitude;
}

static void begin_braking(void)
{
    motor_brake();
    gMotion.brake_ticks = 0U;
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

    encoder_get_counts(
        &gMotion.start_left,
        &gMotion.start_right);

    gMotion.target_left = target_left;
    gMotion.target_right = target_right;
    gMotion.sync_integral = 0;
    gMotion.sync_previous_error = 0;
    gMotion.max_pwm = max_pwm;
    gMotion.brake_ticks = 0U;
    gMotion.elapsed_ticks = 0U;
    gMotion.state = state;

    motor_enable();
    return true;
}

static void update_distance(
    int32_t left_progress,
    int32_t right_progress)
{
    int32_t direction;
    int32_t target;
    int32_t left_travel;
    int32_t right_travel;
    int32_t left_remaining;
    int32_t right_remaining;
    int32_t base_remaining;
    int32_t sync_error;
    int32_t sync_derivative;
    int32_t correction;
    int32_t left_magnitude;
    int32_t right_magnitude;
    bool left_reached;
    bool right_reached;
    int16_t base_pwm;

    direction =
        gMotion.target_left >= 0 ?
        1 : -1;
    target =
        abs_i32(gMotion.target_left);

    /* Normalize both forward and reverse travel onto a positive axis. */
    left_travel =
        left_progress * direction;
    right_travel =
        right_progress * direction;
    left_remaining =
        target - left_travel;
    right_remaining =
        target - right_travel;

    left_reached =
        left_remaining <=
        APP_POSITION_TOLERANCE_COUNT;
    right_reached =
        right_remaining <=
        APP_POSITION_TOLERANCE_COUNT;

    if (left_reached && right_reached) {
        begin_braking();
        return;
    }

    base_remaining =
        left_remaining > right_remaining ?
        left_remaining :
        right_remaining;
    base_pwm =
        decel_command_magnitude(
            base_remaining,
            gMotion.max_pwm);

    /*
     * Positive error means the left wheel is ahead.
     * Reduce left PWM and increase right PWM.
     */
    sync_error =
        left_travel - right_travel;
    gMotion.sync_integral = clamp_i32(
        gMotion.sync_integral + sync_error,
        -APP_SYNC_INTEGRAL_LIMIT,
        APP_SYNC_INTEGRAL_LIMIT);
    sync_derivative =
        sync_error -
        gMotion.sync_previous_error;
    gMotion.sync_previous_error =
        sync_error;

    correction =
        APP_SYNC_KP * sync_error +
        gMotion.sync_integral /
            APP_SYNC_KI_DIV +
        APP_SYNC_KD * sync_derivative;
    correction = clamp_i32(
        correction,
        -APP_SYNC_CORRECTION_LIMIT,
        APP_SYNC_CORRECTION_LIMIT);

    left_magnitude =
        left_reached ?
        0 :
        clamp_i32(
            (int32_t)base_pwm - correction,
            0,
            gMotion.max_pwm);
    right_magnitude =
        right_reached ?
        0 :
        clamp_i32(
            (int32_t)base_pwm + correction,
            0,
            gMotion.max_pwm);

    motor_set(
        (int16_t)(direction * left_magnitude),
        (int16_t)(direction * right_magnitude));
}

static void update_turn(
    int32_t left_progress,
    int32_t right_progress)
{
    int32_t left_remaining;
    int32_t right_remaining;
    bool left_reached;
    bool right_reached;
    int16_t left_command;
    int16_t right_command;

    left_remaining =
        gMotion.target_left -
        left_progress;
    right_remaining =
        gMotion.target_right -
        right_progress;

    left_reached =
        target_reached(
            left_progress,
            gMotion.target_left);
    right_reached =
        target_reached(
            right_progress,
            gMotion.target_right);

    if (left_reached && right_reached) {
        begin_braking();
        return;
    }

    left_command =
        left_reached ?
        0 :
        remaining_to_turn_command(
            left_remaining,
            gMotion.max_pwm);
    right_command =
        right_reached ?
        0 :
        remaining_to_turn_command(
            right_remaining,
            gMotion.max_pwm);

    motor_set(
        left_command,
        right_command);
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
    gMotion.max_pwm =
        APP_POSITION_DEFAULT_MAX_PWM;
    gMotion.brake_ticks = 0U;
    gMotion.elapsed_ticks = 0U;
}

bool motion_start_distance_mm(
    int32_t distance_mm,
    int16_t max_pwm)
{
    int32_t target_count;

    if (distance_mm == 0) {
        return false;
    }

    target_count =
        distance_mm_to_counts(distance_mm);

    return start_targets(
        MOTION_RUNNING_DISTANCE,
        target_count,
        target_count,
        max_pwm);
}

bool motion_start_turn_deg(
    int32_t angle_deg,
    int16_t max_pwm)
{
    int32_t target_count;

    if (angle_deg == 0) {
        return false;
    }

    target_count =
        turn_deg_to_counts(angle_deg);

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
        if (gMotion.brake_ticks <
            APP_ACTIVE_BRAKE_TICKS) {
            gMotion.brake_ticks++;
        } else {
            /* The final stopped state always has PWM compare values at zero. */
            motor_stop();
            gMotion.state = MOTION_DONE;
        }
        return;
    }

    encoder_get_counts(
        &left_count,
        &right_count);

    left_progress =
        left_count -
        gMotion.start_left;
    right_progress =
        right_count -
        gMotion.start_right;

    if (gMotion.state ==
        MOTION_RUNNING_DISTANCE) {
        update_distance(
            left_progress,
            right_progress);
    } else {
        update_turn(
            left_progress,
            right_progress);
    }

    gMotion.elapsed_ticks++;

    if (gMotion.elapsed_ticks >
        APP_MOTION_TIMEOUT_TICKS &&
        gMotion.state != MOTION_BRAKING) {
        motor_stop();
        gMotion.state = MOTION_TIMEOUT;
    }
}

void motion_abort(void)
{
    motor_stop();
    gMotion.state = MOTION_IDLE;
}

bool motion_is_busy(void)
{
    return
        gMotion.state ==
            MOTION_RUNNING_DISTANCE ||
        gMotion.state ==
            MOTION_RUNNING_TURN ||
        gMotion.state ==
            MOTION_BRAKING;
}

MotionState motion_get_state(void)
{
    return gMotion.state;
}
