#include "motor.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

#if ((APP_LEFT_MOTOR_DIRECTION_SIGN != 1) && \
     (APP_LEFT_MOTOR_DIRECTION_SIGN != -1))
#error "APP_LEFT_MOTOR_DIRECTION_SIGN must be +1 or -1"
#endif

#if ((APP_RIGHT_MOTOR_DIRECTION_SIGN != 1) && \
     (APP_RIGHT_MOTOR_DIRECTION_SIGN != -1))
#error "APP_RIGHT_MOTOR_DIRECTION_SIGN must be +1 or -1"
#endif

static int16_t gLeftCommand;
static int16_t gRightCommand;

static int16_t clamp_command(int32_t command)
{
    if (command > APP_MOTOR_COMMAND_MAX) {
        return APP_MOTOR_COMMAND_MAX;
    }
    if (command < -APP_MOTOR_COMMAND_MAX) {
        return -APP_MOTOR_COMMAND_MAX;
    }
    return (int16_t)command;
}

static uint16_t command_to_compare(int16_t command)
{
    uint32_t magnitude;

    if (command < 0) {
        magnitude = (uint32_t)(-(int32_t)command);
    } else {
        magnitude = (uint32_t)command;
    }

    return (uint16_t)(
        (magnitude * (APP_PWM_PERIOD_TICKS - 1U)) /
        APP_MOTOR_COMMAND_MAX);
}

static void set_direction(
    uint32_t forward_pin,
    uint32_t reverse_pin,
    int16_t physical_command)
{
    if (physical_command > 0) {
        DL_GPIO_setPins(MOTOR_CTRL_PORT, forward_pin);
        DL_GPIO_clearPins(MOTOR_CTRL_PORT, reverse_pin);
    } else if (physical_command < 0) {
        DL_GPIO_clearPins(MOTOR_CTRL_PORT, forward_pin);
        DL_GPIO_setPins(MOTOR_CTRL_PORT, reverse_pin);
    } else {
        DL_GPIO_clearPins(
            MOTOR_CTRL_PORT,
            forward_pin | reverse_pin);
    }
}

void motor_init(void)
{
    gLeftCommand = 0;
    gRightCommand = 0;

    DL_TimerA_stopCounter(MOTOR_PWM_INST);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        0U,
        DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        0U,
        DL_TIMER_CC_3_INDEX);

    DL_GPIO_clearPins(
        MOTOR_CTRL_PORT,
        MOTOR_CTRL_TB6612_AIN1_PIN |
        MOTOR_CTRL_TB6612_AIN2_PIN |
        MOTOR_CTRL_TB6612_BIN1_PIN |
        MOTOR_CTRL_TB6612_BIN2_PIN |
        MOTOR_CTRL_TB6612_STBY_PIN);
}

void motor_enable(void)
{
    DL_TimerA_startCounter(MOTOR_PWM_INST);
    DL_GPIO_setPins(
        MOTOR_CTRL_PORT,
        MOTOR_CTRL_TB6612_STBY_PIN);
}

void motor_disable(void)
{
    motor_stop();
    DL_GPIO_clearPins(
        MOTOR_CTRL_PORT,
        MOTOR_CTRL_TB6612_STBY_PIN);
    DL_TimerA_stopCounter(MOTOR_PWM_INST);
}

void motor_set(
    int16_t left_command,
    int16_t right_command)
{
    int16_t physical_left_command;
    int16_t physical_right_command;

    left_command = clamp_command(left_command);
    right_command = clamp_command(right_command);

    /* Preserve logical commands for motion telemetry and fault diagnostics. */
    gLeftCommand = left_command;
    gRightCommand = right_command;

    physical_left_command = (int16_t)(
        left_command * APP_LEFT_MOTOR_DIRECTION_SIGN);
    physical_right_command = (int16_t)(
        right_command * APP_RIGHT_MOTOR_DIRECTION_SIGN);

    set_direction(
        MOTOR_CTRL_TB6612_AIN1_PIN,
        MOTOR_CTRL_TB6612_AIN2_PIN,
        physical_left_command);
    set_direction(
        MOTOR_CTRL_TB6612_BIN1_PIN,
        MOTOR_CTRL_TB6612_BIN2_PIN,
        physical_right_command);

    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        command_to_compare(left_command),
        DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        command_to_compare(right_command),
        DL_TIMER_CC_3_INDEX);
}

void motor_stop(void)
{
    motor_set(0, 0);
}

void motor_brake(void)
{
    gLeftCommand = 0;
    gRightCommand = 0;

    /* TB6612 short-brake: IN1=IN2=HIGH and PWM=HIGH. */
    DL_GPIO_setPins(
        MOTOR_CTRL_PORT,
        MOTOR_CTRL_TB6612_AIN1_PIN |
        MOTOR_CTRL_TB6612_AIN2_PIN |
        MOTOR_CTRL_TB6612_BIN1_PIN |
        MOTOR_CTRL_TB6612_BIN2_PIN);

    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        APP_PWM_PERIOD_TICKS - 1U,
        DL_TIMER_CC_2_INDEX);
    DL_TimerA_setCaptureCompareValue(
        MOTOR_PWM_INST,
        APP_PWM_PERIOD_TICKS - 1U,
        DL_TIMER_CC_3_INDEX);
}

int16_t motor_get_left_command(void)
{
    return gLeftCommand;
}

int16_t motor_get_right_command(void)
{
    return gRightCommand;
}
