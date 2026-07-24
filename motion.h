#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_IDLE = 0,
    MOTION_RUNNING_DISTANCE,
    MOTION_RUNNING_TURN,
    MOTION_BRAKING,
    MOTION_DONE,
    MOTION_TIMEOUT
} MotionState;

void motion_init(void);

/*
 * distance_mm：
 *   正数 = 前进
 *   负数 = 后退
 */
bool motion_start_distance_mm(
    int32_t distance_mm,
    int16_t max_pwm);

/*
 * angle_deg：
 *   正数 = 原地左转
 *   负数 = 原地右转
 */
bool motion_start_turn_deg(
    int32_t angle_deg,
    int16_t max_pwm);

void motion_update(void);
void motion_abort(void);

bool motion_is_busy(void);
MotionState motion_get_state(void);

#endif
