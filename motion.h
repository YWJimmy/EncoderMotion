#ifndef MOTION_H
#define MOTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTION_IDLE = 0,
    MOTION_DISTANCE,
    MOTION_TURN,
    MOTION_BRAKING,
    MOTION_DONE,
    MOTION_TIMEOUT,
    MOTION_ENCODER_FAULT
} MotionState;

typedef struct {
    MotionState state;
    int32_t left_progress;
    int32_t right_progress;
    int32_t target_counts;
    int32_t left_remaining;
    int32_t right_remaining;
    int16_t base_pwm;
    int16_t synchronization_pwm;
    int16_t left_pwm;
    int16_t right_pwm;
    int32_t left_delta;
    int32_t right_delta;
    uint8_t encoder_fault_flags;
} MotionDebug;

void motion_init(void);
bool motion_start_distance_mm(int32_t distance_mm, int16_t max_pwm);
bool motion_start_turn_deg(int32_t angle_deg, int16_t max_pwm);
void motion_update(uint32_t now_ms);
void motion_abort(void);
bool motion_is_busy(void);
MotionState motion_get_state(void);
void motion_get_debug(MotionDebug *debug);

#endif
