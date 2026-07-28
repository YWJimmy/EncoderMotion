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
    MOTION_TIMEOUT,
    MOTION_ENCODER_FAULT
} MotionState;

typedef struct {
    int32_t left_progress;
    int32_t right_progress;
    int32_t target_count;
    int32_t remaining_count;
    int32_t synchronization_error;
    int16_t synchronization_pwm;
    int16_t base_pwm;
    int16_t left_pwm;
    int16_t right_pwm;

    /* Diagnostic speed only: counts per 5 ms tick multiplied by 16. */
    int32_t left_measured_speed_x16;
    int32_t right_measured_speed_x16;

    uint8_t encoder_fault_flags;
    uint16_t control_elapsed_ticks;
    uint16_t encoder_fault_grace_ticks;
} MotionDebugData;

void motion_init(void);
bool motion_start_distance_mm(int32_t distance_mm, int16_t max_pwm);
bool motion_start_turn_deg(int32_t angle_deg, int16_t max_pwm);
void motion_update(uint32_t now_tick);
void motion_abort(void);
bool motion_is_busy(void);
MotionState motion_get_state(void);
void motion_get_debug(MotionDebugData *debug_data);

#endif
