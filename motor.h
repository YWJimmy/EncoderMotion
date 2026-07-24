#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_enable(void);
void motor_disable(void);

void motor_set(
    int16_t left_command,
    int16_t right_command);

void motor_stop(void);
void motor_brake(void);

#endif
