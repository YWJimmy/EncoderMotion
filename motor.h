#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void motor_init(void);
void motor_enable(void);
void motor_disable(void);

/*
 * Commands are expressed in vehicle coordinates:
 * positive = vehicle forward, negative = vehicle reverse.
 */
void motor_set(
    int16_t left_command,
    int16_t right_command);

void motor_stop(void);
void motor_brake(void);

/* Return the last logical commands, not the internally inverted pin polarity. */
int16_t motor_get_left_command(void);
int16_t motor_get_right_command(void);

#endif
