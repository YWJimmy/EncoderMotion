#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    volatile int32_t count;
    volatile int32_t delta;
    uint8_t previous_state;
} Encoder;

extern Encoder gLeftEncoder;
extern Encoder gRightEncoder;

void encoder_init(void);
void encoder_reset_counts(void);

void encoder_get_counts(
    int32_t *left_count,
    int32_t *right_count);

int32_t encoder_take_left_delta(void);
int32_t encoder_take_right_delta(void);

#endif