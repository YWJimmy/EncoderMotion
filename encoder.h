#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    volatile int32_t count;
    volatile int32_t delta;
    uint8_t previous_state;
    volatile uint32_t valid_transition_count;
    volatile uint32_t invalid_transition_count;
    volatile uint32_t duplicate_state_count;
} Encoder;

typedef struct {
    int32_t count;
    uint32_t valid_transition_count;
    uint32_t invalid_transition_count;
    uint32_t duplicate_state_count;
} EncoderStatistics;

extern Encoder gLeftEncoder;
extern Encoder gRightEncoder;

void encoder_init(void);
void encoder_reset_counts(void);
void encoder_reset_statistics(void);
void encoder_get_counts(int32_t *left_count, int32_t *right_count);
void encoder_get_statistics(
    EncoderStatistics *left_statistics,
    EncoderStatistics *right_statistics);
int32_t encoder_take_left_delta(void);
int32_t encoder_take_right_delta(void);

#endif
