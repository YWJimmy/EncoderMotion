#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    volatile int32_t count;
    volatile int32_t delta;
    uint8_t previous_state;
    int8_t transition_accumulator;
    volatile uint32_t valid_transition_count;
    volatile uint32_t committed_cycle_count;
    volatile uint32_t invalid_transition_count;
    volatile uint32_t duplicate_state_count;
    volatile uint32_t a_edge_count;
    volatile uint32_t b_edge_count;
    volatile uint32_t coalesced_edge_count;
} Encoder;

typedef struct {
    int32_t count;
    uint32_t valid_transition_count;
    uint32_t committed_cycle_count;
    uint32_t invalid_transition_count;
    uint32_t duplicate_state_count;
    uint32_t a_edge_count;
    uint32_t b_edge_count;
    uint32_t coalesced_edge_count;
    uint8_t ab_state;
    int8_t transition_accumulator;
} EncoderStatistics;

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
    uint8_t left_ab_state;
    uint8_t right_ab_state;
} EncoderSnapshot;

extern Encoder gLeftEncoder;
extern Encoder gRightEncoder;

void encoder_init(void);
void encoder_reset_counts(void);
void encoder_reset_statistics(void);
void encoder_get_counts(int32_t *left_count, int32_t *right_count);
void encoder_get_statistics(
    EncoderStatistics *left_statistics,
    EncoderStatistics *right_statistics);
void encoder_take_snapshot(EncoderSnapshot *snapshot);
int32_t encoder_take_left_delta(void);
int32_t encoder_take_right_delta(void);

#endif
