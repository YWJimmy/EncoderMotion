#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
    uint8_t left_ab;
    uint8_t right_ab;
} EncoderSnapshot;

typedef struct {
    int32_t count;
    uint32_t valid_transition_count;
    uint32_t invalid_transition_count;
    uint32_t duplicate_state_count;
    uint32_t a_edge_count;
    uint32_t b_edge_count;
    uint32_t merged_edge_count;
    uint8_t ab_state;
} EncoderStatistics;

void encoder_init(void);
void encoder_reset_counts(void);
void encoder_reset_statistics(void);
void encoder_take_snapshot(EncoderSnapshot *snapshot);
void encoder_get_counts(int32_t *left_count, int32_t *right_count);
void encoder_get_statistics(
    EncoderStatistics *left_statistics,
    EncoderStatistics *right_statistics);

/* Test hook: pure quadrature transition evaluation. */
int8_t encoder_decode_transition(uint8_t previous_state, uint8_t new_state);

#endif
