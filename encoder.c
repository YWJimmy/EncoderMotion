#include "encoder.h"

#include <stdbool.h>
#include <stddef.h>

#include "ti_msp_dl_config.h"

#define ENCODER_PIN_MASK \
    (ENCODER_ENC_L_A_PIN | ENCODER_ENC_L_B_PIN | \
     ENCODER_ENC_R_A_PIN | ENCODER_ENC_R_B_PIN)

#define LEFT_ENCODER_SIGN  (-1)
#define RIGHT_ENCODER_SIGN (1)

Encoder gLeftEncoder;
Encoder gRightEncoder;

/* Index = (previous_state << 2) | current_state; A is bit 1, B is bit 0. */
static const int8_t gQuadratureTable[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static uint8_t read_ab_state(
    uint32_t inputs,
    uint32_t pin_a,
    uint32_t pin_b)
{
    uint8_t state = 0U;

    if ((inputs & pin_a) != 0U) {
        state |= 2U;
    }
    if ((inputs & pin_b) != 0U) {
        state |= 1U;
    }

    return state;
}

static void encoder_update(
    Encoder *encoder,
    uint8_t new_state,
    int8_t direction_sign)
{
    uint8_t previous_state;
    uint8_t table_index;
    int8_t step;

    new_state &= 0x03U;
    previous_state = encoder->previous_state & 0x03U;

    if (new_state == previous_state) {
        encoder->duplicate_state_count++;
        return;
    }

    table_index = (uint8_t)((previous_state << 2U) | new_state);
    step = gQuadratureTable[table_index];

    /* Resynchronise even after an illegal two-bit jump, but do not count it. */
    encoder->previous_state = new_state;

    if (step == 0) {
        encoder->invalid_transition_count++;
        return;
    }

    step = (int8_t)(step * direction_sign);
    encoder->count += step;
    encoder->delta += step;
    encoder->valid_transition_count++;
}

void encoder_init(void)
{
    uint32_t inputs;

    DL_GPIO_disableInterrupt(ENCODER_PORT, ENCODER_PIN_MASK);

    DL_GPIO_initDigitalInputFeatures(
        ENCODER_ENC_L_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        ENCODER_ENC_L_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        ENCODER_ENC_R_A_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        ENCODER_ENC_R_B_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    inputs = DL_GPIO_readPins(ENCODER_PORT, ENCODER_PIN_MASK);

    gLeftEncoder.count = 0;
    gLeftEncoder.delta = 0;
    gLeftEncoder.previous_state = read_ab_state(
        inputs, ENCODER_ENC_L_A_PIN, ENCODER_ENC_L_B_PIN);
    gLeftEncoder.valid_transition_count = 0U;
    gLeftEncoder.invalid_transition_count = 0U;
    gLeftEncoder.duplicate_state_count = 0U;
    gLeftEncoder.a_edge_count = 0U;
    gLeftEncoder.b_edge_count = 0U;

    gRightEncoder.count = 0;
    gRightEncoder.delta = 0;
    gRightEncoder.previous_state = read_ab_state(
        inputs, ENCODER_ENC_R_A_PIN, ENCODER_ENC_R_B_PIN);
    gRightEncoder.valid_transition_count = 0U;
    gRightEncoder.invalid_transition_count = 0U;
    gRightEncoder.duplicate_state_count = 0U;
    gRightEncoder.a_edge_count = 0U;
    gRightEncoder.b_edge_count = 0U;

    /* PA28/PA31 are in the upper half; PA12/PA13 are in the lower half. */
    DL_GPIO_setUpperPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_28_EDGE_RISE_FALL |
        DL_GPIO_PIN_31_EDGE_RISE_FALL);
    DL_GPIO_setLowerPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_13_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_PIN_MASK);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_PIN_MASK);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_reset_counts(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gLeftEncoder.count = 0;
    gLeftEncoder.delta = 0;
    gRightEncoder.count = 0;
    gRightEncoder.delta = 0;
    __set_PRIMASK(primask);
}

void encoder_reset_statistics(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gLeftEncoder.valid_transition_count = 0U;
    gLeftEncoder.invalid_transition_count = 0U;
    gLeftEncoder.duplicate_state_count = 0U;
    gLeftEncoder.a_edge_count = 0U;
    gLeftEncoder.b_edge_count = 0U;
    gRightEncoder.valid_transition_count = 0U;
    gRightEncoder.invalid_transition_count = 0U;
    gRightEncoder.duplicate_state_count = 0U;
    gRightEncoder.a_edge_count = 0U;
    gRightEncoder.b_edge_count = 0U;
    __set_PRIMASK(primask);
}

void encoder_get_counts(int32_t *left_count, int32_t *right_count)
{
    uint32_t primask;

    if ((left_count == NULL) || (right_count == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *left_count = gLeftEncoder.count;
    *right_count = gRightEncoder.count;
    __set_PRIMASK(primask);
}

void encoder_get_statistics(
    EncoderStatistics *left_statistics,
    EncoderStatistics *right_statistics)
{
    uint32_t primask;

    if ((left_statistics == NULL) || (right_statistics == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    left_statistics->count = gLeftEncoder.count;
    left_statistics->valid_transition_count =
        gLeftEncoder.valid_transition_count;
    left_statistics->invalid_transition_count =
        gLeftEncoder.invalid_transition_count;
    left_statistics->duplicate_state_count =
        gLeftEncoder.duplicate_state_count;
    left_statistics->a_edge_count = gLeftEncoder.a_edge_count;
    left_statistics->b_edge_count = gLeftEncoder.b_edge_count;
    left_statistics->ab_state = gLeftEncoder.previous_state & 0x03U;

    right_statistics->count = gRightEncoder.count;
    right_statistics->valid_transition_count =
        gRightEncoder.valid_transition_count;
    right_statistics->invalid_transition_count =
        gRightEncoder.invalid_transition_count;
    right_statistics->duplicate_state_count =
        gRightEncoder.duplicate_state_count;
    right_statistics->a_edge_count = gRightEncoder.a_edge_count;
    right_statistics->b_edge_count = gRightEncoder.b_edge_count;
    right_statistics->ab_state = gRightEncoder.previous_state & 0x03U;

    __set_PRIMASK(primask);
}

void encoder_take_snapshot(EncoderSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    snapshot->left_count = gLeftEncoder.count;
    snapshot->right_count = gRightEncoder.count;
    snapshot->left_delta = gLeftEncoder.delta;
    snapshot->right_delta = gRightEncoder.delta;
    snapshot->left_ab_state = gLeftEncoder.previous_state & 0x03U;
    snapshot->right_ab_state = gRightEncoder.previous_state & 0x03U;

    gLeftEncoder.delta = 0;
    gRightEncoder.delta = 0;

    __set_PRIMASK(primask);
}

static int32_t take_delta(Encoder *encoder)
{
    uint32_t primask;
    int32_t delta;

    primask = __get_PRIMASK();
    __disable_irq();
    delta = encoder->delta;
    encoder->delta = 0;
    __set_PRIMASK(primask);
    return delta;
}

int32_t encoder_take_left_delta(void)
{
    return take_delta(&gLeftEncoder);
}

int32_t encoder_take_right_delta(void)
{
    return take_delta(&gRightEncoder);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending;
    uint32_t inputs;

    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) !=
        DL_INTERRUPT_GROUP1_IIDX_GPIOA) {
        return;
    }

    pending = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_PORT, ENCODER_PIN_MASK);
    if (pending == 0U) {
        return;
    }

    /* Clear captured edges early. A later edge remains pending for the next
     * IRQ instead of being accidentally cleared at the end of this handler. */
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, pending);
    inputs = DL_GPIO_readPins(ENCODER_PORT, ENCODER_PIN_MASK);

    if ((pending & ENCODER_ENC_L_A_PIN) != 0U) {
        gLeftEncoder.a_edge_count++;
    }
    if ((pending & ENCODER_ENC_L_B_PIN) != 0U) {
        gLeftEncoder.b_edge_count++;
    }
    if ((pending & ENCODER_ENC_R_A_PIN) != 0U) {
        gRightEncoder.a_edge_count++;
    }
    if ((pending & ENCODER_ENC_R_B_PIN) != 0U) {
        gRightEncoder.b_edge_count++;
    }

    if ((pending & (ENCODER_ENC_L_A_PIN | ENCODER_ENC_L_B_PIN)) != 0U) {
        encoder_update(
            &gLeftEncoder,
            read_ab_state(
                inputs,
                ENCODER_ENC_L_A_PIN,
                ENCODER_ENC_L_B_PIN),
            LEFT_ENCODER_SIGN);
    }

    if ((pending & (ENCODER_ENC_R_A_PIN | ENCODER_ENC_R_B_PIN)) != 0U) {
        encoder_update(
            &gRightEncoder,
            read_ab_state(
                inputs,
                ENCODER_ENC_R_A_PIN,
                ENCODER_ENC_R_B_PIN),
            RIGHT_ENCODER_SIGN);
    }
}
