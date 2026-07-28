#include "encoder.h"

#include <stddef.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define ENCODER_LEFT_MASK \
    (ENCODER_ENC_L_A_PIN | ENCODER_ENC_L_B_PIN)
#define ENCODER_RIGHT_MASK \
    (ENCODER_ENC_R_A_PIN | ENCODER_ENC_R_B_PIN)
#define ENCODER_ALL_MASK (ENCODER_LEFT_MASK | ENCODER_RIGHT_MASK)

typedef struct {
    volatile int32_t count;
    volatile int32_t delta;
    volatile uint32_t valid_transition_count;
    volatile uint32_t invalid_transition_count;
    volatile uint32_t duplicate_state_count;
    volatile uint32_t a_edge_count;
    volatile uint32_t b_edge_count;
    volatile uint32_t merged_edge_count;
    uint8_t previous_state;
} EncoderRuntime;

static EncoderRuntime gLeft;
static EncoderRuntime gRight;

/* Index = previous AB << 2 | current AB. A is bit 1, B is bit 0. */
static const int8_t gQuadratureTable[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

int8_t encoder_decode_transition(uint8_t previous_state, uint8_t new_state)
{
    uint8_t index = (uint8_t)(
        ((previous_state & 0x03U) << 2U) |
        (new_state & 0x03U));
    return gQuadratureTable[index];
}

static uint8_t read_ab(
    uint32_t input_state,
    uint32_t pin_a,
    uint32_t pin_b)
{
    uint8_t state = 0U;

    if ((input_state & pin_a) != 0U) {
        state |= 2U;
    }
    if ((input_state & pin_b) != 0U) {
        state |= 1U;
    }
    return state;
}

static void reset_runtime(EncoderRuntime *encoder, uint8_t ab_state)
{
    encoder->count = 0;
    encoder->delta = 0;
    encoder->valid_transition_count = 0U;
    encoder->invalid_transition_count = 0U;
    encoder->duplicate_state_count = 0U;
    encoder->a_edge_count = 0U;
    encoder->b_edge_count = 0U;
    encoder->merged_edge_count = 0U;
    encoder->previous_state = (uint8_t)(ab_state & 0x03U);
}

static void process_transition(
    EncoderRuntime *encoder,
    uint8_t new_state,
    int8_t direction_sign,
    bool merged_edges)
{
    int8_t step;

    new_state &= 0x03U;

    if (merged_edges) {
        /* The hardware pending bits do not preserve A/B edge order. */
        encoder->merged_edge_count++;
        encoder->previous_state = new_state;
        return;
    }

    if (new_state == encoder->previous_state) {
        encoder->duplicate_state_count++;
        return;
    }

    step = encoder_decode_transition(encoder->previous_state, new_state);
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

    DL_GPIO_disableInterrupt(ENCODER_PORT, ENCODER_ALL_MASK);

    /* Re-apply identical electrical settings on all four inputs. */
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

    DL_GPIO_setUpperPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_28_EDGE_RISE_FALL |
        DL_GPIO_PIN_31_EDGE_RISE_FALL);
    DL_GPIO_setLowerPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_13_EDGE_RISE_FALL);

    inputs = DL_GPIO_readPins(ENCODER_PORT, ENCODER_ALL_MASK);
    reset_runtime(
        &gLeft,
        read_ab(inputs, ENCODER_ENC_L_A_PIN, ENCODER_ENC_L_B_PIN));
    reset_runtime(
        &gRight,
        read_ab(inputs, ENCODER_ENC_R_A_PIN, ENCODER_ENC_R_B_PIN));

    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ALL_MASK);
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    DL_GPIO_enableInterrupt(ENCODER_PORT, ENCODER_ALL_MASK);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_reset_counts(void)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t inputs;

    __disable_irq();
    inputs = DL_GPIO_readPins(ENCODER_PORT, ENCODER_ALL_MASK);
    gLeft.count = 0;
    gLeft.delta = 0;
    gLeft.previous_state = read_ab(
        inputs, ENCODER_ENC_L_A_PIN, ENCODER_ENC_L_B_PIN);
    gRight.count = 0;
    gRight.delta = 0;
    gRight.previous_state = read_ab(
        inputs, ENCODER_ENC_R_A_PIN, ENCODER_ENC_R_B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_ALL_MASK);
    __set_PRIMASK(primask);
}

void encoder_reset_statistics(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    gLeft.valid_transition_count = 0U;
    gLeft.invalid_transition_count = 0U;
    gLeft.duplicate_state_count = 0U;
    gLeft.a_edge_count = 0U;
    gLeft.b_edge_count = 0U;
    gLeft.merged_edge_count = 0U;
    gRight.valid_transition_count = 0U;
    gRight.invalid_transition_count = 0U;
    gRight.duplicate_state_count = 0U;
    gRight.a_edge_count = 0U;
    gRight.b_edge_count = 0U;
    gRight.merged_edge_count = 0U;
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
    snapshot->left_count = gLeft.count;
    snapshot->right_count = gRight.count;
    snapshot->left_delta = gLeft.delta;
    snapshot->right_delta = gRight.delta;
    snapshot->left_ab = gLeft.previous_state;
    snapshot->right_ab = gRight.previous_state;
    gLeft.delta = 0;
    gRight.delta = 0;
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
    *left_count = gLeft.count;
    *right_count = gRight.count;
    __set_PRIMASK(primask);
}

static void copy_statistics(
    const EncoderRuntime *source,
    EncoderStatistics *destination)
{
    destination->count = source->count;
    destination->valid_transition_count = source->valid_transition_count;
    destination->invalid_transition_count = source->invalid_transition_count;
    destination->duplicate_state_count = source->duplicate_state_count;
    destination->a_edge_count = source->a_edge_count;
    destination->b_edge_count = source->b_edge_count;
    destination->merged_edge_count = source->merged_edge_count;
    destination->ab_state = source->previous_state;
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
    copy_statistics(&gLeft, left_statistics);
    copy_statistics(&gRight, right_statistics);
    __set_PRIMASK(primask);
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending;
    uint32_t inputs;
    bool left_merged;
    bool right_merged;

    if (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1) !=
        DL_INTERRUPT_GROUP1_IIDX_GPIOA) {
        return;
    }

    /* One ISR entry performs one pending-status read and one clear only. */
    pending = DL_GPIO_getEnabledInterruptStatus(
        ENCODER_PORT, ENCODER_ALL_MASK);
    if (pending == 0U) {
        return;
    }

    inputs = DL_GPIO_readPins(ENCODER_PORT, ENCODER_ALL_MASK);
    DL_GPIO_clearInterruptStatus(ENCODER_PORT, pending);

    if ((pending & ENCODER_ENC_L_A_PIN) != 0U) {
        gLeft.a_edge_count++;
    }
    if ((pending & ENCODER_ENC_L_B_PIN) != 0U) {
        gLeft.b_edge_count++;
    }
    if ((pending & ENCODER_ENC_R_A_PIN) != 0U) {
        gRight.a_edge_count++;
    }
    if ((pending & ENCODER_ENC_R_B_PIN) != 0U) {
        gRight.b_edge_count++;
    }

    left_merged =
        (pending & ENCODER_LEFT_MASK) == ENCODER_LEFT_MASK;
    right_merged =
        (pending & ENCODER_RIGHT_MASK) == ENCODER_RIGHT_MASK;

    if ((pending & ENCODER_LEFT_MASK) != 0U) {
        process_transition(
            &gLeft,
            read_ab(inputs, ENCODER_ENC_L_A_PIN, ENCODER_ENC_L_B_PIN),
            (int8_t)APP_LEFT_ENCODER_SIGN,
            left_merged);
    }

    if ((pending & ENCODER_RIGHT_MASK) != 0U) {
        process_transition(
            &gRight,
            read_ab(inputs, ENCODER_ENC_R_A_PIN, ENCODER_ENC_R_B_PIN),
            (int8_t)APP_RIGHT_ENCODER_SIGN,
            right_merged);
    }
}
