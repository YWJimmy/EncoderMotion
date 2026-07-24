#include "encoder.h"

#include <stdbool.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define ENCODER_PIN_MASK            \
    (ENCODER_ENC_L_A_PIN |          \
     ENCODER_ENC_L_B_PIN |          \
     ENCODER_ENC_R_A_PIN |          \
     ENCODER_ENC_R_B_PIN)

#define SYSTICK_MASK                (0x00FFFFFFUL)

/*
 * 右编码器机械安装方向与左侧镜像。
 * 正常向前时，软件将左右轮统一为正计数。
 */
#define LEFT_ENCODER_SIGN           (-1)
#define RIGHT_ENCODER_SIGN          (1)

Encoder gLeftEncoder;
Encoder gRightEncoder;

static uint32_t gLastEdgeTick[4];
static uint8_t gEdgeSeen[4];

/*
 * 下标：
 *
 * previous_state << 2 | current_state
 *
 * A 为高位，B 为低位。
 */
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

static bool accept_edge(uint8_t channel)
{
    uint32_t now;
    uint32_t elapsed;

    now = SysTick->VAL & SYSTICK_MASK;

    if (gEdgeSeen[channel] == 0U) {
        gEdgeSeen[channel] = 1U;
        gLastEdgeTick[channel] = now;
        return true;
    }

    /*
     * SysTick 是向下计数器，因此：
     *
     * elapsed = last - now
     */
    elapsed =
        (gLastEdgeTick[channel] - now) &
        SYSTICK_MASK;

    if (elapsed <
        APP_ENCODER_DEGLITCH_CYCLES) {
        return false;
    }

    gLastEdgeTick[channel] = now;
    return true;
}

static void encoder_update(
    Encoder *encoder,
    uint8_t new_state,
    int8_t direction_sign)
{
    uint8_t table_index;
    int8_t step;

    table_index =
        (uint8_t)(
            (encoder->previous_state << 2U) |
            new_state);

    step =
        gQuadratureTable[table_index];

    step =
        (int8_t)(step * direction_sign);

    encoder->count += step;
    encoder->delta += step;
    encoder->previous_state = new_state;
}

void encoder_init(void)
{
    uint32_t inputs;

    /*
     * 使用 SysTick 作为自由运行的边沿间隔计时器，
     * 不开启 SysTick 中断。
     */
    SysTick->LOAD = SYSTICK_MASK;
    SysTick->VAL = 0U;
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_ENABLE_Msk;

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

    inputs =
        DL_GPIO_readPins(
            ENCODER_PORT,
            ENCODER_PIN_MASK);

    gLeftEncoder.count = 0;
    gLeftEncoder.delta = 0;
    gLeftEncoder.previous_state =
        read_ab_state(
            inputs,
            ENCODER_ENC_L_A_PIN,
            ENCODER_ENC_L_B_PIN);

    gRightEncoder.count = 0;
    gRightEncoder.delta = 0;
    gRightEncoder.previous_state =
        read_ab_state(
            inputs,
            ENCODER_ENC_R_A_PIN,
            ENCODER_ENC_R_B_PIN);

    for (uint8_t i = 0U; i < 4U; i++) {
        gEdgeSeen[i] = 0U;
        gLastEdgeTick[i] = 0U;
    }

    /*
     * PA28、PA31 位于 upper pins。
     * PA12、PA13 位于 lower pins。
     */
    DL_GPIO_setUpperPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_28_EDGE_RISE_FALL |
        DL_GPIO_PIN_31_EDGE_RISE_FALL);

    DL_GPIO_setLowerPinsPolarity(
        ENCODER_PORT,
        DL_GPIO_PIN_12_EDGE_RISE_FALL |
        DL_GPIO_PIN_13_EDGE_RISE_FALL);

    DL_GPIO_clearInterruptStatus(
        ENCODER_PORT,
        ENCODER_PIN_MASK);

    DL_GPIO_enableInterrupt(
        ENCODER_PORT,
        ENCODER_PIN_MASK);

    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

void encoder_reset_counts(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    gLeftEncoder.count = 0;
    gLeftEncoder.delta = 0;

    gRightEncoder.count = 0;
    gRightEncoder.delta = 0;

    __set_PRIMASK(primask);
}

void encoder_get_counts(
    int32_t *left_count,
    int32_t *right_count)
{
    uint32_t primask;

    if (left_count == NULL ||
        right_count == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    *left_count = gLeftEncoder.count;
    *right_count = gRightEncoder.count;

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
    bool left_changed = false;
    bool right_changed = false;

    if (DL_Interrupt_getPendingGroup(
            DL_INTERRUPT_GROUP_1) !=
        DL_INTERRUPT_GROUP1_IIDX_GPIOA) {
        return;
    }

    pending =
        DL_GPIO_getEnabledInterruptStatus(
            ENCODER_PORT,
            ENCODER_PIN_MASK);

    inputs =
        DL_GPIO_readPins(
            ENCODER_PORT,
            ENCODER_PIN_MASK);

    if ((pending & ENCODER_ENC_L_A_PIN) != 0U) {
        if (accept_edge(0U)) {
            left_changed = true;
        }
    }

    if ((pending & ENCODER_ENC_L_B_PIN) != 0U) {
        if (accept_edge(1U)) {
            left_changed = true;
        }
    }

    if ((pending & ENCODER_ENC_R_A_PIN) != 0U) {
        if (accept_edge(2U)) {
            right_changed = true;
        }
    }

    if ((pending & ENCODER_ENC_R_B_PIN) != 0U) {
        if (accept_edge(3U)) {
            right_changed = true;
        }
    }

    if (left_changed) {
        encoder_update(
            &gLeftEncoder,
            read_ab_state(
                inputs,
                ENCODER_ENC_L_A_PIN,
                ENCODER_ENC_L_B_PIN),
            LEFT_ENCODER_SIGN);
    }

    if (right_changed) {
        encoder_update(
            &gRightEncoder,
            read_ab_state(
                inputs,
                ENCODER_ENC_R_A_PIN,
                ENCODER_ENC_R_B_PIN),
            RIGHT_ENCODER_SIGN);
    }

    DL_GPIO_clearInterruptStatus(
        ENCODER_PORT,
        pending);
}
