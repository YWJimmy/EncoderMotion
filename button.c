#include "button.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint8_t active_level;
    uint8_t raw_level;
    uint8_t stable_level;
    uint8_t press_pending;
    uint32_t raw_change_ms;
} ButtonRuntime;

static ButtonRuntime gButtons[2] = {
    {BUTTON_SELECT_PORT, BUTTON_SELECT_S1_PIN, 1U, 0U, 0U, 0U, 0U},
    {BUTTON_CONFIRM_PORT, BUTTON_CONFIRM_S2_PIN, 0U, 1U, 1U, 0U, 0U}
};

void button_init(void)
{
    uint8_t index;

    for (index = 0U; index < 2U; index++) {
        uint8_t level = DL_GPIO_readPins(
            gButtons[index].port,
            gButtons[index].pin) != 0U;
        gButtons[index].raw_level = level;
        gButtons[index].stable_level = level;
        gButtons[index].press_pending = 0U;
        gButtons[index].raw_change_ms = 0U;
    }
}

void button_update(uint32_t now_ms)
{
    uint8_t index;

    for (index = 0U; index < 2U; index++) {
        ButtonRuntime *button = &gButtons[index];
        uint8_t level = DL_GPIO_readPins(
            button->port, button->pin) != 0U;

        if (level != button->raw_level) {
            button->raw_level = level;
            button->raw_change_ms = now_ms;
        } else if ((level != button->stable_level) &&
                   ((uint32_t)(now_ms - button->raw_change_ms) >=
                    APP_BUTTON_DEBOUNCE_MS)) {
            button->stable_level = level;
            if (level == button->active_level) {
                button->press_pending = 1U;
            }
        }
    }
}

bool button_take_press(ButtonId button)
{
    bool pressed;

    if ((uint8_t)button >= 2U) {
        return false;
    }

    pressed = gButtons[button].press_pending != 0U;
    gButtons[button].press_pending = 0U;
    return pressed;
}
