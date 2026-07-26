#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_tick.h"
#include "encoder.h"
#include "motion.h"
#include "motor.h"
#include "oled.h"
#include "serial_log.h"

#define SELECT_BUTTON_PORT  (GPIOA)
#define SELECT_BUTTON_PIN   (DL_GPIO_PIN_18)
#define SELECT_BUTTON_IOMUX (IOMUX_PINCM40)

#define CONFIRM_BUTTON_PORT  (GPIOB)
#define CONFIRM_BUTTON_PIN   (DL_GPIO_PIN_21)
#define CONFIRM_BUTTON_IOMUX (IOMUX_PINCM49)

#define MENU_ITEM_COUNT (4U)

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint8_t active_state;
    uint8_t previous_sample;
    uint8_t stable_state;
    uint8_t same_samples;
} Button;

typedef enum {
    MENU_FORWARD_500_MM = 0,
    MENU_REVERSE_200_MM,
    MENU_LEFT_90_DEG,
    MENU_RIGHT_90_DEG
} MenuItem;

static Button gSelectButton = {
    SELECT_BUTTON_PORT, SELECT_BUTTON_PIN, 1U, 0U, 0U, 0U
};
static Button gConfirmButton = {
    CONFIRM_BUTTON_PORT, CONFIRM_BUTTON_PIN, 0U, 1U, 1U, 0U
};

static void buttons_init(void)
{
    DL_GPIO_initDigitalInputFeatures(
        SELECT_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(
        CONFIRM_BUTTON_IOMUX,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    gSelectButton.previous_sample =
        DL_GPIO_readPins(SELECT_BUTTON_PORT, SELECT_BUTTON_PIN) != 0U ?
        1U : 0U;
    gSelectButton.stable_state = gSelectButton.previous_sample;

    gConfirmButton.previous_sample =
        DL_GPIO_readPins(CONFIRM_BUTTON_PORT, CONFIRM_BUTTON_PIN) != 0U ?
        1U : 0U;
    gConfirmButton.stable_state = gConfirmButton.previous_sample;
}

static bool button_take_press(Button *button)
{
    uint8_t sample =
        DL_GPIO_readPins(button->port, button->pin) != 0U ?
        1U : 0U;

    if (sample != button->previous_sample) {
        button->previous_sample = sample;
        button->same_samples = 0U;
        return false;
    }

    if (button->same_samples < APP_BUTTON_DEBOUNCE_TICKS) {
        button->same_samples++;
    }

    if ((button->same_samples == APP_BUTTON_DEBOUNCE_TICKS) &&
        (button->stable_state != sample)) {
        button->stable_state = sample;
        return sample == button->active_state;
    }

    return false;
}

static const char *motion_state_text(MotionState state)
{
    switch (state) {
    case MOTION_RUNNING_DISTANCE:
    case MOTION_RUNNING_TURN:
        return "RUN";
    case MOTION_BRAKING:
        return "BRAKE";
    case MOTION_DONE:
        return "DONE";
    case MOTION_TIMEOUT:
        return "TIMEOUT";
    case MOTION_ENCODER_FAULT:
        return "ENCFAULT";
    case MOTION_IDLE:
    default:
        return "IDLE";
    }
}

static void draw_menu_line(uint8_t page, bool selected, const char *text)
{
    oled_clear_page(page);
    oled_show_string(0U, page, selected ? ">" : " ");
    oled_show_string(12U, page, text);
}

static void draw_status_pages(void)
{
    int32_t left_count;
    int32_t right_count;

    encoder_get_counts(&left_count, &right_count);

    oled_clear_page(6U);
    oled_show_string(0U, 6U, "STATE:");
    oled_show_string(42U, 6U, motion_state_text(motion_get_state()));

    oled_clear_page(7U);
    oled_show_string(0U, 7U, "L:");
    oled_show_i32(12U, 7U, left_count);
    oled_show_string(66U, 7U, "R:");
    oled_show_i32(78U, 7U, right_count);

    (void)oled_update_pages(6U, 2U);
}

static void draw_menu(MenuItem selected)
{
    oled_clear();
    oled_show_string(0U, 0U, "ENCODER MOTION");
    oled_show_string(0U, 1U, "S1:SELECT S2:OK");
    draw_menu_line(2U, selected == MENU_FORWARD_500_MM, "FWD 500MM");
    draw_menu_line(3U, selected == MENU_REVERSE_200_MM, "REV 200MM");
    draw_menu_line(4U, selected == MENU_LEFT_90_DEG, "LEFT 90DEG");
    draw_menu_line(5U, selected == MENU_RIGHT_90_DEG, "RIGHT 90DEG");
    oled_show_string(0U, 6U, "STATE:");
    oled_show_string(42U, 6U, motion_state_text(motion_get_state()));
    (void)oled_update_pages(0U, 8U);
}

static bool start_selected_motion(MenuItem selected)
{
    switch (selected) {
    case MENU_FORWARD_500_MM:
        return motion_start_distance_mm(500, 300);
    case MENU_REVERSE_200_MM:
        return motion_start_distance_mm(-200, 250);
    case MENU_LEFT_90_DEG:
        return motion_start_turn_deg(90, 250);
    case MENU_RIGHT_90_DEG:
        return motion_start_turn_deg(-90, 250);
    default:
        return false;
    }
}

int main(void)
{
    MenuItem selected = MENU_FORWARD_500_MM;
    MotionState displayed_state;
    uint16_t status_refresh_tick = 0U;

    SYSCFG_DL_init();
    motor_init();
    encoder_init();
    motion_init();
    buttons_init();
    (void)oled_init();
    app_tick_init();
    serial_log_init();
    __enable_irq();

    draw_menu(selected);
    displayed_state = motion_get_state();

    while (1) {
        if (app_tick_take()) {
            bool select_pressed = button_take_press(&gSelectButton);
            bool confirm_pressed = button_take_press(&gConfirmButton);

            if (motion_is_busy()) {
                if (confirm_pressed) {
                    motion_abort();
                } else {
                    motion_update();
                }
            } else {
                if (select_pressed) {
                    selected = (MenuItem)(
                        ((uint8_t)selected + 1U) % MENU_ITEM_COUNT);
                    draw_menu(selected);
                    status_refresh_tick = 0U;
                }

                if (confirm_pressed) {
                    (void)start_selected_motion(selected);
                }
            }

            if (motion_get_state() != displayed_state) {
                displayed_state = motion_get_state();
                draw_status_pages();
                status_refresh_tick = 0U;
            } else if (status_refresh_tick >=
                       APP_OLED_STATUS_PERIOD_TICKS) {
                draw_status_pages();
                status_refresh_tick = 0U;
            } else {
                status_refresh_tick++;
            }

            serial_log_task(app_tick_now());
            continue;
        }

        oled_service(app_tick_now());
        serial_log_service();
        __WFI();
    }
}
