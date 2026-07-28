#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_time.h"
#include "button.h"
#include "encoder.h"
#include "motion.h"
#include "motor.h"
#include "oled.h"
#include "serial_log.h"

typedef enum {
    MENU_FORWARD_500 = 0,
    MENU_REVERSE_200,
    MENU_LEFT_90,
    MENU_RIGHT_90,
    MENU_COUNT
} MenuItem;

static const char *motion_state_text(MotionState state)
{
    switch (state) {
    case MOTION_DISTANCE:
    case MOTION_TURN: return "RUN";
    case MOTION_BRAKING: return "BRAKE";
    case MOTION_DONE: return "DONE";
    case MOTION_TIMEOUT: return "TIMEOUT";
    case MOTION_ENCODER_FAULT: return "ENCFAULT";
    case MOTION_IDLE:
    default: return "IDLE";
    }
}

static void draw_menu_line(uint8_t page, bool selected, const char *text)
{
    oled_clear_page(page);
    oled_show_string(0U, page, selected ? ">" : " ");
    oled_show_string(12U, page, text);
}

static void draw_screen(MenuItem selected)
{
    int32_t left_count;
    int32_t right_count;

    encoder_get_counts(&left_count, &right_count);
    oled_clear();
    oled_show_string(0U, 0U, "ENCODER MOTION");
    oled_show_string(0U, 1U, "S1 SELECT S2 OK");
    draw_menu_line(2U, selected == MENU_FORWARD_500, "FWD 500MM");
    draw_menu_line(3U, selected == MENU_REVERSE_200, "REV 200MM");
    draw_menu_line(4U, selected == MENU_LEFT_90, "LEFT 90DEG");
    draw_menu_line(5U, selected == MENU_RIGHT_90, "RIGHT 90DEG");
    oled_show_string(0U, 6U, "STATE:");
    oled_show_string(42U, 6U, motion_state_text(motion_get_state()));
    oled_show_string(0U, 7U, "L:");
    oled_show_i32(12U, 7U, left_count);
    oled_show_string(66U, 7U, "R:");
    oled_show_i32(78U, 7U, right_count);
    oled_request_refresh(0U, 8U);
}

static bool start_menu_motion(MenuItem selected)
{
    switch (selected) {
    case MENU_FORWARD_500:
        return motion_start_distance_mm(500, 300);
    case MENU_REVERSE_200:
        return motion_start_distance_mm(-200, 250);
    case MENU_LEFT_90:
        return motion_start_turn_deg(90, 250);
    case MENU_RIGHT_90:
        return motion_start_turn_deg(-90, 250);
    default:
        return false;
    }
}

int main(void)
{
    MenuItem selected = MENU_FORWARD_500;
    uint32_t last_control_ms = 0U;
    uint32_t last_oled_ms = 0U;
    MotionState last_drawn_state;

    SYSCFG_DL_init();
    motor_init();
    encoder_init();
    motion_init();
    button_init();
    app_time_init();
    serial_log_init();
    (void)oled_init();
    __enable_irq();

    last_drawn_state = motion_get_state();
    draw_screen(selected);

    while (1) {
        uint32_t now_ms = app_time_now_ms();

        button_update(now_ms);

        if ((uint32_t)(now_ms - last_control_ms) >=
            APP_CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;

            if (motion_is_busy()) {
                if (button_take_press(BUTTON_CONFIRM)) {
                    motion_abort();
                } else {
                    motion_update(now_ms);
                }
            } else {
                if (button_take_press(BUTTON_SELECT)) {
                    selected = (MenuItem)(
                        ((uint8_t)selected + 1U) % (uint8_t)MENU_COUNT);
                    draw_screen(selected);
                }
                if (button_take_press(BUTTON_CONFIRM)) {
                    (void)start_menu_motion(selected);
                }
            }
        }

        if ((motion_get_state() != last_drawn_state) ||
            ((uint32_t)(now_ms - last_oled_ms) >=
             APP_OLED_STATUS_PERIOD_MS)) {
            last_drawn_state = motion_get_state();
            last_oled_ms = now_ms;
            draw_screen(selected);
        }

        serial_log_periodic(now_ms);
        serial_log_service();
        oled_service(now_ms);
        __WFI();
    }
}
