#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BUTTON_SELECT = 0,
    BUTTON_CONFIRM
} ButtonId;

void button_init(void);
void button_update(uint32_t now_ms);
bool button_take_press(ButtonId button);

#endif
