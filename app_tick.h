#ifndef APP_TICK_H
#define APP_TICK_H

#include <stdbool.h>
#include <stdint.h>

void app_tick_init(void);
bool app_tick_take(void);
uint32_t app_tick_now(void);
uint32_t app_tick_get_overrun_count(void);

#endif
