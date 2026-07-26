#ifndef APP_TICK_H
#define APP_TICK_H

#include <stdbool.h>
#include <stdint.h>

void app_tick_init(void);

/*
 * Returns true once when one or more real SysTick periods have elapsed.
 * Missed periods are coalesced instead of replayed as zero-time control calls.
 */
bool app_tick_take(void);

uint32_t app_tick_now(void);
uint32_t app_tick_get_overrun_count(void);

#endif
