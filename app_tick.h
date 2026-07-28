#ifndef APP_TICK_H
#define APP_TICK_H

#include <stdbool.h>
#include <stdint.h>

void app_tick_init(void);

/* Returns one coalesced control request and an atomic tick snapshot.
 * Missed periods are never replayed as zero-time control iterations. */
bool app_tick_take(uint32_t *now_tick);

uint32_t app_tick_now(void);
uint32_t app_tick_get_overrun_count(void);

#endif
