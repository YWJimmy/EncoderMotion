#ifndef APP_TIME_H
#define APP_TIME_H

#include <stdint.h>

void app_time_init(void);
uint32_t app_time_now_ms(void);
uint32_t app_time_get_overrun_count(void);

#endif
