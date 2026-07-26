#ifndef SERIAL_LOG_H
#define SERIAL_LOG_H

#include <stdint.h>

void serial_log_init(void);
void serial_log_task(uint32_t now_tick);
void serial_log_service(void);
uint32_t serial_log_get_dropped_message_count(void);
uint16_t serial_log_get_queued_byte_count(void);

#endif
