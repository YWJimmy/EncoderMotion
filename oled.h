#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>

bool oled_init(void);
void oled_clear(void);
void oled_clear_page(uint8_t page);
void oled_show_string(uint8_t x, uint8_t page, const char *text);
void oled_show_i32(uint8_t x, uint8_t page, int32_t value);
void oled_request_refresh(uint8_t first_page, uint8_t page_count);
void oled_service(uint32_t now_ms);
bool oled_is_online(void);
uint32_t oled_get_error_count(void);
uint32_t oled_get_reconnect_count(void);

#endif
