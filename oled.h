#ifndef OLED_H
#define OLED_H

#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_clear_page(uint8_t page);
void oled_show_string(uint8_t x, uint8_t page, const char *text);
void oled_show_i32(uint8_t x, uint8_t page, int32_t value);
void oled_update_pages(uint8_t first_page, uint8_t page_count);

#endif
