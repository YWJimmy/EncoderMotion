#include "oled.h"

#include <string.h>

#include "ti_msp_dl_config.h"

#define OLED_PORT                  (GPIOB)
#define OLED_SCL_PIN               (DL_GPIO_PIN_8)
#define OLED_SDA_PIN               (DL_GPIO_PIN_9)
#define OLED_SCL_IOMUX             (IOMUX_PINCM25)
#define OLED_SDA_IOMUX             (IOMUX_PINCM26)
#define OLED_WIDTH                 (128U)
#define OLED_PAGE_COUNT            (8U)
#define OLED_WRITE_ADDRESS         (0x78U)
#define OLED_POWER_UP_DELAY_CYCLES (1600000U)

static uint8_t gOledBuffer[OLED_PAGE_COUNT][OLED_WIDTH];

static void oled_delay(void)
{
    DL_Common_delayCycles(64U);
}

static void oled_set_scl(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_setPins(OLED_PORT, OLED_SCL_PIN);
        DL_GPIO_disableOutput(OLED_PORT, OLED_SCL_PIN);
    } else {
        DL_GPIO_clearPins(OLED_PORT, OLED_SCL_PIN);
        DL_GPIO_enableOutput(OLED_PORT, OLED_SCL_PIN);
    }
    oled_delay();
}

static void oled_set_sda(uint8_t high)
{
    if (high != 0U) {
        DL_GPIO_setPins(OLED_PORT, OLED_SDA_PIN);
        DL_GPIO_disableOutput(OLED_PORT, OLED_SDA_PIN);
    } else {
        DL_GPIO_clearPins(OLED_PORT, OLED_SDA_PIN);
        DL_GPIO_enableOutput(OLED_PORT, OLED_SDA_PIN);
    }
    oled_delay();
}

static void oled_start(void)
{
    oled_set_sda(1U);
    oled_set_scl(1U);
    oled_set_sda(0U);
    oled_set_scl(0U);
}

static void oled_stop(void)
{
    oled_set_sda(0U);
    oled_set_scl(1U);
    oled_set_sda(1U);
}

static void oled_send_byte(uint8_t value)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; bit++) {
        oled_set_sda((value & 0x80U) != 0U ? 1U : 0U);
        oled_set_scl(1U);
        oled_set_scl(0U);
        value <<= 1U;
    }

    oled_set_sda(1U);
    oled_set_scl(1U);
    oled_set_scl(0U);
}

static void oled_write_command(uint8_t command)
{
    oled_start();
    oled_send_byte(OLED_WRITE_ADDRESS);
    oled_send_byte(0x00U);
    oled_send_byte(command);
    oled_stop();
}

static void oled_write_page(const uint8_t *data)
{
    uint8_t index;

    oled_start();
    oled_send_byte(OLED_WRITE_ADDRESS);
    oled_send_byte(0x40U);
    for (index = 0U; index < OLED_WIDTH; index++) {
        oled_send_byte(data[index]);
    }
    oled_stop();
}

static const uint8_t *oled_glyph(char character)
{
    static const uint8_t glyphs[][5] = {
        {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E},
        {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F}, {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
        {0x00,0x00,0x00,0x00,0x00}, {0x00,0x36,0x36,0x00,0x00},
        {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00},
        {0x02,0x01,0x51,0x09,0x06}, {0x14,0x14,0x3E,0x14,0x14}
    };
    uint8_t index;

    if (character >= '0' && character <= '9') {
        index = (uint8_t)(character - '0');
    } else if (character >= 'A' && character <= 'Z') {
        index = (uint8_t)(10U + character - 'A');
    } else if (character == ' ') {
        index = 36U;
    } else if (character == ':') {
        index = 37U;
    } else if (character == '-') {
        index = 38U;
    } else if (character == '.') {
        index = 39U;
    } else if (character == '>') {
        index = 41U;
    } else {
        index = 40U;
    }

    return glyphs[index];
}

static void oled_show_char(uint8_t x, uint8_t page, char character)
{
    const uint8_t *glyph;
    uint8_t column;

    if (page >= OLED_PAGE_COUNT || x > (OLED_WIDTH - 6U)) {
        return;
    }

    glyph = oled_glyph(character);
    for (column = 0U; column < 5U; column++) {
        gOledBuffer[page][x + column] = glyph[column];
    }
    gOledBuffer[page][x + 5U] = 0U;
}

void oled_init(void)
{
    static const uint8_t commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
        0xA1U, 0xC8U, 0xDAU, 0x12U, 0x81U, 0xCFU, 0xD9U, 0xF1U,
        0xDBU, 0x30U, 0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };
    uint8_t index;

    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_setPins(OLED_PORT, OLED_SCL_PIN | OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_PORT, OLED_SCL_PIN | OLED_SDA_PIN);

    DL_Common_delayCycles(OLED_POWER_UP_DELAY_CYCLES);
    oled_clear();

    for (index = 0U; index < (uint8_t)sizeof(commands); index++) {
        oled_write_command(commands[index]);
    }

    oled_update_pages(0U, OLED_PAGE_COUNT);
}

void oled_clear(void)
{
    (void)memset(gOledBuffer, 0, sizeof(gOledBuffer));
}

void oled_clear_page(uint8_t page)
{
    if (page < OLED_PAGE_COUNT) {
        (void)memset(gOledBuffer[page], 0, OLED_WIDTH);
    }
}

void oled_show_string(uint8_t x, uint8_t page, const char *text)
{
    while (*text != '\0' && x <= (OLED_WIDTH - 6U)) {
        oled_show_char(x, page, *text++);
        x = (uint8_t)(x + 6U);
    }
}

void oled_show_i32(uint8_t x, uint8_t page, int32_t value)
{
    char digits[10];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (value < 0) {
        oled_show_char(x, page, '-');
        x = (uint8_t)(x + 6U);
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[count++] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U && count < (uint8_t)sizeof(digits));

    while (count != 0U && x <= (OLED_WIDTH - 6U)) {
        oled_show_char(x, page, digits[--count]);
        x = (uint8_t)(x + 6U);
    }
}

void oled_update_pages(uint8_t first_page, uint8_t page_count)
{
    uint8_t page;
    uint8_t last_page;

    if (first_page >= OLED_PAGE_COUNT) {
        return;
    }

    last_page = (uint8_t)(first_page + page_count);
    if (last_page > OLED_PAGE_COUNT || last_page < first_page) {
        last_page = OLED_PAGE_COUNT;
    }

    for (page = first_page; page < last_page; page++) {
        oled_write_command((uint8_t)(0xB0U | page));
        oled_write_command(0x10U);
        oled_write_command(0x00U);
        oled_write_page(gOledBuffer[page]);
    }
}
