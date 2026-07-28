#include "oled.h"

#include <string.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define OLED_WIDTH          (128U)
#define OLED_PAGE_COUNT     (8U)
#define OLED_ADDRESS_WRITE  (0x78U)

static uint8_t gBuffer[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t gDirtyMask;
static bool gOnline;
static uint32_t gErrorCount;
static uint32_t gReconnectCount;
static uint32_t gNextReconnectMs;

static void delay_i2c(void)
{
    DL_Common_delayCycles(APP_OLED_I2C_DELAY_CYCLES);
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(OLED_IO_PORT, OLED_IO_SCL_PIN);
    delay_i2c();
}

static void scl_low(void)
{
    DL_GPIO_clearPins(OLED_IO_PORT, OLED_IO_SCL_PIN);
    DL_GPIO_enableOutput(OLED_IO_PORT, OLED_IO_SCL_PIN);
    delay_i2c();
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(OLED_IO_PORT, OLED_IO_SDA_PIN);
    delay_i2c();
}

static void sda_low(void)
{
    DL_GPIO_clearPins(OLED_IO_PORT, OLED_IO_SDA_PIN);
    DL_GPIO_enableOutput(OLED_IO_PORT, OLED_IO_SDA_PIN);
    delay_i2c();
}

static void i2c_start(void)
{
    sda_release();
    scl_release();
    sda_low();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    scl_release();
    sda_release();
}

static bool i2c_write_byte(uint8_t value)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        if ((value & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        scl_release();
        scl_low();
        value <<= 1U;
    }

    sda_release();
    scl_release();
    acknowledged =
        DL_GPIO_readPins(OLED_IO_PORT, OLED_IO_SDA_PIN) == 0U;
    scl_low();
    return acknowledged;
}

static bool write_command(uint8_t command)
{
    bool ok;

    i2c_start();
    ok = i2c_write_byte(OLED_ADDRESS_WRITE);
    ok = i2c_write_byte(0x00U) && ok;
    ok = i2c_write_byte(command) && ok;
    i2c_stop();
    return ok;
}

static bool write_page(uint8_t page)
{
    uint8_t index;
    bool ok;

    ok = write_command((uint8_t)(0xB0U | page));
    ok = write_command(0x10U) && ok;
    ok = write_command(0x00U) && ok;

    i2c_start();
    ok = i2c_write_byte(OLED_ADDRESS_WRITE) && ok;
    ok = i2c_write_byte(0x40U) && ok;
    for (index = 0U; index < OLED_WIDTH; index++) {
        ok = i2c_write_byte(gBuffer[page][index]) && ok;
    }
    i2c_stop();
    return ok;
}

static const uint8_t *glyph(char character)
{
    static const uint8_t glyphs[][5] = {
        {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
        {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},
        {0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},
        {0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
        {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},
        {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E},{0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},
        {0x3F,0x40,0x38,0x40,0x3F},{0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07},{0x61,0x51,0x49,0x45,0x43},
        {0x00,0x00,0x00,0x00,0x00},{0x00,0x36,0x36,0x00,0x00},
        {0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
        {0x02,0x01,0x51,0x09,0x06},{0x14,0x14,0x3E,0x14,0x14}
    };
    uint8_t index;

    if ((character >= '0') && (character <= '9')) {
        index = (uint8_t)(character - '0');
    } else if ((character >= 'A') && (character <= 'Z')) {
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

static void show_char(uint8_t x, uint8_t page, char character)
{
    const uint8_t *data;
    uint8_t column;

    if ((page >= OLED_PAGE_COUNT) || (x > OLED_WIDTH - 6U)) {
        return;
    }

    data = glyph(character);
    for (column = 0U; column < 5U; column++) {
        gBuffer[page][x + column] = data[column];
    }
    gBuffer[page][x + 5U] = 0U;
    gDirtyMask |= (uint8_t)(1U << page);
}

bool oled_init(void)
{
#if APP_OLED_ENABLE
    static const uint8_t commands[] = {
        0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
        0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,
        0xDB,0x30,0xA4,0xA6,0x8D,0x14,0xAF
    };
    uint8_t index;

    gOnline = true;
    oled_clear();

    for (index = 0U; index < (uint8_t)sizeof(commands); index++) {
        if (!write_command(commands[index])) {
            gOnline = false;
            gErrorCount++;
            break;
        }
    }
    gDirtyMask = 0xFFU;
    return gOnline;
#else
    return false;
#endif
}

void oled_clear(void)
{
    (void)memset(gBuffer, 0, sizeof(gBuffer));
    gDirtyMask = 0xFFU;
}

void oled_clear_page(uint8_t page)
{
    if (page < OLED_PAGE_COUNT) {
        (void)memset(gBuffer[page], 0, OLED_WIDTH);
        gDirtyMask |= (uint8_t)(1U << page);
    }
}

void oled_show_string(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL) {
        return;
    }
    while ((*text != '\0') && (x <= OLED_WIDTH - 6U)) {
        show_char(x, page, *text++);
        x = (uint8_t)(x + 6U);
    }
}

void oled_show_i32(uint8_t x, uint8_t page, int32_t value)
{
    char digits[10];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (value < 0) {
        show_char(x, page, '-');
        x = (uint8_t)(x + 6U);
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[count++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while ((magnitude != 0U) && (count < sizeof(digits)));

    while ((count != 0U) && (x <= OLED_WIDTH - 6U)) {
        show_char(x, page, digits[--count]);
        x = (uint8_t)(x + 6U);
    }
}

void oled_request_refresh(uint8_t first_page, uint8_t page_count)
{
    uint8_t page;
    uint8_t last = (uint8_t)(first_page + page_count);

    if (last > OLED_PAGE_COUNT) {
        last = OLED_PAGE_COUNT;
    }
    for (page = first_page; page < last; page++) {
        gDirtyMask |= (uint8_t)(1U << page);
    }
}

void oled_service(uint32_t now_ms)
{
#if APP_OLED_ENABLE
    uint8_t page;

    if (!gOnline) {
        if ((int32_t)(now_ms - gNextReconnectMs) >= 0) {
            gNextReconnectMs = now_ms + APP_OLED_RETRY_PERIOD_MS;
            gReconnectCount++;
            (void)oled_init();
        }
        return;
    }

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        uint8_t mask = (uint8_t)(1U << page);
        if ((gDirtyMask & mask) != 0U) {
            if (write_page(page)) {
                gDirtyMask &= (uint8_t)~mask;
            } else {
                gErrorCount++;
                gOnline = false;
                gNextReconnectMs = now_ms + APP_OLED_RETRY_PERIOD_MS;
            }
            break;
        }
    }
#else
    (void)now_ms;
#endif
}

bool oled_is_online(void)
{
    return gOnline;
}

uint32_t oled_get_error_count(void)
{
    return gErrorCount;
}

uint32_t oled_get_reconnect_count(void)
{
    return gReconnectCount;
}
