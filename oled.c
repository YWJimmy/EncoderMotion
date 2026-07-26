#include "oled.h"

#include <stddef.h>
#include <string.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

#define OLED_PORT          (GPIOB)
#define OLED_SCL_PIN       (DL_GPIO_PIN_8)
#define OLED_SDA_PIN       (DL_GPIO_PIN_9)
#define OLED_SCL_IOMUX     (IOMUX_PINCM25)
#define OLED_SDA_IOMUX     (IOMUX_PINCM26)
#define OLED_WIDTH         (128U)
#define OLED_PAGE_COUNT    (8U)
#define OLED_WRITE_ADDRESS (0x78U)
#define OLED_ALL_PAGES     (0xFFU)

static uint8_t gOledBuffer[OLED_PAGE_COUNT][OLED_WIDTH];
static uint8_t gDirtyPageMask;
static bool gOledOnline;
static uint32_t gLastRetryTick;
static uint32_t gLastPageServiceTick;
static uint32_t gErrorCount;
static uint32_t gReconnectCount;

static void oled_delay(void)
{
    DL_Common_delayCycles(APP_OLED_I2C_DELAY_CYCLES);
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

static uint8_t oled_read_sda(void)
{
    return
        (DL_GPIO_readPins(OLED_PORT, OLED_SDA_PIN) != 0U) ?
        1U : 0U;
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

static bool oled_send_byte(uint8_t value)
{
    uint8_t bit;
    bool acknowledged;

    for (bit = 0U; bit < 8U; bit++) {
        oled_set_sda((value & 0x80U) != 0U ? 1U : 0U);
        oled_set_scl(1U);
        oled_set_scl(0U);
        value <<= 1U;
    }

    oled_set_sda(1U);
    oled_set_scl(1U);
    acknowledged = oled_read_sda() == 0U;
    oled_set_scl(0U);
    return acknowledged;
}

static void oled_bus_recover(void)
{
    uint8_t pulse;

    oled_set_sda(1U);
    for (pulse = 0U; pulse < 9U; pulse++) {
        oled_set_scl(0U);
        oled_set_scl(1U);
    }
    oled_stop();
}

static bool oled_write_buffer_once(
    uint8_t control,
    const uint8_t *data,
    uint16_t length)
{
    uint16_t index;
    bool ok = true;

    oled_start();

    if (!oled_send_byte(OLED_WRITE_ADDRESS)) {
        ok = false;
    } else if (!oled_send_byte(control)) {
        ok = false;
    } else {
        for (index = 0U; index < length; index++) {
            if (!oled_send_byte(data[index])) {
                ok = false;
                break;
            }
        }
    }

    oled_stop();
    return ok;
}

static bool oled_write_buffer(
    uint8_t control,
    const uint8_t *data,
    uint16_t length)
{
    uint8_t attempt;

    for (attempt = 0U;
         attempt <= APP_OLED_TRANSACTION_RETRIES;
         attempt++) {
        if (oled_write_buffer_once(control, data, length)) {
            return true;
        }
        oled_bus_recover();
    }

    gErrorCount++;
    gOledOnline = false;
    return false;
}

static bool oled_write_commands(
    const uint8_t *commands,
    uint16_t command_count)
{
    return oled_write_buffer(0x00U, commands, command_count);
}

static bool oled_write_page(uint8_t page)
{
    uint8_t address_commands[3] = {
        (uint8_t)(0xB0U | page),
        0x10U,
        0x00U
    };
    uint8_t attempt;

    /* Retry the address and data phases together so a partial page write
     * always restarts at column zero. */
    for (attempt = 0U;
         attempt <= APP_OLED_TRANSACTION_RETRIES;
         attempt++) {
        if (oled_write_buffer_once(0x00U, address_commands, 3U) &&
            oled_write_buffer_once(
                0x40U,
                gOledBuffer[page],
                OLED_WIDTH)) {
            return true;
        }
        oled_bus_recover();
    }

    gErrorCount++;
    gOledOnline = false;
    return false;
}

static bool oled_try_initialize(void)
{
    static const uint8_t commands[] = {
        0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U,
        0x00U, 0x40U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
        0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x30U,
        0xA4U, 0xA6U, 0x8DU, 0x14U, 0xAFU
    };

    oled_bus_recover();
    gOledOnline = true;

    if (!oled_write_commands(commands, (uint16_t)sizeof(commands))) {
        return false;
    }

    gDirtyPageMask = OLED_ALL_PAGES;
    return true;
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

static void oled_show_char(uint8_t x, uint8_t page, char character)
{
    const uint8_t *glyph;
    uint8_t column;

    if ((page >= OLED_PAGE_COUNT) || (x > (OLED_WIDTH - 6U))) {
        return;
    }

    glyph = oled_glyph(character);
    for (column = 0U; column < 5U; column++) {
        gOledBuffer[page][x + column] = glyph[column];
    }
    gOledBuffer[page][x + 5U] = 0U;
}

bool oled_init(void)
{
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_setPins(OLED_PORT, OLED_SCL_PIN | OLED_SDA_PIN);
    DL_GPIO_enableOutput(OLED_PORT, OLED_SCL_PIN | OLED_SDA_PIN);

    gDirtyPageMask = 0U;
    gOledOnline = false;
    gLastRetryTick = 0U;
    gLastPageServiceTick = UINT32_MAX;
    gErrorCount = 0U;
    gReconnectCount = 0U;

    DL_Common_delayCycles(APP_OLED_POWER_UP_DELAY_CYCLES);
    oled_clear();
    return oled_try_initialize();
}

void oled_clear(void)
{
    (void)memset(gOledBuffer, 0, sizeof(gOledBuffer));
    gDirtyPageMask = OLED_ALL_PAGES;
}

void oled_clear_page(uint8_t page)
{
    if (page < OLED_PAGE_COUNT) {
        (void)memset(gOledBuffer[page], 0, OLED_WIDTH);
        gDirtyPageMask |= (uint8_t)(1U << page);
    }
}

void oled_show_string(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL) {
        return;
    }

    while ((*text != '\0') && (x <= (OLED_WIDTH - 6U))) {
        oled_show_char(x, page, *text++);
        x = (uint8_t)(x + 6U);
    }

    if (page < OLED_PAGE_COUNT) {
        gDirtyPageMask |= (uint8_t)(1U << page);
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
    } while ((magnitude != 0U) && (count < (uint8_t)sizeof(digits)));

    while ((count != 0U) && (x <= (OLED_WIDTH - 6U))) {
        oled_show_char(x, page, digits[--count]);
        x = (uint8_t)(x + 6U);
    }

    if (page < OLED_PAGE_COUNT) {
        gDirtyPageMask |= (uint8_t)(1U << page);
    }
}

bool oled_update_pages(uint8_t first_page, uint8_t page_count)
{
    uint8_t page;
    uint8_t last_page;

    if (first_page >= OLED_PAGE_COUNT) {
        return false;
    }

    last_page = (uint8_t)(first_page + page_count);
    if ((last_page > OLED_PAGE_COUNT) || (last_page < first_page)) {
        last_page = OLED_PAGE_COUNT;
    }

    for (page = first_page; page < last_page; page++) {
        gDirtyPageMask |= (uint8_t)(1U << page);
    }

    return gOledOnline;
}

void oled_service(uint32_t now_tick)
{
    uint8_t page;
    uint8_t pages_sent = 0U;

    if (!gOledOnline) {
        if ((uint32_t)(now_tick - gLastRetryTick) <
            APP_OLED_RETRY_PERIOD_TICKS) {
            return;
        }

        gLastRetryTick = now_tick;
        if (oled_try_initialize()) {
            gReconnectCount++;
        }
        return;
    }

    if ((gDirtyPageMask == 0U) ||
        (gLastPageServiceTick == now_tick)) {
        return;
    }

    gLastPageServiceTick = now_tick;

    for (page = 0U; page < OLED_PAGE_COUNT; page++) {
        uint8_t page_bit = (uint8_t)(1U << page);
        if ((gDirtyPageMask & page_bit) != 0U) {
            if (!oled_write_page(page)) {
                return;
            }

            gDirtyPageMask &= (uint8_t)~page_bit;
            pages_sent++;
            if (pages_sent >= APP_OLED_PAGES_PER_SERVICE) {
                return;
            }
        }
    }
}

bool oled_is_online(void)
{
    return gOledOnline;
}

uint32_t oled_get_error_count(void)
{
    return gErrorCount;
}

uint32_t oled_get_reconnect_count(void)
{
    return gReconnectCount;
}
