#include "app_tick.h"

#include <stddef.h>

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile bool gTickPending;
static volatile uint32_t gSystemTicks;
static volatile uint32_t gOverrunCount;

void app_tick_init(void)
{
    gTickPending = false;
    gSystemTicks = 0U;
    gOverrunCount = 0U;

    SysTick->CTRL = 0U;
    SysTick->LOAD = APP_CONTROL_TICK_RELOAD;
    SysTick->VAL = 0U;
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_TICKINT_Msk |
        SysTick_CTRL_ENABLE_Msk;
}

bool app_tick_take(uint32_t *now_tick)
{
    uint32_t primask;
    bool available;

    if (now_tick == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    available = gTickPending;
    if (available) {
        gTickPending = false;
        *now_tick = gSystemTicks;
    }

    __set_PRIMASK(primask);
    return available;
}

uint32_t app_tick_now(void)
{
    return gSystemTicks;
}

uint32_t app_tick_get_overrun_count(void)
{
    return gOverrunCount;
}

void SysTick_Handler(void)
{
    gSystemTicks++;

    if (gTickPending) {
        gOverrunCount++;
    }

    /* One request is enough. motion_update() receives the real current tick
     * and calculates the true elapsed interval itself. */
    gTickPending = true;
}
