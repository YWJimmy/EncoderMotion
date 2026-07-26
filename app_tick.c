#include "app_tick.h"

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

bool app_tick_take(void)
{
    uint32_t primask;
    bool available;

    primask = __get_PRIMASK();
    __disable_irq();

    available = gTickPending;
    gTickPending = false;

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

    /* Keep only one pending control request. Real elapsed time is recovered
     * from app_tick_now() inside motion_update(). */
    gTickPending = true;
}
