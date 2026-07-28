#include "app_time.h"

#include "app_config.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t gNowMs;
static volatile uint32_t gOverrunCount;

void app_time_init(void)
{
    gNowMs = 0U;
    gOverrunCount = 0U;

    SysTick->CTRL = 0U;
    SysTick->LOAD = APP_SYSTICK_RELOAD;
    SysTick->VAL = 0U;
    SysTick->CTRL =
        SysTick_CTRL_CLKSOURCE_Msk |
        SysTick_CTRL_TICKINT_Msk |
        SysTick_CTRL_ENABLE_Msk;
}

uint32_t app_time_now_ms(void)
{
    return gNowMs;
}

uint32_t app_time_get_overrun_count(void)
{
    return gOverrunCount;
}

void SysTick_Handler(void)
{
    gNowMs++;
}
