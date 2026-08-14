#include "app_runtime_stats.h"

#include "n32g4fr.h"

#include <stdint.h>

static uint32_t s_cycles_per_us = 1U;

void AppRunTimeStats_TimerInit(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    if (cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }

    s_cycles_per_us = cycles_per_us;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t AppRunTimeStats_GetCounter(void)
{
    return DWT->CYCCNT / s_cycles_per_us;
}
