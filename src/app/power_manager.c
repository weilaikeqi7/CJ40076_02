#include "power_manager.h"

#include "app_state.h"
#include "board.h"

static void modules_off(void)
{
    Board_SetRangePower(false);
    Board_SetGnssPower(false);
    Board_SetImuPower(false);
}

void PowerManager_Init(void)
{
    modules_off();
    AppState_SetPowerMode(APP_POWER_RUN);
}

void PowerManager_EnterFault(void)
{
    modules_off();
    AppState_SetPowerMode(APP_POWER_FAULT);
}

void PowerManager_IdleHook(void)
{
}

void vApplicationIdleHook(void)
{
    PowerManager_IdleHook();
}
