#include "app_state.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static AppStateSnapshot g_state;

void AppState_Init(void)
{
    taskENTER_CRITICAL();
    (void)memset(&g_state, 0, sizeof(g_state));
    g_state.mode = APP_MODE_SINGLE;
    g_state.power_mode = APP_POWER_RUN;
    taskEXIT_CRITICAL();
}

void AppState_SetMode(AppWorkMode mode)
{
    if (mode >= APP_MODE_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.mode = mode;
    taskEXIT_CRITICAL();
}

AppWorkMode AppState_GetMode(void)
{
    AppWorkMode mode;

    taskENTER_CRITICAL();
    mode = g_state.mode;
    taskEXIT_CRITICAL();

    return mode;
}

void AppState_SetPowerMode(AppPowerMode mode)
{
    if (mode > APP_POWER_FAULT)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.power_mode = mode;
    taskEXIT_CRITICAL();
}

AppPowerMode AppState_GetPowerMode(void)
{
    AppPowerMode mode;

    taskENTER_CRITICAL();
    mode = g_state.power_mode;
    taskEXIT_CRITICAL();

    return mode;
}

void AppState_UpdateBattery(const BatteryData* data)
{
    if (data == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.battery = *data;
    taskEXIT_CRITICAL();
}

void AppState_UpdateRange(const RangefinderData* data)
{
    if (data == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.range = *data;
    taskEXIT_CRITICAL();
}

void AppState_ClearRange(void)
{
    taskENTER_CRITICAL();
    (void)memset(&g_state.range, 0, sizeof(g_state.range));
    taskEXIT_CRITICAL();
}

void AppState_UpdateOrientation(const OrientationData* data)
{
    if (data == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.orientation = *data;
    taskEXIT_CRITICAL();
}

void AppState_UpdateGnss(const GnssData* data)
{
    if (data == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    g_state.gnss = *data;
    taskEXIT_CRITICAL();
}

void AppState_SetUptime(uint32_t uptime_ms)
{
    taskENTER_CRITICAL();
    g_state.uptime_ms = uptime_ms;
    taskEXIT_CRITICAL();
}

void AppState_SetMeasureCount(uint32_t count)
{
    taskENTER_CRITICAL();
    g_state.measure_count = count;
    g_state.measure_count_valid = true;
    taskEXIT_CRITICAL();
}

void AppState_Get(AppStateSnapshot* snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = g_state;
    taskEXIT_CRITICAL();
}
