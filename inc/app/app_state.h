#ifndef APP_STATE_H
#define APP_STATE_H

#include "measure_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_MODE_SINGLE = 0,
    APP_MODE_CONTINUOUS,
    APP_MODE_MULTI,
    APP_MODE_COUNT
} AppWorkMode;

typedef enum
{
    APP_POWER_RUN = 0,
    APP_POWER_FAULT
} AppPowerMode;

typedef struct
{
    BatteryData battery;
    RangefinderData range;
    OrientationData orientation;
    GnssData gnss;
    AppWorkMode mode;
    AppPowerMode power_mode;
    uint32_t uptime_ms;
    uint32_t measure_count;
    bool measure_count_valid;
} AppStateSnapshot;

void AppState_Init(void);
void AppState_SetMode(AppWorkMode mode);
AppWorkMode AppState_GetMode(void);
void AppState_SetPowerMode(AppPowerMode mode);
AppPowerMode AppState_GetPowerMode(void);
void AppState_UpdateBattery(const BatteryData* data);
void AppState_UpdateRange(const RangefinderData* data);
void AppState_ClearRange(void);
void AppState_UpdateOrientation(const OrientationData* data);
void AppState_UpdateGnss(const GnssData* data);
void AppState_SetUptime(uint32_t uptime_ms);
void AppState_SetMeasureCount(uint32_t count);
void AppState_Get(AppStateSnapshot* snapshot);

#endif /* APP_STATE_H */
