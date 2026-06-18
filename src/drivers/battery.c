#include "battery.h"

#include "app_log.h"
#include "board_config.h"
#include "bsp_adc.h"

static uint8_t battery_level_from_mv(uint32_t mv)
{
    if (mv > 3900U)
    {
        return 5U;
    }
    if (mv >= 3800U)
    {
        return 4U;
    }
    if (mv >= 3700U)
    {
        return 3U;
    }
    if (mv >= 3600U)
    {
        return 2U;
    }

    return 1U;
}

void Battery_Read(BatteryData* data)
{
    uint16_t raw = 0U;
    uint32_t mv;

    if (data == 0)
    {
        return;
    }

    data->valid = BspAdc_ReadRaw(BOARD_BAT_ADC_CHANNEL, &raw);
    data->raw = raw;
    mv = ((uint32_t)raw * APP_BAT_ADC_REF_MV * APP_BAT_DIVIDER_NUM) /
         (4095U * APP_BAT_DIVIDER_DEN);
    //APP_LOGI("battery", "raw = %d, mv = %d", raw, mv);
    data->voltage_mv = (uint16_t)mv;
    data->level = battery_level_from_mv(mv);
    data->percent = (uint8_t)(data->level * 20U);
}
