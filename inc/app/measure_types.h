#ifndef MEASURE_TYPES_H
#define MEASURE_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool valid;
    uint16_t raw;
    uint16_t voltage_mv;
    uint8_t percent;
    uint8_t level;
} BatteryData;

typedef struct
{
    bool valid;
    uint8_t command;
    uint32_t distance_mm;
    uint32_t first_distance_mm;
    uint32_t last_distance_mm;
    uint8_t status;
    uint8_t target_index;
    bool target_valid;
    bool first_valid;
    bool last_valid;
    uint8_t self_status[4];
    bool self_test_ok;
    bool continuous;
    uint32_t update_ms;
    uint8_t app_mode;
} RangefinderData;

typedef struct
{
    bool valid;
    int16_t roll_cd;
    int16_t pitch_cd;
    int16_t yaw_cd;
    uint32_t update_ms;
} OrientationData;

typedef struct
{
    bool valid;
    bool fix;
    uint8_t satellites;
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
    uint32_t update_ms;
} GnssData;

#endif /* MEASURE_TYPES_H */
