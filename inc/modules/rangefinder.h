#ifndef RANGEFINDER_H
#define RANGEFINDER_H

#include "measure_types.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    RANGE_TARGET_FIRST = 1,
    RANGE_TARGET_LAST = 2,
    RANGE_TARGET_MULTI = 3
} RangeTargetMode;

typedef enum
{
    RANGE_RATE_1HZ = 1,
    RANGE_RATE_2HZ = 2,
    RANGE_RATE_3HZ = 3,
    RANGE_RATE_4HZ = 4,
    RANGE_RATE_5HZ = 5,
    RANGE_RATE_10HZ = 10
} RangeContinuousRate;

void Rangefinder_Reset(void);
void Rangefinder_SelfTest(void);
void Rangefinder_StartSingle(void);
void Rangefinder_StartContinuous(void);
void Rangefinder_Stop(void);
void Rangefinder_SetTargetMode(RangeTargetMode mode);
void Rangefinder_SetContinuousRate(RangeContinuousRate rate);
void Rangefinder_QueryMinGate(void);
void Rangefinder_QueryMaxGate(void);
bool Rangefinder_ProcessByte(uint8_t byte, RangefinderData* data);

#endif /* RANGEFINDER_H */
