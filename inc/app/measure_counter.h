#ifndef MEASURE_COUNTER_H
#define MEASURE_COUNTER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t pitch_install_tenth_deg;
    int16_t yaw_install_tenth_deg;
    int16_t yaw_error_tenth_deg;
} CalibrationOffsets;

void MeasureCounter_Init(void);
uint32_t MeasureCounter_Get(void);
uint32_t MeasureCounter_Increment(void);
void MeasureCounter_Reset(void);
bool MeasureCounter_Save(void);
void MeasureCounter_GetCalibration(CalibrationOffsets* offsets);
bool MeasureCounter_SaveCalibration(const CalibrationOffsets* offsets);

#endif /* MEASURE_COUNTER_H */
