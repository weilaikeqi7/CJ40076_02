#ifndef MEASURE_COUNTER_H
#define MEASURE_COUNTER_H

#include <stdint.h>

void MeasureCounter_Init(void);
uint32_t MeasureCounter_Get(void);
uint32_t MeasureCounter_Increment(void);
void MeasureCounter_Reset(void);

#endif /* MEASURE_COUNTER_H */
