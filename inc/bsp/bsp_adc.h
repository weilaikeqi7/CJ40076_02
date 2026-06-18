#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdbool.h>
#include <stdint.h>

void BspAdc_Init(void);
bool BspAdc_ReadRaw(uint8_t channel, uint16_t* raw);

#endif /* BSP_ADC_H */
