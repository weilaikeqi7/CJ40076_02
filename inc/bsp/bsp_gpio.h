#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "n32g4fr.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    BSP_GPIO_PULL_NONE = 0,
    BSP_GPIO_PULL_UP,
    BSP_GPIO_PULL_DOWN
} BspGpioPull;

void BspGpio_EnableClock(GPIO_Module* port);
void BspGpio_InitOutput(GPIO_Module* port, uint16_t pin, bool high);
void BspGpio_InitInput(GPIO_Module* port, uint16_t pin, BspGpioPull pull);
void BspGpio_InitAnalog(GPIO_Module* port, uint16_t pin);
void BspGpio_InitAlternate(GPIO_Module* port, uint16_t pin, uint32_t alternate);
void BspGpio_InitAlternateInput(GPIO_Module* port, uint16_t pin, uint32_t alternate);
void BspGpio_Write(GPIO_Module* port, uint16_t pin, bool high);
bool BspGpio_Read(GPIO_Module* port, uint16_t pin);

#endif /* BSP_GPIO_H */
