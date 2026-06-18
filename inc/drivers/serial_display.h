#ifndef SERIAL_DISPLAY_H
#define SERIAL_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>

#define SERIAL_DISPLAY_DIGIT_COUNT  27U
#define SERIAL_DISPLAY_SYMBOL_COUNT 35U

typedef struct
{
    int8_t digits[SERIAL_DISPLAY_DIGIT_COUNT];
    uint64_t symbols;
    bool all_on;
    uint8_t brightness;
} SerialDisplayState;

void SerialDisplay_Init(void);
void SerialDisplay_Send(const SerialDisplayState* state);

#endif /* SERIAL_DISPLAY_H */
