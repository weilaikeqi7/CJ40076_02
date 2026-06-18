#ifndef DISPLAY_OUTPUT_H
#define DISPLAY_OUTPUT_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    DISPLAY_SYMBOL_DIR_E = 1,
    DISPLAY_SYMBOL_DIR_S,
    DISPLAY_SYMBOL_DIR_W,
    DISPLAY_SYMBOL_DIR_N,
    DISPLAY_SYMBOL_DIR_NE_E,
    DISPLAY_SYMBOL_BATTERY_1,
    DISPLAY_SYMBOL_BATTERY_2,
    DISPLAY_SYMBOL_BATTERY_3,
    DISPLAY_SYMBOL_BATTERY_4,
    DISPLAY_SYMBOL_BATTERY_FRAME,
    DISPLAY_SYMBOL_UNIT_M,
    DISPLAY_SYMBOL_RANGE_SINGLE,
    DISPLAY_SYMBOL_RANGE_CONTINUOUS,
    DISPLAY_SYMBOL_RANGE_FIRST_F,
    DISPLAY_SYMBOL_RANGE_LAST_E,
    DISPLAY_SYMBOL_RETICLE,
    DISPLAY_SYMBOL_AZIMUTH_DEG,
    DISPLAY_SYMBOL_PITCH_LABEL_P,
    DISPLAY_SYMBOL_PITCH_DEG,
    DISPLAY_SYMBOL_PITCH_SIGN_MINUS,
    DISPLAY_SYMBOL_PITCH_SIGN_PLUS,
    DISPLAY_SYMBOL_LON_W,
    DISPLAY_SYMBOL_LON_E,
    DISPLAY_SYMBOL_LAT_S,
    DISPLAY_SYMBOL_LAT_N,
    DISPLAY_SYMBOL_COORD_LOCAL,
    DISPLAY_SYMBOL_COORD_TARGET,
    DISPLAY_SYMBOL_COORD_FIRST_F,
    DISPLAY_SYMBOL_COORD_LAST_E,
    DISPLAY_SYMBOL_COORD_DEG,
    DISPLAY_SYMBOL_COORD_MIN,
    DISPLAY_SYMBOL_COORD_SEC,
    DISPLAY_SYMBOL_COORD_DOT,
    DISPLAY_SYMBOL_ELEVATION_LABEL_H,
    DISPLAY_SYMBOL_ELEVATION_UNIT_M
} DisplaySymbolId;

void DisplayOutput_Init(void);
void DisplayOutput_ClearBuffer(void);
void DisplayOutput_SetAll(bool on);
void DisplayOutput_SetDigit(uint8_t digit_id, int8_t value);
void DisplayOutput_SetDash(uint8_t digit_id, bool on);
void DisplayOutput_SetNumberRightAligned(const uint8_t* digit_ids, uint8_t digit_count, uint32_t value);
void DisplayOutput_SetSignedNumberRightAligned(const uint8_t* digit_ids, uint8_t digit_count, int32_t value);
void DisplayOutput_SetSymbol(uint8_t symbol_id, bool on);
void DisplayOutput_Flush(void);

#endif /* DISPLAY_OUTPUT_H */
