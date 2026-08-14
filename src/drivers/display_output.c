#include "display_output.h"

#include "serial_display.h"

#include <stddef.h>
#include <string.h>

#define DISPLAY_FIRST_DIGIT_ID 1U
#define DISPLAY_LAST_DIGIT_ID  27U

static SerialDisplayState g_serial_state;

void DisplayOutput_Init(void)
{
    SerialDisplay_Init();
    DisplayOutput_SetAll(false);
    DisplayOutput_Flush();
}

void DisplayOutput_ClearBuffer(void)
{
    (void)memset(&g_serial_state, 0, sizeof(g_serial_state));
    for (uint8_t i = 0U; i < SERIAL_DISPLAY_DIGIT_COUNT; ++i)
    {
        g_serial_state.digits[i] = -1;
    }
}

void DisplayOutput_SetAll(bool on)
{
    DisplayOutput_ClearBuffer();

    if (!on)
    {
        return;
    }

    g_serial_state.all_on = true;
    for (uint8_t digit_id = DISPLAY_FIRST_DIGIT_ID; digit_id <= DISPLAY_LAST_DIGIT_ID; ++digit_id)
    {
        DisplayOutput_SetDigit(digit_id, 8);
    }
    g_serial_state.symbols = (SERIAL_DISPLAY_SYMBOL_COUNT >= 64U) ?
        UINT64_MAX :
        ((1ULL << SERIAL_DISPLAY_SYMBOL_COUNT) - 1ULL);
}

void DisplayOutput_SetDigit(uint8_t digit_id, int8_t value)
{
    if ((digit_id < DISPLAY_FIRST_DIGIT_ID) || (digit_id > DISPLAY_LAST_DIGIT_ID))
    {
        return;
    }

    g_serial_state.digits[digit_id - DISPLAY_FIRST_DIGIT_ID] = ((value >= 0) && (value <= 9)) ? value : -1;
}

void DisplayOutput_SetChar(uint8_t digit_id, char value)
{
    if ((digit_id < DISPLAY_FIRST_DIGIT_ID) || (digit_id > DISPLAY_LAST_DIGIT_ID))
    {
        return;
    }

    if ((value >= '0') && (value <= '9'))
    {
        DisplayOutput_SetDigit(digit_id, (int8_t)(value - '0'));
        return;
    }

    if (value == '-')
    {
        DisplayOutput_SetDash(digit_id, true);
        return;
    }

    switch (value)
    {
    case 'A':
    case 'C':
    case 'E':
    case 'H':
    case 'I':
    case 'P':
    case 'S':
    case 'V':
    case 'n':
    case 'r':
    case 't':
        g_serial_state.digits[digit_id - DISPLAY_FIRST_DIGIT_ID] = (int8_t)value;
        break;
    default:
        g_serial_state.digits[digit_id - DISPLAY_FIRST_DIGIT_ID] = -1;
        break;
    }
}

void DisplayOutput_SetDash(uint8_t digit_id, bool on)
{
    if ((digit_id < DISPLAY_FIRST_DIGIT_ID) || (digit_id > DISPLAY_LAST_DIGIT_ID))
    {
        return;
    }

    g_serial_state.digits[digit_id - DISPLAY_FIRST_DIGIT_ID] = on ? -2 : -1;
}

void DisplayOutput_SetNumberRightAligned(const uint8_t* digit_ids, uint8_t digit_count, uint32_t value)
{
    bool force_one_digit = true;

    if ((digit_ids == 0) || (digit_count == 0U))
    {
        return;
    }

    for (int8_t i = (int8_t)(digit_count - 1U); i >= 0; --i)
    {
        const uint8_t digit = (uint8_t)(value % 10U);
        bool visible;

        value /= 10U;
        visible = (value != 0U) || (digit != 0U) || force_one_digit;
        DisplayOutput_SetDigit(digit_ids[i], visible ? (int8_t)digit : -1);
        force_one_digit = false;
    }
}

void DisplayOutput_SetSignedNumberRightAligned(const uint8_t* digit_ids, uint8_t digit_count, int32_t value)
{
    uint32_t absolute;

    if (value < 0)
    {
        absolute = (uint32_t)(-value);
    }
    else
    {
        absolute = (uint32_t)value;
    }

    DisplayOutput_SetNumberRightAligned(digit_ids, digit_count, absolute);
}

void DisplayOutput_SetSymbol(uint8_t symbol_id, bool on)
{
    if ((symbol_id >= 1U) && (symbol_id <= SERIAL_DISPLAY_SYMBOL_COUNT))
    {
        const uint64_t mask = 1ULL << (symbol_id - 1U);
        if (on)
        {
            g_serial_state.symbols |= mask;
        }
        else
        {
            g_serial_state.symbols &= ~mask;
        }
    }
}

void DisplayOutput_Flush(void)
{
    SerialDisplay_Send(&g_serial_state);
}
