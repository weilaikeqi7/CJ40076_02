#include "serial_display.h"

#include "board_config.h"
#include "bsp_uart.h"

#include <stddef.h>

#define SERIAL_DISPLAY_HEAD1        0xA5U
#define SERIAL_DISPLAY_HEAD2        0x5AU
#define SERIAL_DISPLAY_TYPE_STATE   0x01U
#define SERIAL_DISPLAY_VERSION      0x01U
#define SERIAL_DISPLAY_PAYLOAD_LEN  (1U + 1U + SERIAL_DISPLAY_DIGIT_COUNT + 8U)
#define SERIAL_DISPLAY_FRAME_LEN    (2U + 1U + 1U + 1U + SERIAL_DISPLAY_PAYLOAD_LEN + 1U)
#define SERIAL_DISPLAY_TX_TIMEOUT_MS 20U

static uint8_t g_sequence;

static uint8_t encode_digit(int8_t digit)
{
    if ((digit >= 0) && (digit <= 9))
    {
        return (uint8_t)(digit + 1);
    }

    if (digit == -2)
    {
        return 11U;
    }

    switch ((char)digit)
    {
    case 'A': return 12U;
    case 'C': return 13U;
    case 'E': return 14U;
    case 'H': return 15U;
    case 'I': return 16U;
    case 'P': return 17U;
    case 'S': return 18U;
    case 'V': return 19U;
    case 'n': return 20U;
    case 'r': return 21U;
    case 't': return 22U;
    default: return 0U;
    }
}

void SerialDisplay_Init(void)
{
#if APP_SERIAL_DISPLAY_ENABLE
    BspUart_Reinit(BSP_UART_DISPLAY);
#endif
}

void SerialDisplay_Send(const SerialDisplayState* state)
{
#if APP_SERIAL_DISPLAY_ENABLE
    uint8_t frame[SERIAL_DISPLAY_FRAME_LEN];
    uint8_t index = 0U;
    uint8_t checksum = 0U;
    uint64_t symbols;

    if (state == 0)
    {
        return;
    }

    frame[index++] = SERIAL_DISPLAY_HEAD1;
    frame[index++] = SERIAL_DISPLAY_HEAD2;
    frame[index++] = SERIAL_DISPLAY_TYPE_STATE;
    frame[index++] = g_sequence++;
    frame[index++] = SERIAL_DISPLAY_PAYLOAD_LEN;
    frame[index++] = SERIAL_DISPLAY_VERSION;
    frame[index++] = (uint8_t)((state->all_on ? 0x01U : 0x00U) |
                               ((state->brightness & 0x0FU) << 4U));

    for (uint8_t i = 0U; i < SERIAL_DISPLAY_DIGIT_COUNT; ++i)
    {
        frame[index++] = encode_digit(state->digits[i]);
    }

    symbols = state->symbols;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        frame[index++] = (uint8_t)(symbols & 0xFFU);
        symbols >>= 8U;
    }

    for (uint8_t i = 2U; i < index; ++i)
    {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[index++] = checksum;

    (void)BspUart_Write(BSP_UART_DISPLAY, frame, index, SERIAL_DISPLAY_TX_TIMEOUT_MS);
#else
    (void)state;
#endif
}
