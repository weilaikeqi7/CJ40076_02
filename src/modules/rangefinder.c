#include "rangefinder.h"

#include "bsp_uart.h"

#include <stddef.h>
#include <string.h>

#define RANGE_HEADER_0    0xEEU
#define RANGE_HEADER_1    0x16U
#define RANGE_DEVICE_CODE 0x03U
#define RANGE_MAX_PAYLOAD 16U

typedef enum
{
    RANGE_PARSE_HEADER0 = 0,
    RANGE_PARSE_HEADER1,
    RANGE_PARSE_LENGTH,
    RANGE_PARSE_PAYLOAD,
    RANGE_PARSE_CHECKSUM
} RangeParseState;

static RangeParseState g_state;
static uint8_t g_length;
static uint8_t g_index;
static uint8_t g_payload[RANGE_MAX_PAYLOAD];

static void send_command(uint8_t command, const uint8_t* params, uint8_t param_len)
{
    uint8_t packet[4U + RANGE_MAX_PAYLOAD];
    uint8_t length = (uint8_t)(2U + param_len);
    uint8_t checksum = 0U;
    uint8_t pos = 0U;

    if (length > RANGE_MAX_PAYLOAD)
    {
        return;
    }

    packet[pos++] = RANGE_HEADER_0;
    packet[pos++] = RANGE_HEADER_1;
    packet[pos++] = length;
    packet[pos++] = RANGE_DEVICE_CODE;
    packet[pos++] = command;
    checksum = (uint8_t)(RANGE_DEVICE_CODE + command);

    for (uint8_t i = 0U; i < param_len; ++i)
    {
        packet[pos++] = params[i];
        checksum = (uint8_t)(checksum + params[i]);
    }

    packet[pos++] = checksum;
    (void)BspUart_Write(BSP_UART_RANGE, packet, pos, 100U);
}

void Rangefinder_Reset(void)
{
    g_state = RANGE_PARSE_HEADER0;
    g_length = 0U;
    g_index = 0U;
}

void Rangefinder_SelfTest(void)
{
    send_command(0x01U, 0, 0U);
}

void Rangefinder_StartSingle(void)
{
    send_command(0x02U, 0, 0U);
}

void Rangefinder_SetTargetMode(RangeTargetMode mode)
{
    uint8_t param = (uint8_t)mode;
    send_command(0x03U, &param, 1U);
}

void Rangefinder_StartContinuous(void)
{
    send_command(0x04U, 0, 0U);
}

void Rangefinder_Stop(void)
{
    send_command(0x05U, 0, 0U);
}

void Rangefinder_SetContinuousRate(RangeContinuousRate rate)
{
    uint8_t params[2];

    params[0] = (uint8_t)rate;
    params[1] = 0x01U;
    send_command(0xA1U, params, sizeof(params));
}

void Rangefinder_QueryMinGate(void)
{
    send_command(0xA3U, 0, 0U);
}

void Rangefinder_QueryMaxGate(void)
{
    send_command(0xA5U, 0, 0U);
}

static bool checksum_ok(uint8_t checksum)
{
    uint8_t sum = 0U;

    for (uint8_t i = 0U; i < g_length; ++i)
    {
        sum = (uint8_t)(sum + g_payload[i]);
    }

    return sum == checksum;
}

static bool parse_payload(RangefinderData* data)
{
    uint8_t command;
    uint8_t range_status;
    uint16_t meters;

    if ((data == 0) || (g_length < 2U) || (g_payload[0] != RANGE_DEVICE_CODE))
    {
        return false;
    }

    (void)memset(data, 0, sizeof(*data));

    command = g_payload[1];
    data->command = command;

    if (command == 0x01U)
    {
        if (g_length < 6U)
        {
            return false;
        }
        data->valid = true;
        data->status = g_payload[5];
        data->self_status[0] = g_payload[2];
        data->self_status[1] = g_payload[3];
        data->self_status[2] = g_payload[4];
        data->self_status[3] = g_payload[5];
        data->distance_mm = 0U;
        data->continuous = false;
        data->self_test_ok = ((g_payload[4] & 0x01U) != 0U) &&
                             ((g_payload[5] & 0x01U) != 0U);
        return true;
    }

    if (g_length == 2U)
    {
        return true;
    }

    if (((command != 0x02U) && (command != 0x04U)) || (g_length < 6U))
    {
        return true;
    }

    data->valid = true;
    data->status = g_payload[2];
    range_status = (uint8_t)(data->status & 0x0FU);
    data->target_index = (uint8_t)(data->status >> 4U);
    meters = (uint16_t)(((uint16_t)g_payload[3] << 8U) | g_payload[4]);
    data->target_valid = ((range_status <= 0x02U) && (meters != 0xFFFFU));
    data->first_valid = data->target_valid && (range_status == 0x01U);
    data->last_valid = data->target_valid && (range_status == 0x02U);
    data->distance_mm = data->target_valid ?
        (((uint32_t)meters * 1000U) + ((uint32_t)g_payload[5] * 100U)) : 0U;
    data->continuous = (command == 0x04U);
    return true;
}

bool Rangefinder_ProcessByte(uint8_t byte, RangefinderData* data)
{
    bool parsed = false;

    switch (g_state)
    {
    case RANGE_PARSE_HEADER0:
        if (byte == RANGE_HEADER_0)
        {
            g_state = RANGE_PARSE_HEADER1;
        }
        break;

    case RANGE_PARSE_HEADER1:
        g_state = (byte == RANGE_HEADER_1) ? RANGE_PARSE_LENGTH : RANGE_PARSE_HEADER0;
        break;

    case RANGE_PARSE_LENGTH:
        g_length = byte;
        g_index = 0U;
        g_state = ((g_length > 0U) && (g_length <= RANGE_MAX_PAYLOAD)) ? RANGE_PARSE_PAYLOAD : RANGE_PARSE_HEADER0;
        break;

    case RANGE_PARSE_PAYLOAD:
        g_payload[g_index++] = byte;
        if (g_index >= g_length)
        {
            g_state = RANGE_PARSE_CHECKSUM;
        }
        break;

    case RANGE_PARSE_CHECKSUM:
        if (checksum_ok(byte))
        {
            parsed = parse_payload(data);
        }
        Rangefinder_Reset();
        break;

    default:
        Rangefinder_Reset();
        break;
    }

    return parsed;
}
