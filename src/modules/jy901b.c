#include "jy901b.h"

#include "FreeRTOS.h"
#include "bsp_uart.h"
#include "task.h"

#include <stdbool.h>

#define JY901B_FRAME_LENGTH 11U
#define JY901B_HEADER       0x55U
#define JY901B_FRAME_ANGLE  0x53U

#define JY901B_REG_SAVE     0x00U
#define JY901B_REG_CALSW    0x01U
#define JY901B_REG_RSW      0x02U
#define JY901B_REG_RRATE    0x03U
#define JY901B_REG_ORIENT   0x23U
#define JY901B_REG_AXIS6    0x24U
#define JY901B_REG_KEY      0x69U

#define JY901B_KEY_UNLOCK        0xB588U
#define JY901B_SAVE_PARAM        0x0000U
#define JY901B_CAL_NORMAL        0x0000U
#define JY901B_CAL_ACCELEROMETER 0x0001U
#define JY901B_CAL_MAG_MM        0x0007U
#define JY901B_CAL_REF_ANGLE     0x0008U
#define JY901B_RSW_ANGLE         0x0008U
#define JY901B_RRATE_5HZ         0x0005U
#define JY901B_ORIENT_VERTICAL   0x0001U
#define JY901B_AXIS9_ALGORITHM   0x0000U
#define JY901B_WRITE_SETTLE_MS   200U

static uint8_t g_frame[JY901B_FRAME_LENGTH];
static uint8_t g_index;

static int16_t read_i16_le(const uint8_t* bytes)
{
    return (int16_t)(((uint16_t)bytes[1] << 8U) | bytes[0]);
}

static int16_t angle_to_centidegree(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 18000L) / 32768L);
}

static bool write_register(uint8_t reg, uint16_t value)
{
    const uint8_t packet[5] = {
        0xFFU,
        0xAAU,
        reg,
        (uint8_t)(value & 0xFFU),
        (uint8_t)(value >> 8U)
    };

    return BspUart_Write(BSP_UART_IMU, packet, sizeof(packet), 100U) == sizeof(packet);
}

static bool unlock_registers(void)
{
    if (!write_register(JY901B_REG_KEY, JY901B_KEY_UNLOCK))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(200U));
    return true;
}

static bool save_registers(void)
{
    if (!write_register(JY901B_REG_SAVE, JY901B_SAVE_PARAM))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(300U));
    return true;
}

void Jy901b_Reset(void)
{
    g_index = 0U;
}

bool Jy901b_Configure(void)
{
    if (!unlock_registers() ||
        !write_register(JY901B_REG_RRATE, JY901B_RRATE_5HZ) ||
        !write_register(JY901B_REG_RSW, JY901B_RSW_ANGLE) ||
        !write_register(JY901B_REG_ORIENT, JY901B_ORIENT_VERTICAL) ||
        !write_register(JY901B_REG_AXIS6, JY901B_AXIS9_ALGORITHM))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(JY901B_WRITE_SETTLE_MS));
    return save_registers();
}

bool Jy901b_CalibrateAccelerometer(void)
{
    if (!unlock_registers() ||
        !write_register(JY901B_REG_CALSW, JY901B_CAL_ACCELEROMETER))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(4000U));
    if (!write_register(JY901B_REG_CALSW, JY901B_CAL_NORMAL))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));
    return save_registers();
}

bool Jy901b_CalibrateRefAngle(void)
{
    if (!unlock_registers() ||
        !write_register(JY901B_REG_CALSW, JY901B_CAL_REF_ANGLE))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(3000U));
    return save_registers();
}

bool Jy901b_StartMagCalibration(void)
{
    return unlock_registers() &&
           write_register(JY901B_REG_CALSW, JY901B_CAL_MAG_MM);
}

bool Jy901b_StopMagCalibration(void)
{
    if (!unlock_registers() ||
        !write_register(JY901B_REG_CALSW, JY901B_CAL_NORMAL))
    {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100U));
    return save_registers();
}

bool Jy901b_ProcessByte(uint8_t byte, OrientationData* data)
{
    uint8_t checksum = 0U;

    if (g_index == 0U)
    {
        if (byte != JY901B_HEADER)
        {
            return false;
        }

        g_frame[g_index++] = byte;
        return false;
    }

    if ((g_index == 1U) && (byte != JY901B_FRAME_ANGLE))
    {
        g_index = (byte == JY901B_HEADER) ? 1U : 0U;
        if (g_index == 1U)
        {
            g_frame[0] = byte;
        }
        return false;
    }

    g_frame[g_index++] = byte;
    if (g_index < JY901B_FRAME_LENGTH)
    {
        return false;
    }

    g_index = 0U;
    for (uint8_t i = 0U; i < (JY901B_FRAME_LENGTH - 1U); ++i)
    {
        checksum = (uint8_t)(checksum + g_frame[i]);
    }

    if ((checksum != g_frame[JY901B_FRAME_LENGTH - 1U]) ||
        (g_frame[1] != JY901B_FRAME_ANGLE) ||
        (data == 0))
    {
        if (g_frame[JY901B_FRAME_LENGTH - 1U] == JY901B_HEADER)
        {
            g_frame[0] = JY901B_HEADER;
            g_index = 1U;
        }
        return false;
    }

    data->valid = true;
    data->roll_cd = angle_to_centidegree(read_i16_le(&g_frame[2]));
    data->pitch_cd = angle_to_centidegree(read_i16_le(&g_frame[4]));
    data->yaw_cd = angle_to_centidegree(read_i16_le(&g_frame[6]));
    return true;
}
