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
#define JY901B_REG_KEY      0x69U

#define JY901B_KEY_UNLOCK   0xB588U
#define JY901B_SAVE_PARAM   0x0000U
#define JY901B_CAL_NORMAL   0x0000U
#define JY901B_CAL_GYRO_ACC 0x0001U
#define JY901B_CAL_MAG_MM   0x0007U
#define JY901B_CAL_REF_ANGLE 0x0008U
#define JY901B_RSW_ANGLE    0x0008U
#define JY901B_RRATE_1HZ    0x0003U
#define JY901B_ORIENT_VERTICAL 0x0001U

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

static int16_t compensate_yaw_centidegree(int16_t yaw_cd)
{
    int32_t compensated = 27000L - (int32_t)yaw_cd;

    while (compensated < 0L)
    {
        compensated += 36000L;
    }
    while (compensated >= 36000L)
    {
        compensated -= 36000L;
    }

    return (int16_t)compensated;
}

static void write_register(uint8_t reg, uint16_t value)
{
    const uint8_t packet[5] = {
        0xFFU,
        0xAAU,
        reg,
        (uint8_t)(value & 0xFFU),
        (uint8_t)(value >> 8U)
    };

    (void)BspUart_Write(BSP_UART_IMU, packet, sizeof(packet), 100U);
}

static void write_unlocked_register(uint8_t reg, uint16_t value)
{
    write_register(JY901B_REG_KEY, JY901B_KEY_UNLOCK);
    vTaskDelay(pdMS_TO_TICKS(200U));
    write_register(reg, value);
}

void Jy901b_Reset(void)
{
    g_index = 0U;
}

void Jy901b_Configure(void)
{
    write_unlocked_register(JY901B_REG_RRATE, JY901B_RRATE_1HZ);
    write_unlocked_register(JY901B_REG_RSW, JY901B_RSW_ANGLE);
    write_unlocked_register(JY901B_REG_ORIENT, JY901B_ORIENT_VERTICAL);
    write_unlocked_register(JY901B_REG_SAVE, JY901B_SAVE_PARAM);
    vTaskDelay(pdMS_TO_TICKS(300U));
}

void Jy901b_CalibrateAccGyro(void)
{
    write_unlocked_register(JY901B_REG_CALSW, JY901B_CAL_GYRO_ACC);
    vTaskDelay(pdMS_TO_TICKS(10000U));
    write_unlocked_register(JY901B_REG_CALSW, JY901B_CAL_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(100U));
    write_unlocked_register(JY901B_REG_SAVE, JY901B_SAVE_PARAM);
    vTaskDelay(pdMS_TO_TICKS(300U));
}

void Jy901b_CalibrateRefAngle(void)
{
    write_unlocked_register(JY901B_REG_CALSW, JY901B_CAL_REF_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(3000U));
    write_unlocked_register(JY901B_REG_SAVE, JY901B_SAVE_PARAM);
    vTaskDelay(pdMS_TO_TICKS(300U));
}

void Jy901b_StartMagCalibration(void)
{
    write_unlocked_register(JY901B_REG_CALSW, JY901B_CAL_MAG_MM);
}

void Jy901b_StopMagCalibration(void)
{
    write_unlocked_register(JY901B_REG_CALSW, JY901B_CAL_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(100U));
    write_unlocked_register(JY901B_REG_SAVE, JY901B_SAVE_PARAM);
    vTaskDelay(pdMS_TO_TICKS(300U));
}

bool Jy901b_ProcessByte(uint8_t byte, OrientationData* data)
{
    uint8_t checksum = 0U;

    if ((g_index == 0U) && (byte != JY901B_HEADER))
    {
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

    if ((checksum != g_frame[JY901B_FRAME_LENGTH - 1U]) || (g_frame[1] != JY901B_FRAME_ANGLE) || (data == 0))
    {
        return false;
    }

    data->valid = true;
    data->roll_cd = angle_to_centidegree(read_i16_le(&g_frame[2]));
    data->pitch_cd = angle_to_centidegree(read_i16_le(&g_frame[4]));
    data->yaw_cd = compensate_yaw_centidegree(angle_to_centidegree(read_i16_le(&g_frame[6])));
    return true;
}
