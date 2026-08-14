#include "module_selftest.h"

#include "FreeRTOS.h"
#include "app_log.h"
#include "board.h"
#include "board_config.h"
#include "bsp_uart.h"
#include "bv220.h"
#include "jy901b.h"
#include "rangefinder.h"
#include "task.h"

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static bool wait_range_selftest(uint32_t timeout_ms)
{
    const uint32_t start_ms = now_ms();
    uint32_t last_cmd_ms = 0U;
    uint32_t attempt = 0U;
    uint8_t byte;
    RangefinderData data;

    BspUart_FlushRx(BSP_UART_RANGE);
    Rangefinder_Reset();

    while ((now_ms() - start_ms) < timeout_ms)
    {
        const uint32_t elapsed_ms = now_ms() - start_ms;
        if ((attempt == 0U) ||
            ((elapsed_ms - last_cmd_ms) >= APP_RANGE_SELFTEST_RETRY_MS))
        {
            ++attempt;
            last_cmd_ms = elapsed_ms;
            Rangefinder_SelfTest();
        }

        while (BspUart_ReadByte(BSP_UART_RANGE, &byte))
        {
            if (Rangefinder_ProcessByte(byte, &data) && (data.command == 0x01U))
            {
                return data.self_test_ok;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    APP_LOGW("selftest", "range timeout after %u attempts", (unsigned int)attempt);
    return false;
}

static bool wait_range_command(uint8_t command, uint32_t timeout_ms)
{
    const uint32_t start_ms = now_ms();
    uint8_t byte;
    RangefinderData data;

    while ((now_ms() - start_ms) < timeout_ms)
    {
        while (BspUart_ReadByte(BSP_UART_RANGE, &byte))
        {
            if (Rangefinder_ProcessByte(byte, &data) && (data.command == command))
            {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return false;
}

static void range_protocol2_warmup(void)
{
    Rangefinder_QueryMinGate();
    (void)wait_range_command(0xA3U, 300U);
    Rangefinder_QueryMaxGate();
    (void)wait_range_command(0xA5U, 300U);
}

static bool wait_imu_frame(uint32_t timeout_ms)
{
    const uint32_t start_ms = now_ms();
    uint8_t byte;
    OrientationData data;

    BspUart_FlushRx(BSP_UART_IMU);
    Jy901b_Reset();
    while ((now_ms() - start_ms) < timeout_ms)
    {
        while (BspUart_ReadByte(BSP_UART_IMU, &byte))
        {
            if (Jy901b_ProcessByte(byte, &data))
            {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return false;
}

static bool wait_gnss_any_data(uint32_t timeout_ms)
{
    const uint32_t start_ms = now_ms();
    uint8_t byte;

    BspUart_FlushRx(BSP_UART_GNSS);
    Bv220_Reset();
    while ((now_ms() - start_ms) < timeout_ms)
    {
        if (BspUart_ReadByte(BSP_UART_GNSS, &byte))
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    return false;
}

bool ModuleSelfTest_RunAll(void)
{
    bool ok = true;

    // BspUart_Reinit(BSP_UART_RANGE);
    // Board_SetRangePower(true);
    // ok = wait_range_selftest(APP_RANGE_SELFTEST_TIMEOUT_MS);
    // if (ok)
    // {
    //     range_protocol2_warmup();
    // }
    // Board_SetRangePower(false);
    // if (!ok)
    // {
    //     return false;
    // }

    BspUart_Reinit(BSP_UART_IMU);
    Board_SetImuPower(true);
    ok = Jy901b_Configure();
    if (ok)
    {
        BspUart_FlushRx(BSP_UART_IMU);
        Jy901b_Reset();
        ok = wait_imu_frame(APP_IMU_SELFTEST_TIMEOUT_MS);
    }
    Board_SetImuPower(false);
    if (!ok)
    {
        APP_LOGW("selftest", "imu failed");
        return false;
    }

    // BspUart_Reinit(BSP_UART_GNSS);
    // Board_SetGnssPower(true);
    // ok = wait_gnss_any_data(APP_GNSS_SELFTEST_TIMEOUT_MS);
    // Board_SetGnssPower(false);

    return ok;
}
