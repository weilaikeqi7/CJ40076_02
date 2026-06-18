#include "app_tasks.h"

#include "app_log.h"
#include "app_runtime_stats.h"
#include "app_state.h"
#include "battery.h"
#include "board.h"
#include "board_config.h"
#include "bsp_uart.h"
#include "bv220.h"
#include "jy901b.h"
#include "keys.h"
#include "display_output.h"
#include "measure_counter.h"
#include "module_selftest.h"
#include "power_manager.h"
#include "rangefinder.h"
#include "portable.h"
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define APP_STARTUP_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 3U)
#define APP_IO_TASK_STACK_WORDS       (configMINIMAL_STACK_SIZE * 2U)
#define APP_CONTROL_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 3U)
#define APP_DISPLAY_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 2U)
#define APP_EARTH_RADIUS_M           6371000.0

static bool g_range_powered;
static bool g_imu_powered;
static bool g_gnss_powered;
static bool g_continuous_started;
static bool g_continuous_test_active;
static bool g_imu_calibration_powered;
static volatile bool g_display_calibration_prompt;
static bool g_measure_pending;
static AppWorkMode g_measure_mode = APP_MODE_SINGLE;
static uint32_t g_measure_start_ms;
static uint32_t g_continuous_test_next_ms;
static uint32_t g_mode_click_last_ms;
static uint8_t g_continuous_sample_count;
static uint8_t g_mode_click_count;
static bool g_range_cycle_active;
static bool g_range_cycle_got_any;
static bool g_range_cycle_has_single;
static bool g_range_cycle_has_first;
static bool g_range_cycle_has_last;
static uint8_t g_range_cycle_last_index;
static uint8_t g_range_cycle_max_index;
static uint32_t g_range_cycle_last_rx_ms;
static uint32_t g_range_cycle_single_mm;
static uint32_t g_range_cycle_first_mm;
static uint32_t g_range_cycle_last_mm;
static uint8_t g_range_cycle_last_status;
static uint32_t g_range_rx_bytes_since_start;
static uint32_t g_range_frames_since_start;
static bool g_range_command_ack_received;
static volatile uint8_t g_range_last_ack_command;
static uint8_t g_last_logged_target_coord = 0xFFU;

static uint32_t tick_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void log_sram2_heap_once(void)
{
    const size_t free_now = xPortGetFreeHeapSize();
    const size_t min_free = xPortGetMinimumEverFreeHeapSize();
    const size_t used_now = configTOTAL_HEAP_SIZE - free_now;
    const size_t peak_used = configTOTAL_HEAP_SIZE - min_free;

    APP_LOGI("heap", "sram2 total=%u used=%u free=%u peak=%u min=%u",
             (unsigned int)configTOTAL_HEAP_SIZE,
             (unsigned int)used_now,
             (unsigned int)free_now,
             (unsigned int)peak_used,
             (unsigned int)min_free);
}

static void range_cycle_reset(void);

static void range_power_set(bool enabled)
{
    if (enabled == g_range_powered)
    {
        return;
    }

    if (enabled)
    {
        BspUart_Reinit(BSP_UART_RANGE);
        Board_SetRangePower(true);
        vTaskDelay(pdMS_TO_TICKS(APP_RANGE_POWER_ON_MS));
        BspUart_FlushRx(BSP_UART_RANGE);
        Rangefinder_Reset();
    }
    else
    {
        if (g_continuous_started)
        {
            Rangefinder_Stop();
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        Board_SetRangePower(false);
        g_continuous_started = false;
        g_measure_pending = false;
        g_measure_mode = APP_MODE_SINGLE;
        g_continuous_sample_count = 0U;
        range_cycle_reset();
    }

    g_range_powered = enabled;
}

static void imu_power_set(bool enabled)
{
    if (enabled == g_imu_powered)
    {
        return;
    }

    if (enabled)
    {
        BspUart_Reinit(BSP_UART_IMU);
        Board_SetImuPower(true);
        vTaskDelay(pdMS_TO_TICKS(APP_IMU_POWER_ON_MS));
        BspUart_FlushRx(BSP_UART_IMU);
        Jy901b_Reset();
    }
    else
    {
        Board_SetImuPower(false);
    }

    g_imu_powered = enabled;
}

static void gnss_power_set(bool enabled)
{
    if (enabled == g_gnss_powered)
    {
        return;
    }

    if (enabled)
    {
        BspUart_Reinit(BSP_UART_GNSS);
        Board_SetGnssPower(true);
        vTaskDelay(pdMS_TO_TICKS(APP_GNSS_POWER_ON_MS));
        BspUart_FlushRx(BSP_UART_GNSS);
        Bv220_Reset();
    }
    else
    {
        Board_SetGnssPower(false);
    }

    g_gnss_powered = enabled;
}

static void modules_off_for_sleep(void)
{
    range_power_set(false);
    imu_power_set(false);
    gnss_power_set(false);
}

static void apply_mode_power(AppWorkMode mode)
{
    range_power_set(false);
    imu_power_set(mode == APP_MODE_MULTI);
    gnss_power_set(mode == APP_MODE_MULTI);
}

static void finish_measurement_power(void)
{
    apply_mode_power(AppState_GetMode());
}

static void range_cycle_reset(void)
{
    g_range_cycle_active = false;
    g_range_cycle_got_any = false;
    g_range_cycle_has_single = false;
    g_range_cycle_has_first = false;
    g_range_cycle_has_last = false;
    g_range_cycle_last_index = 0U;
    g_range_cycle_max_index = 0U;
    g_range_cycle_last_rx_ms = 0U;
    g_range_cycle_single_mm = 0U;
    g_range_cycle_first_mm = 0U;
    g_range_cycle_last_mm = 0U;
    g_range_cycle_last_status = 0U;
    g_range_rx_bytes_since_start = 0U;
    g_range_frames_since_start = 0U;
    g_range_command_ack_received = false;
    g_range_last_ack_command = 0U;
}

static void stop_continuous_test(void)
{
    g_continuous_test_active = false;
    finish_measurement_power();
}

static bool wait_range_ack(uint8_t command, uint32_t timeout_ms)
{
    const uint32_t start_ms = tick_ms();

    while ((tick_ms() - start_ms) < timeout_ms)
    {
        if (g_range_last_ack_command == command)
        {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(5U));
    }

    return false;
}

static void publish_range_result(uint32_t now_ms)
{
    RangefinderData result;

    result.valid = true;
    result.command = g_continuous_started ? 0x04U : 0x02U;
    result.distance_mm = g_range_cycle_has_first ? g_range_cycle_first_mm :
        (g_range_cycle_has_single ? g_range_cycle_single_mm :
         (g_range_cycle_has_last ? g_range_cycle_last_mm : 0U));
    result.first_distance_mm = g_range_cycle_first_mm;
    result.last_distance_mm = g_range_cycle_last_mm;
    result.status = g_range_cycle_last_status;
    result.target_index = 0U;
    result.target_valid = g_range_cycle_got_any;
    result.first_valid = g_range_cycle_has_first;
    result.last_valid = g_range_cycle_has_last;
    result.self_status[0] = 0U;
    result.self_status[1] = 0U;
    result.self_status[2] = 0U;
    result.self_status[3] = 0U;
    result.self_test_ok = false;
    result.continuous = g_continuous_started;
    result.update_ms = now_ms;
    result.app_mode = (uint8_t)g_measure_mode;

    AppState_UpdateRange(&result);
    AppState_SetMeasureCount(MeasureCounter_Increment());
    ++g_continuous_sample_count;

    if (result.first_valid && result.last_valid)
    {
        APP_LOGI("range", "mode=%u count=%u first=%u last=%u",
                 (unsigned int)g_measure_mode,
                 (unsigned int)MeasureCounter_Get(),
                 (unsigned int)result.first_distance_mm,
                 (unsigned int)result.last_distance_mm);
    }
    else if (result.first_valid)
    {
        APP_LOGI("range", "mode=%u count=%u first=%u",
                 (unsigned int)g_measure_mode,
                 (unsigned int)MeasureCounter_Get(),
                 (unsigned int)result.first_distance_mm);
    }
    else if (result.last_valid)
    {
        APP_LOGI("range", "mode=%u count=%u last=%u",
                 (unsigned int)g_measure_mode,
                 (unsigned int)MeasureCounter_Get(),
                 (unsigned int)result.last_distance_mm);
    }
    else if (result.target_valid)
    {
        APP_LOGI("range", "mode=%u count=%u dist=%u",
                 (unsigned int)g_measure_mode,
                 (unsigned int)MeasureCounter_Get(),
                 (unsigned int)result.distance_mm);
    }
    else
    {
        APP_LOGI("range", "mode=%u count=%u no target status=0x%02X",
                 (unsigned int)g_measure_mode,
                 (unsigned int)MeasureCounter_Get(),
                 (unsigned int)result.status);
    }
    range_cycle_reset();
}

static void update_range_cycle(const RangefinderData* data, uint32_t now_ms)
{
    if ((data == 0) || ((data->command != 0x02U) && (data->command != 0x04U)))
    {
        return;
    }

    if (g_continuous_started && g_range_cycle_active &&
        (data->target_index == 0U) && (g_range_cycle_last_index != 0U))
    {
        publish_range_result(now_ms);
    }

    if (!g_range_cycle_active)
    {
        g_range_cycle_active = true;
        g_range_cycle_max_index = 0U;
    }

    g_range_cycle_last_rx_ms = now_ms;
    g_range_cycle_last_index = data->target_index;
    g_range_cycle_last_status = data->status;

    if (!data->target_valid)
    {
        return;
    }

    g_range_cycle_got_any = true;
    if (data->first_valid && (!g_range_cycle_has_first))
    {
        g_range_cycle_has_first = true;
        g_range_cycle_first_mm = data->distance_mm;
    }

    if (data->last_valid &&
        ((!g_range_cycle_has_last) || (data->target_index >= g_range_cycle_max_index)))
    {
        g_range_cycle_has_last = true;
        g_range_cycle_max_index = data->target_index;
        g_range_cycle_last_mm = data->distance_mm;
    }

    if ((!data->first_valid) && (!data->last_valid))
    {
        g_range_cycle_has_single = true;
        g_range_cycle_single_mm = data->distance_mm;
    }
}

static void update_range_from_uart(uint32_t now_ms)
{
    uint8_t byte;
    RangefinderData data;

    while (BspUart_ReadByte(BSP_UART_RANGE, &byte))
    {
        if (g_measure_pending || g_continuous_started)
        {
            ++g_range_rx_bytes_since_start;
        }

        if (Rangefinder_ProcessByte(byte, &data))
        {
            if (g_measure_pending || g_continuous_started)
            {
                ++g_range_frames_since_start;
            }
            data.update_ms = now_ms;
            if (data.valid && ((data.command == 0x02U) || (data.command == 0x04U)))
            {
                update_range_cycle(&data, now_ms);
            }
            else if ((!data.valid) && ((data.command == 0x02U) || (data.command == 0x04U)))
            {
                g_range_command_ack_received = true;
                g_range_last_ack_command = data.command;
            }
            else if (!data.valid)
            {
                g_range_last_ack_command = data.command;
            }
            else if (data.valid)
            {
                AppState_UpdateRange(&data);
            }
        }
    }
}

static void update_imu_from_uart(uint32_t now_ms)
{
    uint8_t byte;
    OrientationData data;
    static uint32_t last_log_ms;

    while (BspUart_ReadByte(BSP_UART_IMU, &byte))
    {
        if (Jy901b_ProcessByte(byte, &data))
        {
            data.update_ms = now_ms;
            AppState_UpdateOrientation(&data);
            if ((now_ms - last_log_ms) >= 1000U)
            {
                last_log_ms = now_ms;
                APP_LOGI("att", "yaw=%d.%02d pitch=%d.%02d roll=%d.%02d",
                         data.yaw_cd / 100,
                         (data.yaw_cd < 0) ? (-(data.yaw_cd % 100)) : (data.yaw_cd % 100),
                         data.pitch_cd / 100,
                         (data.pitch_cd < 0) ? (-(data.pitch_cd % 100)) : (data.pitch_cd % 100),
                         data.roll_cd / 100,
                         (data.roll_cd < 0) ? (-(data.roll_cd % 100)) : (data.roll_cd % 100));
            }
        }
    }
}

static void update_gnss_from_uart(uint32_t now_ms)
{
    uint8_t byte;
    GnssData data;
    static uint32_t last_log_ms;

    while (BspUart_ReadByte(BSP_UART_GNSS, &byte))
    {
        if (Bv220_ProcessByte(byte, &data))
        {
            data.update_ms = now_ms;
            AppState_UpdateGnss(&data);
            if ((now_ms - last_log_ms) >= 2000U)
            {
                last_log_ms = now_ms;
                if (data.fix)
                {
                    APP_LOGI("coord", "local lat=%d lon=%d alt=%dcm sats=%u",
                             (int)data.latitude_e7,
                             (int)data.longitude_e7,
                             (int)data.altitude_cm,
                             data.satellites);
                }
            }
        }
    }
}

static void io_task(void* argument)
{
    (void)argument;

    Bv220_Reset();
    Jy901b_Reset();
    Rangefinder_Reset();

    while (1)
    {
        const uint32_t now_ms = tick_ms();
        AppState_SetUptime(now_ms);

        update_range_from_uart(now_ms);
        update_imu_from_uart(now_ms);
        update_gnss_from_uart(now_ms);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

static void handle_mode_key(void)
{
    AppWorkMode mode;

    mode = AppState_GetMode();
    mode = (AppWorkMode)(((uint32_t)mode + 1U) % (uint32_t)APP_MODE_COUNT);
    g_continuous_test_active = false;
    AppState_SetMode(mode);
    AppState_ClearRange();
    apply_mode_power(mode);
}

static void clear_measure_count(void)
{
    MeasureCounter_Reset();
    AppState_SetMeasureCount(MeasureCounter_Get());
    APP_LOGI("control", "measure count cleared");
}

static void enter_imu_calibration_power(void)
{
    g_continuous_test_active = false;
    AppState_ClearRange();
    range_power_set(false);
    gnss_power_set(false);
    imu_power_set(true);
    g_imu_calibration_powered = true;
}

static void leave_imu_calibration_power(void)
{
    g_imu_calibration_powered = false;
    apply_mode_power(AppState_GetMode());
}

static void calibrate_imu_acc_gyro(void)
{
    APP_LOGI("cal", "acc gyro start");
    enter_imu_calibration_power();
    g_display_calibration_prompt = true;
    Jy901b_CalibrateAccGyro();
    g_display_calibration_prompt = false;
    leave_imu_calibration_power();
    APP_LOGI("cal", "acc gyro done");
}

static void calibrate_imu_ref_angle(void)
{
    APP_LOGI("cal", "ref angle start");
    enter_imu_calibration_power();
    g_display_calibration_prompt = true;
    Jy901b_CalibrateRefAngle();
    g_display_calibration_prompt = false;
    leave_imu_calibration_power();
    APP_LOGI("cal", "ref angle done");
}

static void start_imu_mag_calibration(void)
{
    APP_LOGI("cal", "mag start");
    enter_imu_calibration_power();
    g_display_calibration_prompt = true;
    Jy901b_StartMagCalibration();
}

static void stop_imu_mag_calibration(void)
{
    APP_LOGI("cal", "mag stop");
    if (!g_imu_calibration_powered)
    {
        enter_imu_calibration_power();
    }
    Jy901b_StopMagCalibration();
    g_display_calibration_prompt = false;
    leave_imu_calibration_power();
    APP_LOGI("cal", "mag done");
}

static void handle_mode_click_count(uint8_t count)
{
    switch (count)
    {
    case 1U:
        handle_mode_key();
        break;
    case 3U:
        clear_measure_count();
        break;
    case 5U:
        calibrate_imu_acc_gyro();
        break;
    case 6U:
        calibrate_imu_ref_angle();
        break;
    case 7U:
        start_imu_mag_calibration();
        break;
    case 8U:
        stop_imu_mag_calibration();
        break;
    default:
        break;
    }
}

static void note_mode_short_click(uint32_t now_ms)
{
    if ((g_mode_click_count == 0U) ||
        ((now_ms - g_mode_click_last_ms) > APP_MODE_MULTI_CLICK_MS))
    {
        g_mode_click_count = 0U;
    }

    if (g_mode_click_count < 8U)
    {
        ++g_mode_click_count;
    }
    g_mode_click_last_ms = now_ms;
}

static void close_mode_click_window(uint32_t now_ms)
{
    if ((g_mode_click_count == 0U) ||
        ((now_ms - g_mode_click_last_ms) < APP_MODE_MULTI_CLICK_MS))
    {
        return;
    }

    const uint8_t count = g_mode_click_count;
    g_mode_click_count = 0U;
    handle_mode_click_count(count);
}

static void start_measurement(AppWorkMode mode)
{
    g_measure_mode = mode;
    g_measure_pending = true;
    g_continuous_sample_count = 0U;
    range_cycle_reset();
    AppState_ClearRange();

    if (mode == APP_MODE_MULTI)
    {
        imu_power_set(true);
        gnss_power_set(true);
    }
    else
    {
        imu_power_set(false);
        gnss_power_set(false);
    }

    range_power_set(true);

    if (mode == APP_MODE_CONTINUOUS)
    {
#if APP_CONTINUOUS_RANGE_TEST_ENABLE
        g_range_last_ack_command = 0U;
        Rangefinder_SetTargetMode(RANGE_TARGET_MULTI);
        (void)wait_range_ack(0x03U, APP_RANGE_COMMAND_ACK_TIMEOUT_MS);
        g_measure_start_ms = tick_ms();
        g_range_last_ack_command = 0U;
        Rangefinder_StartSingle();
#else
        g_continuous_started = true;
        g_range_last_ack_command = 0U;
        Rangefinder_SetContinuousRate(RANGE_RATE_1HZ);
        (void)wait_range_ack(0xA1U, APP_RANGE_COMMAND_ACK_TIMEOUT_MS);
        g_range_last_ack_command = 0U;
        Rangefinder_SetTargetMode(RANGE_TARGET_MULTI);
        (void)wait_range_ack(0x03U, APP_RANGE_COMMAND_ACK_TIMEOUT_MS);
        g_measure_start_ms = tick_ms();
        g_range_last_ack_command = 0U;
        Rangefinder_StartContinuous();
#endif
    }
    else
    {
        g_range_last_ack_command = 0U;
        Rangefinder_SetTargetMode(RANGE_TARGET_MULTI);
        (void)wait_range_ack(0x03U, APP_RANGE_COMMAND_ACK_TIMEOUT_MS);
        g_measure_start_ms = tick_ms();
        g_range_last_ack_command = 0U;
        Rangefinder_StartSingle();
    }
}

static void handle_power_short(void)
{
    const AppWorkMode mode = AppState_GetMode();

    if ((mode == APP_MODE_CONTINUOUS) && g_continuous_started)
    {
        range_power_set(false);
        return;
    }

    if (mode == APP_MODE_CONTINUOUS)
    {
#if APP_CONTINUOUS_RANGE_TEST_ENABLE
        if (g_continuous_test_active)
        {
            stop_continuous_test();
        }
        else
        {
            g_continuous_test_active = true;
            g_continuous_test_next_ms = tick_ms();
        }
#else
        if (g_measure_pending || g_range_powered)
        {
            range_power_set(false);
        }
        else
        {
            start_measurement(APP_MODE_CONTINUOUS);
        }
#endif
        return;
    }

    start_measurement(mode);
}

static void power_off_sequence(void)
{
    g_continuous_test_active = false;
    modules_off_for_sleep();
    Board_SetDisplayPower(false);
    Board_PowerHold(false);
}

static bool battery_low_voltage(const BatteryData* battery)
{
    return (battery != 0) &&
           battery->valid &&
           (battery->voltage_mv < APP_BAT_EMPTY_MV);
}

static void power_off_wait_forever(void)
{
    power_off_sequence();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

static void power_off_if_battery_low(const char* module, const BatteryData* battery)
{
    if (!battery_low_voltage(battery))
    {
        return;
    }

    APP_LOGE(module,
             "battery low %u mV < %u mV, power off",
             (unsigned int)battery->voltage_mv,
             (unsigned int)APP_BAT_EMPTY_MV);
    power_off_wait_forever();
}

static void update_continuous_test(uint32_t now_ms)
{
    if (!APP_CONTINUOUS_RANGE_TEST_ENABLE)
    {
        (void)now_ms;
        return;
    }

    if ((!g_continuous_test_active) || (AppState_GetMode() != APP_MODE_CONTINUOUS))
    {
        return;
    }

    if (g_measure_pending || g_range_powered)
    {
        return;
    }

    if ((int32_t)(now_ms - g_continuous_test_next_ms) >= 0)
    {
        start_measurement(APP_MODE_CONTINUOUS);
        g_continuous_test_next_ms = now_ms + APP_CONTINUOUS_TEST_INTERVAL_MS;
    }
}

static void close_measurement_if_done(uint32_t now_ms)
{
    if (g_continuous_started)
    {
        if (g_range_cycle_active &&
            ((now_ms - g_range_cycle_last_rx_ms) >= APP_RANGE_CONTINUOUS_GAP_TIMEOUT_MS))
        {
            publish_range_result(now_ms);
        }

        if (g_continuous_sample_count >= APP_RANGE_CONTINUOUS_SAMPLES)
        {
            finish_measurement_power();
        }
        else if ((now_ms - g_measure_start_ms) >= APP_RANGE_CONTINUOUS_TIMEOUT_MS)
        {
            if (g_continuous_sample_count == 0U)
            {
                if (!g_range_command_ack_received)
                {
                    APP_LOGW("control", "continuous range timeout, rx_bytes=%u frames=%u",
                             (unsigned int)g_range_rx_bytes_since_start,
                             (unsigned int)g_range_frames_since_start);
                }
                else
                {
                    APP_LOGI("range", "continuous no target");
                }
                publish_range_result(now_ms);
            }
            finish_measurement_power();
        }
        return;
    }

    if (!g_measure_pending)
    {
        return;
    }

    if (g_range_cycle_active &&
        ((now_ms - g_range_cycle_last_rx_ms) >= APP_RANGE_SINGLE_GAP_TIMEOUT_MS))
    {
        publish_range_result(now_ms);
        finish_measurement_power();
    }
    else
    {
        const uint32_t timeout_ms = (g_measure_mode == APP_MODE_MULTI) ?
            APP_MULTI_MEASURE_TIMEOUT_MS : APP_RANGE_SINGLE_TIMEOUT_MS;

        if ((now_ms - g_measure_start_ms) >= timeout_ms)
        {
            const AppWorkMode finished_mode = g_measure_mode;

            if (!g_range_command_ack_received)
            {
                APP_LOGW("control", "%s range timeout, rx_bytes=%u frames=%u",
                         (finished_mode == APP_MODE_MULTI) ? "multi" : "single",
                         (unsigned int)g_range_rx_bytes_since_start,
                         (unsigned int)g_range_frames_since_start);
            }
            else
            {
                APP_LOGI("range", "%s no target",
                         (finished_mode == APP_MODE_MULTI) ? "multi" : "single");
            }
            publish_range_result(now_ms);
            finish_measurement_power();
        }
    }
}

static void control_task(void* argument)
{
    BatteryData battery;
    uint32_t last_battery_ms = 0U;
#if APP_AUTO_POWER_OFF_ENABLE
    uint32_t last_activity_ms = tick_ms();
#endif

    (void)argument;
    Keys_Init();
    apply_mode_power(AppState_GetMode());
    vTaskDelay(pdMS_TO_TICKS(1U));
    log_sram2_heap_once();

    while (1)
    {
        const uint32_t now_ms = tick_ms();
        const KeyEvent event = Keys_Poll(now_ms);

#if APP_AUTO_POWER_OFF_ENABLE
        if (event != KEY_EVENT_NONE)
        {
            last_activity_ms = now_ms;
        }
#endif

        if (event == KEY_EVENT_MODE_SHORT)
        {
            note_mode_short_click(now_ms);
        }
        else if (event == KEY_EVENT_POWER_SHORT)
        {
            handle_power_short();
        }
        else if (event == KEY_EVENT_POWER_LONG)
        {
            power_off_wait_forever();
        }

        update_continuous_test(tick_ms());
        close_measurement_if_done(tick_ms());
        close_mode_click_window(tick_ms());

        if ((now_ms - last_battery_ms) >= 1000U)
        {
            last_battery_ms = now_ms;
            Battery_Read(&battery);
            AppState_UpdateBattery(&battery);
            power_off_if_battery_low("battery", &battery);
        }

#if APP_AUTO_POWER_OFF_ENABLE
        if ((now_ms - last_activity_ms) >= APP_AUTO_POWER_OFF_MS)
        {
            power_off_wait_forever();
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(20U));
    }
}

static void display_dash_digits(const uint8_t* digit_ids, uint8_t digit_count)
{
    for (uint8_t i = 0U; i < digit_count; ++i)
    {
        DisplayOutput_SetDash(digit_ids[i], true);
    }
}

static void display_number_fixed(const uint8_t* digit_ids, uint8_t digit_count, uint32_t value)
{
    if ((digit_ids == 0) || (digit_count == 0U))
    {
        return;
    }

    for (int8_t i = (int8_t)(digit_count - 1U); i >= 0; --i)
    {
        DisplayOutput_SetDigit(digit_ids[i], (int8_t)(value % 10U));
        value /= 10U;
    }
}

static void display_battery_symbols(uint8_t level)
{
    static const DisplaySymbolId battery_symbols[] = {
        DISPLAY_SYMBOL_BATTERY_1,
        DISPLAY_SYMBOL_BATTERY_2,
        DISPLAY_SYMBOL_BATTERY_3,
        DISPLAY_SYMBOL_BATTERY_4,
    };

    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_BATTERY_FRAME, true);

    if (level > 4U)
    {
        level = 4U;
    }

    for (uint8_t i = 0U; i < (sizeof(battery_symbols) / sizeof(battery_symbols[0])); ++i)
    {
        DisplayOutput_SetSymbol((uint8_t)battery_symbols[i], i < level);
    }
}

static int32_t normalize_degrees(int32_t degrees)
{
    while (degrees < 0)
    {
        degrees += 360;
    }
    while (degrees >= 360)
    {
        degrees -= 360;
    }
    return degrees;
}

static void display_direction_symbols(int32_t yaw_deg)
{
    const int32_t degrees = normalize_degrees(yaw_deg);
    bool north = false;
    bool east = false;
    bool south = false;
    bool west = false;

    if ((degrees >= 337) || (degrees < 22))
    {
        north = true;
    }
    else if (degrees < 67)
    {
        north = true;
        east = true;
    }
    else if (degrees < 112)
    {
        east = true;
    }
    else if (degrees < 157)
    {
        east = true;
        south = true;
    }
    else if (degrees < 202)
    {
        south = true;
    }
    else if (degrees < 247)
    {
        south = true;
        west = true;
    }
    else if (degrees < 292)
    {
        west = true;
    }
    else
    {
        west = true;
        north = true;
    }

    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_DIR_N, north);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_DIR_E, east && !north);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_DIR_NE_E, north && east);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_DIR_S, south);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_DIR_W, west);
}

static void display_mode_symbols(AppWorkMode mode)
{
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RETICLE, true);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_UNIT_M, true);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_SINGLE,
                          (mode == APP_MODE_SINGLE) || (mode == APP_MODE_MULTI));
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_CONTINUOUS, mode == APP_MODE_CONTINUOUS);
}

static double degrees_to_radians(double degrees)
{
    return degrees * 3.14159265358979323846 / 180.0;
}

static double radians_to_degrees(double radians)
{
    return radians * 180.0 / 3.14159265358979323846;
}

static int32_t target_coordinate_e7(int32_t origin_e7,
                                    int32_t paired_origin_e7,
                                    uint32_t distance_mm,
                                    int32_t pitch_cd,
                                    int32_t yaw_cd,
                                    bool latitude)
{
    const double lat1 = degrees_to_radians((double)(latitude ? origin_e7 : paired_origin_e7) / 10000000.0);
    const double lon1 = degrees_to_radians((double)(latitude ? paired_origin_e7 : origin_e7) / 10000000.0);
    const double bearing = degrees_to_radians((double)normalize_degrees(yaw_cd / 100));
    const double pitch = degrees_to_radians((double)pitch_cd / 100.0);
    const double horizontal_m = ((double)distance_mm / 1000.0) * cos(pitch);
    const double angular_distance = horizontal_m / APP_EARTH_RADIUS_M;
    const double lat2 = asin((sin(lat1) * cos(angular_distance)) +
                             (cos(lat1) * sin(angular_distance) * cos(bearing)));
    double lon2 = lon1 + atan2(sin(bearing) * sin(angular_distance) * cos(lat1),
                               cos(angular_distance) - (sin(lat1) * sin(lat2)));

    lon2 = fmod(lon2 + (3.0 * 3.14159265358979323846), 2.0 * 3.14159265358979323846) -
           3.14159265358979323846;

    return (int32_t)((latitude ? radians_to_degrees(lat2) : radians_to_degrees(lon2)) * 10000000.0);
}

static int32_t target_altitude_cm(int32_t origin_altitude_cm, uint32_t distance_mm, int32_t pitch_cd)
{
    const double pitch = degrees_to_radians((double)pitch_cd / 100.0);
    const double height_cm = ((double)distance_mm / 10.0) * sin(pitch);

    return (int32_t)((double)origin_altitude_cm + height_cm);
}

typedef struct
{
    int32_t latitude_e7;
    int32_t longitude_e7;
    int32_t altitude_cm;
} DisplayTargetCoordinate;

typedef struct
{
    uint32_t update_ms;
    bool update_seen;
    bool has_result;
    bool valid;
    bool first_valid;
    bool last_valid;
    bool single_valid;
    DisplayTargetCoordinate first;
    DisplayTargetCoordinate last;
    DisplayTargetCoordinate single;
} DisplayTargetCache;

static DisplayTargetCoordinate calculate_target_coordinate(const GnssData* gnss,
                                                           const OrientationData* orientation,
                                                           uint32_t range_mm)
{
    DisplayTargetCoordinate target;

    target.latitude_e7 = target_coordinate_e7(gnss->latitude_e7,
                                              gnss->longitude_e7,
                                              range_mm,
                                              orientation->pitch_cd,
                                              orientation->yaw_cd,
                                              true);
    target.longitude_e7 = target_coordinate_e7(gnss->longitude_e7,
                                               gnss->latitude_e7,
                                               range_mm,
                                               orientation->pitch_cd,
                                               orientation->yaw_cd,
                                               false);
    target.altitude_cm = target_altitude_cm(gnss->altitude_cm,
                                            range_mm,
                                            orientation->pitch_cd);

    return target;
}

static void display_coordinate_value(int32_t coordinate_e7,
                                     const uint8_t* degree_digits,
                                     const uint8_t* minute_digits,
                                     const uint8_t* minute_fraction_digits)
{
    uint32_t absolute_e7;
    uint32_t degrees;
    uint32_t minutes;
    uint32_t minute_fraction;
    uint32_t remain_e7;

    if (coordinate_e7 < 0)
    {
        absolute_e7 = (uint32_t)(-coordinate_e7);
    }
    else
    {
        absolute_e7 = (uint32_t)coordinate_e7;
    }

    degrees = absolute_e7 / 10000000U;
    remain_e7 = absolute_e7 - (degrees * 10000000U);
    minutes = (remain_e7 * 60U) / 10000000U;
    remain_e7 = (remain_e7 * 60U) - (minutes * 10000000U);
    minute_fraction = (uint32_t)(((uint64_t)remain_e7 * 10000ULL) / 10000000ULL);

    display_number_fixed(degree_digits, 3U, degrees);
    display_number_fixed(minute_digits, 2U, minutes);
    display_number_fixed(minute_fraction_digits, 4U, minute_fraction);
}

static void display_task(void* argument)
{
    AppStateSnapshot snapshot;
    AppPowerMode last_power_mode = APP_POWER_FAULT;
    DisplayTargetCache multi_target_cache = { 0 };
    static const uint8_t azimuth_digits[] = { 1U, 2U, 3U };
    static const uint8_t range_digits[] = { 4U, 5U, 6U, 7U };
    static const uint8_t coord_digits[] = { 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U };
    static const uint8_t coord_degree_digits[] = { 8U, 9U, 10U };
    static const uint8_t coord_minute_digits[] = { 11U, 12U };
    static const uint8_t coord_minute_fraction_digits[] = { 13U, 14U, 15U, 16U };
    static const uint8_t altitude_digits[] = { 17U, 18U, 19U, 20U };
    static const uint8_t count_digits[] = { 21U, 22U, 23U, 24U, 25U };
    static const uint8_t pitch_digits[] = { 26U, 27U };

    (void)argument;
    DisplayOutput_Init();

    while (1)
    {
        AppState_Get(&snapshot);
        if (snapshot.power_mode != APP_POWER_RUN)
        {
            last_power_mode = snapshot.power_mode;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (last_power_mode != APP_POWER_RUN)
        {
            DisplayOutput_Init();
        }
        last_power_mode = APP_POWER_RUN;

        if (g_display_calibration_prompt)
        {
            DisplayOutput_SetAll(true);
            DisplayOutput_Flush();
            vTaskDelay(pdMS_TO_TICKS(200U));
            continue;
        }

        DisplayOutput_ClearBuffer();
        display_mode_symbols(snapshot.mode);
        display_battery_symbols(snapshot.battery.level);
        if (snapshot.measure_count_valid)
        {
            DisplayOutput_SetNumberRightAligned(count_digits, sizeof(count_digits), snapshot.measure_count);
        }
        else
        {
            display_dash_digits(count_digits, sizeof(count_digits));
        }

        const bool range_result_current = snapshot.range.valid &&
            (snapshot.range.app_mode == (uint8_t)snapshot.mode) &&
            ((!g_measure_pending) || (snapshot.range.update_ms >= g_measure_start_ms));
        bool display_range_valid = false;
        bool display_range_is_last = false;
        uint32_t display_range_mm = 0U;

        if (range_result_current)
        {
            if (snapshot.range.first_valid && snapshot.range.last_valid)
            {
                const uint32_t target_toggle_ms =
                    (snapshot.mode == APP_MODE_MULTI) ? 2000U : 1000U;

                display_range_is_last =
                    ((snapshot.uptime_ms / target_toggle_ms) & 1U) != 0U;
                display_range_valid = true;
                display_range_mm = display_range_is_last ?
                    snapshot.range.last_distance_mm : snapshot.range.first_distance_mm;
            }
            else if (snapshot.range.first_valid)
            {
                display_range_valid = true;
                display_range_mm = snapshot.range.first_distance_mm;
            }
            else if (snapshot.range.last_valid)
            {
                display_range_valid = true;
                display_range_is_last = true;
                display_range_mm = snapshot.range.last_distance_mm;
            }
            else if (snapshot.range.target_valid)
            {
                display_range_valid = true;
                display_range_mm = snapshot.range.distance_mm;
            }

            if (display_range_valid)
            {
                DisplayOutput_SetNumberRightAligned(range_digits, sizeof(range_digits), display_range_mm / 1000U);
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_FIRST_F, !display_range_is_last);
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_LAST_E, display_range_is_last);
            }
            else
            {
                display_dash_digits(range_digits, sizeof(range_digits));
            }
        }
        else
        {
            display_dash_digits(range_digits, sizeof(range_digits));
        }

        if (snapshot.mode == APP_MODE_MULTI)
        {
            if (range_result_current &&
                ((!multi_target_cache.update_seen) ||
                 (snapshot.range.update_ms != multi_target_cache.update_ms)))
            {
                const bool range_has_target =
                    snapshot.range.target_valid ||
                    snapshot.range.first_valid ||
                    snapshot.range.last_valid;

                multi_target_cache.update_seen = true;
                multi_target_cache.update_ms = snapshot.range.update_ms;
                multi_target_cache.has_result = range_has_target;
                multi_target_cache.valid = false;
                multi_target_cache.first_valid = false;
                multi_target_cache.last_valid = false;
                multi_target_cache.single_valid = false;

                if (range_has_target && snapshot.orientation.valid && snapshot.gnss.fix)
                {
                    multi_target_cache.first_valid = snapshot.range.first_valid;
                    multi_target_cache.last_valid = snapshot.range.last_valid;
                    multi_target_cache.single_valid =
                        snapshot.range.target_valid &&
                        (!snapshot.range.first_valid) &&
                        (!snapshot.range.last_valid);

                    if (multi_target_cache.first_valid)
                    {
                        multi_target_cache.first =
                            calculate_target_coordinate(&snapshot.gnss,
                                                        &snapshot.orientation,
                                                        snapshot.range.first_distance_mm);
                    }

                    if (multi_target_cache.last_valid)
                    {
                        multi_target_cache.last =
                            calculate_target_coordinate(&snapshot.gnss,
                                                        &snapshot.orientation,
                                                        snapshot.range.last_distance_mm);
                    }

                    if (multi_target_cache.single_valid)
                    {
                        multi_target_cache.single =
                            calculate_target_coordinate(&snapshot.gnss,
                                                        &snapshot.orientation,
                                                        snapshot.range.distance_mm);
                    }

                    multi_target_cache.valid =
                        multi_target_cache.first_valid ||
                        multi_target_cache.last_valid ||
                        multi_target_cache.single_valid;
                }
            }
        }
        else
        {
            multi_target_cache.update_seen = false;
            multi_target_cache.has_result = false;
            multi_target_cache.valid = false;
            multi_target_cache.update_ms = 0U;
        }

        if (snapshot.mode == APP_MODE_MULTI)
        {
            const int32_t yaw_deg = snapshot.orientation.valid ?
                normalize_degrees(snapshot.orientation.yaw_cd / 100) : 0;
            int32_t pitch_deg = snapshot.orientation.pitch_cd / 100;

            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_AZIMUTH_DEG, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_LABEL_P, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_DEG, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_MINUS,
                                  snapshot.orientation.valid && (pitch_deg < 0));
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_PLUS,
                                  snapshot.orientation.valid && (pitch_deg >= 0));

            if (pitch_deg < 0)
            {
                pitch_deg = -pitch_deg;
            }

            if (snapshot.orientation.valid)
            {
                DisplayOutput_SetNumberRightAligned(azimuth_digits, sizeof(azimuth_digits), (uint32_t)yaw_deg);
                DisplayOutput_SetNumberRightAligned(pitch_digits, sizeof(pitch_digits), (uint32_t)pitch_deg);
                display_direction_symbols(yaw_deg);
            }
        }

        if (snapshot.mode == APP_MODE_MULTI)
        {
            int32_t display_latitude = snapshot.gnss.latitude_e7;
            int32_t display_longitude = snapshot.gnss.longitude_e7;
            int32_t display_altitude_cm = snapshot.gnss.altitude_cm;
            const DisplayTargetCoordinate* display_target = 0;
            bool coord_range_is_last = false;
            const bool target_mode_display = multi_target_cache.has_result;
            const bool local_available = snapshot.gnss.fix;
            const bool show_latitude = ((snapshot.uptime_ms / 1000U) & 1U) != 0U;

            if (multi_target_cache.valid)
            {
                if (multi_target_cache.first_valid && multi_target_cache.last_valid)
                {
                    coord_range_is_last =
                        ((snapshot.uptime_ms / 2000U) & 1U) != 0U;
                    display_target = coord_range_is_last ?
                        &multi_target_cache.last : &multi_target_cache.first;
                }
                else if (multi_target_cache.first_valid)
                {
                    display_target = &multi_target_cache.first;
                }
                else if (multi_target_cache.last_valid)
                {
                    coord_range_is_last = true;
                    display_target = &multi_target_cache.last;
                }
                else if (multi_target_cache.single_valid)
                {
                    display_target = &multi_target_cache.single;
                }
            }

            const bool target_available = display_target != 0;

            if (target_available)
            {
                display_latitude = display_target->latitude_e7;
                display_longitude = display_target->longitude_e7;
                display_altitude_cm = display_target->altitude_cm;
            }

            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_LOCAL, !target_mode_display);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_TARGET, target_mode_display);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_FIRST_F, target_available && !coord_range_is_last);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_LAST_E, target_available && coord_range_is_last);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_DEG, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_MIN, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_SEC, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_COORD_DOT, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_ELEVATION_LABEL_H, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_ELEVATION_UNIT_M, true);

            if ((target_mode_display && !target_available) ||
                ((!target_mode_display) && !local_available))
            {
                display_dash_digits(coord_digits, sizeof(coord_digits));
                display_dash_digits(altitude_digits, sizeof(altitude_digits));
            }
            else
            {
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_LAT_N, show_latitude && (display_latitude >= 0));
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_LAT_S, show_latitude && (display_latitude < 0));
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_LON_E, (!show_latitude) && (display_longitude >= 0));
                DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_LON_W, (!show_latitude) && (display_longitude < 0));

                display_coordinate_value(show_latitude ? display_latitude : display_longitude,
                                         coord_degree_digits,
                                         coord_minute_digits,
                                         coord_minute_fraction_digits);

                if (display_altitude_cm < 0)
                {
                    display_altitude_cm = -display_altitude_cm;
                }
                DisplayOutput_SetNumberRightAligned(altitude_digits,
                                                  sizeof(altitude_digits),
                                                  (uint32_t)display_altitude_cm / 100U);

                if (target_available)
                {
                    const uint8_t coord_log_state =
                        (uint8_t)((coord_range_is_last ? 0x02U : 0x00U) |
                                  (show_latitude ? 0x01U : 0x00U));

                    if (coord_log_state != g_last_logged_target_coord)
                    {
                        g_last_logged_target_coord = coord_log_state;
                        APP_LOGI("coord", "target %s %s=%d alt=%dcm",
                                 coord_range_is_last ? "last" : "first",
                                 show_latitude ? "lat" : "lon",
                                 (int)(show_latitude ? display_latitude : display_longitude),
                                 (int)display_altitude_cm);
                    }
                }
            }
        }

        DisplayOutput_Flush();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static BaseType_t create_runtime_tasks(void)
{
    if (xTaskCreate(io_task, "io", APP_IO_TASK_STACK_WORDS, 0, tskIDLE_PRIORITY + 2U, 0) != pdPASS)
    {
        return pdFAIL;
    }

    if (xTaskCreate(control_task, "control", APP_CONTROL_TASK_STACK_WORDS, 0, tskIDLE_PRIORITY + 2U, 0) != pdPASS)
    {
        return pdFAIL;
    }

    if (xTaskCreate(display_task, "display", APP_DISPLAY_TASK_STACK_WORDS, 0, tskIDLE_PRIORITY + 1U, 0) != pdPASS)
    {
        return pdFAIL;
    }

    if (AppRunTimeStats_Start() != pdPASS)
    {
        return pdFAIL;
    }

    return pdPASS;
}

static void startup_task(void* argument)
{
    BatteryData battery;

    (void)argument;

    PowerManager_Init();
    MeasureCounter_Init();
    AppState_SetMeasureCount(MeasureCounter_Get());
    vTaskDelay(pdMS_TO_TICKS(APP_STARTUP_SETTLE_MS));

    Battery_Read(&battery);
    AppState_UpdateBattery(&battery);
    power_off_if_battery_low("startup", &battery);

    APP_LOGI("startup", "module self-test start");

    if (!ModuleSelfTest_RunAll())
    {
        APP_LOGE("startup", "module self-test failed, power off");
        power_off_wait_forever();
    }

    APP_LOGI("startup", "module self-test passed");
    if (create_runtime_tasks() != pdPASS)
    {
        APP_LOGE("startup", "failed to create runtime tasks");
        PowerManager_EnterFault();
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    vTaskDelete(0);
}

BaseType_t AppTasks_Start(void)
{
    AppState_Init();

    return xTaskCreate(startup_task,
                       "startup",
                       APP_STARTUP_TASK_STACK_WORDS,
                       0,
                       tskIDLE_PRIORITY + 3U,
                       0);
}
