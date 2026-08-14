#include "app_tasks.h"

#include "app_log.h"
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
#include "task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define APP_STARTUP_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 3U)
#define APP_IO_TASK_STACK_WORDS       (configMINIMAL_STACK_SIZE * 2U)
#define APP_CONTROL_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 3U)
#define APP_DISPLAY_TASK_STACK_WORDS  (configMINIMAL_STACK_SIZE * 2U)
#define APP_EARTH_RADIUS_M           6371000.0
#define APP_CALIBRATION_HOLD_START_MS 600U
#define APP_CALIBRATION_REPEAT_MS     100U
#define APP_CALIBRATION_NEXT_PAGE_MS 1000U

typedef enum
{
    CALIBRATION_PAGE_NONE = 0,
    CALIBRATION_PAGE_PITCH_INSTALL,
    CALIBRATION_PAGE_YAW_INSTALL,
    CALIBRATION_PAGE_YAW_ERROR,
    CALIBRATION_PAGE_SAVE
} CalibrationPage;

static bool g_range_powered;
static bool g_imu_powered;
static bool g_gnss_powered;
static bool g_continuous_started;
#if APP_WORK_TIME_TEST_MODE_ENABLE
static bool g_work_time_test_active;
static uint32_t g_work_time_test_next_ms;
#endif
static bool g_imu_calibration_powered;
static bool g_imu_mag_calibration_active;
static volatile bool g_lcd_calibration_prompt;
static volatile CalibrationPage g_calibration_page;
static CalibrationOffsets g_calibration_offsets;
static CalibrationOffsets g_calibration_edit_offsets;
static uint32_t g_calibration_power_hold_ms;
static uint32_t g_calibration_mode_hold_ms;
static uint32_t g_calibration_power_repeat_ms;
static uint32_t g_calibration_mode_repeat_ms;
static uint32_t g_calibration_both_hold_ms;
static bool g_calibration_power_repeated;
static bool g_calibration_mode_repeated;
static bool g_calibration_both_active;
static bool g_calibration_wait_release;
static bool g_measure_pending;
static AppWorkMode g_measure_mode = APP_MODE_SINGLE;
static uint32_t g_measure_start_ms;
static uint32_t g_mode_click_last_ms;
static uint8_t g_continuous_sample_count;
static uint8_t g_mode_click_count;
static bool g_range_cycle_active;
static bool g_range_cycle_has_first;
static bool g_range_cycle_has_last;
static uint8_t g_range_cycle_last_index;
static uint8_t g_range_cycle_max_index;
static uint32_t g_range_cycle_last_rx_ms;
static uint32_t g_range_cycle_first_mm;
static uint32_t g_range_cycle_last_mm;
static uint8_t g_range_cycle_last_status;
static uint32_t g_range_rx_bytes_since_start;
static uint32_t g_range_frames_since_start;
static bool g_range_command_ack_received;
static volatile uint8_t g_range_last_ack_command;

static void display_battery_symbols(uint8_t level);
static int32_t normalize_degrees(int32_t degrees);

static uint32_t tick_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static int32_t normalize_yaw_centidegree(int32_t yaw_cd)
{
    while (yaw_cd > 18000)
    {
        yaw_cd -= 36000;
    }
    while (yaw_cd <= -18000)
    {
        yaw_cd += 36000;
    }
    return yaw_cd;
}

static bool calibration_settings_active(void)
{
    return g_calibration_page != CALIBRATION_PAGE_NONE;
}

static void get_active_calibration_offsets(CalibrationOffsets* offsets)
{
    if (offsets == 0)
    {
        return;
    }

    taskENTER_CRITICAL();
    *offsets = calibration_settings_active() ? g_calibration_edit_offsets : g_calibration_offsets;
    taskEXIT_CRITICAL();
}

static void apply_orientation_offsets(OrientationData* data)
{
    CalibrationOffsets offsets;
    int32_t pitch_cd;
    int32_t yaw_cd;

    if (data == 0)
    {
        return;
    }

    get_active_calibration_offsets(&offsets);
    pitch_cd = -(int32_t)data->pitch_cd + ((int32_t)offsets.pitch_install_tenth_deg * 10);
    yaw_cd = -(int32_t)data->yaw_cd +
             ((int32_t)(offsets.yaw_install_tenth_deg + offsets.yaw_error_tenth_deg) * 10);

    if (pitch_cd > INT16_MAX)
    {
        pitch_cd = INT16_MAX;
    }
    else if (pitch_cd < INT16_MIN)
    {
        pitch_cd = INT16_MIN;
    }

    data->pitch_cd = (int16_t)pitch_cd;
    data->yaw_cd = (int16_t)normalize_yaw_centidegree(yaw_cd);
}

static bool mode_uses_multifunction(AppWorkMode mode)
{
#if APP_WORK_TIME_TEST_MODE_ENABLE
    return (mode == APP_MODE_MULTI) || (mode == APP_MODE_WORK_TIME_TEST);
#else
    return mode == APP_MODE_MULTI;
#endif
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
        AppState_ClearOrientation();
        BspUart_Reinit(BSP_UART_IMU);
        Board_SetImuPower(true);
        BspUart_FlushRx(BSP_UART_IMU);
        Jy901b_Reset();
        AppState_ClearOrientation();
        g_imu_powered = true;
    }
    else
    {
        g_imu_powered = false;
        Board_SetImuPower(false);
        BspUart_FlushRx(BSP_UART_IMU);
        Jy901b_Reset();
        AppState_ClearOrientation();
    }
}

static void gnss_power_set(bool enabled)
{
    if (enabled == g_gnss_powered)
    {
        return;
    }

    if (enabled)
    {
        AppState_ClearGnss();
        BspUart_Reinit(BSP_UART_GNSS);
        Board_SetGnssPower(true);
        BspUart_FlushRx(BSP_UART_GNSS);
        Bv220_Reset();
        AppState_ClearGnss();
        g_gnss_powered = true;
    }
    else
    {
        g_gnss_powered = false;
        Board_SetGnssPower(false);
        BspUart_FlushRx(BSP_UART_GNSS);
        Bv220_Reset();
        AppState_ClearGnss();
    }
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
    imu_power_set(mode_uses_multifunction(mode));
    gnss_power_set(mode_uses_multifunction(mode));
}

static void finish_measurement_power(void)
{
    apply_mode_power(AppState_GetMode());
}

static void range_cycle_reset(void)
{
    g_range_cycle_active = false;
    g_range_cycle_has_first = false;
    g_range_cycle_has_last = false;
    g_range_cycle_last_index = 0U;
    g_range_cycle_max_index = 0U;
    g_range_cycle_last_rx_ms = 0U;
    g_range_cycle_first_mm = 0U;
    g_range_cycle_last_mm = 0U;
    g_range_cycle_last_status = 0U;
    g_range_rx_bytes_since_start = 0U;
    g_range_frames_since_start = 0U;
    g_range_command_ack_received = false;
    g_range_last_ack_command = 0U;
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
        (g_range_cycle_has_last ? g_range_cycle_last_mm : 0U);
    result.first_distance_mm = g_range_cycle_first_mm;
    result.last_distance_mm = g_range_cycle_last_mm;
    result.status = g_range_cycle_last_status;
    result.target_index = 0U;
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

    if ((!data->first_valid) && (!data->last_valid))
    {
        return;
    }

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
    while (BspUart_ReadByte(BSP_UART_IMU, &byte))
    {
        if (Jy901b_ProcessByte(byte, &data))
        {
            if (!g_imu_powered)
            {
                continue;
            }
            apply_orientation_offsets(&data);
            data.update_ms = now_ms;
            AppState_UpdateOrientation(&data);
        }
    }
}

static void update_gnss_from_uart(uint32_t now_ms)
{
    uint8_t byte;
    GnssData data;
    while (BspUart_ReadByte(BSP_UART_GNSS, &byte))
    {
        if (Bv220_ProcessByte(byte, &data))
        {
            if (!g_gnss_powered)
            {
                continue;
            }
            data.update_ms = now_ms;
            AppState_UpdateGnss(&data);
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
#if APP_WORK_TIME_TEST_MODE_ENABLE
    mode = (AppWorkMode)(((uint32_t)mode + 1U) % (uint32_t)APP_MODE_COUNT);
#else
    mode = (mode >= APP_MODE_MULTI) ? APP_MODE_SINGLE : (AppWorkMode)((uint32_t)mode + 1U);
#endif
    AppState_SetMode(mode);
    AppState_ClearRange();
    apply_mode_power(mode);
}

static void clear_measure_count(void)
{
    MeasureCounter_Reset();
    AppState_SetMeasureCount(MeasureCounter_Get());
}

static void reset_calibration_key_state(void)
{
    g_calibration_power_hold_ms = 0U;
    g_calibration_mode_hold_ms = 0U;
    g_calibration_power_repeat_ms = 0U;
    g_calibration_mode_repeat_ms = 0U;
    g_calibration_both_hold_ms = 0U;
    g_calibration_power_repeated = false;
    g_calibration_mode_repeated = false;
    g_calibration_both_active = false;
    g_calibration_wait_release = false;
}

static void enter_calibration_settings(void)
{
#if APP_WORK_TIME_TEST_MODE_ENABLE
    g_work_time_test_active = false;
#endif
    AppState_ClearRange();
    range_power_set(false);
    gnss_power_set(false);
    imu_power_set(true);

    taskENTER_CRITICAL();
    g_calibration_edit_offsets = g_calibration_offsets;
    g_calibration_page = CALIBRATION_PAGE_PITCH_INSTALL;
    taskEXIT_CRITICAL();

    reset_calibration_key_state();
}

static void leave_calibration_settings(void)
{
    taskENTER_CRITICAL();
    g_calibration_page = CALIBRATION_PAGE_NONE;
    taskEXIT_CRITICAL();
    reset_calibration_key_state();
    AppState_ClearOrientation();
    apply_mode_power(AppState_GetMode());
}

static void save_calibration_settings(void)
{
    CalibrationOffsets offsets;

    taskENTER_CRITICAL();
    offsets = g_calibration_edit_offsets;
    taskEXIT_CRITICAL();

    if (!MeasureCounter_SaveCalibration(&offsets))
    {
        APP_LOGE("cal", "failed to save angle offsets");
        return;
    }

    taskENTER_CRITICAL();
    g_calibration_offsets = offsets;
    taskEXIT_CRITICAL();
    leave_calibration_settings();
}

static void next_calibration_page(void)
{
    taskENTER_CRITICAL();
    if ((g_calibration_page >= CALIBRATION_PAGE_PITCH_INSTALL) &&
        (g_calibration_page < CALIBRATION_PAGE_SAVE))
    {
        g_calibration_page = (CalibrationPage)((uint32_t)g_calibration_page + 1U);
    }
    else
    {
        g_calibration_page = CALIBRATION_PAGE_PITCH_INSTALL;
    }
    taskEXIT_CRITICAL();
}

static void adjust_calibration_value(int16_t delta)
{
    int16_t* value = 0;
    int16_t minimum = 0;
    int16_t maximum = 0;
    int32_t adjusted;

    taskENTER_CRITICAL();
    switch (g_calibration_page)
    {
    case CALIBRATION_PAGE_PITCH_INSTALL:
        value = &g_calibration_edit_offsets.pitch_install_tenth_deg;
        minimum = -900;
        maximum = 900;
        break;
    case CALIBRATION_PAGE_YAW_INSTALL:
        value = &g_calibration_edit_offsets.yaw_install_tenth_deg;
        minimum = -1800;
        maximum = 1800;
        break;
    case CALIBRATION_PAGE_YAW_ERROR:
        value = &g_calibration_edit_offsets.yaw_error_tenth_deg;
        minimum = -1800;
        maximum = 1800;
        break;
    default:
        break;
    }

    if (value != 0)
    {
        adjusted = (int32_t)*value + delta;
        if (adjusted < minimum)
        {
            adjusted = minimum;
        }
        else if (adjusted > maximum)
        {
            adjusted = maximum;
        }
        *value = (int16_t)adjusted;
    }
    taskEXIT_CRITICAL();
}

static void handle_calibration_settings_input(KeyEvent event, uint32_t now_ms)
{
    const bool power_pressed = Keys_IsPowerPressed();
    const bool mode_pressed = Keys_IsModePressed();
    const CalibrationPage page = g_calibration_page;

    if (g_calibration_wait_release)
    {
        if ((!power_pressed) && (!mode_pressed))
        {
            reset_calibration_key_state();
        }
        return;
    }

    if (g_calibration_both_active)
    {
        if (power_pressed && mode_pressed)
        {
            if ((now_ms - g_calibration_both_hold_ms) >= APP_CALIBRATION_NEXT_PAGE_MS)
            {
                next_calibration_page();
                g_calibration_both_active = false;
                g_calibration_wait_release = true;
            }
        }
        else
        {
            g_calibration_both_active = false;
            g_calibration_wait_release = true;
        }
        return;
    }

    if (power_pressed && mode_pressed)
    {
        g_calibration_both_active = true;
        g_calibration_both_hold_ms = now_ms;
        g_calibration_power_repeated = true;
        g_calibration_mode_repeated = true;
        return;
    }

    if (page == CALIBRATION_PAGE_SAVE)
    {
        if (event == KEY_EVENT_POWER_SHORT)
        {
            save_calibration_settings();
        }
        else if (event == KEY_EVENT_MODE_SHORT)
        {
            leave_calibration_settings();
        }
        return;
    }

    if ((event == KEY_EVENT_POWER_SHORT) && (!g_calibration_power_repeated))
    {
        adjust_calibration_value(1);
    }
    else if ((event == KEY_EVENT_MODE_SHORT) && (!g_calibration_mode_repeated))
    {
        adjust_calibration_value(-1);
    }

    if (power_pressed)
    {
        if (g_calibration_power_hold_ms == 0U)
        {
            g_calibration_power_hold_ms = now_ms;
            g_calibration_power_repeat_ms = now_ms;
        }
        else if (((now_ms - g_calibration_power_hold_ms) >= APP_CALIBRATION_HOLD_START_MS) &&
                 ((now_ms - g_calibration_power_repeat_ms) >= APP_CALIBRATION_REPEAT_MS))
        {
            adjust_calibration_value(1);
            g_calibration_power_repeat_ms = now_ms;
            g_calibration_power_repeated = true;
        }
    }
    else
    {
        g_calibration_power_hold_ms = 0U;
        g_calibration_power_repeat_ms = 0U;
        g_calibration_power_repeated = false;
    }

    if (mode_pressed)
    {
        if (g_calibration_mode_hold_ms == 0U)
        {
            g_calibration_mode_hold_ms = now_ms;
            g_calibration_mode_repeat_ms = now_ms;
        }
        else if (((now_ms - g_calibration_mode_hold_ms) >= APP_CALIBRATION_HOLD_START_MS) &&
                 ((now_ms - g_calibration_mode_repeat_ms) >= APP_CALIBRATION_REPEAT_MS))
        {
            adjust_calibration_value(-1);
            g_calibration_mode_repeat_ms = now_ms;
            g_calibration_mode_repeated = true;
        }
    }
    else
    {
        g_calibration_mode_hold_ms = 0U;
        g_calibration_mode_repeat_ms = 0U;
        g_calibration_mode_repeated = false;
    }
}

static void enter_imu_calibration_power(void)
{
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

static void calibrate_imu_accelerometer(void)
{
    bool ok;

    enter_imu_calibration_power();
    g_lcd_calibration_prompt = true;
    ok = Jy901b_CalibrateAccelerometer();
    g_lcd_calibration_prompt = false;
    leave_imu_calibration_power();
    if (!ok)
    {
        APP_LOGE("cal", "accelerometer command failed");
    }
}

static void calibrate_imu_ref_angle(void)
{
    bool ok;

    enter_imu_calibration_power();
    g_lcd_calibration_prompt = true;
    ok = Jy901b_CalibrateRefAngle();
    g_lcd_calibration_prompt = false;
    leave_imu_calibration_power();
    if (!ok)
    {
        APP_LOGE("cal", "ref angle command failed");
    }
}

static void start_imu_mag_calibration(void)
{
    enter_imu_calibration_power();
    g_lcd_calibration_prompt = true;
    if (Jy901b_StartMagCalibration())
    {
        g_imu_mag_calibration_active = true;
        APP_LOGI("cal", "mag calibration started");
    }
    else
    {
        g_lcd_calibration_prompt = false;
        leave_imu_calibration_power();
        APP_LOGE("cal", "mag start command failed");
    }
}

static void stop_imu_mag_calibration(void)
{
    bool ok;

    if (!g_imu_mag_calibration_active)
    {
        return;
    }

    ok = Jy901b_StopMagCalibration();
    g_imu_mag_calibration_active = false;
    g_lcd_calibration_prompt = false;
    leave_imu_calibration_power();
    if (!ok)
    {
        APP_LOGE("cal", "mag stop command failed");
    }
    else
    {
        APP_LOGI("cal", "mag calibration stopped, save command sent");
    }
}

static void handle_mode_click_count(uint8_t count)
{
    if (g_imu_mag_calibration_active)
    {
        if (count == 8U)
        {
            stop_imu_mag_calibration();
        }
        return;
    }

    switch (count)
    {
    case 1U:
        handle_mode_key();
        break;
    case 3U:
        clear_measure_count();
        break;
    case 5U:
        calibrate_imu_accelerometer();
        break;
    case 4U:
        enter_calibration_settings();
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
#if APP_WORK_TIME_TEST_MODE_ENABLE
    if (g_work_time_test_active)
    {
        return;
    }
#endif
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

    if (mode_uses_multifunction(mode))
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

    if (g_imu_mag_calibration_active)
    {
        return;
    }

#if APP_WORK_TIME_TEST_MODE_ENABLE
    if (mode == APP_MODE_WORK_TIME_TEST)
    {
        if (g_work_time_test_active)
        {
            g_work_time_test_active = false;
            range_power_set(false);
        }
        else
        {
            g_work_time_test_active = true;
            g_work_time_test_next_ms = tick_ms();
        }
        return;
    }
#endif

    if ((mode == APP_MODE_CONTINUOUS) && g_continuous_started)
    {
        range_power_set(false);
        return;
    }

    if (mode == APP_MODE_CONTINUOUS)
    {
        if (g_measure_pending || g_range_powered)
        {
            range_power_set(false);
        }
        else
        {
            start_measurement(APP_MODE_CONTINUOUS);
        }
        return;
    }

    start_measurement(mode);
}

static void power_off_sequence(bool save_measure_count)
{
#if APP_WORK_TIME_TEST_MODE_ENABLE
    g_work_time_test_active = false;
#endif
    g_imu_mag_calibration_active = false;
    g_lcd_calibration_prompt = false;
    modules_off_for_sleep();
    if (save_measure_count && !MeasureCounter_Save())
    {
        APP_LOGE("counter", "failed to save measure count");
    }
    Board_PowerHold(false);
}

static bool battery_low_voltage(const BatteryData* battery)
{
    return (battery != 0) &&
           battery->valid &&
           (battery->voltage_mv < APP_BAT_EMPTY_MV);
}

static void power_off_wait_forever(bool save_measure_count)
{
    power_off_sequence(save_measure_count);
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
    power_off_wait_forever(false);
}

#if APP_WORK_TIME_TEST_MODE_ENABLE
static void update_work_time_test(uint32_t now_ms)
{
    if ((!g_work_time_test_active) ||
        (AppState_GetMode() != APP_MODE_WORK_TIME_TEST) ||
        g_measure_pending || g_range_powered)
    {
        return;
    }

    if ((int32_t)(now_ms - g_work_time_test_next_ms) >= 0)
    {
        g_work_time_test_next_ms = now_ms + APP_WORK_TIME_TEST_INTERVAL_MS;
        start_measurement(APP_MODE_WORK_TIME_TEST);
    }
}
#endif

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
        const uint32_t timeout_ms = mode_uses_multifunction(g_measure_mode) ?
            APP_MULTI_MEASURE_TIMEOUT_MS : APP_RANGE_SINGLE_TIMEOUT_MS;

        if ((now_ms - g_measure_start_ms) >= timeout_ms)
        {
            const AppWorkMode finished_mode = g_measure_mode;

            if (!g_range_command_ack_received)
            {
                APP_LOGW("control", "%s range timeout, rx_bytes=%u frames=%u",
                         mode_uses_multifunction(finished_mode) ? "multi" : "single",
                         (unsigned int)g_range_rx_bytes_since_start,
                         (unsigned int)g_range_frames_since_start);
            }
            else
            {
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

        if (calibration_settings_active())
        {
            handle_calibration_settings_input(event, now_ms);
        }
        else
        {
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
                power_off_wait_forever(true);
            }
        }

        close_measurement_if_done(tick_ms());
#if APP_WORK_TIME_TEST_MODE_ENABLE
        if (!calibration_settings_active())
        {
            update_work_time_test(tick_ms());
        }
#endif
        if (!calibration_settings_active())
        {
            close_mode_click_window(tick_ms());
        }

        if ((now_ms - last_battery_ms) >= 1000U)
        {
            last_battery_ms = now_ms;
            Battery_Read(&battery);
            AppState_UpdateBattery(&battery);
            power_off_if_battery_low("battery", &battery);
        }

#if APP_AUTO_POWER_OFF_ENABLE
        if (
#if APP_WORK_TIME_TEST_MODE_ENABLE
            (!g_work_time_test_active) &&
#endif
            ((now_ms - last_activity_ms) >= APP_AUTO_POWER_OFF_MS))
        {
            power_off_wait_forever(true);
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

static void display_calibration_label(const uint8_t* digit_ids, CalibrationPage page)
{
    const char* label = "   ";

    switch (page)
    {
    case CALIBRATION_PAGE_PITCH_INSTALL:
        label = "PIt";
        break;
    case CALIBRATION_PAGE_YAW_INSTALL:
        label = "HIt";
        break;
    case CALIBRATION_PAGE_YAW_ERROR:
        label = "HEr";
        break;
    default:
        break;
    }

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        DisplayOutput_SetChar(digit_ids[i], label[i]);
    }
}

static void display_calibration_save(const uint8_t* digit_ids)
{
    static const char label[] = "SAVE";

    for (uint8_t i = 0U; i < 4U; ++i)
    {
        DisplayOutput_SetChar(digit_ids[i], label[i]);
    }
}

static void display_calibration_live_angle(const uint8_t* digit_ids,
                                           CalibrationPage page,
                                           const OrientationData* orientation)
{
    int32_t tenths;

    if ((orientation == 0) || (!orientation->valid))
    {
        display_dash_digits(digit_ids, 4U);
        return;
    }

    if (page == CALIBRATION_PAGE_PITCH_INSTALL)
    {
        tenths = orientation->pitch_cd / 10;
        if (tenths < -999)
        {
            tenths = -999;
        }
        else if (tenths > 9999)
        {
            tenths = 9999;
        }

        if (tenths < 0)
        {
            DisplayOutput_SetDash(digit_ids[0], true);
            display_number_fixed(&digit_ids[1], 3U, (uint32_t)(-tenths));
        }
        else
        {
            DisplayOutput_SetNumberRightAligned(digit_ids, 4U, (uint32_t)tenths);
        }
    }
    else
    {
        tenths = normalize_yaw_centidegree(orientation->yaw_cd);
        if (tenths < 0)
        {
            tenths += 36000;
        }
        tenths /= 10;
        DisplayOutput_SetNumberRightAligned(digit_ids, 4U, (uint32_t)tenths);
    }
}

static void display_calibration_settings(const uint8_t* label_digits,
                                         const uint8_t* live_angle_digits,
                                         const uint8_t* value_digits,
                                         const AppStateSnapshot* snapshot)
{
    CalibrationPage page;
    CalibrationOffsets offsets;
    int16_t value = 0;
    uint32_t absolute;

    taskENTER_CRITICAL();
    page = g_calibration_page;
    offsets = g_calibration_edit_offsets;
    taskEXIT_CRITICAL();

    DisplayOutput_ClearBuffer();
    display_battery_symbols(snapshot->battery.level);
    display_calibration_label(label_digits, page);

    if (page == CALIBRATION_PAGE_SAVE)
    {
        display_calibration_save(live_angle_digits);
        DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_FIRST_F, true);
        DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_LAST_E, true);
        return;
    }

    display_calibration_live_angle(live_angle_digits, page, &snapshot->orientation);

    switch (page)
    {
    case CALIBRATION_PAGE_PITCH_INSTALL:
        value = offsets.pitch_install_tenth_deg;
        break;
    case CALIBRATION_PAGE_YAW_INSTALL:
        value = offsets.yaw_install_tenth_deg;
        DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_SINGLE, true);
        break;
    case CALIBRATION_PAGE_YAW_ERROR:
        value = offsets.yaw_error_tenth_deg;
        DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_CONTINUOUS, true);
        break;
    default:
        break;
    }

    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_MINUS, value < 0);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_PLUS, value >= 0);
    absolute = (uint32_t)((value < 0) ? -value : value);
    DisplayOutput_SetNumberRightAligned(value_digits, 4U, absolute);
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

static void log_lcd_refresh_data(const char* view, const AppStateSnapshot* snapshot)
{
    if (snapshot == 0)
    {
        return;
    }

    if (snapshot->orientation.valid)
    {
        APP_LOGI("lcd",
                 "%s mode=%u imu[r_cd=%d p_cd=%d y_cd=%d t=%u]",
                 view,
                 (unsigned int)snapshot->mode,
                 (int)snapshot->orientation.roll_cd,
                 (int)snapshot->orientation.pitch_cd,
                 (int)snapshot->orientation.yaw_cd,
                 (unsigned int)snapshot->orientation.update_ms);
    }

    if (snapshot->range.valid &&
        (snapshot->range.first_valid || snapshot->range.last_valid))
    {
        APP_LOGI("lcd",
                 "%s mode=%u range[cmd=0x%02X f=%u/%u l=%u/%u st=%u t=%u]",
                 view,
                 (unsigned int)snapshot->mode,
                 (unsigned int)snapshot->range.command,
                 snapshot->range.first_valid ? 1U : 0U,
                 (unsigned int)snapshot->range.first_distance_mm,
                 snapshot->range.last_valid ? 1U : 0U,
                 (unsigned int)snapshot->range.last_distance_mm,
                 (unsigned int)snapshot->range.status,
                 (unsigned int)snapshot->range.update_ms);
    }

    if (snapshot->gnss.valid && snapshot->gnss.fix)
    {
        APP_LOGI("lcd",
                 "%s mode=%u gps[fix=%u sat=%u lat_e7=%d lon_e7=%d alt_cm=%d t=%u]",
                 view,
                 (unsigned int)snapshot->mode,
                 snapshot->gnss.fix ? 1U : 0U,
                 (unsigned int)snapshot->gnss.satellites,
                 (int)snapshot->gnss.latitude_e7,
                 (int)snapshot->gnss.longitude_e7,
                 (int)snapshot->gnss.altitude_cm,
                 (unsigned int)snapshot->gnss.update_ms);
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
    const bool multifunction_mode = mode_uses_multifunction(mode);
#if APP_WORK_TIME_TEST_MODE_ENABLE
    const bool work_time_test_mode = mode == APP_MODE_WORK_TIME_TEST;
#else
    const bool work_time_test_mode = false;
#endif

    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RETICLE, true);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_UNIT_M, true);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_SINGLE,
                          (mode == APP_MODE_SINGLE) || multifunction_mode);
    DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_RANGE_CONTINUOUS,
                          (mode == APP_MODE_CONTINUOUS) || work_time_test_mode);
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
    const double bearing = degrees_to_radians((double)yaw_cd / 100.0);
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
    DisplayTargetCoordinate first;
    DisplayTargetCoordinate last;
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
    AppWorkMode last_display_mode = APP_MODE_COUNT;
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

        if (snapshot.mode != last_display_mode)
        {
            multi_target_cache = (DisplayTargetCache){ 0 };
            last_display_mode = snapshot.mode;
        }

        if (g_lcd_calibration_prompt)
        {
            DisplayOutput_SetAll(true);
            DisplayOutput_Flush();
            log_lcd_refresh_data("imu_cal", &snapshot);
            vTaskDelay(pdMS_TO_TICKS(200U));
            continue;
        }

        if (calibration_settings_active())
        {
            display_calibration_settings(azimuth_digits,
                                         range_digits,
                                         altitude_digits,
                                         &snapshot);
            DisplayOutput_Flush();
            log_lcd_refresh_data("offset", &snapshot);
            vTaskDelay(pdMS_TO_TICKS(100U));
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
                    mode_uses_multifunction(snapshot.mode) ? 2000U : 1000U;

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

        if (mode_uses_multifunction(snapshot.mode))
        {
            if (range_result_current &&
                ((!multi_target_cache.update_seen) ||
                 (snapshot.range.update_ms != multi_target_cache.update_ms)))
            {
                const bool range_has_target =
                    snapshot.range.first_valid || snapshot.range.last_valid;

                multi_target_cache.update_seen = true;
                multi_target_cache.update_ms = snapshot.range.update_ms;
                multi_target_cache.has_result = range_has_target;
                multi_target_cache.valid = false;
                multi_target_cache.first_valid = false;
                multi_target_cache.last_valid = false;

                if (range_has_target && snapshot.orientation.valid && snapshot.gnss.fix)
                {
                    multi_target_cache.first_valid = snapshot.range.first_valid;
                    multi_target_cache.last_valid = snapshot.range.last_valid;

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

                    multi_target_cache.valid =
                        multi_target_cache.first_valid ||
                        multi_target_cache.last_valid;
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

        if (mode_uses_multifunction(snapshot.mode))
        {
            const int32_t yaw_deg = snapshot.orientation.valid ?
                normalize_degrees(snapshot.orientation.yaw_cd / 100) : 0;
            int32_t pitch_deg = snapshot.orientation.pitch_cd / 100;

            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_AZIMUTH_DEG, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_LABEL_P, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_DEG, true);
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_MINUS,
                                  snapshot.orientation.valid && (pitch_deg < 0));
            DisplayOutput_SetSymbol((uint8_t)DISPLAY_SYMBOL_PITCH_SIGN_PLUS, false);

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

        if (mode_uses_multifunction(snapshot.mode))
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

            }
        }

        DisplayOutput_Flush();
        log_lcd_refresh_data("normal", &snapshot);

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

    return pdPASS;
}

static void startup_task(void* argument)
{
    BatteryData battery;

    (void)argument;

    PowerManager_Init();
    MeasureCounter_Init();
    MeasureCounter_GetCalibration(&g_calibration_offsets);
    AppState_SetMeasureCount(MeasureCounter_Get());

    Battery_Read(&battery);
    AppState_UpdateBattery(&battery);
    power_off_if_battery_low("startup", &battery);

    if (!ModuleSelfTest_RunAll())
    {
        APP_LOGE("startup", "module self-test failed, power off");
        power_off_wait_forever(false);
    }

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
