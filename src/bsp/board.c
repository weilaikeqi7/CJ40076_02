#include "board.h"

#include "board_config.h"
#include "bsp_adc.h"
#include "bsp_gpio.h"
#include "misc.h"
#include "n32g4fr_rcc.h"

static void write_active(GPIO_Module* port, uint16_t pin, uint32_t active_high, bool enabled)
{
    BspGpio_Write(port, pin, (enabled == (active_high != 0U)));
}

static bool read_active(GPIO_Module* port, uint16_t pin, uint32_t active_high)
{
    const bool high = BspGpio_Read(port, pin);
    return high == (active_high != 0U);
}

void Board_PowerHold(bool enabled)
{
    write_active(BOARD_PWR_HOLD_PORT, BOARD_PWR_HOLD_PIN, BOARD_PWR_HOLD_ACTIVE_HIGH, enabled);
}

void Board_SetDisplayPower(bool enabled)
{
    write_active(BOARD_PWR_DISPLAY_PORT, BOARD_PWR_DISPLAY_PIN, BOARD_PWR_DISPLAY_ACTIVE_HIGH, enabled);
}

void Board_SetRangePower(bool enabled)
{
    write_active(BOARD_PWR_RANGE_PORT, BOARD_PWR_RANGE_PIN, BOARD_PWR_RANGE_ACTIVE_HIGH, enabled);
}

void Board_SetGnssPower(bool enabled)
{
    write_active(BOARD_PWR_GNSS_PORT, BOARD_PWR_GNSS_PIN, BOARD_PWR_GNSS_ACTIVE_HIGH, enabled);
}

void Board_SetImuPower(bool enabled)
{
    write_active(BOARD_PWR_IMU_PORT, BOARD_PWR_IMU_PIN, BOARD_PWR_IMU_ACTIVE_HIGH, enabled);
}

bool Board_ReadPowerKey(void)
{
    return read_active(BOARD_KEY_POWER_PORT, BOARD_KEY_POWER_PIN, BOARD_KEY_POWER_ACTIVE_HIGH);
}

bool Board_ReadModeKey(void)
{
    return read_active(BOARD_KEY_MODE_PORT, BOARD_KEY_MODE_PIN, BOARD_KEY_MODE_ACTIVE_HIGH);
}

void Board_Init(void)
{
    BspGpioPull mode_key_pull;
    BspGpioPull power_key_pull;

    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA | RCC_APB2_PERIPH_GPIOB, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* PA15/PB3/PB4 默认是 JTAG 引脚，禁用 JTAG-DP 仅保留 SW-DP 后才能作普通 GPIO 使用 */
    GPIO_ConfigPinRemap(GPIO_RMP_SW_JTAG_SW_ENABLE, ENABLE);

    BspGpio_InitOutput(BOARD_PWR_RANGE_PORT, BOARD_PWR_RANGE_PIN, false);
    BspGpio_InitOutput(BOARD_PWR_IMU_PORT, BOARD_PWR_IMU_PIN, false);
    BspGpio_InitOutput(BOARD_PWR_GNSS_PORT, BOARD_PWR_GNSS_PIN, false);

    mode_key_pull = (BOARD_KEY_MODE_ACTIVE_HIGH != 0U) ? BSP_GPIO_PULL_DOWN : BSP_GPIO_PULL_UP;
    power_key_pull = (BOARD_KEY_POWER_ACTIVE_HIGH != 0U) ? BSP_GPIO_PULL_DOWN : BSP_GPIO_PULL_UP;
    BspGpio_InitInput(BOARD_KEY_MODE_PORT, BOARD_KEY_MODE_PIN, mode_key_pull);
    BspGpio_InitInput(BOARD_KEY_POWER_PORT, BOARD_KEY_POWER_PIN, power_key_pull);

    BspGpio_InitOutput(BOARD_PWR_HOLD_PORT, BOARD_PWR_HOLD_PIN, false);
    BspGpio_InitOutput(BOARD_PWR_DISPLAY_PORT, BOARD_PWR_DISPLAY_PIN, false);

    while (!Board_ReadPowerKey())
    {
        /* Wait for the active-low PA5 power key before latching power. */
    }

    Board_PowerHold(true);
    Board_SetDisplayPower(true);

    BspGpio_InitAnalog(BOARD_BAT_ADC_PORT, BOARD_BAT_ADC_PIN);

    BspAdc_Init();

}
