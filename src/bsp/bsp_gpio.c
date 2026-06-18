#include "bsp_gpio.h"

#include "n32g4fr_rcc.h"

static GPIO_ModeType gpio_input_mode(BspGpioPull pull)
{
    if (pull == BSP_GPIO_PULL_UP)
    {
        return GPIO_Mode_IPU;
    }
    if (pull == BSP_GPIO_PULL_DOWN)
    {
        return GPIO_Mode_IPD;
    }
    return GPIO_Mode_IN_FLOATING;
}

void BspGpio_EnableClock(GPIO_Module* port)
{
    if (port == GPIOA)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    }
    else if (port == GPIOB)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    }
    else if (port == GPIOC)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);
    }
    else if (port == GPIOD)
    {
        RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOD, ENABLE);
    }
}

void BspGpio_InitOutput(GPIO_Module* port, uint16_t pin, bool high)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_WriteBit(port, pin, high ? Bit_SET : Bit_RESET);

    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Speed = GPIO_Speed_2MHz;
    init.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitPeripheral(port, &init);
}

void BspGpio_InitInput(GPIO_Module* port, uint16_t pin, BspGpioPull pull)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Speed = GPIO_INPUT;
    init.GPIO_Mode  = gpio_input_mode(pull);
    GPIO_InitPeripheral(port, &init);
}

void BspGpio_InitAnalog(GPIO_Module* port, uint16_t pin)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Speed = GPIO_INPUT;
    init.GPIO_Mode  = GPIO_Mode_AIN;
    GPIO_InitPeripheral(port, &init);
}

void BspGpio_InitAlternate(GPIO_Module* port, uint16_t pin, uint32_t alternate)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Speed = GPIO_Speed_10MHz;
    init.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitPeripheral(port, &init);
    if (alternate != 0U)
    {
        GPIO_ConfigPinRemap(alternate, ENABLE);
    }
}

void BspGpio_InitAlternateInput(GPIO_Module* port, uint16_t pin, uint32_t alternate)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Speed = GPIO_INPUT;
    init.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitPeripheral(port, &init);
    if (alternate != 0U)
    {
        GPIO_ConfigPinRemap(alternate, ENABLE);
    }
}

void BspGpio_Write(GPIO_Module* port, uint16_t pin, bool high)
{
    GPIO_WriteBit(port, pin, high ? Bit_SET : Bit_RESET);
}

bool BspGpio_Read(GPIO_Module* port, uint16_t pin)
{
    return GPIO_ReadInputDataBit(port, pin) != 0U;
}
