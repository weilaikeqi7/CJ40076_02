#include "n32g4fr_it.h"

#include "bsp_uart.h"

void NMI_Handler(void)
{
}

#ifndef APP_USE_CMB_FAULT_HANDLER
void HardFault_Handler(void)
{
    while (1)
    {
    }
}
#endif

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void DebugMon_Handler(void)
{
}

void USART1_IRQHandler(void)
{
    BspUart_HandleIrq(BSP_UART_GNSS);
}

void USART2_IRQHandler(void)
{
    BspUart_HandleIrq(BSP_UART_IMU);
}

void UART6_IRQHandler(void)
{
    BspUart_HandleIrq(BSP_UART_RANGE);
}

void UART5_IRQHandler(void)
{
    BspUart_HandleIrq(BSP_UART_DISPLAY);
}
