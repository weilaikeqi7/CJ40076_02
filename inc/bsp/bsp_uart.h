#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    BSP_UART_GNSS = 0,
    BSP_UART_IMU,
    BSP_UART_RANGE,
    BSP_UART_DISPLAY,
    BSP_UART_COUNT
} BspUartId;

void BspUart_InitAll(void);
void BspUart_Reinit(BspUartId id);
void BspUart_HandleIrq(BspUartId id);
bool BspUart_ReadByte(BspUartId id, uint8_t* byte);
size_t BspUart_Write(BspUartId id, const uint8_t* data, size_t length, uint32_t timeout_ms);
void BspUart_FlushRx(BspUartId id);

#endif /* BSP_UART_H */
