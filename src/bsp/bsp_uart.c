#include "bsp_uart.h"

#include "FreeRTOS.h"
#include "board_config.h"
#include "bsp_gpio.h"
#include "misc.h"
#include "n32g4fr_dma.h"
#include "n32g4fr_rcc.h"
#include "n32g4fr_usart.h"
#include "task.h"

#define BSP_UART_GNSS_RX_BUFFER_SIZE  512U
#define BSP_UART_IMU_RX_BUFFER_SIZE   128U
#define BSP_UART_RANGE_RX_BUFFER_SIZE 64U
#define BSP_UART_DISPLAY_RX_BUFFER_SIZE  20U

typedef struct
{
    USART_Module* module;
    IRQn_Type irq;
    GPIO_Module* tx_port;
    uint16_t tx_pin;
    GPIO_Module* rx_port;
    uint16_t rx_pin;
    uint32_t gpio_remap;
    uint32_t apb1_clock;
    uint32_t apb2_clock;
    uint32_t baudrate;
    uint8_t irq_priority;
    uint8_t* rx_buffer;
    uint16_t rx_buffer_size;
    uint8_t* rx_dma_buffer;
    uint16_t rx_dma_size;
    DMA_ChannelType* rx_dma;
} BspUartConfig;

static uint8_t g_gnss_rx_buffer[BSP_UART_GNSS_RX_BUFFER_SIZE];
static uint8_t g_imu_rx_buffer[BSP_UART_IMU_RX_BUFFER_SIZE];
static uint8_t g_range_rx_buffer[BSP_UART_RANGE_RX_BUFFER_SIZE];
static uint8_t g_display_rx_buffer[BSP_UART_DISPLAY_RX_BUFFER_SIZE];

static uint8_t g_gnss_dma_rx_buffer[BSP_UART_GNSS_RX_BUFFER_SIZE];
static uint8_t g_imu_dma_rx_buffer[BSP_UART_IMU_RX_BUFFER_SIZE];
static uint8_t g_range_dma_rx_buffer[BSP_UART_RANGE_RX_BUFFER_SIZE];
static uint8_t g_display_dma_rx_buffer[BSP_UART_DISPLAY_RX_BUFFER_SIZE];

static const BspUartConfig g_uart_config[BSP_UART_COUNT] = {
    [BSP_UART_GNSS] = {
        .module = USART1,
        .irq = USART1_IRQn,
        .tx_port = BOARD_GNSS_TX_PORT,
        .tx_pin = BOARD_GNSS_TX_PIN,
        .rx_port = BOARD_GNSS_RX_PORT,
        .rx_pin = BOARD_GNSS_RX_PIN,
        .gpio_remap = BOARD_GNSS_REMAP,
        .apb1_clock = 0U,
        .apb2_clock = RCC_APB2_PERIPH_USART1,
        .baudrate = APP_GNSS_UART_BAUD,
        .irq_priority = 5U,
        .rx_buffer = g_gnss_rx_buffer,
        .rx_buffer_size = BSP_UART_GNSS_RX_BUFFER_SIZE,
        .rx_dma_buffer = g_gnss_dma_rx_buffer,
        .rx_dma_size = BSP_UART_GNSS_RX_BUFFER_SIZE,
        .rx_dma = DMA1_CH5
    },
    [BSP_UART_IMU] = {
        .module = USART2,
        .irq = USART2_IRQn,
        .tx_port = BOARD_USART2_TX_PORT,
        .tx_pin = BOARD_USART2_TX_PIN,
        .rx_port = BOARD_USART2_RX_PORT,
        .rx_pin = BOARD_USART2_RX_PIN,
        .gpio_remap = BOARD_USART2_REMAP,
        .apb1_clock = RCC_APB1_PERIPH_USART2,
        .apb2_clock = 0U,
        .baudrate = APP_JY901B_UART_BAUD,
        .irq_priority = 6U,
        .rx_buffer = g_imu_rx_buffer,
        .rx_buffer_size = BSP_UART_IMU_RX_BUFFER_SIZE,
        .rx_dma_buffer = g_imu_dma_rx_buffer,
        .rx_dma_size = BSP_UART_IMU_RX_BUFFER_SIZE,
        .rx_dma = DMA1_CH6
    },
    [BSP_UART_RANGE] = {
        .module = UART6,
        .irq = UART6_IRQn,
        .tx_port = BOARD_RANGE_TX_PORT,
        .tx_pin = BOARD_RANGE_TX_PIN,
        .rx_port = BOARD_RANGE_RX_PORT,
        .rx_pin = BOARD_RANGE_RX_PIN,
        .gpio_remap = BOARD_RANGE_REMAP,
        .apb1_clock = 0U,
        .apb2_clock = RCC_APB2_PERIPH_UART6,
        .baudrate = APP_RANGE_UART_BAUD,
        .irq_priority = 7U,
        .rx_buffer = g_range_rx_buffer,
        .rx_buffer_size = BSP_UART_RANGE_RX_BUFFER_SIZE,
        .rx_dma_buffer = g_range_dma_rx_buffer,
        .rx_dma_size = BSP_UART_RANGE_RX_BUFFER_SIZE,
        .rx_dma = DMA2_CH1
    },
    [BSP_UART_DISPLAY] = {
        .module = UART5,
        .irq = UART5_IRQn,
        .tx_port = BOARD_DISPLAY_TX_PORT,
        .tx_pin = BOARD_DISPLAY_TX_PIN,
        .rx_port = BOARD_DISPLAY_RX_PORT,
        .rx_pin = BOARD_DISPLAY_RX_PIN,
        .gpio_remap = BOARD_DISPLAY_REMAP,
        .apb1_clock = RCC_APB1_PERIPH_UART5,
        .apb2_clock = 0U,
        .baudrate = APP_DISPLAY_UART_BAUD,
        .irq_priority = 8U,
        .rx_buffer = g_display_rx_buffer,
        .rx_buffer_size = BSP_UART_DISPLAY_RX_BUFFER_SIZE,
        .rx_dma_buffer = g_display_dma_rx_buffer,
        .rx_dma_size = BSP_UART_DISPLAY_RX_BUFFER_SIZE,
        .rx_dma = DMA1_CH8
    },
};

static volatile uint16_t g_rx_head[BSP_UART_COUNT];
static volatile uint16_t g_rx_tail[BSP_UART_COUNT];
static volatile uint16_t g_dma_rx_consumed[BSP_UART_COUNT];
static bool g_uart_initialized[BSP_UART_COUNT];

static bool uart_id_valid(BspUartId id)
{
    return ((uint32_t)id) < (uint32_t)BSP_UART_COUNT;
}

static void uart_enable_clock(const BspUartConfig* config)
{
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO, ENABLE);

    if (config->apb1_clock != 0U)
    {
        RCC_EnableAPB1PeriphClk(config->apb1_clock, ENABLE);
    }
    if (config->apb2_clock != 0U)
    {
        RCC_EnableAPB2PeriphClk(config->apb2_clock, ENABLE);
    }
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA1 | RCC_AHB_PERIPH_DMA2, ENABLE);
}

static void uart_gpio_init(GPIO_Module* port,
                           uint16_t pin,
                           GPIO_ModeType mode)
{
    GPIO_InitType init;

    BspGpio_EnableClock(port);
    GPIO_InitStruct(&init);
    init.Pin        = pin;
    init.GPIO_Mode  = mode;
    init.GPIO_Speed = (mode == GPIO_Mode_AF_PP) ? GPIO_Speed_10MHz : GPIO_INPUT;
    GPIO_InitPeripheral(port, &init);
}

static void uart_clear_pending_rx(const BspUartConfig* config)
{
    volatile uint32_t discard = 0U;
    uint32_t guard = (uint32_t)config->rx_buffer_size + 4U;

    while (guard-- > 0U)
    {
        const uint32_t status = config->module->STS;
        if ((status & (USART_FLAG_RXDNE |
                       USART_FLAG_IDLEF |
                       USART_FLAG_OREF |
                       USART_FLAG_NEF |
                       USART_FLAG_FEF |
                       USART_FLAG_PEF)) == 0U)
        {
            break;
        }

        discard = config->module->DAT;
    }

    (void)discard;
    NVIC_ClearPendingIRQ(config->irq);
}

static void uart_push_rx_byte(BspUartId id, uint8_t byte)
{
    const BspUartConfig* config = &g_uart_config[id];
    const uint16_t next = (uint16_t)((g_rx_head[id] + 1U) % config->rx_buffer_size);

    if (next != g_rx_tail[id])
    {
        config->rx_buffer[g_rx_head[id]] = byte;
        g_rx_head[id] = next;
    }
}

static void uart_drain_dma_rx(BspUartId id)
{
    const BspUartConfig* config = &g_uart_config[id];
    uint16_t write_pos;
    uint16_t consumed;

    if (!g_uart_initialized[id])
    {
        return;
    }

    write_pos = (uint16_t)(config->rx_dma_size - DMA_GetCurrDataCounter(config->rx_dma));
    if (write_pos > config->rx_dma_size)
    {
        write_pos = config->rx_dma_size;
    }

    consumed = g_dma_rx_consumed[id];
    if (consumed >= config->rx_dma_size)
    {
        consumed = 0U;
    }

    if (write_pos == config->rx_dma_size)
    {
        for (uint16_t i = consumed; i < config->rx_dma_size; ++i)
        {
            uart_push_rx_byte(id, config->rx_dma_buffer[i]);
        }
        g_dma_rx_consumed[id] = 0U;
        return;
    }

    if (write_pos > consumed)
    {
        for (uint16_t i = consumed; i < write_pos; ++i)
        {
            uart_push_rx_byte(id, config->rx_dma_buffer[i]);
        }
    }
    else if (write_pos < consumed)
    {
        for (uint16_t i = consumed; i < config->rx_dma_size; ++i)
        {
            uart_push_rx_byte(id, config->rx_dma_buffer[i]);
        }
        for (uint16_t i = 0U; i < write_pos; ++i)
        {
            uart_push_rx_byte(id, config->rx_dma_buffer[i]);
        }
    }

    g_dma_rx_consumed[id] = write_pos;
}

static void uart_poll_dma_rx(BspUartId id)
{
    const BspUartConfig* config = &g_uart_config[id];

    if (!g_uart_initialized[id])
    {
        return;
    }

    NVIC_DisableIRQ(config->irq);
    uart_drain_dma_rx(id);
    NVIC_EnableIRQ(config->irq);
}

static void uart_init_one(BspUartId id)
{
    const BspUartConfig* config = &g_uart_config[id];
    USART_InitType usart_init;
    DMA_InitType dma_init;
    NVIC_InitType nvic_init;

    uart_enable_clock(config);
    if (config->gpio_remap != 0U)
    {
        GPIO_ConfigPinRemap(config->gpio_remap, ENABLE);
    }
    uart_gpio_init(config->tx_port, config->tx_pin, GPIO_Mode_AF_PP);
    uart_gpio_init(config->rx_port, config->rx_pin, GPIO_Mode_IPU);

    DMA_DeInit(config->rx_dma);
    g_dma_rx_consumed[id] = 0U;
    DMA_StructInit(&dma_init);
    dma_init.PeriphAddr = (uint32_t)config->module + 4U;
    dma_init.MemAddr = (uint32_t)config->rx_dma_buffer;
    dma_init.Direction = DMA_DIR_PERIPH_SRC;
    dma_init.BufSize = config->rx_dma_size;
    dma_init.PeriphInc = DMA_PERIPH_INC_DISABLE;
    dma_init.DMA_MemoryInc = DMA_MEM_INC_ENABLE;
    dma_init.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    dma_init.MemDataSize = DMA_MemoryDataSize_Byte;
    dma_init.CircularMode = DMA_MODE_CIRCULAR;
    dma_init.Priority = DMA_PRIORITY_VERY_HIGH;
    dma_init.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(config->rx_dma, &dma_init);

    nvic_init.NVIC_IRQChannel                   = (uint8_t)config->irq;
    nvic_init.NVIC_IRQChannelPreemptionPriority = config->irq_priority;
    nvic_init.NVIC_IRQChannelSubPriority        = 0U;
    nvic_init.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic_init);

    USART_StructInit(&usart_init);
    usart_init.BaudRate            = config->baudrate;
    usart_init.WordLength          = USART_WL_8B;
    usart_init.StopBits            = USART_STPB_1;
    usart_init.Parity              = USART_PE_NO;
    usart_init.Mode                = USART_MODE_RX | USART_MODE_TX;
    usart_init.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_Init(config->module, &usart_init);

    USART_ConfigInt(config->module, USART_INT_IDLEF, ENABLE);
    USART_ConfigInt(config->module, USART_INT_RXDNE, DISABLE);
    USART_EnableDMA(config->module, USART_DMAREQ_RX, ENABLE);
    DMA_EnableChannel(config->rx_dma, ENABLE);
    USART_Enable(config->module, ENABLE);

    uart_clear_pending_rx(config);
    g_uart_initialized[id] = true;
}

void BspUart_InitAll(void)
{
    for (uint32_t id = 0U; id < (uint32_t)BSP_UART_COUNT; ++id)
    {
        BspUart_FlushRx((BspUartId)id);
        uart_init_one((BspUartId)id);
    }
}

void BspUart_Reinit(BspUartId id)
{
    const BspUartConfig* config;

    if (!uart_id_valid(id))
    {
        return;
    }

    config = &g_uart_config[id];

    taskENTER_CRITICAL();
    NVIC_DisableIRQ(config->irq);
    USART_ConfigInt(config->module, USART_INT_IDLEF, DISABLE);
    USART_ConfigInt(config->module, USART_INT_RXDNE, DISABLE);
    USART_EnableDMA(config->module, USART_DMAREQ_RX, DISABLE);
    USART_Enable(config->module, DISABLE);
    DMA_EnableChannel(config->rx_dma, DISABLE);
    uart_clear_pending_rx(config);
    g_rx_head[id] = 0U;
    g_rx_tail[id] = 0U;
    g_dma_rx_consumed[id] = 0U;
    g_uart_initialized[id] = false;
    taskEXIT_CRITICAL();

    uart_init_one(id);
}

void BspUart_HandleIrq(BspUartId id)
{
    const BspUartConfig* config;

    if (!uart_id_valid(id))
    {
        return;
    }

    config = &g_uart_config[id];

    if (USART_GetIntStatus(config->module, USART_INT_IDLEF) != RESET)
    {
        (void)config->module->STS;
        (void)config->module->DAT;

        uart_drain_dma_rx(id);
    }

    if ((USART_GetFlagStatus(config->module, USART_FLAG_OREF) != RESET) ||
        (USART_GetFlagStatus(config->module, USART_FLAG_NEF) != RESET) ||
        (USART_GetFlagStatus(config->module, USART_FLAG_PEF) != RESET) ||
        (USART_GetFlagStatus(config->module, USART_FLAG_FEF) != RESET))
    {
        (void)config->module->STS;
        (void)config->module->DAT;
    }
}

bool BspUart_ReadByte(BspUartId id, uint8_t* byte)
{
    bool has_byte;
    uint16_t tail;

    if ((!uart_id_valid(id)) || (byte == 0))
    {
        return false;
    }

    uart_poll_dma_rx(id);

    taskENTER_CRITICAL();
    has_byte = (g_rx_head[id] != g_rx_tail[id]);
    if (has_byte)
    {
        tail = g_rx_tail[id];
        *byte = g_uart_config[id].rx_buffer[tail];
        g_rx_tail[id] = (uint16_t)((tail + 1U) % g_uart_config[id].rx_buffer_size);
    }
    taskEXIT_CRITICAL();

    return has_byte;
}

size_t BspUart_Write(BspUartId id, const uint8_t* data, size_t length, uint32_t timeout_ms)
{
    const BspUartConfig* config;
    TickType_t start;
    size_t sent = 0U;

    if ((!uart_id_valid(id)) || (data == 0))
    {
        return 0U;
    }

    config = &g_uart_config[id];
    start = xTaskGetTickCount();

    while (sent < length)
    {
        while (USART_GetFlagStatus(config->module, USART_FLAG_TXDE) == RESET)
        {
            if ((timeout_ms != 0U) &&
                ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)))
            {
                return sent;
            }
        }
        USART_SendData(config->module, data[sent]);
        ++sent;
    }

    while (USART_GetFlagStatus(config->module, USART_FLAG_TXC) == RESET)
    {
        if ((timeout_ms != 0U) &&
            ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)))
        {
            break;
        }
    }

    return sent;
}

void BspUart_FlushRx(BspUartId id)
{
    if (!uart_id_valid(id))
    {
        return;
    }

    taskENTER_CRITICAL();
    if (g_uart_initialized[id])
    {
        NVIC_DisableIRQ(g_uart_config[id].irq);
        DMA_EnableChannel(g_uart_config[id].rx_dma, DISABLE);
        uart_clear_pending_rx(&g_uart_config[id]);
        DMA_SetCurrDataCounter(g_uart_config[id].rx_dma, g_uart_config[id].rx_dma_size);
        DMA_EnableChannel(g_uart_config[id].rx_dma, ENABLE);
        NVIC_EnableIRQ(g_uart_config[id].irq);
    }
    g_rx_head[id] = 0U;
    g_rx_tail[id] = 0U;
    g_dma_rx_consumed[id] = 0U;
    taskEXIT_CRITICAL();
}
