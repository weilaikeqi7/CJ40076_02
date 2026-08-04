/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ka025vg_oled.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* DMA 环形接收缓冲：USART1_RX 经 DMA2_Stream2 循环写入，配合空闲中断分帧 */
#define RX_DMA_BUF_SIZE 256U
static uint8_t rx_dma_buf[RX_DMA_BUF_SIZE];
static volatile uint16_t rx_dma_old_pos;
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PROTOCOL_HEAD1      0xA5U
#define PROTOCOL_HEAD2      0x5AU
#define PROTOCOL_TYPE_STATE 0x01U
#define PROTOCOL_PAYLOAD_LEN 37U // 1 (version) + 1 (all_on & brightness) + 27 (digits) + 8 (symbols)
#define PROTOCOL_FRAME_LEN  43U // 2 (head) + 1 (type) + 1 (seq) + 1 (len) + 37 (payload) + 1 (checksum)

static uint8_t rx_buf[PROTOCOL_FRAME_LEN * 2];
static uint16_t rx_len = 0;
static uint8_t parse_and_update_state(void)
{
  uint16_t i = 0;
  while ((rx_len - i) >= PROTOCOL_FRAME_LEN)
  {
    if ((rx_buf[i] == PROTOCOL_HEAD1) && (rx_buf[i+1] == PROTOCOL_HEAD2))
    {
      if (rx_buf[i+2] == PROTOCOL_TYPE_STATE)
      {
        uint8_t payload_len = rx_buf[i+4];
        if (payload_len == PROTOCOL_PAYLOAD_LEN)
        {
          uint8_t checksum = 0;
          for (uint16_t j = 2; j < PROTOCOL_FRAME_LEN - 1; ++j)
          {
            checksum += rx_buf[i + j];
          }
          if (checksum == rx_buf[i + PROTOCOL_FRAME_LEN - 1])
          {
            // 校验通过，提取数据
            uint8_t ctrl = rx_buf[i + 6];
            uint8_t all_on = ctrl & 0x01U;
            uint8_t brightness = (ctrl >> 4U) & 0x0FU;
            uint8_t digits[27];
            (void)memcpy(digits, &rx_buf[i + 7], 27);
            uint64_t symbols = 0;
            for (uint8_t s = 0; s < 8; ++s)
            {
              symbols |= ((uint64_t)rx_buf[i + 34 + s]) << (s * 8);
            }
            KA025VG_UpdateState(digits, symbols, all_on, brightness);

            // 移除已处理的帧及之前的所有无效字节
            rx_len -= (i + PROTOCOL_FRAME_LEN);
            (void)memmove(rx_buf, &rx_buf[i + PROTOCOL_FRAME_LEN], rx_len);
            return 1;
          }
        }
      }
      i++; // 校验和不正确或长度不对，滑动1字节继续寻找包头
    }
    else
    {
      i++;
    }
  }

  if (i > 0)
  {
    rx_len -= i;
    (void)memmove(rx_buf, &rx_buf[i], rx_len);
  }
  return 0;
}

/* 将一段新到字节追加进协议解析缓冲，溢出时丢弃最旧数据保证不死锁 */
static void rx_buf_push(const uint8_t *data, uint16_t length)
{
  for (uint16_t k = 0U; k < length; ++k)
  {
    if (rx_len >= sizeof(rx_buf))
    {
      /* 缓冲满：丢弃最旧 1 字节，滑窗继续，避免整帧因抖动卡死 */
      (void)memmove(rx_buf, &rx_buf[1], rx_len - 1U);
      rx_len--;
    }
    rx_buf[rx_len++] = data[k];
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (KA025VG_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }

  // 开机先显示全屏测试图案，确认 OLED 初始化和 QSPI 刷新正常
  if (KA025VG_ShowFixedScreen() != HAL_OK)
  {
    Error_Handler();
  }

  // 启动 DMA + 空闲中断接收：DMA 循环填充 rx_dma_buf，空闲/半满/满时触发 RxEventCallback
  rx_dma_old_pos = 0U;
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
  /* 关闭 DMA 半传输中断，仅在空闲/满传输时处理，减少无谓回调 */
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (parse_and_update_state())
    {
      KA025VG_RenderAndShow();
    }
    HAL_Delay(10U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* DMA + 空闲中断接收事件：Size 为 DMA 当前写入位置（累计），按环形差量取出新字节 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART1)
  {
    return;
  }

  if (Size != rx_dma_old_pos)
  {
    if (Size > rx_dma_old_pos)
    {
      /* 未回绕：一段连续数据 */
      rx_buf_push(&rx_dma_buf[rx_dma_old_pos], (uint16_t)(Size - rx_dma_old_pos));
    }
    else
    {
      /* 已回绕：先取尾部，再取头部 */
      rx_buf_push(&rx_dma_buf[rx_dma_old_pos], (uint16_t)(RX_DMA_BUF_SIZE - rx_dma_old_pos));
      if (Size > 0U)
      {
        rx_buf_push(&rx_dma_buf[0], Size);
      }
    }
    rx_dma_old_pos = Size;
  }

  if (rx_dma_old_pos >= RX_DMA_BUF_SIZE)
  {
    rx_dma_old_pos = 0U;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == OLED_TE_Pin)
  {
    KA025VG_TeInterruptHandler();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 清除溢出/帧/噪声错误标志，否则错误后接收链路会永久停摆 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    /* 重启 DMA + 空闲中断接收 */
    rx_dma_old_pos = 0U;
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
