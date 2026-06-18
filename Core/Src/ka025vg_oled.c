#include "ka025vg_oled.h"
#include "main.h"

#include <string.h>

#define KA025VG_QSPI_TIMEOUT_MS     1000U
#define KA025VG_FRAME_BYTES         (KA025VG_WIDTH * KA025VG_HEIGHT)

#define KA025VG_QSPI_WRITE_IMAGE    0x32U
#define KA025VG_IMAGE_ALT_BYTES     0x003C00U

static QSPI_HandleTypeDef *ka025vg_qspi;
static volatile uint8_t ka025vg_qspi_tx_done;
static volatile uint8_t ka025vg_te_cmd;
static volatile uint8_t ka025vg_te_ready;
static volatile uint8_t ka025vg_te_frame;
static volatile uint8_t ka025vg_te_count;
static uint8_t ka025vg_framebuffer[KA025VG_FRAME_BYTES];

static HAL_StatusTypeDef KA025VG_QspiWriteCommand(uint8_t command,
                                                  const uint8_t *params,
                                                  uint32_t length);

/* 与另一个工程 Core/Src/quadspi.c::qspi_instruct 一致：配置命令 + DMA发送，不在本函数等待。 */
static void KA025VG_QspiInstruct(uint8_t *data, uint16_t length)
{
  QSPI_CommandTypeDef sCommand;

  sCommand.InstructionMode = QSPI_INSTRUCTION_NONE;
  sCommand.Instruction = 0x00U;
  sCommand.AddressMode = QSPI_ADDRESS_NONE;
  sCommand.AddressSize = QSPI_ADDRESS_24_BITS;
  sCommand.Address = 0U;
  sCommand.DataMode = QSPI_DATA_1_LINE;
  sCommand.DummyCycles = 0U;
  sCommand.NbData = length;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DdrMode = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(ka025vg_qspi, &sCommand, KA025VG_QSPI_TIMEOUT_MS) != HAL_OK)
  {
    return;
  }

  (void)HAL_QSPI_Transmit_DMA(ka025vg_qspi, data);
}

static HAL_StatusTypeDef KA025VG_QspiInstructAndWait(uint8_t *data,
                                                     uint16_t length,
                                                     uint32_t delay_ms)
{
  uint32_t start = HAL_GetTick();

  ka025vg_qspi_tx_done = 0U;
  KA025VG_QspiInstruct(data, length);

  while ((ka025vg_qspi_tx_done == 0U) && ((HAL_GetTick() - start) < KA025VG_QSPI_TIMEOUT_MS))
  {
  }

  if (ka025vg_qspi_tx_done == 0U)
  {
    return HAL_TIMEOUT;
  }

  ka025vg_qspi_tx_done = 0U;
  if (delay_ms > 0U)
  {
    HAL_Delay(delay_ms);
  }

  return HAL_OK;
}

static HAL_StatusTypeDef KA025VG_QspiTransmitDma(uint8_t *data)
{
  uint32_t start = HAL_GetTick();

  ka025vg_qspi_tx_done = 0U;
  if (HAL_QSPI_Transmit_DMA(ka025vg_qspi, data) != HAL_OK)
  {
    return HAL_ERROR;
  }

  while ((ka025vg_qspi_tx_done == 0U) && ((HAL_GetTick() - start) < KA025VG_QSPI_TIMEOUT_MS))
  {
  }

  return (ka025vg_qspi_tx_done != 0U) ? HAL_OK : HAL_TIMEOUT;
}

static HAL_StatusTypeDef KA025VG_QspiTransmitPacket(const uint8_t *packet, uint32_t length)
{
  QSPI_CommandTypeDef command = {0};

  if ((ka025vg_qspi == NULL) || (packet == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  command.InstructionMode = QSPI_INSTRUCTION_NONE;
  command.AddressMode = QSPI_ADDRESS_NONE;
  command.AddressSize = QSPI_ADDRESS_24_BITS;
  command.Address = 0U;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  command.DataMode = QSPI_DATA_1_LINE;
  command.DummyCycles = 0U;
  command.NbData = length;
  command.DdrMode = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(ka025vg_qspi, &command, KA025VG_QSPI_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return KA025VG_QspiTransmitDma((uint8_t *)packet);
}

static HAL_StatusTypeDef KA025VG_QspiWriteCommand(uint8_t command,
                                                  const uint8_t *params,
                                                  uint32_t length)
{
  uint8_t packet[36];

  if ((ka025vg_qspi == NULL) ||
      ((length > 0U) && (params == NULL)) ||
      ((length + 4U) > sizeof(packet)))
  {
    return HAL_ERROR;
  }

  packet[0] = 0x02U;
  packet[1] = 0x00U;
  packet[2] = command;
  packet[3] = 0x00U;
  if (length > 0U)
  {
    (void)memcpy(&packet[4], params, length);
  }

  return KA025VG_QspiTransmitPacket(packet, length + 4U);
}

HAL_StatusTypeDef KA025VG_WriteCommand(uint8_t command, const uint8_t *params, uint32_t length)
{
  return KA025VG_QspiWriteCommand(command, params, length);
}

/* 与另一个工程 Core/Src/gpio.c 的 OLED_TE_Pin 中断逻辑保持一致。 */
void KA025VG_TeInterruptHandler(void)
{
  if (ka025vg_te_ready == 0U)
  {
    ka025vg_te_count++;
    if (ka025vg_te_count == 2U)
    {
      ka025vg_te_cmd = 1U;
    }
    if (ka025vg_te_count == 16U)
    {
      ka025vg_te_ready = 1U;
    }
  }
  else
  {
    ka025vg_te_frame = 1U;
  }
}

HAL_StatusTypeDef KA025VG_Init(QSPI_HandleTypeDef *hqspi)
{
  /* 下面这组命令按另一个工程 OLED/QSPI/qspi.c 的 B_Version oled_QSPI_init() 顺序整理。
     cd36 原工程定义为5字节但按8字节发送，这里显式补齐为8字节，避免数组越界，同时保持实际发送行为。 */
  uint8_t cd80[5] = {0x02U, 0x00U, 0x80U, 0x00U, 0x01U};
  uint8_t cd91[5] = {0x02U, 0x00U, 0x91U, 0x00U, 0x90U};
  uint8_t cd89[8] = {0x02U, 0x00U, 0x89U, 0x00U, 0xF1U, 0xF1U, 0x00U, 0x09U};
  uint8_t cd8A[8] = {0x02U, 0x00U, 0x8AU, 0x00U, 0x18U, 0x03U, 0x07U, 0x04U};
  uint8_t cd96[6] = {0x02U, 0x00U, 0x96U, 0x00U, 0x31U, 0x00U};
  uint8_t cd97[6] = {0x02U, 0x00U, 0x97U, 0x00U, 0x01U, 0x00U};
  uint8_t cd95[6] = {0x02U, 0x00U, 0x95U, 0x00U, 0x81U, 0x0BU};
  uint8_t cd51[6] = {0x02U, 0x00U, 0x51U, 0x00U, 0xFFU, 0x03U};
  uint8_t cd35[5] = {0x02U, 0x00U, 0x35U, 0x00U, 0x00U};
  uint8_t cd44[5] = {0x02U, 0x00U, 0x44U, 0x00U, 0x01U};
  uint8_t cdA8[5] = {0x02U, 0x00U, 0xA8U, 0x00U, 0x01U};
  uint8_t cdAA[5] = {0x02U, 0x00U, 0xAAU, 0x00U, 0x15U};
  uint8_t cdA9[5] = {0x02U, 0x00U, 0xA9U, 0x00U, 0x01U};
  uint8_t cd11[5] = {0x02U, 0x00U, 0x11U, 0x00U, 0x00U};
  uint8_t cd29[5] = {0x02U, 0x00U, 0x29U, 0x00U, 0x00U};
  uint8_t cd36[8] = {0x02U, 0x00U, 0x36U, 0x00U, 0x03U, 0x02U, 0x00U, 0x35U};

  if (hqspi == NULL)
  {
    return HAL_ERROR;
  }

  ka025vg_qspi = hqspi;
  ka025vg_qspi_tx_done = 0U;
  ka025vg_te_cmd = 0U;
  ka025vg_te_ready = 0U;
  ka025vg_te_frame = 0U;
  ka025vg_te_count = 0U;

  /* power_init() */
  HAL_GPIO_WritePin(EN_3V3_GPIO_Port, EN_3V3_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(EN_1V8_GPIO_Port, EN_1V8_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(EN_IOVCC_GPIO_Port, EN_IOVCC_Pin, GPIO_PIN_SET);
  HAL_Delay(1U);

  /* oled_QSPI_reset() */
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(200U);

  if (KA025VG_QspiInstructAndWait(cd80, 5U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd91, 5U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd89, 8U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd8A, 8U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd96, 6U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd97, 6U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd95, 6U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd51, 6U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd35, 5U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cd44, 5U, 10U) != HAL_OK) { return HAL_ERROR; }

  /* 亮度调节 */
  if (KA025VG_QspiInstructAndWait(cdA8, 5U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cdAA, 5U, 10U) != HAL_OK) { return HAL_ERROR; }
  if (KA025VG_QspiInstructAndWait(cdA9, 5U, 10U) != HAL_OK) { return HAL_ERROR; }

  /* 设置 elvdd 为高，拉高电平 */
  HAL_GPIO_WritePin(EN_1V2_GPIO_Port, EN_1V2_Pin, GPIO_PIN_SET);
  HAL_Delay(100U);

  if (KA025VG_QspiInstructAndWait(cd11, 5U, 100U) != HAL_OK) { return HAL_ERROR; }

  /* 与另一个工程一致：等第2个TE后再Display ON。 */
  while (ka025vg_te_cmd == 0U)
  {
  }

  if (KA025VG_QspiInstructAndWait(cd29, 5U, 0U) != HAL_OK) { return HAL_ERROR; }

  /* 设置 elvss 为 -6V，拉高电平 */
  HAL_GPIO_WritePin(EN_ELVSS_GPIO_Port, EN_ELVSS_Pin, GPIO_PIN_SET);

  if (KA025VG_QspiInstructAndWait(cd36, 8U, 10U) != HAL_OK) { return HAL_ERROR; }

  return HAL_OK;
}

HAL_StatusTypeDef KA025VG_DrawGray8(const uint8_t *pixels, size_t pixel_count)
{
  QSPI_CommandTypeDef command = {0};

  if ((ka025vg_qspi == NULL) || (pixels == NULL) || (pixel_count == 0U) || (pixel_count > KA025VG_FRAME_BYTES))
  {
    return HAL_ERROR;
  }

  if (pixel_count == KA025VG_FRAME_BYTES)
  {
    /*
     * 与参考工程 OLED/QSPI/qspi.c::QSPI_send_image() 逐行完全一致：
     *   teflag=0 → while(teflag==0){}  → teflag=0
     *   → qspi_send_message(row[0])    → while(tx_done==0){} → tx_done=0
     *   → QSPI_delay(200)              → 循环 row[1..479] + QSPI_delay(70)
     *
     * 关键点：
     *   1. 等 TE 信号，不等盲目的 HAL_Delay
     *   2. QSPI_delay 是非 volatile 空循环 (-O3 下被优化掉)
     *   3. 忙等不用 HAL_GetTick，纯 while(tx_done==0)
     *   4. tx_done 在忙等之后才清零
     */
    ka025vg_te_frame = 0U;
    while (ka025vg_te_frame == 0U) {}
    ka025vg_te_frame = 0U;

    /* 预先构造命令结构体，行间复用，不改动 */
    command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
    command.Instruction       = KA025VG_QSPI_WRITE_IMAGE;
    command.AddressMode       = QSPI_ADDRESS_NONE;
    command.AddressSize       = QSPI_ADDRESS_24_BITS;
    command.Address           = 0U;
    command.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
    command.AlternateBytes    = KA025VG_IMAGE_ALT_BYTES;
    command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_24_BITS;
    command.DataMode          = QSPI_DATA_4_LINES;
    command.DummyCycles       = 0U;
    command.NbData            = KA025VG_WIDTH;
    command.DdrMode           = QSPI_DDR_MODE_DISABLE;
    command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
    command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

    for (uint32_t row = 0U; row < KA025VG_HEIGHT; row++)
    {
      /* 与 qspi_send_message 一致：HAL_QSPI_Command + HAL_QSPI_Transmit_DMA，
         不在本函数内等待 tx_done */
      if (HAL_QSPI_Command(ka025vg_qspi, &command, KA025VG_QSPI_TIMEOUT_MS) != HAL_OK)
      {
        return HAL_ERROR;
      }
      if (HAL_QSPI_Transmit_DMA(ka025vg_qspi, (uint8_t *)&pixels[row * KA025VG_WIDTH]) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* 与 QSPI_send_image 一致：纯 while(tx_done==0)，不用 HAL_GetTick */
      while (ka025vg_qspi_tx_done == 0U) {}
      ka025vg_qspi_tx_done = 0U;

      /* 与 QSPI_delay 一致：非 volatile 空循环，-O3 下编译器删掉 */
      {
        uint16_t d;
        uint16_t loops = (row == 0U) ? 200U : 70U;
        for (d = 0U; d < loops; ++d) {}
      }
    }
    return HAL_OK;
  }

  /* 单行发送 */
  command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  command.Instruction       = KA025VG_QSPI_WRITE_IMAGE;
  command.AddressMode       = QSPI_ADDRESS_NONE;
  command.AddressSize       = QSPI_ADDRESS_24_BITS;
  command.Address           = 0U;
  command.AlternateByteMode = QSPI_ALTERNATE_BYTES_1_LINE;
  command.AlternateBytes    = KA025VG_IMAGE_ALT_BYTES;
  command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_24_BITS;
  command.DataMode          = QSPI_DATA_4_LINES;
  command.DummyCycles       = 0U;
  command.NbData            = (uint32_t)pixel_count;
  command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(ka025vg_qspi, &command, KA025VG_QSPI_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return KA025VG_QspiTransmitDma((uint8_t *)pixels);
}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi)
{
  if (hqspi == ka025vg_qspi)
  {
    ka025vg_qspi_tx_done = 1U;
  }
}

HAL_StatusTypeDef KA025VG_Fill(uint8_t gray)
{
  (void)memset(ka025vg_framebuffer, gray, sizeof(ka025vg_framebuffer));
  return KA025VG_DrawGray8(ka025vg_framebuffer, sizeof(ka025vg_framebuffer));
}

#define REF_CUSTOM_DISPLAY_DIGIT_COUNT 27U

#define REF_SYM_DIR_E             1U
#define REF_SYM_DIR_S             2U
#define REF_SYM_DIR_W             3U
#define REF_SYM_DIR_N             4U
#define REF_SYM_DIR_NE_E          5U
#define REF_SYM_BATTERY_1         6U
#define REF_SYM_BATTERY_2         7U
#define REF_SYM_BATTERY_3         8U
#define REF_SYM_BATTERY_4         9U
#define REF_SYM_BATTERY_FRAME     10U
#define REF_SYM_UNIT_M            11U
#define REF_SYM_RANGE_SINGLE      12U
#define REF_SYM_RANGE_CONTINUOUS  13U
#define REF_SYM_RANGE_FIRST_F     14U
#define REF_SYM_RANGE_LAST_E      15U
#define REF_SYM_RETICLE           16U
#define REF_SYM_AZIMUTH_DEG       17U
#define REF_SYM_PITCH_LABEL_P     18U
#define REF_SYM_PITCH_DEG         19U
#define REF_SYM_PITCH_SIGN_MINUS  20U
#define REF_SYM_PITCH_SIGN_PLUS   21U
#define REF_SYM_LON_W             22U
#define REF_SYM_LON_E             23U
#define REF_SYM_LAT_S             24U
#define REF_SYM_LAT_N             25U
#define REF_SYM_COORD_LOCAL       26U
#define REF_SYM_COORD_TARGET      27U
#define REF_SYM_COORD_FIRST_F     28U
#define REF_SYM_COORD_LAST_E      29U
#define REF_SYM_COORD_DEG         30U
#define REF_SYM_COORD_MIN         31U
#define REF_SYM_COORD_SEC         32U
#define REF_SYM_COORD_DOT         33U
#define REF_SYM_ELEVATION_LABEL_H 34U
#define REF_SYM_ELEVATION_UNIT_M  35U

typedef struct
{
  int8_t digits[REF_CUSTOM_DISPLAY_DIGIT_COUNT];
  uint64_t symbols;
  uint8_t all_on;
  uint8_t brightness;
} RefDisplayState;

static uint8_t REF_SymbolOn(const RefDisplayState *state, uint8_t symbol)
{
  if ((state == NULL) || (symbol == 0U) || (symbol > 63U))
  {
    return 0U;
  }

  return (uint8_t)((state->all_on != 0U) || ((state->symbols & (1ULL << (symbol - 1U))) != 0ULL));
}

static char REF_DigitToChar(const RefDisplayState *state, uint8_t digit_id)
{
  int8_t value;

  if ((state == NULL) || (digit_id == 0U) || (digit_id > REF_CUSTOM_DISPLAY_DIGIT_COUNT))
  {
    return ' ';
  }

  if (state->all_on != 0U)
  {
    return '8';
  }

  value = state->digits[digit_id - 1U];
  if ((value >= 0) && (value <= 9))
  {
    return (char)('0' + value);
  }
  if (value == -2)
  {
    return '-';
  }

  return ' ';
}

static uint8_t REF_PixelBril(const RefDisplayState *state)
{
  static const uint8_t levels[] = {0x20U, 0x40U, 0x70U, 0xA0U, 0xFFU};
  uint8_t level = 4U;

  if ((state != NULL) && (state->brightness > 0U) && (state->brightness <= 5U))
  {
    level = (uint8_t)(state->brightness - 1U);
  }

  return levels[level];
}

static void REF_DrawRectangle(uint16_t addr_x, uint16_t addr_y, uint16_t width, uint16_t height, uint8_t bril)
{
  if ((((uint32_t)addr_x + width) >= KA025VG_HEIGHT) || (((uint32_t)addr_y + height) >= KA025VG_WIDTH))
  {
    return;
  }

  for (uint16_t x = addr_x; x < (uint16_t)(addr_x + width); x++)
  {
    (void)memset(&ka025vg_framebuffer[((uint32_t)x * KA025VG_WIDTH) + addr_y], bril, height);
  }
}

static void REF_DrawBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t bril)
{
  REF_DrawRectangle(x, y, h, w, bril);
}

static void REF_GlyphRows(char ch, uint8_t rows[7])
{
  static const uint8_t blank[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
  static const uint8_t dash[7]  = {0U, 0U, 0U, 31U, 0U, 0U, 0U};
  const uint8_t *src = blank;

  switch (ch)
  {
  case '0': { static const uint8_t r[7] = {14U, 17U, 19U, 21U, 25U, 17U, 14U}; src = r; break; }
  case '1': { static const uint8_t r[7] = {4U, 12U, 4U, 4U, 4U, 4U, 14U}; src = r; break; }
  case '2': { static const uint8_t r[7] = {14U, 17U, 1U, 2U, 4U, 8U, 31U}; src = r; break; }
  case '3': { static const uint8_t r[7] = {30U, 1U, 1U, 14U, 1U, 1U, 30U}; src = r; break; }
  case '4': { static const uint8_t r[7] = {2U, 6U, 10U, 18U, 31U, 2U, 2U}; src = r; break; }
  case '5': { static const uint8_t r[7] = {31U, 16U, 16U, 30U, 1U, 1U, 30U}; src = r; break; }
  case '6': { static const uint8_t r[7] = {14U, 16U, 16U, 30U, 17U, 17U, 14U}; src = r; break; }
  case '7': { static const uint8_t r[7] = {31U, 1U, 2U, 4U, 8U, 8U, 8U}; src = r; break; }
  case '8': { static const uint8_t r[7] = {14U, 17U, 17U, 14U, 17U, 17U, 14U}; src = r; break; }
  case '9': { static const uint8_t r[7] = {14U, 17U, 17U, 15U, 1U, 1U, 14U}; src = r; break; }
  case 'A': { static const uint8_t r[7] = {14U, 17U, 17U, 31U, 17U, 17U, 17U}; src = r; break; }
  case 'C': { static const uint8_t r[7] = {14U, 17U, 16U, 16U, 16U, 17U, 14U}; src = r; break; }
  case 'E': { static const uint8_t r[7] = {31U, 16U, 16U, 30U, 16U, 16U, 31U}; src = r; break; }
  case 'F': { static const uint8_t r[7] = {31U, 16U, 16U, 30U, 16U, 16U, 16U}; src = r; break; }
  case 'H': { static const uint8_t r[7] = {17U, 17U, 17U, 31U, 17U, 17U, 17U}; src = r; break; }
  case 'L': { static const uint8_t r[7] = {16U, 16U, 16U, 16U, 16U, 16U, 31U}; src = r; break; }
  case 'M': { static const uint8_t r[7] = {17U, 27U, 21U, 21U, 17U, 17U, 17U}; src = r; break; }
  case 'N': { static const uint8_t r[7] = {17U, 25U, 21U, 19U, 17U, 17U, 17U}; src = r; break; }
  case 'P': { static const uint8_t r[7] = {30U, 17U, 17U, 30U, 16U, 16U, 16U}; src = r; break; }
  case 'S': { static const uint8_t r[7] = {15U, 16U, 16U, 14U, 1U, 1U, 30U}; src = r; break; }
  case 'T': { static const uint8_t r[7] = {31U, 4U, 4U, 4U, 4U, 4U, 4U}; src = r; break; }
  case 'W': { static const uint8_t r[7] = {17U, 17U, 17U, 21U, 21U, 21U, 10U}; src = r; break; }
  case '+': { static const uint8_t r[7] = {0U, 4U, 4U, 31U, 4U, 4U, 0U}; src = r; break; }
  case '-': src = dash; break;
  case '.': { static const uint8_t r[7] = {0U, 0U, 0U, 0U, 0U, 12U, 12U}; src = r; break; }
  case '\'': { static const uint8_t r[7] = {12U, 12U, 8U, 0U, 0U, 0U, 0U}; src = r; break; }
  case '"': { static const uint8_t r[7] = {10U, 10U, 10U, 0U, 0U, 0U, 0U}; src = r; break; }
  default: src = blank; break;
  }

  (void)memcpy(rows, src, 7U);
}

static void REF_DrawChar(uint16_t x, uint16_t y, char ch, uint8_t scale, uint8_t bril)
{
  uint8_t rows[7];

  REF_GlyphRows(ch, rows);
  for (uint8_t row = 0U; row < 7U; ++row)
  {
    for (uint8_t col = 0U; col < 5U; ++col)
    {
      if ((rows[row] & (uint8_t)(1U << (4U - col))) != 0U)
      {
        REF_DrawRectangle((uint16_t)(x + (row * scale)),
                          (uint16_t)(y + (col * scale)),
                          scale,
                          scale,
                          bril);
      }
    }
  }
}

static void REF_DrawText(uint16_t x, uint16_t y, const char *text, uint8_t scale, uint8_t bril)
{
  while ((text != NULL) && (*text != '\0'))
  {
    REF_DrawChar(x, y, *text, scale, bril);
    y = (uint16_t)(y + (6U * scale));
    ++text;
  }
}

static void REF_DrawDigitIds(const RefDisplayState *state,
                             const uint8_t *ids,
                             uint8_t count,
                             uint16_t x,
                             uint16_t y,
                             uint8_t scale,
                             uint8_t bril)
{
  for (uint8_t i = 0U; i < count; ++i)
  {
    REF_DrawChar(x, (uint16_t)(y + (i * 6U * scale)), REF_DigitToChar(state, ids[i]), scale, bril);
  }
}

static void REF_DrawReticle(uint8_t bril)
{
  /* 中心十字准星：屏幕中心 (row 239, col 319) */
  REF_DrawBar(238U, 240U, 160U, 3U, bril);   /* 水平线 */
  REF_DrawBar(160U, 318U, 3U, 160U, bril);   /* 垂直线 */
  REF_DrawBar(233U, 313U, 14U, 14U, bril);   /* 中心圆点外框 */
  REF_DrawBar(236U, 316U, 8U, 8U, 0U);       /* 中心镂空 */
}

static void REF_DrawBattery(const RefDisplayState *state, uint8_t bril)
{
  /* 右上角电池框：横向放置，位于方位角读数右侧 */
  const uint16_t x = 78U;   /* row */
  const uint16_t y = 470U;  /* col */
  uint8_t bars = 0U;

  if (REF_SymbolOn(state, REF_SYM_BATTERY_FRAME) == 0U)
  {
    return;
  }

  /* 电池外框 (宽 70 col x 高 26 row) */
  REF_DrawBar(x, y, 70U, 3U, bril);                       /* 上边 */
  REF_DrawBar((uint16_t)(x + 23U), y, 70U, 3U, bril);     /* 下边 */
  REF_DrawBar(x, y, 3U, 26U, bril);                       /* 左边 */
  REF_DrawBar(x, (uint16_t)(y + 67U), 3U, 26U, bril);     /* 右边 */
  REF_DrawBar((uint16_t)(x + 7U), (uint16_t)(y + 70U), 12U, 5U, bril); /* 正极头 */

  bars += REF_SymbolOn(state, REF_SYM_BATTERY_1) ? 1U : 0U;
  bars += REF_SymbolOn(state, REF_SYM_BATTERY_2) ? 1U : 0U;
  bars += REF_SymbolOn(state, REF_SYM_BATTERY_3) ? 1U : 0U;
  bars += REF_SymbolOn(state, REF_SYM_BATTERY_4) ? 1U : 0U;

  for (uint8_t i = 0U; i < bars; ++i)
  {
    REF_DrawBar((uint16_t)(x + 6U), (uint16_t)(y + 6U + (i * 15U)), 11U, 14U, bril);
  }
}

static void REF_DrawDirection(const RefDisplayState *state, uint8_t bril)
{
  /* 顶部方向标 E S W N E 横向排列一行，居中于屏幕顶部 */
  const uint16_t row = 45U;
  if (REF_SymbolOn(state, REF_SYM_DIR_E))    { REF_DrawText(row, 284U, "E", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_DIR_S))    { REF_DrawText(row, 300U, "S", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_DIR_W))    { REF_DrawText(row, 316U, "W", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_DIR_N))    { REF_DrawText(row, 332U, "N", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_DIR_NE_E)) { REF_DrawText(row, 348U, "E", 2U, bril); }
}

static void REF_DrawRange(const RefDisplayState *state, uint8_t bril)
{
  static const uint8_t range_ids[] = {4U, 5U, 6U, 7U};

  /* 距离：屏幕中部偏上的大字号 4 位数字 + M 单位 */
  REF_DrawDigitIds(state, range_ids, 4U, 150U, 250U, 5U, bril);
  if (REF_SymbolOn(state, REF_SYM_UNIT_M)) { REF_DrawText(160U, 380U, "M", 3U, bril); }
  /* 首/末次回波标 F/E：距离数字左上方 */
  if (REF_SymbolOn(state, REF_SYM_RANGE_FIRST_F)) { REF_DrawText(150U, 218U, "F", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_RANGE_LAST_E)) { REF_DrawText(150U, 218U, "E", 3U, bril); }
  /* 单次/连续测距标 S/C：距离数字右侧 */
  if (REF_SymbolOn(state, REF_SYM_RANGE_SINGLE)) { REF_DrawText(150U, 410U, "S", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_RANGE_CONTINUOUS)) { REF_DrawText(174U, 410U, "C", 2U, bril); }
}

static void REF_DrawOrientation(const RefDisplayState *state, uint8_t bril)
{
  static const uint8_t azimuth_ids[] = {1U, 2U, 3U};
  static const uint8_t pitch_ids[] = {26U, 27U};

  /* 方位角：顶部方向标下方，3 位数字 + 度点，居中 */
  REF_DrawDigitIds(state, azimuth_ids, 3U, 78U, 286U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_AZIMUTH_DEG)) { REF_DrawChar(78U, 340U, '.', 2U, bril); }

  /* 俯仰角：屏幕左侧，P 标签 + 符号 + 2 位数字 + 度点 */
  if (REF_SymbolOn(state, REF_SYM_PITCH_LABEL_P)) { REF_DrawText(232U, 60U, "P", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_PITCH_SIGN_MINUS)) { REF_DrawText(232U, 80U, "-", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_PITCH_SIGN_PLUS)) { REF_DrawText(232U, 80U, "+", 3U, bril); }
  REF_DrawDigitIds(state, pitch_ids, 2U, 232U, 100U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_PITCH_DEG)) { REF_DrawChar(232U, 138U, '.', 2U, bril); }
}

static void REF_DrawCoordinate(const RefDisplayState *state, uint8_t bril)
{
  static const uint8_t degree_ids[] = {8U, 9U, 10U};
  static const uint8_t minute_ids[] = {11U, 12U};
  static const uint8_t fraction_ids[] = {13U, 14U, 15U, 16U};
  static const uint8_t altitude_ids[] = {17U, 18U, 19U, 20U};
  static const uint8_t count_ids[] = {21U, 22U, 23U, 24U, 25U};

  /* 坐标行：本机/目标标 + 首末标 + 经纬方向 + 度分秒，位于准星下方 */
  const uint16_t coord_row = 320U;
  if (REF_SymbolOn(state, REF_SYM_COORD_LOCAL)) { REF_DrawText(coord_row, 132U, "L", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_COORD_TARGET)) { REF_DrawText(coord_row, 132U, "T", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_COORD_FIRST_F)) { REF_DrawText(coord_row, 158U, "F", 2U, bril); }
  if (REF_SymbolOn(state, REF_SYM_COORD_LAST_E)) { REF_DrawText(coord_row, 158U, "E", 2U, bril); }

  if (REF_SymbolOn(state, REF_SYM_LAT_N)) { REF_DrawText(coord_row, 178U, "N", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_LAT_S)) { REF_DrawText(coord_row, 178U, "S", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_LON_E)) { REF_DrawText(coord_row, 178U, "E", 3U, bril); }
  if (REF_SymbolOn(state, REF_SYM_LON_W)) { REF_DrawText(coord_row, 178U, "W", 3U, bril); }

  REF_DrawDigitIds(state, degree_ids, 3U, coord_row, 206U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_COORD_DEG)) { REF_DrawChar(coord_row, 260U, '.', 2U, bril); }
  REF_DrawDigitIds(state, minute_ids, 2U, coord_row, 278U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_COORD_MIN)) { REF_DrawChar(coord_row, 314U, '\'', 2U, bril); }
  REF_DrawDigitIds(state, fraction_ids, 4U, coord_row, 330U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_COORD_SEC)) { REF_DrawChar(coord_row, 402U, '"', 2U, bril); }

  /* 高程行 + 测量计数：坐标行下方 */
  const uint16_t elev_row = 388U;
  if (REF_SymbolOn(state, REF_SYM_ELEVATION_LABEL_H)) { REF_DrawText(elev_row, 188U, "H", 3U, bril); }
  REF_DrawDigitIds(state, altitude_ids, 4U, elev_row, 222U, 3U, bril);
  if (REF_SymbolOn(state, REF_SYM_ELEVATION_UNIT_M)) { REF_DrawText(elev_row, 294U, "M", 2U, bril); }

  REF_DrawDigitIds(state, count_ids, 5U, elev_row, 360U, 2U, bril);
}

static void REF_Render(const RefDisplayState *state)
{
  uint8_t bril = REF_PixelBril(state);

  if (state == NULL)
  {
    return;
  }

  (void)memset(ka025vg_framebuffer, 0x00, sizeof(ka025vg_framebuffer));
  if (REF_SymbolOn(state, REF_SYM_RETICLE)) { REF_DrawReticle(bril); }
  REF_DrawBattery(state, bril);
  REF_DrawDirection(state, bril);
  REF_DrawRange(state, bril);
  REF_DrawOrientation(state, bril);
  REF_DrawCoordinate(state, bril);
}

static void REF_ShowBootScreen(void)
{
  RefDisplayState state;

  /* 全亮测试画面：点亮所有数字段(显示 8)与全部标记，对应参考图全字符界面 */
  (void)memset(&state, 0, sizeof(state));
  state.brightness = 5U;
  state.all_on = 1U;

  REF_Render(&state);
}

HAL_StatusTypeDef KA025VG_ShowFixedScreen(void)
{
  REF_ShowBootScreen();

  return KA025VG_DrawGray8(ka025vg_framebuffer, sizeof(ka025vg_framebuffer));
}

static RefDisplayState g_current_display_state;

void KA025VG_UpdateState(const uint8_t *digits, uint64_t symbols, uint8_t all_on, uint8_t brightness)
{
  if (digits != NULL)
  {
    for (uint8_t i = 0U; i < REF_CUSTOM_DISPLAY_DIGIT_COUNT; ++i)
    {
      /* 协议层中，1-10 映射为 0-9 数字，11 映射为 -2（负号/破折号），0 映射为 -1（不显示） */
      uint8_t val = digits[i];
      if ((val >= 1U) && (val <= 10U))
      {
        g_current_display_state.digits[i] = (int8_t)(val - 1U);
      }
      else if (val == 11U)
      {
        g_current_display_state.digits[i] = -2;
      }
      else
      {
        g_current_display_state.digits[i] = -1;
      }
    }
  }
  g_current_display_state.symbols = symbols;
  g_current_display_state.all_on = all_on;
  g_current_display_state.brightness = brightness;
}

void KA025VG_RenderAndShow(void)
{
  REF_Render(&g_current_display_state);
  (void)KA025VG_DrawGray8(ka025vg_framebuffer, sizeof(ka025vg_framebuffer));
}

