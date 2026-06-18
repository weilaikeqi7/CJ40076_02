#ifndef KA025VG_OLED_H
#define KA025VG_OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <stdint.h>

#define KA025VG_WIDTH   640U
#define KA025VG_HEIGHT  480U

HAL_StatusTypeDef KA025VG_Init(QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef KA025VG_WriteCommand(uint8_t command, const uint8_t *params, uint32_t length);
HAL_StatusTypeDef KA025VG_Fill(uint8_t gray);
HAL_StatusTypeDef KA025VG_DrawGray8(const uint8_t *pixels, size_t pixel_count);
HAL_StatusTypeDef KA025VG_ShowFixedScreen(void);
void KA025VG_TeInterruptHandler(void);
void KA025VG_UpdateState(const uint8_t *digits, uint64_t symbols, uint8_t all_on, uint8_t brightness);
void KA025VG_RenderAndShow(void);

#ifdef __cplusplus
}
#endif

#endif /* KA025VG_OLED_H */
