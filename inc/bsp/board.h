#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>

void Board_Init(void);
void Board_PowerHold(bool enabled);
void Board_SetRangePower(bool enabled);
void Board_SetGnssPower(bool enabled);
void Board_SetImuPower(bool enabled);
bool Board_ReadPowerKey(void);
bool Board_ReadModeKey(void);

#endif /* BOARD_H */
