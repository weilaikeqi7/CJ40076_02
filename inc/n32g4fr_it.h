#ifndef N32G4FR_IT_H
#define N32G4FR_IT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g4fr.h"

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* N32G4FR_IT_H */
