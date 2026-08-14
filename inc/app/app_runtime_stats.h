#ifndef APP_RUNTIME_STATS_H
#define APP_RUNTIME_STATS_H

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

void AppRunTimeStats_TimerInit(void);
uint32_t AppRunTimeStats_GetCounter(void);

#endif /* APP_RUNTIME_STATS_H */
