#include "FreeRTOS.h"

#if (configAPPLICATION_ALLOCATED_HEAP == 1)
uint8_t ucHeap[configTOTAL_HEAP_SIZE] __attribute__((section(".freertos_heap"), aligned(8)));
#endif
