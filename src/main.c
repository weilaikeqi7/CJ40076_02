#include "main.h"

#include "app_debug.h"
#include "app_log.h"
#include "app_tasks.h"
#include "board.h"
#include "build_info.h"
#include "task.h"

#include <stdint.h>

#if defined(N32_EXPECT_FPU) && (N32_EXPECT_FPU == 1) && !defined(__clang__)
#if (__FPU_USED != 1)
#error "FPU was requested, but CMSIS reports __FPU_USED != 1. Check -mfpu and -mfloat-abi."
#endif
#endif

extern uint8_t _sfreertos_heap[];
extern uint8_t _efreertos_heap[];

static void freertos_heap_region_init(void)
{
    volatile uint8_t* heap = _sfreertos_heap;

    while (heap < _efreertos_heap)
    {
        *heap++ = 0U;
    }

    __DSB();
}

int main(void)
{
    BaseType_t tasks_created;

    freertos_heap_region_init();
    AppDebug_Init();
    APP_LOGI("main", "boot");
    APP_LOGI("main", "build: %s", APP_BUILD_TIMESTAMP);

    Board_Init();
    tasks_created = AppTasks_Start();
    if (tasks_created != pdPASS)
    {
        APP_LOGE("main", "failed to create app tasks");
        Error_Handler();
    }

    APP_LOGI("main", "starting scheduler");
    vTaskStartScheduler();
    APP_LOGE("main", "scheduler returned");
    Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

void AppAssertFailed(const char* file, int line)
{
    AppDebug_AssertFailed(file, line);
    Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
    APP_LOGE("rtos", "malloc failed");
    Error_Handler();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* task_name)
{
    (void)task;
    APP_LOGE("rtos", "stack overflow: %s", (task_name != NULL) ? task_name : "-");
    Error_Handler();
}

#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    (void)expr;
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
