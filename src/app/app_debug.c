#include "app_debug.h"

#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "app_config.h"
#include "cm_backtrace.h"
#include "task.h"

#include <stdarg.h>
#include <stdint.h>

void AppDebug_Init(void)
{
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(0, "Terminal", 0, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    SEGGER_RTT_WriteString(0, "\r\n");
    cm_backtrace_init(APP_FIRMWARE_NAME, APP_HARDWARE_VER, APP_SOFTWARE_VER);
    cm_backtrace_firmware_info();
}

void AppDebug_Printf(const char* fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);
}

void AppDebug_AssertFailed(const char* file, int line)
{
    SEGGER_RTT_printf(0, "assert failed: %s:%d\r\n", file, line);
    cm_backtrace_assert(cmb_get_sp());
}

uint32_t* vTaskStackAddr(void)
{
    TaskStatus_t task_status;

    vTaskGetInfo(NULL, &task_status, pdFALSE, eInvalid);
    return (uint32_t*)task_status.pxStackBase;
}

uint32_t vTaskStackSize(void)
{
    TaskStatus_t task_status;

    vTaskGetInfo(NULL, &task_status, pdFALSE, eInvalid);
    return (uint32_t)(task_status.pxEndOfStack - task_status.pxStackBase + 1);
}

char* vTaskName(void)
{
    return pcTaskGetName(NULL);
}
