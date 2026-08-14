#include "app_log.h"

#include "FreeRTOS.h"
#include "SEGGER_RTT.h"
#include "n32g4fr.h"
#include "task.h"

#include <stdarg.h>
#include <stdint.h>

static AppLogLevel s_log_level = APP_LOG_LEVEL_INFO;

static const char* const s_level_names[] = {
    "D",
    "I",
    "W",
    "E",
};

static uint32_t app_log_now_ms(void)
{
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U)
    {
        return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return 0U;
    }

    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static const char* app_log_task_name(void)
{
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U)
    {
        return "isr";
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return "boot";
    }

    return pcTaskGetName(NULL);
}

void AppLog_SetLevel(AppLogLevel level)
{
    s_log_level = level;
}

AppLogLevel AppLog_GetLevel(void)
{
    return s_log_level;
}

void AppLog_Print(AppLogLevel level, const char* module, const char* fmt, ...)
{
    va_list args;
    const char* safe_module = (module != NULL) ? module : "-";
    const char* task_name   = app_log_task_name();

    if ((level < s_log_level) || (level >= APP_LOG_LEVEL_OFF))
    {
        return;
    }

    if (task_name == NULL)
    {
        task_name = "-";
    }

    SEGGER_RTT_printf(
        0,
        "[%08u][%s][%s][%s] ",
        app_log_now_ms(),
        s_level_names[level],
        task_name,
        safe_module);

    va_start(args, fmt);
    SEGGER_RTT_vprintf(0, fmt, &args);
    va_end(args);

    SEGGER_RTT_WriteString(0, "\r\n");
}
