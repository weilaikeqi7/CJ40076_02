#include "app_runtime_stats.h"

#include "app_log.h"
#include "n32g4fr.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_RUNTIME_STATS_TASK_STACK_WORDS (configMINIMAL_STACK_SIZE * 2U)
#define APP_RUNTIME_STATS_TASK_PRIORITY    (tskIDLE_PRIORITY + 1U)
#define APP_RUNTIME_STATS_PERIOD_MS        5000U
#define APP_RUNTIME_STATS_MAX_TASKS        12U

typedef struct
{
    UBaseType_t task_number;
    configRUN_TIME_COUNTER_TYPE run_time;
    bool valid;
} AppRunTimeStatsSample;

static TaskStatus_t s_task_status[APP_RUNTIME_STATS_MAX_TASKS];
static AppRunTimeStatsSample s_last_samples[APP_RUNTIME_STATS_MAX_TASKS];
static uint32_t s_cycles_per_us = 1U;

void AppRunTimeStats_TimerInit(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;

    if (cycles_per_us == 0U)
    {
        cycles_per_us = 1U;
    }

    s_cycles_per_us = cycles_per_us;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t AppRunTimeStats_GetCounter(void)
{
    return DWT->CYCCNT / s_cycles_per_us;
}

static char task_state_char(eTaskState state)
{
    switch (state)
    {
    case eRunning:
        return 'R';
    case eReady:
        return 'Y';
    case eBlocked:
        return 'B';
    case eSuspended:
        return 'S';
    case eDeleted:
        return 'D';
    default:
        return '?';
    }
}

static AppRunTimeStatsSample* find_last_sample(UBaseType_t task_number)
{
    for (uint32_t i = 0U; i < APP_RUNTIME_STATS_MAX_TASKS; ++i)
    {
        if (s_last_samples[i].valid && (s_last_samples[i].task_number == task_number))
        {
            return &s_last_samples[i];
        }
    }

    return NULL;
}

static void save_samples(const TaskStatus_t* tasks, UBaseType_t task_count)
{
    for (uint32_t i = 0U; i < APP_RUNTIME_STATS_MAX_TASKS; ++i)
    {
        s_last_samples[i].valid = false;
    }

    for (UBaseType_t i = 0U; (i < task_count) && (i < APP_RUNTIME_STATS_MAX_TASKS); ++i)
    {
        s_last_samples[i].task_number = tasks[i].xTaskNumber;
        s_last_samples[i].run_time = tasks[i].ulRunTimeCounter;
        s_last_samples[i].valid = true;
    }
}

static void print_runtime_stats(bool have_last_sample)
{
    configRUN_TIME_COUNTER_TYPE total_run_time = 0U;
    uint32_t total_delta = 0U;
    UBaseType_t task_count = uxTaskGetSystemState(s_task_status,
                                                  APP_RUNTIME_STATS_MAX_TASKS,
                                                  &total_run_time);

    (void)total_run_time;

    if (task_count == 0U)
    {
        APP_LOGW("stats", "task array too small, max=%u", (unsigned int)APP_RUNTIME_STATS_MAX_TASKS);
        return;
    }

    for (UBaseType_t i = 0U; i < task_count; ++i)
    {
        AppRunTimeStatsSample* last = find_last_sample(s_task_status[i].xTaskNumber);

        if (have_last_sample && (last != NULL))
        {
            total_delta += (uint32_t)(s_task_status[i].ulRunTimeCounter - last->run_time);
        }
    }

    if (!have_last_sample || (total_delta == 0U))
    {
        APP_LOGI("stats", "runtime stats baseline tasks=%u", (unsigned int)task_count);
        save_samples(s_task_status, task_count);
        return;
    }

    APP_LOGI("stats", "task cpu over %ums", (unsigned int)APP_RUNTIME_STATS_PERIOD_MS);

    for (UBaseType_t i = 0U; i < task_count; ++i)
    {
        const TaskStatus_t* task = &s_task_status[i];
        const AppRunTimeStatsSample* last = find_last_sample(task->xTaskNumber);
        uint32_t delta = 0U;
        uint32_t cpu_x10 = 0U;

        if (last != NULL)
        {
            delta = (uint32_t)(task->ulRunTimeCounter - last->run_time);
            cpu_x10 = (uint32_t)(((uint64_t)delta * 1000ULL) / total_delta);
        }

        APP_LOGI("stats",
                 "%s cpu=%u.%u%% stack=%u prio=%u state=%c",
                 (task->pcTaskName != NULL) ? task->pcTaskName : "-",
                 (unsigned int)(cpu_x10 / 10U),
                 (unsigned int)(cpu_x10 % 10U),
                 (unsigned int)task->usStackHighWaterMark,
                 (unsigned int)task->uxCurrentPriority,
                 task_state_char(task->eCurrentState));
    }

    save_samples(s_task_status, task_count);
}

static void runtime_stats_task(void* argument)
{
    bool have_last_sample = false;

    (void)argument;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(APP_RUNTIME_STATS_PERIOD_MS));
        print_runtime_stats(have_last_sample);
        have_last_sample = true;
    }
}

BaseType_t AppRunTimeStats_Start(void)
{
    return xTaskCreate(runtime_stats_task,
                       "stats",
                       APP_RUNTIME_STATS_TASK_STACK_WORDS,
                       NULL,
                       APP_RUNTIME_STATS_TASK_PRIORITY,
                       NULL);
}
