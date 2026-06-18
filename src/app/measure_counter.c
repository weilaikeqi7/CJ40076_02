#include "measure_counter.h"

#include "n32g4fr_flash.h"

#define MEASURE_COUNTER_FLASH_ADDR  (0x08000000UL + (512UL * 1024UL) - 0x800UL)
#define MEASURE_COUNTER_MAGIC       0x434A0076UL

typedef struct
{
    uint32_t count;
    uint32_t magic;
} MeasureCounterRecord;

static MeasureCounterRecord g_record;

static void save_record(void)
{
    FLASH_Unlock();
    if (FLASH_COMPL == FLASH_EraseOnePage(MEASURE_COUNTER_FLASH_ADDR))
    {
        (void)FLASH_ProgramWord(MEASURE_COUNTER_FLASH_ADDR, g_record.count);
        (void)FLASH_ProgramWord(MEASURE_COUNTER_FLASH_ADDR + 4UL, g_record.magic);
    }
    FLASH_Lock();
}

void MeasureCounter_Init(void)
{
    const MeasureCounterRecord* stored = (const MeasureCounterRecord*)MEASURE_COUNTER_FLASH_ADDR;

    g_record = *stored;
    if (g_record.magic != MEASURE_COUNTER_MAGIC)
    {
        g_record.count = 0U;
        g_record.magic = MEASURE_COUNTER_MAGIC;
        save_record();
    }
}

uint32_t MeasureCounter_Get(void)
{
    return g_record.count;
}

uint32_t MeasureCounter_Increment(void)
{
    if (g_record.count < 99999U)
    {
        ++g_record.count;
    }
    save_record();
    return g_record.count;
}

void MeasureCounter_Reset(void)
{
    g_record.count = 0U;
    g_record.magic = MEASURE_COUNTER_MAGIC;
    save_record();
}
