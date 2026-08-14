#include "measure_counter.h"

#include "n32g4fr_flash.h"

#include <stdbool.h>

#define MEASURE_COUNTER_FLASH_ADDR        (0x08000000UL + (512UL * 1024UL) - 0x800UL)
#define MEASURE_COUNTER_LEGACY_MAGIC      0x434A0076UL
#define MEASURE_COUNTER_PREVIOUS_MAGIC_V1 0x434A0077UL
#define MEASURE_COUNTER_PREVIOUS_MAGIC_V2 0x434A0078UL
#define MEASURE_COUNTER_MAGIC             0x434A0079UL
#define MEASURE_COUNTER_CHECK_SEED        0xA55A5AA5UL

#define PITCH_INSTALL_MIN_TENTH_DEG     (-900)
#define PITCH_INSTALL_MAX_TENTH_DEG       900
#define YAW_OFFSET_MIN_TENTH_DEG       (-1800)
#define YAW_OFFSET_MAX_TENTH_DEG         1800
#define PITCH_INSTALL_DEFAULT_TENTH_DEG     0
#define YAW_INSTALL_DEFAULT_TENTH_DEG        0
#define YAW_ERROR_DEFAULT_TENTH_DEG          0

typedef struct
{
    uint32_t count;
    uint32_t magic;
    int16_t pitch_install_tenth_deg;
    int16_t yaw_install_tenth_deg;
    int16_t yaw_error_tenth_deg;
    uint16_t reserved;
    uint32_t check;
} MeasureCounterRecord;

_Static_assert(sizeof(MeasureCounterRecord) == 20U, "unexpected settings record layout");

static MeasureCounterRecord g_record;
static uint32_t g_saved_count;
static bool g_dirty;

static uint32_t record_check(const MeasureCounterRecord* record)
{
    const uint32_t offsets_01 = (uint32_t)(uint16_t)record->pitch_install_tenth_deg |
                                ((uint32_t)(uint16_t)record->yaw_install_tenth_deg << 16U);
    const uint32_t offsets_2 = (uint32_t)(uint16_t)record->yaw_error_tenth_deg |
                               ((uint32_t)record->reserved << 16U);

    return record->count ^ record->magic ^ offsets_01 ^ offsets_2 ^ MEASURE_COUNTER_CHECK_SEED;
}

static bool offsets_valid(const CalibrationOffsets* offsets)
{
    return (offsets != 0) &&
           (offsets->pitch_install_tenth_deg >= PITCH_INSTALL_MIN_TENTH_DEG) &&
           (offsets->pitch_install_tenth_deg <= PITCH_INSTALL_MAX_TENTH_DEG) &&
           (offsets->yaw_install_tenth_deg >= YAW_OFFSET_MIN_TENTH_DEG) &&
           (offsets->yaw_install_tenth_deg <= YAW_OFFSET_MAX_TENTH_DEG) &&
           (offsets->yaw_error_tenth_deg >= YAW_OFFSET_MIN_TENTH_DEG) &&
           (offsets->yaw_error_tenth_deg <= YAW_OFFSET_MAX_TENTH_DEG);
}

static void set_default_offsets(MeasureCounterRecord* record)
{
    record->pitch_install_tenth_deg = PITCH_INSTALL_DEFAULT_TENTH_DEG;
    record->yaw_install_tenth_deg = YAW_INSTALL_DEFAULT_TENTH_DEG;
    record->yaw_error_tenth_deg = YAW_ERROR_DEFAULT_TENTH_DEG;
    record->reserved = 0U;
}

static bool save_record(const MeasureCounterRecord* source)
{
    bool saved = false;
    MeasureCounterRecord record;

    if (source == 0)
    {
        return false;
    }

    record = *source;
    record.magic = MEASURE_COUNTER_MAGIC;
    record.reserved = 0U;
    record.check = record_check(&record);

    FLASH_Unlock();
    if (FLASH_COMPL == FLASH_EraseOnePage(MEASURE_COUNTER_FLASH_ADDR))
    {
        saved = true;
        for (uint32_t offset = 0U; offset < sizeof(record); offset += sizeof(uint32_t))
        {
            const uint32_t word = *(const uint32_t*)((const uint8_t*)&record + offset);

            if (FLASH_COMPL != FLASH_ProgramWord(MEASURE_COUNTER_FLASH_ADDR + offset, word))
            {
                saved = false;
                break;
            }
        }
    }
    FLASH_Lock();

    return saved;
}

void MeasureCounter_Init(void)
{
    const MeasureCounterRecord* stored = (const MeasureCounterRecord*)MEASURE_COUNTER_FLASH_ADDR;
    CalibrationOffsets offsets;
    bool previous_record_valid;

    g_record = *stored;
    offsets.pitch_install_tenth_deg = g_record.pitch_install_tenth_deg;
    offsets.yaw_install_tenth_deg = g_record.yaw_install_tenth_deg;
    offsets.yaw_error_tenth_deg = g_record.yaw_error_tenth_deg;
    previous_record_valid = ((g_record.magic == MEASURE_COUNTER_PREVIOUS_MAGIC_V1) ||
                             (g_record.magic == MEASURE_COUNTER_PREVIOUS_MAGIC_V2)) &&
                            (g_record.check == record_check(&g_record)) &&
                            offsets_valid(&offsets);

    if ((g_record.magic == MEASURE_COUNTER_MAGIC) &&
        (g_record.check == record_check(&g_record)) &&
        offsets_valid(&offsets))
    {
        g_saved_count = g_record.count;
        g_dirty = false;
    }
    else
    {
        g_record.count = ((stored->magic == MEASURE_COUNTER_LEGACY_MAGIC) || previous_record_valid) ?
                         stored->count : 0U;
        g_record.magic = MEASURE_COUNTER_MAGIC;
        set_default_offsets(&g_record);
        g_record.check = record_check(&g_record);
        g_saved_count = g_record.count;
        g_dirty = true;
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
        g_dirty = true;
    }
    return g_record.count;
}

void MeasureCounter_Reset(void)
{
    g_record.count = 0U;
    g_record.magic = MEASURE_COUNTER_MAGIC;
    g_dirty = true;
}

bool MeasureCounter_Save(void)
{
    if (!g_dirty)
    {
        return true;
    }

    if (!save_record(&g_record))
    {
        return false;
    }

    g_saved_count = g_record.count;
    g_dirty = false;
    return true;
}

void MeasureCounter_GetCalibration(CalibrationOffsets* offsets)
{
    if (offsets == 0)
    {
        return;
    }

    offsets->pitch_install_tenth_deg = g_record.pitch_install_tenth_deg;
    offsets->yaw_install_tenth_deg = g_record.yaw_install_tenth_deg;
    offsets->yaw_error_tenth_deg = g_record.yaw_error_tenth_deg;
}

bool MeasureCounter_SaveCalibration(const CalibrationOffsets* offsets)
{
    MeasureCounterRecord saved_record;

    if (!offsets_valid(offsets))
    {
        return false;
    }

    saved_record = g_record;
    saved_record.count = g_saved_count;
    saved_record.pitch_install_tenth_deg = offsets->pitch_install_tenth_deg;
    saved_record.yaw_install_tenth_deg = offsets->yaw_install_tenth_deg;
    saved_record.yaw_error_tenth_deg = offsets->yaw_error_tenth_deg;

    if (!save_record(&saved_record))
    {
        return false;
    }

    g_record.pitch_install_tenth_deg = offsets->pitch_install_tenth_deg;
    g_record.yaw_install_tenth_deg = offsets->yaw_install_tenth_deg;
    g_record.yaw_error_tenth_deg = offsets->yaw_error_tenth_deg;
    g_dirty = g_record.count != g_saved_count;
    return true;
}
