#include "bv220.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define BV220_LINE_MAX 128U
#define BV220_FIELD_MAX 24U

static char g_line[BV220_LINE_MAX];
static uint8_t g_line_pos;

static int hex_digit(char ch)
{
    if ((ch >= '0') && (ch <= '9'))
    {
        return ch - '0';
    }
    if ((ch >= 'A') && (ch <= 'F'))
    {
        return 10 + ch - 'A';
    }
    if ((ch >= 'a') && (ch <= 'f'))
    {
        return 10 + ch - 'a';
    }
    return -1;
}

static bool checksum_valid(const char* line)
{
    const char* star = strchr(line, '*');
    uint8_t checksum = 0U;
    int high;
    int low;

    if ((line[0] != '$') || (star == 0) || (star[1] == '\0') || (star[2] == '\0'))
    {
        return false;
    }

    for (const char* p = line + 1; p < star; ++p)
    {
        checksum ^= (uint8_t)*p;
    }

    high = hex_digit(star[1]);
    low = hex_digit(star[2]);
    if ((high < 0) || (low < 0))
    {
        return false;
    }

    return checksum == (uint8_t)((high << 4) | low);
}

static uint8_t split_fields(char* line, char* fields[], uint8_t max_fields)
{
    uint8_t count = 0U;
    char* p = line;

    while ((*p != '\0') && (count < max_fields))
    {
        fields[count++] = p;
        while ((*p != ',') && (*p != '*') && (*p != '\0'))
        {
            ++p;
        }
        if (*p == '\0')
        {
            break;
        }
        *p++ = '\0';
    }

    return count;
}

static int32_t coordinate_to_e7(const char* field, const char* hemi)
{
    double value;
    int degrees;
    double minutes;
    int32_t e7;

    if ((field == 0) || (field[0] == '\0') || (hemi == 0) || (hemi[0] == '\0'))
    {
        return 0;
    }

    value = strtod(field, 0);
    degrees = (int)(value / 100.0);
    minutes = value - ((double)degrees * 100.0);
    e7 = (int32_t)(((double)degrees + (minutes / 60.0)) * 10000000.0);

    if ((hemi[0] == 'S') || (hemi[0] == 'W'))
    {
        e7 = -e7;
    }

    return e7;
}

static bool parse_gga(char* line, GnssData* data)
{
    char* fields[BV220_FIELD_MAX];
    uint8_t count = split_fields(line, fields, BV220_FIELD_MAX);
    int fix_quality;

    if ((data == 0) || (count < 10U))
    {
        return false;
    }

    fix_quality = atoi(fields[6]);
    data->valid = true;
    data->fix = fix_quality > 0;
    data->latitude_e7 = coordinate_to_e7(fields[2], fields[3]);
    data->longitude_e7 = coordinate_to_e7(fields[4], fields[5]);
    data->satellites = (uint8_t)atoi(fields[7]);
    data->altitude_cm = (int32_t)(strtod(fields[9], 0) * 100.0);
    return true;
}

static bool parse_sentence(const char* sentence, GnssData* data)
{
    char work[BV220_LINE_MAX];

    if (!checksum_valid(sentence))
    {
        return false;
    }

    (void)strncpy(work, sentence, sizeof(work) - 1U);
    work[sizeof(work) - 1U] = '\0';

    if (strncmp(work, "$GNGGA", 6U) == 0)
    {
        return parse_gga(work, data);
    }

    return false;
}

void Bv220_Reset(void)
{
    g_line_pos = 0U;
}

bool Bv220_ProcessByte(uint8_t byte, GnssData* data)
{
    if (byte == '$')
    {
        g_line_pos = 0U;
    }

    if ((byte == '\r') || (byte == '\n'))
    {
        if (g_line_pos == 0U)
        {
            return false;
        }
        g_line[g_line_pos] = '\0';
        g_line_pos = 0U;
        return parse_sentence(g_line, data);
    }

    if (g_line_pos >= (BV220_LINE_MAX - 1U))
    {
        g_line_pos = 0U;
        return false;
    }

    g_line[g_line_pos++] = (char)byte;
    return false;
}
