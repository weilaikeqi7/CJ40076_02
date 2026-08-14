#include "keys.h"

#include "board.h"
#include "board_config.h"

#include <stdbool.h>

typedef struct
{
    bool stable_pressed;
    bool last_raw_pressed;
    bool long_sent;
    uint32_t changed_ms;
    uint32_t pressed_ms;
} DebouncedKey;

static DebouncedKey g_power_key;
static DebouncedKey g_mode_key;
static bool g_power_wait_release;

static void key_reset(DebouncedKey* key, bool pressed, uint32_t now_ms)
{
    key->stable_pressed = pressed;
    key->last_raw_pressed = pressed;
    key->long_sent = false;
    key->changed_ms = now_ms;
    key->pressed_ms = pressed ? now_ms : 0U;
}

void Keys_Init(void)
{
    const bool power_pressed = Board_ReadPowerKey();

    key_reset(&g_power_key, power_pressed, 0U);
    key_reset(&g_mode_key, Board_ReadModeKey(), 0U);
    g_power_wait_release = power_pressed;
}

static KeyEvent update_power_key(bool raw_pressed, uint32_t now_ms)
{
    KeyEvent event = KEY_EVENT_NONE;

    if (g_power_wait_release)
    {
        if (raw_pressed != g_power_key.last_raw_pressed)
        {
            g_power_key.last_raw_pressed = raw_pressed;
            g_power_key.changed_ms = now_ms;
        }

        if ((!raw_pressed) &&
            ((now_ms - g_power_key.changed_ms) >= APP_KEY_DEBOUNCE_MS))
        {
            key_reset(&g_power_key, false, now_ms);
            g_power_wait_release = false;
        }
        return KEY_EVENT_NONE;
    }

    if (raw_pressed != g_power_key.last_raw_pressed)
    {
        g_power_key.last_raw_pressed = raw_pressed;
        g_power_key.changed_ms = now_ms;
    }

    if ((now_ms - g_power_key.changed_ms) >= APP_KEY_DEBOUNCE_MS)
    {
        if (raw_pressed != g_power_key.stable_pressed)
        {
            g_power_key.stable_pressed = raw_pressed;
            if (raw_pressed)
            {
                g_power_key.pressed_ms = now_ms;
                g_power_key.long_sent = false;
            }
            else if (!g_power_key.long_sent)
            {
                event = KEY_EVENT_POWER_SHORT;
            }
        }
    }

    if (g_power_key.stable_pressed && (!g_power_key.long_sent) &&
        ((now_ms - g_power_key.pressed_ms) >= APP_POWER_LONG_PRESS_MS))
    {
        g_power_key.long_sent = true;
        event = KEY_EVENT_POWER_LONG;
    }

    return event;
}

static KeyEvent update_mode_key(bool raw_pressed, uint32_t now_ms)
{
    KeyEvent event = KEY_EVENT_NONE;

    if (raw_pressed != g_mode_key.last_raw_pressed)
    {
        g_mode_key.last_raw_pressed = raw_pressed;
        g_mode_key.changed_ms = now_ms;
    }

    if ((now_ms - g_mode_key.changed_ms) >= APP_KEY_DEBOUNCE_MS)
    {
        if (raw_pressed != g_mode_key.stable_pressed)
        {
            g_mode_key.stable_pressed = raw_pressed;
            if (raw_pressed)
            {
                g_mode_key.pressed_ms = now_ms;
            }
            else
            {
                event = KEY_EVENT_MODE_SHORT;
            }
        }
    }

    return event;
}

KeyEvent Keys_Poll(uint32_t now_ms)
{
    KeyEvent event;

    event = update_power_key(Board_ReadPowerKey(), now_ms);
    if (event != KEY_EVENT_NONE)
    {
        return event;
    }

    return update_mode_key(Board_ReadModeKey(), now_ms);
}

bool Keys_IsPowerPressed(void)
{
    return g_power_key.stable_pressed;
}

bool Keys_IsModePressed(void)
{
    return g_mode_key.stable_pressed;
}
