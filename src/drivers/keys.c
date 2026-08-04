#include "keys.h"

#include "app_log.h"
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
    uint32_t last_hold_log_seconds;
} DebouncedKey;

static DebouncedKey g_power_key;
static DebouncedKey g_mode_key;

static void key_reset(DebouncedKey* key, bool pressed, uint32_t now_ms)
{
    key->stable_pressed = pressed;
    key->last_raw_pressed = pressed;
    key->long_sent = false;
    key->changed_ms = now_ms;
    key->pressed_ms = pressed ? now_ms : 0U;
    key->last_hold_log_seconds = 0U;
}

void Keys_Init(void)
{
    const bool power_pressed = Board_ReadPowerKey();
    const bool mode_pressed = Board_ReadModeKey();

    key_reset(&g_power_key, power_pressed, 0U);
    key_reset(&g_mode_key, mode_pressed, 0U);

    if (power_pressed)
    {
        APP_LOGI("key", "power pressed at startup");
    }
    if (mode_pressed)
    {
        APP_LOGI("key", "mode pressed at startup");
    }
}

static KeyEvent update_power_key(bool raw_pressed, uint32_t now_ms)
{
    KeyEvent event = KEY_EVENT_NONE;

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
                g_power_key.last_hold_log_seconds = 0U;
                APP_LOGI("key", "power pressed");
            }
            else
            {
                const uint32_t held_ms = now_ms - g_power_key.pressed_ms;
                APP_LOGI("key", "power released after %u ms", (unsigned int)held_ms);
                if (!g_power_key.long_sent)
                {
                    APP_LOGI("key", "power short press");
                    event = KEY_EVENT_POWER_SHORT;
                }
            }
        }
    }

    if (g_power_key.stable_pressed)
    {
        const uint32_t held_seconds =
            (now_ms - g_power_key.pressed_ms) / 1000U;
        if (held_seconds > g_power_key.last_hold_log_seconds)
        {
            g_power_key.last_hold_log_seconds = held_seconds;
            APP_LOGI("key", "power held %u s", (unsigned int)held_seconds);
        }
    }

    if (g_power_key.stable_pressed && (!g_power_key.long_sent) &&
        ((now_ms - g_power_key.pressed_ms) >= APP_POWER_LONG_PRESS_MS))
    {
        g_power_key.long_sent = true;
        APP_LOGI("key", "power long press");
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
                g_mode_key.last_hold_log_seconds = 0U;
                APP_LOGI("key", "mode pressed");
            }
            else
            {
                const uint32_t held_ms = now_ms - g_mode_key.pressed_ms;
                APP_LOGI("key", "mode released after %u ms", (unsigned int)held_ms);
                APP_LOGI("key", "mode short press");
                event = KEY_EVENT_MODE_SHORT;
            }
        }
    }

    if (g_mode_key.stable_pressed)
    {
        const uint32_t held_seconds =
            (now_ms - g_mode_key.pressed_ms) / 1000U;
        if (held_seconds > g_mode_key.last_hold_log_seconds)
        {
            g_mode_key.last_hold_log_seconds = held_seconds;
            APP_LOGI("key", "mode held %u s", (unsigned int)held_seconds);
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
