#ifndef KEYS_H
#define KEYS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_POWER_SHORT,
    KEY_EVENT_POWER_LONG,
    KEY_EVENT_MODE_SHORT
} KeyEvent;

void Keys_Init(void);
KeyEvent Keys_Poll(uint32_t now_ms);
bool Keys_IsPowerPressed(void);
bool Keys_IsModePressed(void);

#endif /* KEYS_H */
