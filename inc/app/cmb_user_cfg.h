#ifndef CMB_USER_CFG_H
#define CMB_USER_CFG_H

#include "SEGGER_RTT.h"

#define cmb_println(...)                 \
    do                                   \
    {                                    \
        SEGGER_RTT_printf(0, __VA_ARGS__); \
        SEGGER_RTT_WriteString(0, "\r\n"); \
    } while (0)

#define CMB_USING_OS_PLATFORM
#define CMB_OS_PLATFORM_TYPE  CMB_OS_PLATFORM_FREERTOS
#define CMB_CPU_PLATFORM_TYPE CMB_CPU_ARM_CORTEX_M4
#define CMB_USING_DUMP_STACK_INFO
#define CMB_PRINT_LANGUAGE    CMB_PRINT_LANGUAGE_ENGLISH

#define CMB_CSTACK_BLOCK_START _sstack
#define CMB_CSTACK_BLOCK_END   _estack
#define CMB_CODE_SECTION_START _stext
#define CMB_CODE_SECTION_END   _etext

#endif /* CMB_USER_CFG_H */
