cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE
        "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi-gcc.cmake"
        CACHE FILEPATH "N32 GNU Arm toolchain file")
endif()

project(CJ40076_02 C ASM)

set(APP_TARGET cj40076_02)
set(APP_OUTPUT_NAME ${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor/Nations.N32G4FR_Library.2.6.0"
    CACHE PATH "Nations N32G4FR SDK root")
set(N32_TARGET n32g4fr CACHE STRING "Nations target platform name")
set(N32_FLASH_ORIGIN "0x08000000" CACHE STRING "Flash origin")
set(N32_FLASH_SIZE "512K" CACHE STRING "Flash size for N32G4FRHEQ7")
set(N32_RAM_ORIGIN "0x20000000" CACHE STRING "RAM origin" FORCE)
set(N32_RAM_SIZE "144K" CACHE STRING "RAM size for N32G4FRHEQ7" FORCE)
set(N32_STACK_SIZE "0x800" CACHE STRING "Stack size" FORCE)
set(N32_HEAP_SIZE "0x200" CACHE STRING "Heap size")
option(N32_USE_FPU "Build for Cortex-M4F single precision FPU" ON)
set(N32_FLOAT_ABI "hard" CACHE STRING "GCC float ABI: hard, softfp, or soft")

set(CMSIS_CORE_DIR "${SDK_ROOT}/firmware/CMSIS/core")
set(CMSIS_DEVICE_DIR "${SDK_ROOT}/firmware/CMSIS/device")
set(STDPERIPH_DIR "${SDK_ROOT}/firmware/${N32_TARGET}_std_periph_driver")
set(STARTUP_FILE "${CMSIS_DEVICE_DIR}/startup/startup_${N32_TARGET}_gcc.s")
set(FREERTOS_KERNEL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/FreeRTOS-Kernel")
set(SEGGER_RTT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/RTT-8.58.0")
set(CMBACKTRACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vendor/CmBacktrace-1.5.0")

foreach(required_path IN ITEMS
        "${CMSIS_CORE_DIR}"
        "${CMSIS_DEVICE_DIR}"
        "${STDPERIPH_DIR}/inc"
        "${STDPERIPH_DIR}/src"
        "${STARTUP_FILE}"
        "${FREERTOS_KERNEL_DIR}/include"
        "${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F"
        "${SEGGER_RTT_DIR}/RTT"
        "${SEGGER_RTT_DIR}/Config"
        "${CMBACKTRACE_DIR}/cm_backtrace")
    if(NOT EXISTS "${required_path}")
        message(FATAL_ERROR "Missing SDK path: ${required_path}")
    endif()
endforeach()

set(LINKER_SCRIPT_IN "${CMAKE_CURRENT_SOURCE_DIR}/linker/n32g4fr_flash.ld.in")
set(LINKER_SCRIPT "${CMAKE_CURRENT_BINARY_DIR}/n32g4fr_flash.ld")
set(BUILD_INFO_HEADER "${CMAKE_CURRENT_BINARY_DIR}/generated/build_info.h")
configure_file("${LINKER_SCRIPT_IN}" "${LINKER_SCRIPT}" @ONLY)

add_custom_target(generate_build_info ALL
    COMMAND ${CMAKE_COMMAND}
        -DOUTPUT="${BUILD_INFO_HEADER}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/generate_build_info.cmake"
    BYPRODUCTS "${BUILD_INFO_HEADER}"
    COMMENT "Generating build info"
)

file(GLOB STDPERIPH_SOURCES CONFIGURE_DEPENDS
    "${STDPERIPH_DIR}/src/*.c"
)

set(PROJECT_SOURCES
    src/main.c
    "${BUILD_INFO_HEADER}"
    src/app/app_debug.c
    src/app/app_log.c
    src/app/app_runtime_stats.c
    src/app/app_state.c
    src/app/app_tasks.c
    src/app/measure_counter.c
    src/app/module_selftest.c
    src/app/power_manager.c
    src/bsp/board.c
    src/bsp/bsp_adc.c
    src/bsp/bsp_gpio.c
    src/bsp/bsp_uart.c
    src/drivers/battery.c
    src/drivers/display_output.c
    src/drivers/keys.c
    src/drivers/serial_display.c
    src/modules/bv220.c
    src/modules/jy901b.c
    src/modules/rangefinder.c
    src/n32g4fr_it.c
    src/syscalls.c
    src/freertos_heap.c
    "${CMSIS_DEVICE_DIR}/system_n32g4fr.c"
    "${STARTUP_FILE}"
    "${FREERTOS_KERNEL_DIR}/tasks.c"
    "${FREERTOS_KERNEL_DIR}/queue.c"
    "${FREERTOS_KERNEL_DIR}/list.c"
    "${FREERTOS_KERNEL_DIR}/timers.c"
    "${FREERTOS_KERNEL_DIR}/event_groups.c"
    "${FREERTOS_KERNEL_DIR}/stream_buffer.c"
    "${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F/port.c"
    "${FREERTOS_KERNEL_DIR}/portable/MemMang/heap_4.c"
    "${SEGGER_RTT_DIR}/RTT/SEGGER_RTT.c"
    "${SEGGER_RTT_DIR}/RTT/SEGGER_RTT_printf.c"
    "${CMBACKTRACE_DIR}/cm_backtrace/cm_backtrace.c"
    "${CMBACKTRACE_DIR}/cm_backtrace/fault_handler/gcc/cmb_fault.S"
    ${STDPERIPH_SOURCES}
)

add_executable(${APP_TARGET} ${PROJECT_SOURCES})
add_dependencies(${APP_TARGET} generate_build_info)

set_target_properties(${APP_TARGET} PROPERTIES
    OUTPUT_NAME ${APP_OUTPUT_NAME}
)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
)

if(N32_USE_FPU)
    list(APPEND MCU_FLAGS -mfpu=fpv4-sp-d16 -mfloat-abi=${N32_FLOAT_ABI})
else()
    list(APPEND MCU_FLAGS -mfloat-abi=soft)
endif()

target_compile_definitions(${APP_TARGET} PRIVATE
    N32G4FR
    USE_STDPERIPH_DRIVER
    CMB_USER_CFG
    APP_USE_CMB_FAULT_HANDLER=1
    $<$<CONFIG:Debug>:APP_DEBUG_BUILD=1>
    $<$<CONFIG:Debug>:APP_DEBUG_DISABLE_IDLE_SLEEP=1>
    $<$<BOOL:${N32_USE_FPU}>:N32_EXPECT_FPU=1>
)

target_include_directories(${APP_TARGET} PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
    inc
    inc/app
    inc/bsp
    inc/drivers
    inc/modules
    "${CMSIS_CORE_DIR}"
    "${CMSIS_DEVICE_DIR}"
    "${STDPERIPH_DIR}/inc"
    "${FREERTOS_KERNEL_DIR}/include"
    "${FREERTOS_KERNEL_DIR}/portable/GCC/ARM_CM4F"
    "${SEGGER_RTT_DIR}/RTT"
    "${SEGGER_RTT_DIR}/Config"
    "${CMBACKTRACE_DIR}/cm_backtrace"
)

target_compile_options(${APP_TARGET} PRIVATE
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-sign-compare
    -ffunction-sections
    -fdata-sections
    -fno-common
    -fmessage-length=0
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
    $<$<CONFIG:Debug>:-Og -g3 -gdwarf-2>
    $<$<CONFIG:Release>:-O2 -g0>
)

target_link_options(${APP_TARGET} PRIVATE
    ${MCU_FLAGS}
    -T${LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${APP_OUTPUT_NAME}.map
    -Wl,--cref
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments
    --specs=nano.specs
    --specs=nosys.specs
)

target_link_libraries(${APP_TARGET} PRIVATE
    c
    m
    nosys
)

add_custom_command(TARGET ${APP_TARGET} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${APP_TARGET}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex -S $<TARGET_FILE:${APP_TARGET}> ${APP_OUTPUT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${APP_TARGET}> ${APP_OUTPUT_NAME}.bin
    COMMENT "Generating HEX and BIN images"
)
