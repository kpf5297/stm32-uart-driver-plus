cmake_minimum_required(VERSION 3.20)

project(stm32_uart_driver_plus VERSION 1.3.0 LANGUAGES C)

option(UART_DRIVER_BUILD_EMBEDDED_LIB "Build STM32 HAL/CMSIS UART driver library" ON)
option(UART_DRIVER_BUILD_TESTS "Build host-side deterministic unit tests" OFF)

if(UART_DRIVER_BUILD_EMBEDDED_LIB)
    add_library(stm32_uart_driver_plus STATIC
        src/uart_driver.c
        src/uart_rtos_adapter.c
        src/command_module.c
        src/commands.c
        src/logging.c
        src/fault_module.c
        src/uart_deterministic_queue.c
    )

    target_include_directories(stm32_uart_driver_plus PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_compile_features(stm32_uart_driver_plus PUBLIC c_std_11)

    # CubeMX can override HAL family via UART_STM32_HAL_HEADER definition.
    target_compile_definitions(stm32_uart_driver_plus PUBLIC
        UART_STM32_HAL_HEADER="\"stm32f4xx_hal.h\""
    )
endif()

if(UART_DRIVER_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
