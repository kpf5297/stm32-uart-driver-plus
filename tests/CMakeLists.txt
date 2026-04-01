add_library(test_unity STATIC
    framework/unity.c
)

target_include_directories(test_unity PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/framework
)

add_executable(test_uart_deterministic_queue
    test_uart_deterministic_queue.c
    ../src/uart_deterministic_queue.c
)

target_include_directories(test_uart_deterministic_queue PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(test_uart_deterministic_queue PRIVATE test_unity)
target_compile_features(test_uart_deterministic_queue PRIVATE c_std_11)

add_test(NAME test_uart_deterministic_queue COMMAND test_uart_deterministic_queue)

add_executable(test_uart_driver_determinism
    test_uart_driver_determinism.c
    mocks/Mockstm32f4xx_hal.c
    ../src/uart_driver.c
    ../src/uart_deterministic_queue.c
    ../src/uart_rtos_adapter.c
)

target_include_directories(test_uart_driver_determinism PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/framework
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_compile_definitions(test_uart_driver_determinism PRIVATE
    UART_RTOS_USE_CMSIS=0
    UART_STM32_HAL_HEADER=\"Mockstm32f4xx_hal.h\"
    USE_CMD_INTERPRETER=0
    LOGGING_ENABLED=0
)

target_link_libraries(test_uart_driver_determinism PRIVATE test_unity)
target_compile_features(test_uart_driver_determinism PRIVATE c_std_11)

add_test(NAME test_uart_driver_determinism COMMAND test_uart_driver_determinism)
