# Usage Guide

This document explains how to configure and integrate **STM32 UART Driver Plus** into your project. Configuration is now module-level via `include/uart_driver.h` and per-module headers such as `include/logging.h` and `include/command_module.h`.

## Configuration Macros

The following macros control feature inclusion and memory footprints. Adjust them to suit your application.

| Macro | Description |
|-------|-------------|
|`UART_TX_MODE`|Transmission mode. This top-level module enforces DMA-only operation; per-module defaults can be found in `include/uart_driver.h` and per-module headers.| 
|`USE_CMD_INTERPRETER`|Set to `1` to enable the command interpreter task.|
|`LOGGING_ENABLED`|Enable the logging module when non-zero.|
|`TELEMETRY_ENABLED`|Enable the telemetry helper when non-zero.|
|`CMD_MAX_LINE_LEN`|Maximum length of a received command line.|
|`CMD_MAX_PARAMS`|Maximum number of parameters parsed from a line.|
|`CMD_TASK_PRIO`|FreeRTOS priority for the command interpreter task.|
|`CMD_TASK_STACK`|Stack size for the command interpreter task in words.|
|`LOG_QUEUE_DEPTH`|Depth of the internal log queue.|
|`TELEMETRY_QUEUE_DEPTH`|Depth of the telemetry queue.|
|`MAX_LOG_PAYLOAD`|Maximum characters stored for each log entry.|
|`MAX_TELEMETRY_PAYLOAD`|Maximum payload bytes in a telemetry packet.|
|`DEFAULT_LOG_LEVEL`|Initial severity filter for log messages.|
|`LOG_TASK_STACK`|Stack size for the log processing task.|
|`LOG_TASK_PRIO`|Priority of the log processing task.|

These defaults are visible in the module headers: `include/uart_driver.h`, `include/logging.h`, and `include/command_module.h`.

## Basic Driver Setup

1. Configure and initialize the underlying UART peripheral using the STM32 HAL before calling the driver.
2. Create a `uart_drv_t` instance and call:

   ```c
   uart_drv_t uart;
   uart_system_init(&uart, &huart, &hdma_tx, &hdma_rx); // DMA handles may be NULL
   ```

3. Use the TX APIs to transmit and enable circular RX with IDLE detection for receive:

   ```c
   uart_send_blocking(&uart, data, len, timeout_ms);
   uart_send_nb(&uart, data, len);    // non-blocking, enqueues data to TX queue (DMA driven)
   ```
## Command Interpreter

`uart_system_init()` automatically starts the interpreter when `USE_CMD_INTERPRETER` is set. You can also call `cmd_init()` manually if finer control is required.

Commands are defined by filling a `Command` table (see [`commands.c`](src/commands.c)) and implementing handlers that accept an `Args *` structure.

## Logging and Telemetry

When `LOGGING_ENABLED` is non-zero, `uart_system_init()` invokes
`log_init()` automatically. You can still call `log_init()` yourself
if the driver was brought up manually. Then log messages from any task:

```c
log_write(LOG_LEVEL_INFO, "Started");
```

Telemetry packets can be queued via `telemetry_send(&pkt);` when `TELEMETRY_ENABLED` is non-zero.

## Enabling Floating-Point printf in CMake

Many embedded toolchains omit floating-point support from `printf` to save
flash. When using CMake with newlib-nano, enable `%f` formatting by linking
with `_printf_float`:

```cmake
target_link_options(your_target PRIVATE -Wl,-u,_printf_float)
```

**In gcc-arm-none-eabi.cmake**

```cmake
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u _printf_float")
```

Add `_scanf_float` in the same manner if floating-point scanning is required.

## Examples

Complete integration examples using FreeRTOS are provided under [`examples/freertos_example`](examples/freertos_example).
## Circular DMA (RX) + IDLE Detection

The driver supports circular DMA mode for low-latency receive with IDLE line detection. This is ideal for processing variable-length ASCII lines or packets with lower CPU overhead.

To start circular DMA for RX and enable IDLE detection, call:

```c
uint8_t rxbuf[CMD_MAX_LINE_LEN];
uart_register_callback(&uart, uart_event_cb, NULL); // register callback
uart_start_circular_rx(&uart, rxbuf, sizeof(rxbuf));
```

When the UART detects an IDLE condition, the driver will call the registered callback with `UART_EVT_RX_AVAILABLE`. In that callback, call `uart_read_from_circular` to pull up to a desired number of bytes from the circular buffer. This avoids repeated 1-byte DMA re-arms and reduces ISR work.

`main_wo_Command.c` demonstrates a minimal setup without the command interpreter, while `main_w_Command.c` showcases a feature-rich setup with the command interpreter enabled.
