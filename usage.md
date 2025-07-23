# Usage Guide

This document explains how to configure and integrate **STM32 UART Driver Plus** into your project. All configuration options reside in `include/uart_driver_config.h`.

## Configuration Macros

The following macros control feature inclusion and memory footprints. Adjust them to suit your application.

| Macro | Description |
|-------|-------------|
|`USE_CMD_INTERPRETER`|Set to `1` to enable the command interpreter task.| 
|`CMD_MAX_LINE_LEN`|Maximum length of a received command line.|
|`CMD_MAX_PARAMS`|Maximum number of parameters parsed from a line.|
|`CMD_TASK_PRIO`|FreeRTOS priority for the command interpreter task.|
|`CMD_TASK_STACK`|Stack size for the command interpreter task in words.|
|`LOGGING_ENABLED`|Enable the logging module when non-zero.| 
|`TELEMETRY_ENABLED`|Enable the telemetry helper when non-zero.|
|`LOG_TX_USE_DMA`|Use DMA for log transmissions when set.|
|`TELEMETRY_TX_USE_DMA`|Use DMA when sending telemetry packets.|
|`LOG_QUEUE_DEPTH`|Depth of the internal log queue.| 
|`TELEMETRY_QUEUE_DEPTH`|Depth of the telemetry queue.| 
|`MAX_LOG_PAYLOAD`|Maximum characters stored for each log entry.| 
|`MAX_TELEMETRY_PAYLOAD`|Maximum payload bytes in a telemetry packet.| 
|`DEFAULT_LOG_LEVEL`|Initial severity filter for log messages.| 
|`LOG_TASK_STACK`|Stack size for the log processing task.| 
|`LOG_TASK_PRIO`|Priority of the log processing task.| 
|`UART_BACKEND`|Selects driver backend: `UART_BACKEND_HAL` or `UART_BACKEND_CMSIS`.|

These defaults are visible in [`uart_driver_config.h`](include/uart_driver_config.h).

## Basic Driver Setup

1. Configure and initialize the underlying UART peripheral (HAL or CMSIS) before calling the driver.
2. Create a `uart_drv_t` instance and call:
   ```c
   uart_drv_t uart;
   uart_init(&uart, &huart, &hdma_tx, &hdma_rx); // DMA handles may be NULL
   ```
3. Use the blocking or non‑blocking APIs to transmit and receive:
   ```c
   uart_send_blocking(&uart, data, len, timeout_ms);
   uart_receive_blocking(&uart, buf, len, timeout_ms);
   uart_send_nb(&uart, data, len);    // interrupt mode
   uart_start_dma_rx(&uart, buf, len); // DMA mode
   ```
4. Register a callback to be notified from ISR context:
   ```c
   void my_uart_cb(uart_event_t evt, void *ctx);
   uart_register_callback(&uart, my_uart_cb, NULL);
   ```

## Command Interpreter

When `USE_CMD_INTERPRETER` is enabled call `cmd_init()` after initializing the UART driver:

```c
cmd_init(&uart);
```

Commands are defined by filling a `Command` table (see [`sample_commands.c`](src/sample_commands.c)) and implementing handlers that accept an `Args *` structure.

## Logging and Telemetry

If `LOGGING_ENABLED` is set, initialize the logger with:
```c
log_init(&uart);
```
Then log messages from any task:
```c
log_write(LOG_LEVEL_INFO, "Started");
```
Telemetry packets can be queued via `telemetry_send(&pkt);` when `TELEMETRY_ENABLED` is non-zero.

## Examples

Complete integration examples using FreeRTOS are provided under [`examples/freertos_example`](examples/freertos_example). See `main_wDriver.c`, `main_wCommand.c` and `main_wLoggingCommand.c` for minimal and feature-rich setups.
