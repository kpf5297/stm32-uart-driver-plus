# STM32 UART Driver Plus

**Version 1.2 - Now with CMSIS/FreeRTOS/LL Support**

A professional, modular UART driver package for STM32 microcontrollers. Designed for robust integration in FreeRTOS-based systems with support for both HAL and LL (Low Layer) drivers.

## 🆕 New in Version 1.2

- **LL (Low Layer) Driver Support** - Choose between HAL and LL for optimal performance
- **Enhanced CMSIS-RTOS Integration** - Full compatibility with CMSIS-RTOS v1/v2
- **Improved FreeRTOS Support** - Better task-safe operations and resource management
- **Unified Abstraction Layer** - Seamless switching between HAL and LL backends
- **Performance Optimizations** - Lower memory footprint and faster execution with LL

## Supported Configurations

| Driver Layer | RTOS | Performance | Memory Usage | Complexity |
|-------------|------|-------------|--------------|------------|
| **HAL** | None | Good | Medium | Low |
| **HAL** | FreeRTOS | Good | High | Low |
| **HAL** | CMSIS-RTOS | Good | High | Low |
| **LL** | None | Excellent | Low | High |
| **LL** | FreeRTOS | Excellent | Medium | Medium |
| **LL** | CMSIS-RTOS | Excellent | Medium | Medium |

---

## Quick Start

### 1. Choose Your Configuration

Edit `include/uart_driver_config.h`:

```c
/* Driver Layer Selection */
#define USE_STM32_LL_DRIVERS  0    // 0=HAL (easier), 1=LL (faster)

/* RTOS Support */
#define USE_FREERTOS          1    // Enable FreeRTOS
#define USE_CMSIS_RTOS        1    // Enable CMSIS-RTOS wrapper
```

### 2. HAL + FreeRTOS (Recommended for beginners)

```c
#include "uart_driver.h"

// Initialize your HAL peripherals first
UART_HandleTypeDef huart2;
// ... configure huart2 ...

// Initialize UART driver
uart_drv_t uart_driver;
uart_system_init(&uart_driver, &huart2, NULL, NULL);

// Use in FreeRTOS task
void uart_task(void *pvParameters) {
    uint8_t data[] = "Hello World!
";
    uart_send_blocking(&uart_driver, data, strlen((char*)data), 1000);
}
```

### 3. LL + FreeRTOS (Recommended for performance)

```c
#include "uart_driver.h"

// Configure LL peripherals first
// ... LL initialization code ...

// Initialize UART driver
uart_drv_t uart_driver;
uart_system_init_ll(&uart_driver, USART2, DMA1, DMA1, 
                    LL_DMA_STREAM_6, LL_DMA_STREAM_5);

// Use with same API as HAL version
uint8_t data[] = "Hello LL World!
";
uart_send_blocking(&uart_driver, data, strlen((char*)data), 1000);
```

### 4. Examples

- **`examples/freertos_example/`** - Complete HAL + FreeRTOS project
- **`examples/freertos_ll_example.c`** - LL + FreeRTOS implementation
- **See `CMSIS_FREERTOS_LL_GUIDE.md`** for detailed configuration guide

---

- UART peripheral driver (interrupt and DMA modes)
- Thread-safe APIs with mutex protection
- Optional Command Interpreter module
- Optional Logging and Telemetry module
- Centralized configuration
- Example integration points for FreeRTOS users

---

## Features

### UART Driver Module

- Initialize UART with configurable baud rate, data bits, parity, stop bits.
- Transmit/receive in interrupt or DMA mode.
- **NEW**: Support for both HAL and LL (Low Layer) drivers
- APIs:
  - `uart_send(uint8_t *data, size_t len)`
  - `uart_receive(uint8_t *buf, size_t len)`
  - Non-blocking versions with status return.
- Error handling and reporting (overrun, framing, noise).
- Mutex-protected, thread-safe design.
- **NEW**: Enhanced CMSIS-RTOS compatibility

### Command Interpreter Module
- Compile-time registration of commands via a struct table.
- Tokenize incoming UART data and dispatch handlers.
- Configurable max command/parameter limits.
- FreeRTOS task integration optional.
- Enable/disable via `uart_driver_config.h`.

### Logging & Telemetry Module
- Structured log entry and telemetry packet formats.
- Queue buffering with configurable depth and payload size.
- `log_write(LogLevel, const char *fmt, ...)`
- `telemetry_send(const TelemetryPacket *pkt)`
- Background transmission using interrupt or DMA.
- Optional FreeRTOS task.

### Configuration Module
- Single point configuration (`uart_driver_config.h`).
- Enable/disable each feature.
- Select interrupt vs. DMA mode.
- Configure queue sizes, stack sizes, priorities.

### FreeRTOS Integration
- Initialization helper: `uart_system_init()`
- Optional task creation for Command and Logging modules.
- Weak task definitions allow user override.
- Clean shutdown API provided.

---

## Configuration Flags

All configuration options are centralized in `uart_driver_config.h`:
- `UART_TX_MODE` selects blocking, interrupt, or DMA operation globally
- `USE_CMD_INTERPRETER`, `LOGGING_ENABLED`, `TELEMETRY_ENABLED` toggle optional modules
- Task names, stack sizes, priorities
- Buffer sizes, queue depths

---

## Concurrency & Safety

- Internal mutex guarding all register and shared resource access.
- Safe to call from multiple FreeRTOS tasks or ISRs.

---

## Testing Setup

### Recommended Hardware:
- STM32 Nucleo or Discovery board (e.g., STM32F446RE recommended).
- UART-to-USB bridge for PC testing.

### Test checklist:
- Interrupt-mode loopback test.
- DMA-mode loopback test.
- FreeRTOS multi-task interaction.
- Error injection (overrun, framing).
- Logging and telemetry stress tests.

---

## Non-Functional Goals
- **Portability:** Uses the standard STM32 HAL drivers.
- **Configurability:** All memory footprints tunable via config.
- **Reliability:** Recovery from all reported errors without deadlock.
- **Documentation:** Doxygen comments throughout; full user README.

---

## License
MIT License — see [LICENSE](LICENSE).

---

## Topics / Tags
`stm32` `uart` `freertos` `embedded` `driver` `command-interpreter` `logging` `telemetry`
