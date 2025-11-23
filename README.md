# STM32 UART Driver Plus

**Version 1.2**

A professional, modular UART driver package for STM32 microcontrollers designed for robust integration with CMSIS-RTOS v2 (FreeRTOS backend). Uses STM32 HAL drivers exclusively (LL removed).

## Key Features

- **HAL Driver Support**: Uses STM32 HAL drivers for portability and simplicity.
-- **RTOS Integration**: Full compatibility with CMSIS-RTOS v2 (FreeRTOS backend)
- **Modular Architecture**: Optional command interpreter, logging, and telemetry modules
- **Thread-Safe Operations**: Mutex-protected APIs for multi-task environments
 - **Operation Mode**: DMA-only (blocking/non-blocking APIs delegate to DMA)
- **Error Handling**: Comprehensive error detection and recovery mechanisms

## Architecture Overview

| STM32 HAL | None | Good | Medium | Low |
| STM32 HAL | FreeRTOS/CMSIS | Good | High | Low |

Configuration is now module-level; edit `include/uart_driver.h` and per-module headers to select your desired defaults:

```c
#define USE_STM32_HAL_DRIVERS  1    // HAL is the supported driver layer
/* CMSIS v2 (FreeRTOS backend) is required; configuration should use CMSIS-RTOS v2 APIs */
```

### Basic Integration

**HAL + FreeRTOS Implementation:**
```c
#include "uart_driver.h"

// Initialize with HAL handle
uart_drv_t uart_driver;
uart_system_init(&uart_driver, &huart2, NULL, NULL);

// Thread-safe transmission
uint8_t data[] = "Hello World!\n";
uart_send_blocking(&uart_driver, data, strlen((char*)data), 1000);
```

(LL driver examples were removed; see `examples/freertos_example/` for HAL-only usage.)

### Examples and Documentation

- Complete HAL + FreeRTOS project: `examples/freertos_example/`
-- HAL + FreeRTOS implementation: `examples/freertos_example/`
- Detailed configuration guide: `CMSIS_FREERTOS_GUIDE.md`
- API usage examples: `usage.md`

## Module Architecture

### UART Driver Core
- Configurable communication parameters (baud rate, data bits, parity, stop bits)
 - Transmission mode: DMA-only (blocking/non-blocking APIs delegate to DMA). Non-blocking APIs are queue-backed (TX/RX queues) and safe to call from ISRs or tasks.
- Comprehensive error handling (overrun, framing, noise detection)
- Thread-safe operation with mutex protection
- Support for STM32 HAL drivers

### Command Interpreter (Optional)
- Compile-time command registration via structured tables
- Automatic tokenization and command dispatch
- Configurable parameter limits and buffer sizes
- Optional FreeRTOS task integration

### Logging & Telemetry (Optional)
- Structured logging with configurable severity levels
- Queue-based buffering for non-blocking operation
- Telemetry packet transmission support
- Background processing with dedicated tasks

### Configuration Management
 - Module-level configuration in `include/uart_driver.h`, `include/logging.h`, and `include/command_module.h`
- Runtime feature enabling/disabling
- Configurable memory footprints and task parameters

## System Requirements

**Hardware:**
- STM32 microcontroller with UART/USART peripheral
- Optional DMA controller for high-performance applications

**Software:**
- STM32 HAL drivers
- CMSIS-RTOS v2 (FreeRTOS backend) required for RTOS-enabled features
- Standard C library (newlib/newlib-nano supported)

**Configuration Parameters:**
- `UART_TX_MODE`: Transmission mode selection
- `USE_CMD_INTERPRETER`, `LOGGING_ENABLED`, `TELEMETRY_ENABLED`: Feature toggles
- Task stack sizes, priorities, and buffer depths

## Quality Assurance

**Design Principles:**
- **Portability:** Standard STM32 HAL driver compatibility
- **Configurability:** Tunable memory footprints and feature sets
- **Reliability:** Error recovery mechanisms without system deadlock
- **Thread Safety:** Mutex-protected operations for concurrent access

**Validation:**
- Multi-mode testing (blocking, interrupt, DMA)
- RTOS integration verification
- Error injection and recovery testing
- Resource contention validation

## License

MIT License — see [LICENSE](LICENSE) for details.

---

*For detailed API documentation, configuration examples, and integration guides, refer to the accompanying documentation files.*
