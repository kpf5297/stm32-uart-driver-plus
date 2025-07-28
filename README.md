# STM32 UART Driver Plus

**Version 1.1**

A professional, modular UART driver package for STM32 microcontrollers. Designed for robust integration in FreeRTOS-based systems, this package provides:

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
- APIs:
  - `uart_send(uint8_t *data, size_t len)`
  - `uart_receive(uint8_t *buf, size_t len)`
  - Non-blocking versions with status return.
- Error handling and reporting (overrun, framing, noise).
- Mutex-protected, thread-safe design.

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

All configuration options centralized in `uart_driver_config.h`:
- `CMD_TX_USE_DMA` to send command responses with DMA when a DMA handle is present
- `LOG_TX_USE_DMA` to use DMA for log output
- `TELEMETRY_TX_USE_DMA` to use DMA for telemetry
- Task names, stack sizes, priorities.
- Buffer sizes, queue depths.

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
