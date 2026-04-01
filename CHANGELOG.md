# Changelog

All notable changes to this project will be documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/) and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.3.0] — 2026-04-01

### Fixed (Bugs / Race Conditions)

- **`uart_send_nb` race condition**: The check of `tx_status != UART_BUSY` and the subsequent DMA start were not atomic. An ISR completing between the check and the DMA launch could result in two simultaneous DMA transfers. Fixed by atomically testing-and-setting `tx_status` inside a critical section before launching DMA from outside it.

- **`HAL_UART_TxCpltCallback` race condition**: `notify_event(UART_EVT_TX_COMPLETE)` was called before dequeuing the next item, causing `tx_status` to briefly become `UART_OK` while pending items remained. A task calling `uart_send_nb` in that window could erroneously start a second DMA concurrently. Fixed by dequeuing and starting the next transfer *before* reporting completion — `UART_EVT_TX_COMPLETE` is now only fired when the queue is fully drained.

- **`uart_instances` array ISR safety**: `find_drv` (called from HAL IRQ callbacks) iterated the instance registry concurrently with `uart_init`/`uart_deinit` in task context, with no mutual exclusion. A partially-updated registry observed by an ISR is undefined behaviour. Fixed by wrapping all registry writes in `FAULT_ENTER/EXIT_CRITICAL` in `uart_init` and `uart_deinit`, and reads in `find_drv`.

- **`uart_start_circular_rx` DMA register patched unsafely**: `DMA_SxCR_CIRC` was set via a direct register write before `HAL_UART_Receive_DMA`, which reinitialises the DMA stream from `hdmarx->Init` and would overwrite the bit. The direct write has been removed; DMA circular mode must now be configured via `hdmarx->Init.Mode = DMA_CIRCULAR` before calling `MX_DMA_Init()` (as CubeMX generates).

- **`process_telemetry_queue` NULL dereference**: `osMessageQueueGet` was called on `telemetry_queue` without checking whether queue creation had succeeded in `log_init`. Added a `NULL` guard.

- **Missing NULL guards**: `uart_reconfigure`, `uart_flush_rx`, and `uart_flush_tx` did not validate their `drv` argument. Added guards matching all other public functions.

### Fixed (Logic Errors)

- **`logging.c` timestamp incorrect at non-1000 Hz tick rates**: `get_current_timestamp` divided raw RTOS ticks by the hardcoded constant `1000`, which was only correct at 1 kHz. Now uses `osKernelGetTickFreq()` to be correct at any configured tick rate (consistent with `fault_module.c`).

- **`cmsis_mutex_lock` short-timeout truncation**: Integer truncation in the ms-to-ticks conversion silently turned sub-tick timeouts (e.g. 1 ms at 100 Hz) into 0 ticks, causing `osMutexAcquire` to perform a non-blocking poll. Added a minimum-1-tick guard matching the existing behaviour in `cmsis_delay_ms`.

### Fixed (Minor / Style)

- **`cmd_printf` non-reentrant static buffer**: The formatted output buffer was declared `static`, making the function non-reentrant. Concurrent calls from separate RTOS tasks silently corrupted each other's output. Changed to stack-local; also simplified away the unnecessary 31-byte chunking loop since `CMD_MAX_LINE_LEN` fits within one TX pool node.

- **`osThreadNew` return values unchecked**: In `command_module.c` and `logging.c`, a failed thread creation would leave a NULL task ID. In `command_module.c` this would later cause `osThreadFlagsSet(NULL, 1u)` — undefined behaviour — when the UART event callback fired. Now both sites check the return value and handle failure cleanly.

- **`snprintf` return value passed as transmit length without bounds check**: In `logging.c`, `snprintf` return was passed directly as the byte count to `log_tx`. A truncated format string returns a value ≥ `len`, which would overflow the buffer on transmission. Added a clamp to `len - 1`.

- **`uart_get_status` backslash line continuation**: Replaced the backslash-continued ternary with three clear `if`/`return` statements. Added NULL guard.

- **Stale comments and orphaned API declarations in `uart_driver.h`**: Removed dangling doc blocks for `uart_receive_blocking` and `uart_receive_nb` (removed in v1.2), and a `/** @brief Begin a DMA based receive. */` comment with no corresponding declaration. Added a note on `TICKS_PER_SECOND`/`MS_TO_TICKS` warning about per-expansion function calls.

- **Duplicate `#include "command_module.h"` in `commands.h`**: Removed the redundant second inclusion.

### Added

- `include/uart_driver_version.h`: Version constants (`UART_DRIVER_VERSION_MAJOR/MINOR/PATCH/STR`) for compile-time version checking.
- `VERSION` file at repository root containing the canonical version string.
- CMakeLists.txt now declares the project version via `project(... VERSION 1.3.0 ...)`.

---

## [1.2.0] — 2025

### Changed
- Removed LL driver support; HAL-only.
- Refactored for CMSIS-RTOS v2 (FreeRTOS backend).
- Replaced abstraction layer with direct `uart_rtos_adapter_t` function-pointer table.
- Added `uart_send_dma_blocking` (blocking DMA transmit with timeout).
- Added NUCLEO-446RE bare-system sample code.

### Removed
- LL (low-layer) driver and all LL-related examples.
- `uart_receive_blocking` and `uart_receive_nb` (replaced by circular DMA RX).
- Old RTOS abstraction layer header.

---

## [1.1.0]

### Added
- Non-blocking queued TX via deterministic pool (`uart_tx_node_t`).
- `uart_deterministic_queue` — fixed-size, zero-allocation ring buffer.
- Host-side unit tests (Unity framework, CMake-driven).
- RTOS adapter pattern (`uart_rtos_adapter_t`) for swappable RTOS backends.

---

## [1.0.0]

- Initial release: HAL + FreeRTOS UART driver with blocking and DMA TX/RX, command interpreter, logging, and fault management modules.
