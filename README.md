# STM32 UART Driver Plus

**Version 1.3.0** — [Changelog](CHANGELOG.md)

A thread-safe, DMA-backed UART driver for STM32 microcontrollers targeting CMSIS-RTOS v2 / FreeRTOS applications. Drop it under your CubeMX project, link one target, and get non-blocking queued TX, circular DMA RX with IDLE detection, and optional command interpreter, structured logging, and fault management modules.

---

## Table of Contents

1. [Features](#features)
2. [Repository Layout](#repository-layout)
3. [CubeMX Setup (Step-by-Step)](#cubemx-setup-step-by-step)
4. [Adding the Driver to Your CMake Project](#adding-the-driver-to-your-cmake-project)
5. [Initialization and Basic Usage](#initialization-and-basic-usage)
6. [API Reference](#api-reference)
7. [Optional Modules](#optional-modules)
8. [Configuration Reference](#configuration-reference)
9. [Running Host Tests](#running-host-tests)
10. [Memory Footprint](#memory-footprint)
11. [Troubleshooting](#troubleshooting)
12. [License](#license)

---

## Features

- **HAL only** — STM32 HAL drivers; no LL layer required.
- **DMA TX** — queued non-blocking (`uart_send_nb`) and blocking-with-timeout (`uart_send_dma_blocking`) transmit, both backed by a deterministic fixed-size node pool (no heap).
- **Circular DMA RX** with UART IDLE interrupt — call `uart_start_circular_rx` once; read whenever with `uart_read_from_circular`.
- **CMSIS-RTOS v2** mutex protection on all public APIs; safe to call from multiple tasks.
- **Swappable RTOS adapter** (`uart_rtos_adapter_t`) — replace the CMSIS backend with any RTOS for porting or testing.
- **Up to 4 concurrent UART instances** registered globally; HAL callbacks are routed automatically.
- **Optional modules**: command interpreter (CLI over UART), structured logging with severity levels, and fault management.
- **Host-side unit tests** — build and run on macOS/Linux with no STM32 toolchain required.

---

## Repository Layout

```text
stm32-uart-driver-plus/
├── include/
│   ├── uart_driver.h              # Core driver API + configuration macros
│   ├── uart_driver_version.h      # Version constants
│   ├── uart_deterministic_queue.h # Lock-free fixed-size ring buffer
│   ├── uart_rtos_adapter.h        # RTOS abstraction interface
│   ├── command_module.h           # Optional CLI module
│   ├── logging.h                  # Optional logging / telemetry module
│   └── fault_module.h             # Optional fault management module
├── src/
│   ├── uart_driver.c
│   ├── uart_deterministic_queue.c
│   ├── uart_rtos_adapter.c
│   ├── command_module.c
│   ├── commands.c
│   ├── logging.c
│   └── fault_module.c
├── tests/                         # Host-side Unity tests
├── sample_code/                   # Board-specific integration samples
├── CMakeLists.txt
├── VERSION
└── CHANGELOG.md
```

---

## CubeMX Setup (Step-by-Step)

This section walks through configuring a NUCLEO-F446RE (or any STM32F4) project in STM32CubeMX to work with this driver. Adjust peripheral names to match your board.

### 1. UART Peripheral

1. Open **Pinout & Configuration → Connectivity → USARTx**.
2. Set **Mode** to `Asynchronous`.
3. Configure baud rate, data bits, parity, and stop bits as required.
4. Under **DMA Settings**, add two DMA requests:
   - `USARTx_TX` — Direction: **Memory to Peripheral**, Mode: **Normal**, Data width: Byte/Byte.
   - `USARTx_RX` — Direction: **Peripheral to Memory**, Mode: **Circular**, Data width: Byte/Byte.

   > **Important:** The RX DMA stream **must** be set to **Circular** mode here in CubeMX. The driver relies on `hdmarx->Init.Mode = DMA_CIRCULAR` being set before `HAL_UART_Receive_DMA` is called and does not patch the register itself.

5. Under **NVIC Settings**, enable:
   - `USARTx global interrupt`
   - Both DMA stream interrupts that CubeMX generated for TX and RX.

### 2. FreeRTOS / CMSIS-RTOS v2

1. Open **Middleware → FreeRTOS**.
2. Set **Interface** to `CMSIS_V2`.
3. Increase **TOTAL_HEAP_SIZE** to at least `16384` bytes (16 KB). Each driver instance creates two mutexes; each optional module creates one or more message queues and tasks.
4. Set **configUSE_MUTEXES** = `1` (usually already enabled in the CMSIS V2 template).

   Recommended heap sizing for the full driver stack (driver + logging + command interpreter):

| Component | Approximate heap usage |
|---|---|
| 2 CMSIS mutexes (per driver instance) | ~200 bytes |
| Log message queue (32 entries × 148 bytes) | ~4.7 KB |
| Telemetry queue (16 entries × 8 bytes) | ~128 bytes |
| Log task stack | configurable, default 1024 words |
| Command task stack | configurable, default 512 words |
| **Minimum recommended TOTAL_HEAP_SIZE** | **16 384 bytes** |

5. No FreeRTOS tasks need to be created in CubeMX for this driver — the optional modules spawn their own tasks internally.

### 3. Linker Script

If using the default STM32CubeMX linker script, ensure the heap is large enough:

```
_Min_Heap_Size  = 0x4000;   /* 16 KB */
_Min_Stack_Size = 0x800;    /* 2 KB  */
```

### 4. UART IRQ Handler

CubeMX generates `USARTx_IRQHandler` in `stm32f4xx_it.c`. Ensure it calls the HAL handler:

```c
void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart2);
}
```

This is already generated by CubeMX. Do **not** remove this call — it is what triggers `HAL_UART_TxCpltCallback` and `HAL_UART_ErrorCallback`, which the driver overrides globally.

### 5. UART IDLE Interrupt for Circular RX

The driver calls `uart_process_idle` to detect end-of-packet on the circular RX buffer. Wire it into your UART IRQ handler:

```c
/* stm32f4xx_it.c */
#include "uart_driver.h"

void USART2_IRQHandler(void) {
    uart_process_idle(&huart2);   /* must come BEFORE HAL_UART_IRQHandler */
    HAL_UART_IRQHandler(&huart2);
}
```

> `uart_process_idle` checks for the IDLE flag, clears it by reading SR then DR (F4-specific), and invokes the registered callback with `UART_EVT_RX_AVAILABLE` if circular mode is active.

---

## Adding the Driver to Your CMake Project

Place this repository anywhere under your project — the conventional location is alongside the HAL drivers:

```
MyProject/
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/
│   └── stm32-uart-driver-plus/   ← here
└── CMakeLists.txt
```

In your top-level `CMakeLists.txt` (or the one CubeMX generates):

```cmake
add_subdirectory(Drivers/stm32-uart-driver-plus)
target_link_libraries(${PROJECT_NAME} PRIVATE stm32_uart_driver_plus)
```

For a non-F4 series, override the HAL umbrella header:

```cmake
target_compile_definitions(stm32_uart_driver_plus PUBLIC
    UART_STM32_HAL_HEADER="stm32h7xx_hal.h"
)
```

The embedded library target (`UART_DRIVER_BUILD_EMBEDDED_LIB`, default `ON`) links against no system libraries of its own — it expects the parent project to provide the HAL and CMSIS-RTOS v2 headers via its own include paths.

---

## Initialization and Basic Usage

### Minimal initialization (driver core only)

```c
#include "uart_driver.h"

/* Declare statically — no heap allocation required */
static uart_drv_t g_uart;

/* Call from a FreeRTOS task AFTER the scheduler has started */
void app_task(void *arg) {
    uart_init(&g_uart, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);

    /* Start circular DMA RX into a static buffer */
    static uint8_t rx_buf[64];
    uart_start_circular_rx(&g_uart, rx_buf, sizeof(rx_buf));

    for (;;) {
        uint8_t tmp[64];
        size_t n = uart_read_from_circular(&g_uart, tmp, sizeof(tmp));
        if (n > 0) {
            uart_send_nb(&g_uart, tmp, n);   /* echo back, non-blocking */
        }
        osDelay(1);
    }
}
```

### Using the convenience initializer (all modules)

```c
/* uart_system_init calls uart_init, then cmd_init and log_init
 * when USE_CMD_INTERPRETER and LOGGING_ENABLED are set (default 1). */
uart_system_init(&g_uart, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);
```

### Blocking transmit with timeout

```c
const char *msg = "Hello\r\n";
uart_send_dma_blocking(&g_uart, (const uint8_t *)msg, strlen(msg), 100 /*ms*/);
```

### Event-driven RX with callback

```c
static void on_uart_event(uart_event_t evt, void *ctx) {
    if (evt == UART_EVT_RX_AVAILABLE) {
        /* Signal a task — do not call uart_read_from_circular from ISR */
        osThreadFlagsSet(rx_task_id, 0x01U);
    }
}

uart_register_callback(&g_uart, on_uart_event, NULL);
```

---

## API Reference

### Lifecycle

| Function | Description |
|----------|-------------|
| `uart_init(drv, huart, hdma_tx, hdma_rx)` | Initialize driver instance. Call from a task, after RTOS scheduler start. |
| `uart_system_init(drv, huart, hdma_tx, hdma_rx)` | Initialize driver + optional modules (logging, command interpreter). |
| `uart_deinit(drv)` | Release resources and remove from registry. |
| `uart_reconfigure(drv, huart, hdma_tx, hdma_rx)` | Swap in new HAL handles at runtime (e.g. after a peripheral reset). |
| `uart_set_rtos_adapter(drv, adapter, ctx)` | Override the default CMSIS adapter. Call **before** `uart_init`. |
| `uart_register_callback(drv, cb, ctx)` | Register event callback (invoked from ISR or IRQ context). |

### Transmit

| Function | Description |
|----------|-------------|
| `uart_send_nb(drv, data, len)` | Enqueue a transmit request. Returns immediately; callback fires on completion. `len` ≤ `UART_TX_NODE_SIZE` (default 256). |
| `uart_send_dma_blocking(drv, data, len, timeout_ms)` | Block the calling task until DMA transmit completes or timeout elapses. |
| `uart_send_blocking(drv, data, len, timeout_ms)` | Alias for `uart_send_dma_blocking`. |
| `uart_start_dma_tx(drv, data, len)` | Raw DMA transmit without the queue (caller must manage buffer lifetime). |

### Receive (Circular DMA)

| Function | Description |
|----------|-------------|
| `uart_start_circular_rx(drv, buf, len)` | Configure circular DMA RX. `buf` must remain valid for the lifetime of the driver. |
| `uart_read_from_circular(drv, dest, maxlen)` | Copy up to `maxlen` bytes from the circular buffer. Returns bytes copied. |
| `uart_bytes_available(drv)` | Bytes waiting in the circular buffer. |

### Diagnostics

| Function | Description |
|----------|-------------|
| `uart_get_status(drv)` | Combined TX/RX status (`UART_OK`, `UART_BUSY`, `UART_ERROR`). |
| `uart_flush_rx(drv)` | Clear UART overrun error flag. |
| `uart_flush_tx(drv)` | Clear TX complete flag. |
| `uart_get_tx_queue_count(drv)` | Pending TX nodes in the queue. |
| `uart_get_tx_pool_free_count(drv)` | Free TX pool nodes remaining. |
| `uart_get_tx_fallback_count(drv)` | Times TX pool allocation failed (pool exhaustion counter). |

### ISR Entry Point

```c
/* Call from your USARTx_IRQHandler, before HAL_UART_IRQHandler */
void uart_process_idle(UART_HandleTypeDef *hu);
```

---

## Optional Modules

All optional modules default to **enabled**. Override before including headers or via compiler flags to exclude them.

### Command Interpreter (`USE_CMD_INTERPRETER`)

Provides a CLI over the UART: line editing, tokenization, and command dispatch. Built-in commands: `help`, `echo`, `add`, `faults`, `fault_clear`, `log_level`.

```c
/* Add your own commands in src/commands.c by extending cmd_list[] */
const Command cmd_list[] = {
    { "help",        cmd_help        },
    { "echo",        cmd_echo        },
    { "my_command",  my_cmd_handler  },  /* add here */
};
const size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);
```

Configuration macros (set before including `command_module.h`):

| Macro | Default | Description |
|-------|---------|-------------|
| `USE_CMD_INTERPRETER` | `1` | Enable/disable the module entirely |
| `CMD_MAX_LINE_LEN` | `128` | Maximum input line length in bytes |
| `CMD_MAX_PARAMS` | `8` | Maximum tokenized arguments per command |
| `CMD_TASK_PRIO` | `osPriorityNormal` | CMSIS task priority |
| `CMD_TASK_STACK` | `512` | Task stack in bytes |

### Logging (`LOGGING_ENABLED`)

Queue-based structured logging with severity levels. A background task drains the log queue and transmits over the driver UART.

```c
log_write(LOG_LEVEL_INFO,  "System started, heap free: %u", xPortGetFreeHeapSize());
log_write(LOG_LEVEL_ERROR, "DMA fault: status=0x%02X", status);
log_set_level(LOG_LEVEL_WARN);   /* suppress DEBUG and INFO */
```

Configuration macros:

| Macro | Default | Description |
|-------|---------|-------------|
| `LOGGING_ENABLED` | `1` | Enable/disable the module |
| `LOG_QUEUE_DEPTH` | `32` | Log entry queue depth |
| `TELEMETRY_QUEUE_DEPTH` | `16` | Telemetry packet queue depth |
| `MAX_LOG_PAYLOAD` | `128` | Maximum formatted message length |
| `DEFAULT_LOG_LEVEL` | `LOG_LEVEL_INFO` | Initial filter level |
| `LOG_TASK_STACK` | `1024` | Logger task stack in bytes |
| `LOG_TASK_PRIO` | `osPriorityNormal` | Logger task priority |

### Fault Management

Bit-mask fault state with timestamps. Integrates with the logging module to emit a periodic fault status line when any fault is active.

```c
fault_raise(FAULT_OVERCURRENT);
if (fault_is_active(FAULT_OVERCURRENT)) { /* ... */ }
fault_clear(FAULT_OVERCURRENT);
fault_clear_all();
```

---

## Configuration Reference

The most commonly overridden macros in `uart_driver.h`:

| Macro | Default | Description |
|-------|---------|-------------|
| `UART_TX_NODE_SIZE` | `256` | Maximum bytes per non-blocking TX request |
| `UART_TX_NODE_COUNT` | `16` | TX pool and queue depth (static allocation) |
| `UART_RX_NODE_COUNT` | `16` | Reserved for future use |
| `UART_RTOS_USE_CMSIS` | `1` | Use CMSIS-RTOS v2 adapter (set `0` to supply your own) |
| `UART_STM32_HAL_HEADER` | `"stm32f4xx_hal.h"` | HAL umbrella header; override for other STM32 series |
| `FAULT_ENTER_CRITICAL` | `__disable_irq()` | Critical section entry; replace if using BASEPRI masking |
| `FAULT_EXIT_CRITICAL` | `__enable_irq()` | Critical section exit |

Override at the compiler command line (preferred, avoids editing the driver source):

```cmake
target_compile_definitions(${PROJECT_NAME} PRIVATE
    UART_TX_NODE_SIZE=512
    UART_TX_NODE_COUNT=8
    UART_STM32_HAL_HEADER="stm32h7xx_hal.h"
)
```

---

## Running Host Tests

The test suite runs on macOS and Linux with any C11-capable host compiler. No STM32 toolchain or hardware required.

```bash
cmake -S . -B build-host \
    -DUART_DRIVER_BUILD_EMBEDDED_LIB=OFF \
    -DUART_DRIVER_BUILD_TESTS=ON

cmake --build build-host

ctest --test-dir build-host --output-on-failure
```

Expected output:

```
[UNITY] Suite: test_uart_deterministic_queue
[UNITY] Tests run: 3  Tests failed: 0

[UNITY] Suite: test_uart_driver_determinism
[UNITY] Tests run: 3  Tests failed: 0

100% tests passed, 0 tests failed out of 2
```

Test coverage:

| Suite | Tests |
|-------|-------|
| `test_uart_deterministic_queue` | FIFO order, wrap-around, full/empty boundary, invalid input guards |
| `test_uart_driver_determinism` | Blocking DMA timeout determinism, non-blocking TX FIFO drain order, IDLE event routing |

---

## Memory Footprint

Approximate figures for an STM32F446 at `-Os`, with full driver + logging + command interpreter:

| Region | Size |
|--------|------|
| Flash (driver + modules) | ~12 KB |
| RAM — static pool per driver instance (`UART_TX_NODE_COUNT × (UART_TX_NODE_SIZE + 8)`) | ~4.2 KB at defaults |
| RAM — log queue (`LOG_QUEUE_DEPTH × sizeof(LogEntry)`) | ~4.7 KB at defaults |
| FreeRTOS heap (mutexes + queues + task TCBs) | ~6–8 KB |

To reduce RAM, lower `UART_TX_NODE_COUNT`, `UART_TX_NODE_SIZE`, and `LOG_QUEUE_DEPTH`.

---

## Troubleshooting

### Hard fault on startup

- Ensure `uart_init` is called **after** `osKernelStart()`. Calling it before the scheduler is running causes `osMutexNew` to fail.
- Verify `TOTAL_HEAP_SIZE` is large enough (see [CubeMX Setup](#cubemx-setup-step-by-step) table).

### No data received in circular buffer

- Confirm the RX DMA stream is in **Circular** mode in CubeMX (DMA Settings → Mode = Circular).
- Confirm `uart_process_idle` is called from `USARTx_IRQHandler` **before** `HAL_UART_IRQHandler`.
- Check UART global interrupt and RX DMA stream interrupt are enabled in NVIC.

### TX pool exhaustion (`uart_send_nb` returns `UART_BUSY`)

- Monitor `uart_get_tx_fallback_count` and `uart_get_tx_pool_free_count` during development.
- Increase `UART_TX_NODE_COUNT` or reduce transmit frequency.
- Ensure the logging module is not flooding the TX queue — lower `DEFAULT_LOG_LEVEL` in production builds.

### `UART_BUSY` returned from `uart_send_dma_blocking`

- Another task holds the TX mutex. Increase `timeout_ms` or reduce contention.
- Check that `HAL_UART_TxCpltCallback` is being called (UART IRQ is enabled and handler calls `HAL_UART_IRQHandler`).

### Incorrect log timestamps (seconds too slow or fast)

- Verify your FreeRTOS tick rate (`configTICK_RATE_HZ`) matches `osKernelGetTickFreq()`. The driver uses the RTOS API directly and is not sensitive to the specific tick frequency.

### Build fails: `stm32f4xx_hal.h` not found

- The embedded library target does not bundle HAL headers. Your project must add the HAL include path. CubeMX projects do this automatically via the generated `CMakeLists.txt`. If using a manual build, add:
  ```cmake
  target_include_directories(stm32_uart_driver_plus PUBLIC
      path/to/STM32F4xx_HAL_Driver/Inc
      path/to/CMSIS/Include
      path/to/CMSIS/Device/ST/STM32F4xx/Include
  )
  ```
  Or override the header macro as described in [Adding the Driver](#adding-the-driver-to-your-cmake-project).

---

## License

MIT License — see [LICENSE](LICENSE) for details.
