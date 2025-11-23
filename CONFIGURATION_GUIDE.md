# Configuration Guide for CMSIS/FreeRTOS/LL Support

This guide helps you quickly configure your STM32 UART Driver Plus for different use cases.

## Configuration Matrix

| Use Case | HAL/LL | FreeRTOS | CMSIS-RTOS | Config Settings |
|----------|--------|----------|------------|----------------|
| **Bare Metal HAL** | HAL | ❌ | ❌ | `USE_FREERTOS=0` |
| **FreeRTOS HAL** | HAL | ✅ | ❌ | Deprecated - use CMSIS-RTOS v2 wrapper |
| **CMSIS-RTOS HAL** | HAL | ✅ | ✅ | Use CMSIS-RTOS v2 (no macros required) |
| **DMA-only HAL** | HAL | ✅ | ✅ | This repo enforces DMA-only for performance and predictability |

## Quick Configuration Steps

### Step 1: Edit module headers (`include/uart_driver.h`, `include/logging.h`, `include/command_module.h`)

```c
/* Choose your configuration by setting these defines: */
/* RTOS Configuration */
/* CMSIS-RTOS v2 is required for RTOS features; this repo is DMA-only */
```

### Step 2: Include Appropriate Headers

#### For HAL Configuration

```c
#include "stm32f4xx_hal.h"          // Adjust for your STM32 series
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"
```

#### For HAL Configuration

```c
#include "stm32f4xx_hal.h"          // Adjust for your STM32 series
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"
```

#### For FreeRTOS

```c
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
```

#### For CMSIS-RTOS

```c
#include "cmsis_os.h"               // CMSIS-RTOS v1
// or
#include "cmsis_os2.h"              // CMSIS-RTOS v2
```

### Step 3: Initialization Examples

#### HAL + FreeRTOS Example

```c
// 1. Configure HAL peripherals (usually done by CubeMX)
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

// 2. Initialize UART driver
uart_drv_t uart_driver;
uart_status_t result = uart_init(&uart_driver, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);

// 3. Use in FreeRTOS tasks
void uart_task(void *pvParameters) {
    uint8_t buffer[64];
    // Circular DMA RX: polling/read loop
    uint8_t circ_storage[64];
    uart_start_circular_rx(&uart_driver, circ_storage, sizeof(circ_storage));
    while(1) {
        size_t available = uart_bytes_available(&uart_driver);
        if (available) {
            uint8_t b;
            size_t n = uart_read_from_circular(&uart_driver, &b, 1);
            if (n) {
                uart_send_blocking(&uart_driver, &b, 1, 100);
            }
        }
        osDelay(1);
    }
}
```

_(LL configurations and examples removed; the repository supports HAL-only. See `examples/freertos_example/` for HAL+RTOS examples)_
// 3. Use same API as HAL version
void uart_task(void *pvParameters) {
    uint8_t buffer[64];
    while(1) {
        if (uart_receive_blocking(&uart_driver, buffer, 1, 100) == UART_OK) {
            uart_send_blocking(&uart_driver, buffer, 1, 100);
        }
    }
}

```

#### CMSIS-RTOS Example:
```c
// CMSIS-RTOS v2 thread
void uart_thread(void *argument) {
    uart_drv_t *uart = (uart_drv_t*)argument;
    uint8_t buffer[64];
    
    // In CMSIS-RTOS use circular DMA and event-driven or polling reads.
    uint8_t circ_storage[64];
    uart_start_circular_rx(uart, circ_storage, sizeof(circ_storage));
    while(1) {
        size_t available = uart_bytes_available(uart);
        if (available) {
            uint8_t b;
            if (uart_read_from_circular(uart, &b, 1)) {
                uart_send_blocking(uart, &b, 1, 100);
            }
        }
        osDelay(1);
    }
}

// Create thread
osThreadId_t uart_thread_id;
const osThreadAttr_t uart_thread_attr = {
    .name = "uart_thread",
    .stack_size = 512,
    .priority = osPriorityNormal,
};
uart_thread_id = osThreadNew(uart_thread, &uart_driver, &uart_thread_attr);
```

## Performance Comparison

### Memory Usage (approximate, STM32F4)

| Configuration | Flash (KB) | RAM (KB) | Notes |
|---------------|------------|----------|-------|
| Bare Metal HAL | 8-12 | 1-2 | Minimal overhead |
| FreeRTOS HAL | 15-20 | 4-8 | Includes RTOS overhead |
| CMSIS-RTOS HAL | 16-22 | 5-9 | Additional wrapper overhead |
  
### Execution Speed (relative)

| Configuration | TX Speed | RX Speed | Interrupt Latency |
|---------------|----------|----------|-------------------|
| Bare Metal HAL | 100% | 100% | 100% |
| FreeRTOS HAL | 85-95% | 85-95% | 110-120% |
  
## Migration Path

_(LL mode support and migration instructions removed — this repo is HAL-only.)_

### From Bare Metal to FreeRTOS

1. Add FreeRTOS to your project
2. Change `USE_FREERTOS` to `1`
3. Replace polling loops with FreeRTOS tasks
4. Use blocking APIs instead of non-blocking where appropriate
5. Configure FreeRTOS heap and stack sizes appropriately

### From FreeRTOS to CMSIS-RTOS

1. Add CMSIS-RTOS to your project
2. Change `USE_CMSIS_RTOS` to `1`
3. Replace FreeRTOS API calls with CMSIS-RTOS equivalents:
   - `xTaskCreate()` → `osThreadNew()`
   - `vTaskDelay()` → `osDelay()`
   - `xSemaphoreCreateMutex()` → `osMutexNew()`

## Troubleshooting

### Common Issues

1. **Compile errors about missing headers:**
   - Check that you have the correct STM32 HAL/LL libraries in your project
   - Verify STM32CubeMX configuration includes required peripherals

2. **Linker errors about undefined symbols:**
   - Ensure FreeRTOS is properly integrated if `USE_FREERTOS=1`
   - Check that all required source files are included in build

3. **Runtime crashes or hard faults:**
   - Verify FreeRTOS heap size is sufficient (check `configTOTAL_HEAP_SIZE`)
   - Linker file needs to match your memory layout:

    ``` c
        //Increase heap to accommodate RTOS dynamic allocations: message queues, TCBs, and buffers
        _Min_Heap_Size = 0x4000;   // required amount of heap  (16 KB)
        // Increase minimum stack to allow nested interrupts and larger stacks for main thread
        _Min_Stack_Size = 0x800; // required amount of stack (2 KB) 
    ```

   - Ensure interrupt priorities are configured correctly for FreeRTOS
   - Check that stack sizes for tasks are adequate

1. **Performance Tips:**
    - Verify clock configuration is optimal
    - Check that compiler optimizations are enabled
    - Ensure interrupt handlers are efficient

### Debug Tips

1. **Enable assertions** in FreeRTOS for development builds
2. **Use RTT or SWO** for debugging instead of UART when testing UART code
3. **Monitor stack usage** in FreeRTOS tasks
4. **Profile interrupt frequency** to ensure system stability

## Advanced Configurations

### Custom RTOS Integration

To integrate with other RTOS (not FreeRTOS/CMSIS-RTOS):

1. Set `USE_FREERTOS=0`
2. Implement your own mutex/semaphore wrappers in your application or in `include/uart_driver.h` if you want to override defaults.
3. Define custom tick and critical section macros

### Mixed HAL usage

This repository and the driver are HAL-only. If you need to mix HAL with other approaches, be sure to keep clock configuration and IRQ handling consistent.

### Real-time Applications

For hard real-time requirements:

1. Choose appropriate HAL/DMA configurations and avoid blocking in high-priority interrupts
2. Set appropriate FreeRTOS task priorities
3. Consider using static memory allocation when possible
4. Minimize interrupt disable time in critical sections
