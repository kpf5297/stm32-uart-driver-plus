# Configuration Guide for CMSIS/FreeRTOS/LL Support

This guide helps you quickly configure your STM32 UART Driver Plus for different use cases.

## Configuration Matrix

| Use Case | HAL/LL | FreeRTOS | CMSIS-RTOS | Config Settings |
|----------|--------|----------|------------|----------------|
| **Bare Metal HAL** | HAL | ❌ | ❌ | `USE_STM32_LL_DRIVERS=0`, `USE_FREERTOS=0` |
| **Bare Metal LL** | LL | ❌ | ❌ | `USE_STM32_LL_DRIVERS=1`, `USE_FREERTOS=0` |
| **FreeRTOS HAL** | HAL | ✅ | ❌ | `USE_STM32_LL_DRIVERS=0`, `USE_FREERTOS=1`, `USE_CMSIS_RTOS=0` |
| **FreeRTOS LL** | LL | ✅ | ❌ | `USE_STM32_LL_DRIVERS=1`, `USE_FREERTOS=1`, `USE_CMSIS_RTOS=0` |
| **CMSIS-RTOS HAL** | HAL | ✅ | ✅ | `USE_STM32_LL_DRIVERS=0`, `USE_FREERTOS=1`, `USE_CMSIS_RTOS=1` |
| **CMSIS-RTOS LL** | LL | ✅ | ✅ | `USE_STM32_LL_DRIVERS=1`, `USE_FREERTOS=1`, `USE_CMSIS_RTOS=1` |

## Quick Configuration Steps

### Step 1: Edit `uart_driver_config.h`

```c
/* Choose your configuration by setting these defines: */

/* Driver Layer Selection (choose one) */
#define USE_STM32_LL_DRIVERS  0    // 0 = HAL, 1 = LL

/* RTOS Configuration */
#define USE_FREERTOS          1    // 0 = bare metal, 1 = FreeRTOS
#define USE_CMSIS_RTOS        0    // 0 = direct FreeRTOS, 1 = CMSIS-RTOS wrapper
```

### Step 2: Include Appropriate Headers

#### For HAL Configuration:
```c
#include "stm32f4xx_hal.h"          // Adjust for your STM32 series
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"
```

#### For LL Configuration:
```c
#include "stm32f4xx_ll_usart.h"     // Adjust for your STM32 series
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
```

#### For FreeRTOS:
```c
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
```

#### For CMSIS-RTOS:
```c
#include "cmsis_os.h"               // CMSIS-RTOS v1
// or
#include "cmsis_os2.h"              // CMSIS-RTOS v2
```

### Step 3: Initialization Examples

#### HAL + FreeRTOS Example:
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
    while(1) {
        if (uart_receive_blocking(&uart_driver, buffer, 1, 100) == UART_OK) {
            uart_send_blocking(&uart_driver, buffer, 1, 100);
        }
    }
}
```

#### LL + FreeRTOS Example:
```c
// 1. Configure LL peripherals manually
void configure_ll_peripherals(void) {
    // GPIO configuration
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinAlternateFunction(GPIOA, LL_GPIO_PIN_2, LL_GPIO_AF_7);
    // ... more GPIO config ...
    
    // UART configuration
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_USART_SetBaudRate(USART2, 84000000, LL_USART_OVERSAMPLING_16, 115200);
    LL_USART_Enable(USART2);
}

// 2. Initialize UART driver
uart_drv_t uart_driver;
uart_status_t result = uart_init_ll(&uart_driver, USART2, DMA1, DMA1, 
                                    LL_DMA_STREAM_6, LL_DMA_STREAM_5);

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
    
    while(1) {
        if (uart_receive_blocking(uart, buffer, 1, 100) == UART_OK) {
            uart_send_blocking(uart, buffer, 1, 100);
        }
        osDelay(1);  // CMSIS-RTOS delay instead of vTaskDelay
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

### Memory Usage (approximate, STM32F4):

| Configuration | Flash (KB) | RAM (KB) | Notes |
|---------------|------------|----------|-------|
| Bare Metal HAL | 8-12 | 1-2 | Minimal overhead |
| Bare Metal LL | 4-6 | 0.5-1 | Smallest footprint |
| FreeRTOS HAL | 15-20 | 4-8 | Includes RTOS overhead |
| FreeRTOS LL | 12-16 | 3-6 | Better than HAL |
| CMSIS-RTOS HAL | 16-22 | 5-9 | Additional wrapper overhead |
| CMSIS-RTOS LL | 13-18 | 4-7 | Still better than HAL |

### Execution Speed (relative):

| Configuration | TX Speed | RX Speed | Interrupt Latency |
|---------------|----------|----------|-------------------|
| Bare Metal HAL | 100% | 100% | 100% |
| Bare Metal LL | 120-150% | 120-150% | 80-90% |
| FreeRTOS HAL | 85-95% | 85-95% | 110-120% |
| FreeRTOS LL | 100-120% | 100-120% | 90-100% |

## Migration Path

### From HAL to LL:
1. Change `USE_STM32_LL_DRIVERS` to `1`
2. Replace HAL includes with LL includes
3. Rewrite peripheral initialization code to use LL APIs
4. Update interrupt handlers to use LL flag checking
5. Change from `uart_init()` to `uart_init_ll()`

### From Bare Metal to FreeRTOS:
1. Add FreeRTOS to your project
2. Change `USE_FREERTOS` to `1`
3. Replace polling loops with FreeRTOS tasks
4. Use blocking APIs instead of non-blocking where appropriate
5. Configure FreeRTOS heap and stack sizes appropriately

### From FreeRTOS to CMSIS-RTOS:
1. Add CMSIS-RTOS to your project
2. Change `USE_CMSIS_RTOS` to `1`
3. Replace FreeRTOS API calls with CMSIS-RTOS equivalents:
   - `xTaskCreate()` → `osThreadNew()`
   - `vTaskDelay()` → `osDelay()`
   - `xSemaphoreCreateMutex()` → `osMutexNew()`

## Troubleshooting

### Common Issues:

1. **Compile errors about missing headers:**
   - Check that you have the correct STM32 HAL/LL libraries in your project
   - Verify STM32CubeMX configuration includes required peripherals

2. **Linker errors about undefined symbols:**
   - Ensure FreeRTOS is properly integrated if `USE_FREERTOS=1`
   - Check that all required source files are included in build

3. **Runtime crashes or hard faults:**
   - Verify FreeRTOS heap size is sufficient (check `configTOTAL_HEAP_SIZE`)
   - Ensure interrupt priorities are configured correctly for FreeRTOS
   - Check that stack sizes for tasks are adequate

4. **Poor performance with LL drivers:**
   - Verify clock configuration is optimal
   - Check that LL optimizations are enabled in compiler settings
   - Ensure interrupt handlers are efficient

### Debug Tips:

1. **Enable assertions** in FreeRTOS for development builds
2. **Use RTT or SWO** for debugging instead of UART when testing UART code
3. **Monitor stack usage** in FreeRTOS tasks
4. **Profile interrupt frequency** to ensure system stability

## Advanced Configurations

### Custom RTOS Integration:
To integrate with other RTOS (not FreeRTOS/CMSIS-RTOS):

1. Set `USE_FREERTOS=0`
2. Implement your own mutex/semaphore wrappers in `uart_driver_config.h`
3. Define custom tick and critical section macros

### Mixed HAL/LL Usage:
You can use HAL for some peripherals and LL for others:

1. Configure only UART driver to use LL: modify abstraction layer
2. Keep other peripherals using HAL
3. Be careful about clock configuration consistency

### Real-time Applications:
For hard real-time requirements:

1. Use LL drivers for deterministic timing
2. Set appropriate FreeRTOS task priorities
3. Consider using static memory allocation
4. Minimize interrupt disable time in critical sections
