# STM32 UART Driver Plus - CMSIS/FreeRTOS/HAL Guide

This document explains how to use the STM32 UART Driver Plus library with the STM32 HAL driver and RTOS configurations (FreeRTOS/CMSIS-RTOS v1/v2).

## Supported Configuration

- **HAL (Hardware Abstraction Layer)** - Default, high-level API (Only HAL is supported in this repository)

## RTOS Support  
- **FreeRTOS** - Full task-safe operation with semaphores and mutexes
- **CMSIS-RTOS v1/v2** - Optional CMSIS wrapper over FreeRTOS
- **Bare Metal** - No RTOS, polling/interrupt-based operation

## Configuration

### Edit `include/uart_driver_config.h`

Set these macros to match your configuration:

```c
/* RTOS Configuration */
#define USE_FREERTOS          1    // 0 = bare metal, 1 = FreeRTOS
#define USE_CMSIS_RTOS        1    // 0 = direct FreeRTOS, 1 = CMSIS-RTOS wrapper
```

### HAL Configuration (Default)

For HAL mode, include these headers in your project:
```c
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"
```

Usage example:
```c
#include "uart_driver.h"

// Initialize HAL UART and DMA first
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

// Setup your HAL peripherals... (e.g., CubeMX generated init code)
MX_USART2_UART_Init();
MX_DMA_Init();

// Initialize UART driver
uart_drv_t uart_driver;
uart_status_t result = uart_init(&uart_driver, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);
```

## FreeRTOS Integration

See `examples/freertos_example/` for a working HAL-based project with FreeRTOS and CMSIS-RTOS support.

## Migration & Best Practices

This repository uses HAL only (LL support removed). If you are migrating from LL, convert peripheral initialization code to HAL equivalents and use `uart_init()` instead of any LL-specific init calls.

**Best practices** remain the same: use DMA for high throughput, ensure correct interrupt priorities, and use mutexes for multi-task access.
