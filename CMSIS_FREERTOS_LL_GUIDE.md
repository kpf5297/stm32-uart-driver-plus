# STM32 UART Driver Plus - CMSIS/FreeRTOS/LL Compatibility Guide

This document explains how to use the STM32 UART Driver Plus library with different STM32 driver layers and RTOS configurations.

## Supported Configurations

### Driver Layer Support
- **HAL (Hardware Abstraction Layer)** - Default, high-level API
- **LL (Low Layer)** - Direct register access, more efficient, lower memory footprint

### RTOS Support  
- **FreeRTOS** - Full task-safe operation with semaphores and mutexes
- **CMSIS-RTOS v1/v2** - CMSIS wrapper over FreeRTOS
- **Bare Metal** - No RTOS, polling/interrupt based operation

## Configuration

### 1. Driver Layer Selection

Edit `include/uart_driver_config.h`:

```c
/* Choose driver layer:
 * 0 = HAL (Hardware Abstraction Layer) 
 * 1 = LL (Low Layer)
 */
#define USE_STM32_LL_DRIVERS  0    // Use HAL (default)
// #define USE_STM32_LL_DRIVERS  1 // Use LL for better performance

/* RTOS Configuration */
#define USE_FREERTOS          1    // Enable FreeRTOS support
#define USE_CMSIS_RTOS        1    // Enable CMSIS-RTOS wrapper
```

### 2. HAL Configuration (Default)

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

// Setup your HAL peripherals...
MX_USART2_UART_Init();
MX_DMA_Init();

// Initialize UART driver
uart_drv_t uart_driver;
uart_status_t result = uart_init(&uart_driver, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);
```

### 3. LL Configuration

For LL mode, include these headers:
```c
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
```

Usage example:
```c
#include "uart_driver.h"

// Configure LL UART peripheral
void MX_USART2_UART_Init_LL(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // Configure GPIO pins
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure UART
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(USART2, &USART_InitStruct);
    
    LL_USART_Enable(USART2);
}

// Initialize UART driver with LL
uart_drv_t uart_driver;
uart_status_t result = uart_init_ll(&uart_driver, USART2, DMA1, DMA1, 
                                    LL_DMA_STREAM_6, LL_DMA_STREAM_5);
```

## FreeRTOS Integration

### Basic FreeRTOS Usage

```c
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart_driver.h"

void uart_task(void *pvParameters)
{
    uart_drv_t *uart = (uart_drv_t*)pvParameters;
    uint8_t buffer[64];
    
    while(1) {
        // Thread-safe blocking receive
        uart_status_t status = uart_receive_blocking(uart, buffer, sizeof(buffer), 1000);
        if (status == UART_OK) {
            // Echo back
            uart_send_blocking(uart, buffer, strlen((char*)buffer), 1000);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Create task
xTaskCreate(uart_task, "UART", 256, &uart_driver, tskIDLE_PRIORITY + 1, NULL);
```

### CMSIS-RTOS v2 Usage

```c
#include "cmsis_os2.h"
#include "uart_driver.h"

osThreadId_t uart_thread_id;
const osThreadAttr_t uart_thread_attr = {
    .name = "uart_thread",
    .stack_size = 512,
    .priority = osPriorityNormal,
};

void uart_thread(void *argument)
{
    uart_drv_t *uart = (uart_drv_t*)argument;
    uint8_t buffer[64];
    
    while(1) {
        // CMSIS-RTOS compatible calls
        uart_status_t status = uart_receive_blocking(uart, buffer, sizeof(buffer), 1000);
        if (status == UART_OK) {
            uart_send_blocking(uart, buffer, strlen((char*)buffer), 1000);
        }
        osDelay(10);
    }
}

// Create thread
uart_thread_id = osThreadNew(uart_thread, &uart_driver, &uart_thread_attr);
```

## Performance Comparison

| Configuration | Memory Usage | Performance | Complexity |
|---------------|-------------|-------------|------------|
| HAL + FreeRTOS | High | Good | Low |
| LL + FreeRTOS | Medium | Excellent | Medium |
| HAL + Bare Metal | Medium | Fair | Low |
| LL + Bare Metal | Low | Excellent | High |

## Advanced Features

### Interrupt Callbacks with LL

```c
// LL mode supports direct interrupt handling
void USART2_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USART2)) {
        uint8_t data = LL_USART_ReceiveData8(USART2);
        // Handle received data
        handle_uart_rx_data(data);
    }
    
    if (LL_USART_IsActiveFlag_TC(USART2)) {
        LL_USART_ClearFlag_TC(USART2);
        // Handle transmission complete
        handle_uart_tx_complete();
    }
}
```

### DMA with LL

```c
// Configure DMA for LL UART
void configure_uart_dma_ll(void)
{
    LL_DMA_InitTypeDef DMA_InitStruct = {0};
    
    // Enable DMA clock
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    
    // Configure DMA for TX
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART2->DR;
    DMA_InitStruct.MemoryOrM2MDstAddress = 0; // Set dynamically
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
    DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_InitStruct.NbData = 0; // Set dynamically
    DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
    DMA_InitStruct.Priority = LL_DMA_PRIORITY_LOW;
    DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
    
    LL_DMA_Init(DMA1, LL_DMA_STREAM_6, &DMA_InitStruct);
}
```

## Migration Guide

### From HAL to LL

1. Change configuration: `#define USE_STM32_LL_DRIVERS 1`
2. Replace HAL headers with LL headers
3. Update initialization calls from `uart_init()` to `uart_init_ll()`
4. Configure LL peripherals manually instead of using CubeMX HAL code

### From Bare Metal to FreeRTOS

1. Enable FreeRTOS: `#define USE_FREERTOS 1`
2. Add FreeRTOS includes and initialization
3. Replace polling loops with FreeRTOS tasks
4. Use blocking APIs instead of non-blocking where appropriate

## Error Handling

```c
uart_status_t status = uart_send_blocking(&uart_driver, data, len, 1000);
switch(status) {
    case UART_OK:
        // Success
        break;
    case UART_BUSY:
        // UART is busy, try again later
        break;
    case UART_ERROR:
        // Hardware error, check connection
        uart_flush_rx(&uart_driver);
        uart_flush_tx(&uart_driver);
        break;
}
```

## Best Practices

1. **Choose LL for performance-critical applications** where memory and speed are important
2. **Use HAL for rapid prototyping** and when development speed is more important than performance
3. **Always enable FreeRTOS** when using multiple tasks that access UART
4. **Use DMA** for large data transfers to reduce CPU load
5. **Implement proper error handling** for robust applications
6. **Configure appropriate timeouts** based on your baud rate and data size

## Example Projects

See the `examples/` directory for complete working examples:
- `examples/freertos_example/` - FreeRTOS + HAL configuration
- `examples/freertos_ll_example/` - FreeRTOS + LL configuration (if available)
- `examples/bare_metal_example/` - Bare metal configuration (if available)
