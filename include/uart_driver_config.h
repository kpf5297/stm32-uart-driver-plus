#ifndef UART_DRIVER_CONFIG_H
#define UART_DRIVER_CONFIG_H

/* Configuration options for UART driver modules */

/*******************************************************************************
 * STM32 Driver Layer Selection
 ******************************************************************************/
/* Choose which STM32 driver layer to use:
 * 0 = HAL (Hardware Abstraction Layer)
 * 1 = LL (Low Layer) 
 */
#define USE_STM32_LL_DRIVERS  0

/* FreeRTOS/CMSIS-RTOS Support */
#define USE_FREERTOS          1
#define USE_CMSIS_RTOS        1

/* Global UART transmission mode */
#define UART_MODE_BLOCKING   0
#define UART_MODE_INTERRUPT  1
#define UART_MODE_DMA        2
/* Select one of the modes above to apply across all modules */
#define UART_TX_MODE         UART_MODE_BLOCKING

/* Feature enable macros */
#define USE_CMD_INTERPRETER  1
#define LOGGING_ENABLED      1
#define TELEMETRY_ENABLED    1

/* Command interpreter options */
#define CMD_MAX_LINE_LEN     128
#define CMD_MAX_PARAMS       8
#define CMD_TASK_PRIO        (tskIDLE_PRIORITY + 1)
#define CMD_TASK_STACK       256

/* Logging and telemetry options */
#define LOG_QUEUE_DEPTH      32
#define TELEMETRY_QUEUE_DEPTH 16
#define MAX_LOG_PAYLOAD      128
#define MAX_TELEMETRY_PAYLOAD 64
#define DEFAULT_LOG_LEVEL    LOG_LEVEL_INFO
#define LOG_TASK_STACK       512
#define LOG_TASK_PRIO        (tskIDLE_PRIORITY + 1)


/*******************************************************************************
 * Tick source & critical‐section abstraction
 ******************************************************************************/

#ifdef USE_FREERTOS
  #include "FreeRTOS.h"
  #include "task.h"
  #include "semphr.h"
  #ifdef USE_CMSIS_RTOS
    #include "cmsis_os.h"
  #endif
  #define TICKS_PER_SECOND       configTICK_RATE_HZ
  #define GET_TICKS()            xTaskGetTickCount()
  #define FAULT_ENTER_CRITICAL() taskENTER_CRITICAL()
  #define FAULT_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
  /* Bare metal or other RTOS */
  #if USE_STM32_LL_DRIVERS
    #include "stm32f4xx_ll_usart.h"
    #include "stm32f4xx_ll_dma.h"
    #include "stm32f4xx_ll_gpio.h"
    #include "stm32f4xx_ll_rcc.h"
  #else
    #include "stm32f4xx_hal.h"
  #endif
  #define TICKS_PER_SECOND       1000U
  #define GET_TICKS()            HAL_GetTick()
  #define FAULT_ENTER_CRITICAL() __disable_irq()
  #define FAULT_EXIT_CRITICAL()  __enable_irq()
#endif


#endif /* UART_DRIVER_CONFIG_H */
