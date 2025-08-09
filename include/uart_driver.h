/**
 * @file uart_driver.h
 * @brief Thread-safe UART driver abstraction for STM32.
 *
 * This module provides blocking and non-blocking APIs for UART
 * transmission and reception using either interrupts or DMA.
 * All functions are safe to call from multiple FreeRTOS tasks.
 * 
 * Supports both STM32 HAL and LL (Low Layer) drivers with CMSIS-RTOS integration.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "uart_driver_config.h"
#include "uart_driver_abstraction.h"

#if USE_FREERTOS
  #include "FreeRTOS.h"
  #include "semphr.h"
  #if USE_CMSIS_RTOS
    #include "cmsis_os.h"
  #endif
#endif

#if USE_STM32_LL_DRIVERS
  #include "stm32f4xx_ll_usart.h"
  #include "stm32f4xx_ll_dma.h"
#else
  #include "stm32f4xx_hal.h"   // adjust to your STM32 series HAL header
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Opaque driver handle
typedef struct uart_drv_handle_s uart_drv_t;

// Return/status codes
typedef enum {
    UART_OK = 0,
    UART_BUSY,
    UART_ERROR
} uart_status_t;

// Events for callbacks
typedef enum {
    UART_EVT_TX_COMPLETE,
    UART_EVT_RX_COMPLETE,
    UART_EVT_ERROR
} uart_event_t;

// Callback prototype
typedef void (*uart_callback_t)(uart_event_t evt, void *user_ctx);

/**
 * @brief Initialize a UART driver instance.
 *
 * For HAL mode: Caller MUST have already initialized the underlying HAL UART peripheral.
 * For LL mode: Caller MUST provide valid LL UART instance and configure it appropriately.
 */
#if USE_STM32_LL_DRIVERS
uart_status_t uart_init_ll(uart_drv_t *drv,
                           USART_TypeDef *uart_instance,
                           DMA_TypeDef   *dma_tx_instance,
                           DMA_TypeDef   *dma_rx_instance,
                           uint32_t      dma_tx_stream,
                           uint32_t      dma_rx_stream);
#else
uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
                        DMA_HandleTypeDef  *hdma_tx,
                        DMA_HandleTypeDef  *hdma_rx);
#endif

/**
 * @brief Convenience initializer that also sets up optional modules.
 *
 * This will invoke the appropriate uart_init function and then call cmd_init and
 * log_init automatically when the corresponding feature macros
 * are enabled in `uart_driver_config.h`.
 */
#if USE_STM32_LL_DRIVERS
uart_status_t uart_system_init_ll(uart_drv_t *drv,
                                  USART_TypeDef *uart_instance,
                                  DMA_TypeDef   *dma_tx_instance,
                                  DMA_TypeDef   *dma_rx_instance,
                                  uint32_t      dma_tx_stream,
                                  uint32_t      dma_rx_stream);
#else
uart_status_t uart_system_init(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx);
#endif

/**
 * @brief Deinitialize a driver instance (frees its mutexes).
 */
void uart_deinit(uart_drv_t *drv);

/**
 * @brief Swap in a different UART/DMA handle set at runtime.
 */
#if USE_STM32_LL_DRIVERS
uart_status_t uart_reconfigure_ll(uart_drv_t *drv,
                                  USART_TypeDef *uart_instance,
                                  DMA_TypeDef   *dma_tx_instance,
                                  DMA_TypeDef   *dma_rx_instance,
                                  uint32_t      dma_tx_stream,
                                  uint32_t      dma_rx_stream);
#else
uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx);
#endif

/**
 * @brief Transmit data in a blocking manner.
 *
 * This function grabs the TX mutex and calls HAL_UART_Transmit().
 * It will block the calling task for up to @p timeout_ms.
 */
uart_status_t uart_send_blocking   (uart_drv_t *drv, uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Receive data in a blocking manner.
 *
 * Similar to uart_send_blocking() but for reception.
 */
uart_status_t uart_receive_blocking(uart_drv_t *drv, uint8_t *buf,  size_t len, uint32_t timeout_ms);

/** @brief Begin an interrupt-driven transmit. */
uart_status_t uart_send_nb   (uart_drv_t *drv, uint8_t *data, size_t len);
/** @brief Begin an interrupt-driven receive. */
uart_status_t uart_receive_nb(uart_drv_t *drv, uint8_t *buf,  size_t len);

/** @brief Begin a DMA based transmit. */
uart_status_t uart_start_dma_tx(uart_drv_t *drv, uint8_t *data, size_t len);
/** @brief Begin a DMA based receive. */
uart_status_t uart_start_dma_rx(uart_drv_t *drv, uint8_t *buf,  size_t len);
/** @brief Blocking transmit using DMA when available. */
uart_status_t uart_send_dma_blocking(uart_drv_t *drv, uint8_t *data,
                                     size_t len, uint32_t timeout_ms);

/** Return number of bytes remaining in DMA RX buffer (0 if not using DMA). */
size_t        uart_bytes_available(uart_drv_t *drv);
/** Clear any RX error flags. */
void          uart_flush_rx        (uart_drv_t *drv);
/** Clear TX completion flag. */
void          uart_flush_tx        (uart_drv_t *drv);
/** Retrieve last operation status. */
uart_status_t uart_get_status      (uart_drv_t *drv);

/**
 * @brief Internal state structure for a UART instance.
 */
struct uart_drv_handle_s {
    /* Abstraction layer handle - works with both HAL and LL */
    uart_abstraction_handle_t abstraction;
    
    /* Legacy HAL interface (for backward compatibility) */
#if !USE_STM32_LL_DRIVERS
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;
#endif

    /* FreeRTOS synchronization (when enabled) */
#if USE_FREERTOS
    SemaphoreHandle_t   tx_mutex;
    SemaphoreHandle_t   rx_mutex;
#endif /* USE_FREERTOS */
    
    /* Status and callback management */
    uart_callback_t     cb;
    void               *ctx;
    volatile uart_status_t status;
};

/** Register a callback invoked from ISR context on UART events. */
void uart_register_callback(uart_drv_t *drv, uart_callback_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H
