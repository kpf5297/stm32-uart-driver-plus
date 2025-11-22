/**
 * @file uart_driver.h
 * @brief Thread-safe UART driver abstraction for STM32.
 *
 * This module provides blocking and non-blocking APIs for UART
 * transmission and reception using either interrupts or DMA.
 * All functions are safe to call from multiple FreeRTOS tasks.
 * 
 * Uses STM32 HAL drivers (LL support has been removed) with CMSIS-RTOS integration.
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

  #include "stm32f4xx_hal.h"   // adjust to your STM32 series HAL header

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
 * Caller MUST have initialized the underlying HAL UART peripheral and optional DMA handles.
 */
uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
                        DMA_HandleTypeDef  *hdma_tx,
                        DMA_HandleTypeDef  *hdma_rx);

/**
 * @brief Convenience initializer that also sets up optional modules.
 *
 * This will invoke the appropriate uart_init function and then call cmd_init and
 * log_init automatically when the corresponding feature macros
 * are enabled in `uart_driver_config.h`.
 */
uart_status_t uart_system_init(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx);

/**
 * @brief Deinitialize a driver instance (frees its mutexes).
 */
void uart_deinit(uart_drv_t *drv);

/**
 * @brief Swap in a different UART/DMA handle set at runtime.
 */
uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx);

/**
 * @brief Transmit data in a blocking manner.
 *
 * This function grabs the TX mutex and calls HAL_UART_Transmit().
 * It will block the calling task for up to @p timeout_ms.
 */
uart_status_t uart_send_blocking   (uart_drv_t *drv, const uint8_t *data, size_t len, uint32_t timeout_ms);

/**
 * @brief Receive data in a blocking manner.
 *
 * Similar to uart_send_blocking() but for reception.
 */
uart_status_t uart_receive_blocking(uart_drv_t *drv, uint8_t *buf,  size_t len, uint32_t timeout_ms);

/** @brief Begin an interrupt-driven transmit. */
uart_status_t uart_send_nb   (uart_drv_t *drv, const uint8_t *data, size_t len);
/** @brief Begin an interrupt-driven receive. */
uart_status_t uart_receive_nb(uart_drv_t *drv, uint8_t *buf,  size_t len);

/** @brief Begin a DMA based transmit. */
uart_status_t uart_start_dma_tx(uart_drv_t *drv, const uint8_t *data, size_t len);
/** @brief Begin a DMA based receive. */
uart_status_t uart_start_dma_rx(uart_drv_t *drv, uint8_t *buf,  size_t len);
/** @brief Blocking transmit using DMA when available. */
uart_status_t uart_send_dma_blocking(uart_drv_t *drv, const uint8_t *data,
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
    /* Abstraction layer handle (HAL-only) */
    uart_abstraction_handle_t abstraction;
    
    /* Legacy HAL interface (for backward compatibility) */
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;

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
