/**
 * @file uart_driver.h
 * @brief Thread-safe UART driver abstraction for STM32.
 *
 * This module provides blocking and non-blocking APIs for UART
 * transmission and reception using either interrupts or DMA.
 * All functions are safe to call from multiple FreeRTOS tasks.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "uart_driver_config.h"
#if UART_BACKEND == UART_BACKEND_HAL
#include "stm32f4xx_hal.h"   // adjust to your STM32 series HAL header
#else
#include "cmsis_uart_adapter.h"
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
 * Caller MUST have already initialized the underlying hardware peripheral.
 * The specific requirements depend on the selected backend (HAL or CMSIS).
 */
#if UART_BACKEND == UART_BACKEND_HAL
uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
                        DMA_HandleTypeDef  *hdma_tx,
                        DMA_HandleTypeDef  *hdma_rx);
#else
uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
                        void              *unused_tx,
                        void              *unused_rx);
#endif

/**
 * @brief Deinitialize a driver instance (frees its mutexes).
 */
void uart_deinit(uart_drv_t *drv);

/**
 * @brief Swap in a different UART/DMA handle set at runtime.
 */
#if UART_BACKEND == UART_BACKEND_HAL
uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx);
#else
uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               void              *unused_tx,
                               void              *unused_rx);
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
    UART_HandleTypeDef *huart;
#if UART_BACKEND == UART_BACKEND_HAL
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;
#endif
    SemaphoreHandle_t   tx_mutex;
    SemaphoreHandle_t   rx_mutex;
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
