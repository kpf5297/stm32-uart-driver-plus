/*
 * uart_driver.h
 *
 *  Created on: Jul 19, 2025
 *      Author: kevinfox
 */

// uart_driver.h

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
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
 * Caller MUST have already:
 *  • called HAL_UART_Init(huart)
 *  • (optional) called HAL_DMA_Init(hdma_tx) / HAL_DMA_Init(hdma_rx)
 *  • configured NVIC for USART and DMA interrupts
 *
 * @param drv      pointer to your uart_drv_t storage (must live as long as you use it)
 * @param huart    pointer to an already-initialized UART_HandleTypeDef
 * @param hdma_tx  optional pointer to an already-initialized DMA_HandleTypeDef for TX (or NULL)
 * @param hdma_rx  optional pointer to an already-initialized DMA_HandleTypeDef for RX (or NULL)
 * @return UART_OK on success, UART_ERROR on failure (e.g. out of mutexes or too many instances)
 */
uart_status_t uart_init(uart_drv_t *drv,
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

// Blocking APIs (take mutex, then HAL transmit/receive)
uart_status_t uart_send_blocking   (uart_drv_t *drv, uint8_t *data, size_t len, uint32_t timeout_ms);
uart_status_t uart_receive_blocking(uart_drv_t *drv, uint8_t *buf,  size_t len, uint32_t timeout_ms);

// Interrupt-driven, non-blocking
uart_status_t uart_send_nb   (uart_drv_t *drv, uint8_t *data, size_t len);
uart_status_t uart_receive_nb(uart_drv_t *drv, uint8_t *buf,  size_t len);

// DMA-driven
uart_status_t uart_start_dma_tx(uart_drv_t *drv, uint8_t *data, size_t len);
uart_status_t uart_start_dma_rx(uart_drv_t *drv, uint8_t *buf,  size_t len);

// Buffer/status management
size_t        uart_bytes_available(uart_drv_t *drv);
void          uart_flush_rx        (uart_drv_t *drv);
void          uart_flush_tx        (uart_drv_t *drv);
uart_status_t uart_get_status      (uart_drv_t *drv);

struct uart_drv_handle_s {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;
    SemaphoreHandle_t   tx_mutex;
    SemaphoreHandle_t   rx_mutex;
    uart_callback_t     cb;
    void               *ctx;
    volatile uart_status_t status;
};

// Event registration
void uart_register_callback(uart_drv_t *drv, uart_callback_t cb, void *user_ctx);

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H
