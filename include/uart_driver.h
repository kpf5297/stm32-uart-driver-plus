/**
 * @file uart_driver.h
 * @brief Thread-safe UART driver abstraction for STM32.
 *
 * This module provides blocking and non-blocking APIs for UART
 * transmission and reception using DMA. All functions are safe to call
 * from multiple CMSIS-RTOS v2 tasks.
 * 
 * Uses STM32 HAL drivers (LL support has been removed) with CMSIS-RTOS v2 integration.
 */

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "uart_deterministic_queue.h"
#include "uart_rtos_adapter.h"

#ifndef UART_RTOS_USE_CMSIS
#define UART_RTOS_USE_CMSIS 1
#endif

#if UART_RTOS_USE_CMSIS
#include "cmsis_os2.h"
typedef osMutexId_t uart_mutex_handle_t;
#else
typedef void *uart_mutex_handle_t;
#endif

/* Allow CubeMX projects to override the exact STM32 HAL umbrella header. */
#ifndef UART_STM32_HAL_HEADER
#define UART_STM32_HAL_HEADER "stm32f4xx_hal.h"
#endif
#include UART_STM32_HAL_HEADER

/* Module-level configuration defaults; applications may redefine before including this header. */
#ifndef UART_TX_NODE_SIZE
#define UART_TX_NODE_SIZE 256
#endif
#ifndef UART_TX_NODE_COUNT
#define UART_TX_NODE_COUNT 16
#endif
/* RX queue node count */
#ifndef UART_RX_NODE_COUNT
#define UART_RX_NODE_COUNT 16
#endif

/* Kernel tick helpers.
 * NOTE: TICKS_PER_SECOND and MS_TO_TICKS call osKernelGetTickFreq() on every
 * expansion. Cache the result in a local variable when used in a hot path or
 * when multiple evaluations must be consistent (e.g. quotient + remainder). */
#define GET_TICKS()            osKernelGetTickCount()
#define MS_TO_TICKS(ms)        ((uint32_t)(((uint64_t)(ms) * osKernelGetTickFreq()) / 1000U))
#define TICKS_PER_SECOND       osKernelGetTickFreq()

/* Fault critical section helpers (defaults) */
#ifndef FAULT_ENTER_CRITICAL
#define FAULT_ENTER_CRITICAL() __disable_irq()
#endif
#ifndef FAULT_EXIT_CRITICAL
#define FAULT_EXIT_CRITICAL()  __enable_irq()
#endif

/* CMSIS-RTOS v2 (FreeRTOS backend) is required in this repo */
/* HAL header is included above; adjust to your STM32 series if necessary */

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
    UART_EVT_RX_AVAILABLE,
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
 * log_init automatically when the corresponding feature macros are enabled
 * in module headers such as `include/command_module.h` and `include/logging.h`.
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
 * This function grabs the TX mutex and starts a HAL UART DMA transfer.
 * It will block the calling task until the DMA transfer completes or @p timeout_ms elapses.
 */
uart_status_t uart_send_blocking   (uart_drv_t *drv, const uint8_t *data, size_t len, uint32_t timeout_ms);

/** @brief Begin a queued non-blocking transmit (DMA-based). */
uart_status_t uart_send_nb   (uart_drv_t *drv, const uint8_t *data, size_t len);

/** @brief Begin a direct DMA transmit (no queue). */
uart_status_t uart_start_dma_tx(uart_drv_t *drv, const uint8_t *data, size_t len);
/* RX is circular-only; use uart_start_circular_rx and uart_read_from_circular. */
/* Start circular DMA receive into provided buffer and enable IDLE detection */
uart_status_t uart_start_circular_rx(uart_drv_t *drv, uint8_t *buf, size_t len);

/* Read up to maxlen bytes from circular RX buffer, returns number of bytes read */
size_t uart_read_from_circular(uart_drv_t *drv, uint8_t *dest, size_t maxlen);

/* Called from IRQ handler to process UART IDLE events */
void uart_process_idle(UART_HandleTypeDef *hu);
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

/** Return current queued TX node count for debugging/diagnostics. */
size_t uart_get_tx_queue_count(uart_drv_t *drv);
/** Return number of free tx pool nodes (for diagnostics) */
size_t uart_get_tx_pool_free_count(uart_drv_t *drv);
/** Return total tx pool node count (for diagnostics) */
size_t uart_get_tx_pool_total_count(uart_drv_t *drv);
/** Return number of times queued TX allocation failed (fallbacks) */
size_t uart_get_tx_fallback_count(uart_drv_t *drv);

/**
 * @brief Internal state structure for a UART instance.
 */
/* Node for queued DMA transfers. Holds payload inline to avoid malloc/free. */
typedef struct uart_tx_node_s {
    struct uart_tx_node_s *next;
    uint16_t len;
    bool in_use;
    uint8_t payload[UART_TX_NODE_SIZE];
} uart_tx_node_t;


struct uart_drv_handle_s {
    /* No abstraction layer: driver operates directly on HAL UART / DMA handles */
    
    /* Legacy HAL interface (for backward compatibility) */
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;

    /* CMSIS-RTOS v2 synchronization */
    uart_mutex_handle_t tx_mutex;
    uart_mutex_handle_t rx_mutex;
    
    /* Status and callback management */
    uart_callback_t     cb;
    void               *ctx;
    volatile uart_status_t tx_status;
    volatile uart_status_t rx_status;
    /* Deterministic pending TX queue of pool-node pointers. */
    uart_deterministic_queue_t tx_pending_queue;
    struct uart_tx_node_s *tx_queue_storage[UART_TX_NODE_COUNT];
    struct uart_tx_node_s *active_tx_node; /* Currently active DMA node */
    /* Per-instance deterministic TX pool (no cross-instance sharing). */
    struct uart_tx_node_s tx_node_pool[UART_TX_NODE_COUNT];
    struct uart_tx_node_s *tx_free_list;
    bool tx_pool_initialized;
    volatile size_t tx_fallback_count;
    /* No queued RX support: circular-only RX implementation now */
    /* Circular DMA RX support */
    uint8_t *circular_buf;    /* Buffer used by circular DMA */
    size_t   circular_len;    /* Length of circular buffer */
    size_t   circular_last_pos;/* Last read position in circular buffer */
    bool     circular_enabled; /* Is circular RX active? */
    const uart_rtos_adapter_t *rtos_adapter;
    void *rtos_ctx;
};

/* (already declared above) */

/** Register a callback invoked from ISR context on UART events. */
void uart_register_callback(uart_drv_t *drv, uart_callback_t cb, void *user_ctx);

/**
 * @brief Set a custom RTOS adapter for mutex, tick, and delay operations.
 *
 * Call before uart_init() to override default CMSIS-based behavior.
 */
void uart_set_rtos_adapter(uart_drv_t *drv,
                           const uart_rtos_adapter_t *adapter,
                           void *rtos_ctx);

#ifdef __cplusplus
}
#endif

#endif // UART_DRIVER_H
