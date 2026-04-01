/**
 * @file uart_driver.c
 * @brief Thread-safe UART driver implementation.
 * 
 * Provides blocking and queued non-blocking DMA-based UART communication
 * with CMSIS-RTOS v2 integration. Supports multiple UART instances with
 * automatic callback routing and error handling.
 */

#include "uart_driver.h"
#include <string.h>
#include "command_module.h"
#include "logging.h"

#define UART_DRV_MAX_INSTANCES 4
#define DEFAULT_UART_TIMEOUT_MS 100
#define TASK_DELAY_MS 1

/* Registry of all inited instances */
static uart_drv_t *uart_instances[UART_DRV_MAX_INSTANCES];
static size_t      uart_instance_count = 0;
/* No RX node pool: circular-only RX mode enabled */

static void tx_pool_init(uart_drv_t *drv) {
    if (!drv || drv->tx_pool_initialized) return;
    /* Use IRQ disable for pool protection (safe in ISR) */
    for (int i = 0; i < UART_TX_NODE_COUNT - 1; ++i) {
        drv->tx_node_pool[i].next = &drv->tx_node_pool[i+1];
        drv->tx_node_pool[i].in_use = false;
        drv->tx_node_pool[i].len = 0;
    }
    drv->tx_node_pool[UART_TX_NODE_COUNT - 1].next = NULL;
    drv->tx_node_pool[UART_TX_NODE_COUNT - 1].in_use = false;
    drv->tx_node_pool[UART_TX_NODE_COUNT - 1].len = 0;
    drv->tx_free_list = &drv->tx_node_pool[0];
    drv->tx_pool_initialized = true;
    drv->tx_fallback_count = 0;
}

static uart_tx_node_t *tx_pool_alloc(uart_drv_t *drv) {
    if (!drv) return NULL;
    if (!drv->tx_pool_initialized) tx_pool_init(drv);
    FAULT_ENTER_CRITICAL();
    uart_tx_node_t *n = drv->tx_free_list;
    if (n) {
        drv->tx_free_list = n->next;
        n->next = NULL;
        n->in_use = true;
    }
    FAULT_EXIT_CRITICAL();
    if (!n) {
        /* Keep a running count when allocations fail so we can diagnose TX pool exhaustion */
        drv->tx_fallback_count++;
    }
    return n;
}

static void tx_pool_free(uart_drv_t *drv, uart_tx_node_t *n) {
    if (!drv || !n) return;
    FAULT_ENTER_CRITICAL();
    n->next = drv->tx_free_list;
    n->in_use = false;
    n->len = 0;
    drv->tx_free_list = n;
    FAULT_EXIT_CRITICAL();
}

static const uart_rtos_adapter_t *get_rtos_adapter(uart_drv_t *drv) {
    if (!drv->rtos_adapter) {
        drv->rtos_adapter = uart_rtos_get_default_adapter();
    }
    return drv->rtos_adapter;
}

/* No queued RX pool in circular-only mode */

/**
 * @brief Find the driver instance matching a given UART handle.
 * @param hu Pointer to HAL UART handle
 * @return Pointer to matching driver instance, or NULL if not found
 */
static uart_drv_t *find_drv(UART_HandleTypeDef *hu) {
    uart_drv_t *result = NULL;
    FAULT_ENTER_CRITICAL();
    for (size_t i = 0; i < uart_instance_count; ++i) {
        if (uart_instances[i]->huart == hu) {
            result = uart_instances[i];
            break;
        }
    }
    FAULT_EXIT_CRITICAL();
    return result;
}

/**
 * @brief Invoke the user callback and update driver status.
 * @param drv Pointer to driver instance
 * @param evt Event type to report
 */
static void notify_event(uart_drv_t *drv, uart_event_t evt) {
    if (evt == UART_EVT_TX_COMPLETE) {
        drv->tx_status = UART_OK;
    } else if (evt == UART_EVT_RX_COMPLETE) {
        drv->rx_status = UART_OK;
    } else {
        /* propagate errors into both for safety */
        drv->tx_status = UART_ERROR;
        drv->rx_status = UART_ERROR;
    }
    if (drv->cb) {
        drv->cb(evt, drv->ctx);
    }
}

/* Queue helpers */
static int drv_queue_enqueue(uart_drv_t *drv, const uint8_t *data, size_t len) {
    if (!drv || !data || len == 0 || len > UART_TX_NODE_SIZE) return -1;
    uart_tx_node_t *n = tx_pool_alloc(drv);
    if (!n) return -1;
    memcpy(n->payload, data, len);
    n->len = (uint16_t)len;
    n->next = NULL;

    FAULT_ENTER_CRITICAL();
    uart_queue_status_t qstatus = uart_queue_push(&drv->tx_pending_queue, &n);
    FAULT_EXIT_CRITICAL();
    if (qstatus != UART_QUEUE_OK) {
        tx_pool_free(drv, n);
        return -1;
    }
    return 0;
}

static uart_tx_node_t *drv_queue_dequeue(uart_drv_t *drv) {
    if (!drv) return NULL;
    uart_tx_node_t *n = NULL;
    FAULT_ENTER_CRITICAL();
    if (uart_queue_pop(&drv->tx_pending_queue, &n) != UART_QUEUE_OK) {
        n = NULL;
    }
    FAULT_EXIT_CRITICAL();
    return n;
}

/* Queue receive operations removed — use circular RX */

/* No queued RX operations (dequeue removed) */

uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
                        DMA_HandleTypeDef  *hdma_tx,
                        DMA_HandleTypeDef  *hdma_rx)
{
    if (!drv || !huart) {
        return UART_ERROR;
    }
    if (uart_instance_count >= UART_DRV_MAX_INSTANCES) {
        return UART_ERROR;
    }
    const uart_rtos_adapter_t *rtos = get_rtos_adapter(drv);
    drv->huart = huart;
    drv->hdma_tx = hdma_tx;
    drv->hdma_rx = hdma_rx;
    /* Link DMA handles if provided */
    if (drv->hdma_tx) {
        drv->huart->hdmatx = drv->hdma_tx;
    }
    if (drv->hdma_rx) {
        drv->huart->hdmarx = drv->hdma_rx;
    }
    /* Create synchronization primitives via selected RTOS adapter. */
    drv->tx_mutex = rtos->create_mutex(drv->rtos_ctx);
    drv->rx_mutex = rtos->create_mutex(drv->rtos_ctx);
        /* circular-only RX: no queued RX nodes */
    drv->tx_free_list = NULL;
    drv->tx_pool_initialized = false;
    drv->tx_fallback_count = 0;
    tx_pool_init(drv);
    (void)uart_queue_init(&drv->tx_pending_queue,
                          (uint8_t *)drv->tx_queue_storage,
                          sizeof(drv->tx_queue_storage[0]),
                          UART_TX_NODE_COUNT);
        /* no rx pool to init for circular-only RX */
    drv->cb       = NULL;
    drv->ctx      = NULL;
    drv->tx_status = UART_OK;
    drv->rx_status = UART_OK;
    /* Initialize circular RX fields */
    drv->circular_buf = NULL;
    drv->circular_len = 0;
    drv->circular_last_pos = 0;
    drv->circular_enabled = false;
    /* Register instance; guard against ISR preemption during the write */
    FAULT_ENTER_CRITICAL();
    uart_instances[uart_instance_count++] = drv;
    FAULT_EXIT_CRITICAL();
    return (drv->tx_mutex && drv->rx_mutex) ? UART_OK : UART_ERROR;
}

/**
 * @brief Clean up and remove driver from registry.
 * @param drv Pointer to driver instance to deinitialize
 */
void uart_deinit(uart_drv_t *drv)
{
    if (!drv) {
        return;
    }
    const uart_rtos_adapter_t *rtos = get_rtos_adapter(drv);
    /* Remove from registry; guard against ISR preemption during the write */
    FAULT_ENTER_CRITICAL();
    for (size_t i = 0; i < uart_instance_count; ++i) {
        if (uart_instances[i] == drv) {
            for (size_t j = i; j + 1 < uart_instance_count; ++j) {
                uart_instances[j] = uart_instances[j+1];
            }
            --uart_instance_count;
            break;
        }
    }
    FAULT_EXIT_CRITICAL();
    /* Clean up RTOS resources */
    if (drv->tx_mutex) {
        rtos->delete_mutex(drv->rtos_ctx, drv->tx_mutex);
    }
    if (drv->rx_mutex) {
        rtos->delete_mutex(drv->rtos_ctx, drv->rx_mutex);
    }
    /* Free any queued nodes for this driver */
    uart_tx_node_t *n;
    while ((n = drv_queue_dequeue(drv)) != NULL) {
        tx_pool_free(drv, n);
    }
    /* No queued RX nodes to free when using circular-only RX */
    /* tx_queue_mutex is not present in new implementation */
    /* Disable circular DMA and UART IDLE IRQ */
    if (drv->hdma_rx && drv->hdma_rx->Instance) {
        drv->hdma_rx->Instance->CR &= ~DMA_SxCR_CIRC;
    }
    if (drv->huart) {
        __HAL_UART_DISABLE_IT(drv->huart, UART_IT_IDLE);
    }
}

uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx)
{
    if (!drv || !huart) return UART_ERROR;
    drv->huart = huart;
    drv->hdma_tx = hdma_tx;
    drv->hdma_rx = hdma_rx;
    if (drv->hdma_tx) {
        drv->huart->hdmatx = drv->hdma_tx;
    }
    if (drv->hdma_rx) {
        drv->huart->hdmarx = drv->hdma_rx;
    }
    drv->tx_status = UART_OK;
    drv->rx_status = UART_OK;
    drv->circular_enabled = false;
    return UART_OK;
}

/*******************************************************************************
 * Blocking APIs
 ******************************************************************************/

uart_status_t uart_send_blocking(uart_drv_t *drv, const uint8_t *data, size_t len, uint32_t timeout_ms) {
    /* For DMA-only policy, blocking send uses DMA and waits for completion. */
    return uart_send_dma_blocking(drv, data, len, timeout_ms);
}

/* uart_receive_blocking removed - use circular RX with uart_read_from_circular */

/*******************************************************************************
 * Non-blocking DMA-driven APIs
 ******************************************************************************/

uart_status_t uart_send_nb(uart_drv_t *drv, const uint8_t *data, size_t len) {
    if (!drv || !data || len == 0) return UART_ERROR;
    if (!drv->hdma_tx) return UART_ERROR;
    if (len > UART_TX_NODE_SIZE) return UART_ERROR;
    if (drv_queue_enqueue(drv, data, len) != 0) {
        return UART_BUSY; // queue full or allocation failed
    }
    /* Atomically claim the TX channel if idle, then start outside the critical section */
    FAULT_ENTER_CRITICAL();
    bool start_tx = (drv->tx_status != UART_BUSY);
    if (start_tx) {
        drv->tx_status = UART_BUSY;
    }
    FAULT_EXIT_CRITICAL();
    if (start_tx) {
        uart_tx_node_t *n = drv_queue_dequeue(drv);
        if (n) {
            drv->active_tx_node = n;
            if (HAL_UART_Transmit_DMA(drv->huart, (uint8_t *)n->payload, n->len) != HAL_OK) {
                drv->tx_status = UART_ERROR;
                drv->active_tx_node = NULL;
                tx_pool_free(drv, n);
                return UART_ERROR;
            }
        } else {
            drv->tx_status = UART_OK;
        }
    }
    return UART_OK;
}

/* uart_receive_nb removed - use circular RX with uart_start_circular_rx and uart_read_from_circular */

/*******************************************************************************
 * DMA-driven APIs
 ******************************************************************************/

uart_status_t uart_start_dma_tx(uart_drv_t *drv, const uint8_t *data, size_t len) {
    if (!drv->hdma_tx) {
        return UART_ERROR;
    }
    return (HAL_UART_Transmit_DMA(drv->huart, (uint8_t *)data, len) == HAL_OK)
            ? UART_OK : UART_ERROR;
}

/* uart_start_dma_rx removed - use uart_start_circular_rx for circular mode */

uart_status_t uart_start_circular_rx(uart_drv_t *drv, uint8_t *buf, size_t len) {
    if (!drv || !buf || len == 0) return UART_ERROR;
    if (!drv->hdma_rx) return UART_ERROR;
    /* Mark circular mode and configure DMA */
    drv->circular_buf = buf;
    drv->circular_len = len;
    drv->circular_last_pos = 0;
    drv->circular_enabled = true;
    /* NOTE: DMA circular mode must be configured via hdmarx->Init.Mode = DMA_CIRCULAR
     * before calling MX_DMA_Init(). Patching DMA_SxCR_CIRC here is unsafe because
     * HAL_UART_Receive_DMA reconfigures the stream from Init and would overwrite it. */
    /* Enable UART IDLE interrupt so we can detect end of packet */
    __HAL_UART_ENABLE_IT(drv->huart, UART_IT_IDLE);
    if (HAL_UART_Receive_DMA(drv->huart, buf, len) != HAL_OK) {
        drv->circular_enabled = false;
        return UART_ERROR;
    }
    return UART_OK;
}

// Blocking DMA transmit helper
uart_status_t uart_send_dma_blocking(uart_drv_t *drv, const uint8_t *data,
                                     size_t len, uint32_t timeout_ms)
{
    if (!drv || !data || len == 0) {
        return UART_ERROR;
    }
    const uart_rtos_adapter_t *rtos = get_rtos_adapter(drv);
    if (!drv->hdma_tx) {
        return UART_ERROR;
    }
    if (rtos->mutex_lock(drv->rtos_ctx, drv->tx_mutex, timeout_ms) != UART_RTOS_OP_OK) {
        return UART_BUSY;
    }

    /* If a non-blocking queued TX is in progress, wait until it finishes */
    while (drv->tx_status == UART_BUSY) {
        rtos->delay_ms(drv->rtos_ctx, TASK_DELAY_MS);
    }
    drv->tx_status = UART_BUSY;
    if (HAL_UART_Transmit_DMA(drv->huart, (uint8_t *)data, len) != HAL_OK) {
                drv->tx_status = UART_ERROR;
        rtos->mutex_unlock(drv->rtos_ctx, drv->tx_mutex);
        return UART_ERROR;
    }

    uint32_t start = rtos->get_ticks(drv->rtos_ctx);
    while (drv->tx_status == UART_BUSY) {
        if ((rtos->get_ticks(drv->rtos_ctx) - start) >= timeout_ms) {
            drv->tx_status = UART_ERROR;
            rtos->mutex_unlock(drv->rtos_ctx, drv->tx_mutex);
            return UART_ERROR;
        }
        rtos->delay_ms(drv->rtos_ctx, TASK_DELAY_MS);
    }
    rtos->mutex_unlock(drv->rtos_ctx, drv->tx_mutex);
    return drv->tx_status;
}

/*******************************************************************************
 * Buffer & status APIs  
 ******************************************************************************/

size_t uart_bytes_available(uart_drv_t *drv) {
    if (!drv || !drv->hdma_rx) return 0;
    if (drv->circular_enabled && drv->circular_len > 0) {
        size_t ndtr = drv->hdma_rx->Instance->NDTR;
        size_t write_pos = drv->circular_len - ndtr;
        size_t available = (write_pos + drv->circular_len - drv->circular_last_pos) % drv->circular_len;
        return available;
    }
    /* Non-circular mode removed; only circular mode is supported. */
    return 0;
}

size_t uart_get_tx_queue_count(uart_drv_t *drv) {
    if (!drv) return 0;
    return uart_queue_count(&drv->tx_pending_queue);
}

size_t uart_get_tx_pool_free_count(uart_drv_t *drv) {
    if (!drv) return 0;
    if (!drv->tx_pool_initialized) return UART_TX_NODE_COUNT;
    size_t c = 0;
    FAULT_ENTER_CRITICAL();
    uart_tx_node_t *n = drv->tx_free_list;
    while (n) { c++; n = n->next; }
    FAULT_EXIT_CRITICAL();
    return c;
}

size_t uart_get_tx_pool_total_count(uart_drv_t *drv) {
    (void)drv;
    return UART_TX_NODE_COUNT;
}

size_t uart_get_tx_fallback_count(uart_drv_t *drv) {
    if (!drv) return 0;
    return drv->tx_fallback_count;
}

void uart_flush_rx(uart_drv_t *drv) {
    if (!drv || !drv->huart) return;
    __HAL_UART_CLEAR_OREFLAG(drv->huart);
}

void uart_flush_tx(uart_drv_t *drv) {
    if (!drv || !drv->huart) return;
    __HAL_UART_CLEAR_FLAG(drv->huart, UART_FLAG_TC);
}

uart_status_t uart_get_status(uart_drv_t *drv) {
    if (!drv) return UART_ERROR;
    if (drv->tx_status == UART_BUSY || drv->rx_status == UART_BUSY) return UART_BUSY;
    if (drv->tx_status == UART_ERROR || drv->rx_status == UART_ERROR) return UART_ERROR;
    return UART_OK;
}

uart_status_t uart_system_init(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx)
{
    uart_status_t ret = uart_init(drv, huart, hdma_tx, hdma_rx);
    if (ret != UART_OK) {
        return ret;
    }

#if USE_CMD_INTERPRETER
    cmd_init(drv);
#endif
#if LOGGING_ENABLED
    log_init(drv);
#endif
    return UART_OK;
}

/* Read up to maxlen bytes from circular RX buffer */
size_t uart_read_from_circular(uart_drv_t *drv, uint8_t *dest, size_t maxlen) {
    if (!drv || !drv->circular_enabled || !drv->hdma_rx || !dest) return 0;
    size_t ndtr = drv->hdma_rx->Instance->NDTR;
    size_t write_pos = drv->circular_len - ndtr;
    size_t available = (write_pos + drv->circular_len - drv->circular_last_pos) % drv->circular_len;
    if (available == 0) return 0;
    size_t to_read = (available < maxlen) ? available : maxlen;
    for (size_t i = 0; i < to_read; ++i) {
        dest[i] = drv->circular_buf[(drv->circular_last_pos + i) % drv->circular_len];
    }
    drv->circular_last_pos = (drv->circular_last_pos + to_read) % drv->circular_len;
    return to_read;
}

/* Called from IRQ handler when IDLE detected; not safe to call from ISR for heavy processing */
void uart_process_idle(UART_HandleTypeDef *hu) {
    if (!__HAL_UART_GET_FLAG(hu, UART_FLAG_IDLE)) return;
    /* Clear IDLE flag by reading SR then DR */
    volatile uint32_t tmp;
    tmp = hu->Instance->SR;
    tmp = hu->Instance->DR;
    (void)tmp; /* Silence unused variable warnings; reading SR/DR clears IDLE */
    uart_drv_t *drv = find_drv(hu);
    if (!drv) return;
    if (!drv->circular_enabled) return;
    /* Notify user callback that data is available */
    if (drv->cb) {
        drv->cb(UART_EVT_RX_AVAILABLE, drv->ctx);
    }
}

/*******************************************************************************
 * Callback registration
 ******************************************************************************/

void uart_register_callback(uart_drv_t *drv, uart_callback_t cb, void *user_ctx) {
    if (!drv) {
        return;
    }
    drv->cb  = cb;
    drv->ctx = user_ctx;
}

void uart_set_rtos_adapter(uart_drv_t *drv,
                           const uart_rtos_adapter_t *adapter,
                           void *rtos_ctx) {
    if (!drv) {
        return;
    }
    drv->rtos_adapter = adapter;
    drv->rtos_ctx = rtos_ctx;
}

/*******************************************************************************
 * HAL UART IRQ callbacks (called by HAL_UART_IRQHandler)
 ******************************************************************************/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (!drv) return;
    /* Free the just-completed node */
    if (drv->active_tx_node) {
        tx_pool_free(drv, drv->active_tx_node);
        drv->active_tx_node = NULL;
    }
    /* Dequeue before updating tx_status so it is never briefly UART_OK while
     * items are still pending — this closes the race with uart_send_nb. */
    uart_tx_node_t *n = drv_queue_dequeue(drv);
    if (n) {
        drv->active_tx_node = n;
        drv->tx_status = UART_BUSY;
        if (HAL_UART_Transmit_DMA(drv->huart, (uint8_t *)n->payload, n->len) != HAL_OK) {
            drv->tx_status = UART_ERROR;
            tx_pool_free(drv, n);
            drv->active_tx_node = NULL;
            if (drv->cb) drv->cb(UART_EVT_ERROR, drv->ctx);
        }
    } else {
        /* Queue fully drained; only now signal completion */
        notify_event(drv, UART_EVT_TX_COMPLETE);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (drv) {
        /* Non-circular RX completed (unlikely in circular-only setup). */
        notify_event(drv, UART_EVT_RX_COMPLETE);
        drv->rx_status = UART_OK;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (drv) {
        notify_event(drv, UART_EVT_ERROR);
    }
}


