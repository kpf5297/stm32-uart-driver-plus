/*
 * uart_driver.c
 *
 *  Created on: Jul 19, 2025
 *      Author: kevinfox
 */
// uart_driver.c

#include "uart_driver.h"
#include "uart_driver_config.h"

#define UART_DRV_MAX_INSTANCES 4

// Registry of all inited instances
static uart_drv_t *uart_instances[UART_DRV_MAX_INSTANCES];
static size_t      uart_instance_count = 0;

/** Find the drv instance matching a given huart pointer */
static uart_drv_t *find_drv(UART_HandleTypeDef *hu) {
    for (size_t i = 0; i < uart_instance_count; ++i) {
        if (uart_instances[i]->huart == hu) {
            return uart_instances[i];
        }
    }
    return NULL;
}

/** Invoke the user callback and update status */
static void notify_event(uart_drv_t *drv, uart_event_t evt) {
    drv->status = (evt == UART_EVT_TX_COMPLETE || evt == UART_EVT_RX_COMPLETE)
                  ? UART_OK
                  : UART_ERROR;
    if (drv->cb) drv->cb(evt, drv->ctx);
}

uart_status_t uart_init(uart_drv_t *drv,
                        UART_HandleTypeDef *huart,
#if UART_BACKEND == UART_BACKEND_HAL
                        DMA_HandleTypeDef  *hdma_tx,
                        DMA_HandleTypeDef  *hdma_rx)
#else
                        void              *unused_tx,
                        void              *unused_rx)
#endif
{
    if (uart_instance_count >= UART_DRV_MAX_INSTANCES) {
        return UART_ERROR;
    }
    drv->huart = huart;
#if UART_BACKEND == UART_BACKEND_HAL
    drv->hdma_tx = hdma_tx;
    drv->hdma_rx = hdma_rx;
    // Link DMA handles if provided
    if (drv->hdma_tx) drv->huart->hdmatx = drv->hdma_tx;
    if (drv->hdma_rx) drv->huart->hdmarx = drv->hdma_rx;
#else
    (void)unused_tx;
    (void)unused_rx;
#endif
    // Create mutexes
    drv->tx_mutex = xSemaphoreCreateMutex();
    drv->rx_mutex = xSemaphoreCreateMutex();
    drv->cb       = NULL;
    drv->ctx      = NULL;
    drv->status   = UART_OK;
    // Register instance
    uart_instances[uart_instance_count++] = drv;
    return (drv->tx_mutex && drv->rx_mutex) ? UART_OK : UART_ERROR;
}

void uart_deinit(uart_drv_t *drv)
{
    // Remove from registry
    for (size_t i = 0; i < uart_instance_count; ++i) {
        if (uart_instances[i] == drv) {
            for (size_t j = i; j + 1 < uart_instance_count; ++j) {
                uart_instances[j] = uart_instances[j+1];
            }
            --uart_instance_count;
            break;
        }
    }
    if (drv->tx_mutex) vSemaphoreDelete(drv->tx_mutex);
    if (drv->rx_mutex) vSemaphoreDelete(drv->rx_mutex);
}

uart_status_t uart_reconfigure(uart_drv_t *drv,
                               UART_HandleTypeDef *huart,
#if UART_BACKEND == UART_BACKEND_HAL
                               DMA_HandleTypeDef  *hdma_tx,
                               DMA_HandleTypeDef  *hdma_rx)
#else
                               void              *unused_tx,
                               void              *unused_rx)
#endif
{
    drv->huart = huart;
#if UART_BACKEND == UART_BACKEND_HAL
    drv->hdma_tx = hdma_tx;
    drv->hdma_rx = hdma_rx;
    if (drv->hdma_tx) drv->huart->hdmatx = drv->hdma_tx;
    if (drv->hdma_rx) drv->huart->hdmarx = drv->hdma_rx;
#else
    (void)unused_tx;
    (void)unused_rx;
#endif
    drv->status = UART_OK;
    return UART_OK;
}

// Blocking APIs

uart_status_t uart_send_blocking(uart_drv_t *drv, uint8_t *data, size_t len, uint32_t timeout_ms) {
    if (xSemaphoreTake(drv->tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return UART_BUSY;
    HAL_StatusTypeDef h = HAL_UART_Transmit(drv->huart, data, len, timeout_ms);
    xSemaphoreGive(drv->tx_mutex);
    return (h == HAL_OK ? UART_OK : UART_ERROR);
}

uart_status_t uart_receive_blocking(uart_drv_t *drv, uint8_t *buf, size_t len, uint32_t timeout_ms) {
    if (xSemaphoreTake(drv->rx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return UART_BUSY;
    HAL_StatusTypeDef h = HAL_UART_Receive(drv->huart, buf, len, timeout_ms);
    xSemaphoreGive(drv->rx_mutex);
    return (h == HAL_OK ? UART_OK : UART_ERROR);
}

// Non-blocking IRQ-driven APIs

uart_status_t uart_send_nb(uart_drv_t *drv, uint8_t *data, size_t len) {
    if (drv->status == UART_BUSY) return UART_BUSY;
    drv->status = UART_BUSY;
    if (HAL_UART_Transmit_IT(drv->huart, data, len) != HAL_OK) {
        drv->status = UART_ERROR;
        return UART_ERROR;
    }
    return UART_OK;
}

uart_status_t uart_receive_nb(uart_drv_t *drv, uint8_t *buf, size_t len) {
    if (drv->status == UART_BUSY) return UART_BUSY;
    drv->status = UART_BUSY;
    if (HAL_UART_Receive_IT(drv->huart, buf, len) != HAL_OK) {
        drv->status = UART_ERROR;
        return UART_ERROR;
    }
    return UART_OK;
}

// DMA-driven APIs

uart_status_t uart_start_dma_tx(uart_drv_t *drv, uint8_t *data, size_t len) {
#if UART_BACKEND == UART_BACKEND_HAL
    if (!drv->hdma_tx) return UART_ERROR;
    return (HAL_UART_Transmit_DMA(drv->huart, data, len) == HAL_OK
            ? UART_OK : UART_ERROR);
#else
    (void)data; (void)len; (void)drv;
    return UART_ERROR;
#endif
}

uart_status_t uart_start_dma_rx(uart_drv_t *drv, uint8_t *buf, size_t len) {
#if UART_BACKEND == UART_BACKEND_HAL
    if (!drv->hdma_rx) return UART_ERROR;
    return (HAL_UART_Receive_DMA(drv->huart, buf, len) == HAL_OK
            ? UART_OK : UART_ERROR);
#else
    (void)buf;
    (void)len;
    (void)drv;
    return UART_ERROR;
#endif
}

// Blocking DMA transmit helper
uart_status_t uart_send_dma_blocking(uart_drv_t *drv, uint8_t *data,
                                     size_t len, uint32_t timeout_ms)
{
#if UART_BACKEND == UART_BACKEND_HAL
    if (!drv->hdma_tx)
        return UART_ERROR;
    if (xSemaphoreTake(drv->tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
        return UART_BUSY;

    drv->status = UART_BUSY;
    if (HAL_UART_Transmit_DMA(drv->huart, data, len) != HAL_OK) {
        drv->status = UART_ERROR;
        xSemaphoreGive(drv->tx_mutex);
        return UART_ERROR;
    }

    TickType_t start = xTaskGetTickCount();
    while (drv->status == UART_BUSY) {
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            drv->status = UART_ERROR;
            xSemaphoreGive(drv->tx_mutex);
            return UART_ERROR;
        }
        vTaskDelay(1);
    }

    xSemaphoreGive(drv->tx_mutex);
    return drv->status;
#else
    (void)drv; (void)data; (void)len; (void)timeout_ms;
    return UART_ERROR;
#endif
}

// Buffer & status

#if UART_BACKEND == UART_BACKEND_HAL
size_t uart_bytes_available(uart_drv_t *drv) {
    return drv->hdma_rx ? drv->hdma_rx->Instance->NDTR : 0;
}
#else
size_t uart_bytes_available(uart_drv_t *drv) {
    (void)drv; return 0;
}
#endif

void uart_flush_rx(uart_drv_t *drv) {
    __HAL_UART_CLEAR_OREFLAG(drv->huart);
}

void uart_flush_tx(uart_drv_t *drv) {
//    __HAL_UART_CLEAR_TCFLAG(drv->huart);
	__HAL_UART_CLEAR_FLAG(drv->huart, UART_FLAG_TC);
}

uart_status_t uart_get_status(uart_drv_t *drv) {
    return drv->status;
}

// Callback registration

void uart_register_callback(uart_drv_t *drv, uart_callback_t cb, void *user_ctx) {
    drv->cb  = cb;
    drv->ctx = user_ctx;
}

// HAL UART IRQ callbacks (called by HAL_UART_IRQHandler)

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (drv) notify_event(drv, UART_EVT_TX_COMPLETE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (drv) notify_event(drv, UART_EVT_RX_COMPLETE);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *hu) {
    uart_drv_t *drv = find_drv(hu);
    if (drv) notify_event(drv, UART_EVT_ERROR);
}


