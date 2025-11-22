/**
 * @file uart_driver_abstraction.c
 * @brief HAL-only UART abstraction implementation
 *
 * This file implements the unified API for the STM32 HAL driver,
 * providing integration with CMSIS-RTOS and FreeRTOS.
 */

#include "uart_driver_abstraction.h"
#include <string.h>

#define UART_TX_IDLE_STATE 0
#define UART_TX_ACTIVE_STATE 1
#define UART_RX_IDLE_STATE 0
#define UART_RX_ACTIVE_STATE 1

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

/*
 * Private Functions
 */
/* HAL Implementation */
static uart_abstraction_status_t uart_hal_init(uart_abstraction_handle_t *handle)
{
    /* HAL UART should already be initialized by user */
    if (handle->huart == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    
    handle->tx_state = UART_TX_IDLE_STATE;
    handle->rx_state = UART_RX_IDLE_STATE;
    
    return UART_ABSTRACTION_OK;
}

/**
 * @brief Convert HAL status to abstraction status
 */
static uart_abstraction_status_t hal_to_abstraction_status(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
        case HAL_OK:      return UART_ABSTRACTION_OK;
        case HAL_BUSY:    return UART_ABSTRACTION_BUSY;
        case HAL_TIMEOUT: return UART_ABSTRACTION_TIMEOUT;
        default:          return UART_ABSTRACTION_ERROR;
    }
}

/* end HAL-only implementation */

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

uart_abstraction_status_t uart_abstraction_init(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    
    /* Initialize state */
    handle->tx_state = UART_TX_IDLE_STATE;
    handle->rx_state = UART_RX_IDLE_STATE;
    handle->tx_buffer = NULL;
    handle->rx_buffer = NULL;
    handle->tx_size = 0;
    handle->rx_size = 0;
    handle->tx_count = 0;
    handle->rx_count = 0;
    
#if USE_FREERTOS
    /* Create FreeRTOS synchronization objects */
    handle->tx_mutex = xSemaphoreCreateMutex();
    handle->rx_mutex = xSemaphoreCreateMutex();
    handle->tx_complete_sem = xSemaphoreCreateBinary();
    handle->rx_complete_sem = xSemaphoreCreateBinary();
    
    if (!handle->tx_mutex || !handle->rx_mutex || 
        !handle->tx_complete_sem || !handle->rx_complete_sem) {
        return UART_ABSTRACTION_ERROR;
    }
#endif /* USE_FREERTOS */
    
    return uart_hal_init(handle);
}

void uart_abstraction_deinit(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
#if USE_FREERTOS
    /* Clean up FreeRTOS objects */
    if (handle->tx_mutex) {
        vSemaphoreDelete(handle->tx_mutex);
    }
    if (handle->rx_mutex) {
        vSemaphoreDelete(handle->rx_mutex);
    }
    if (handle->tx_complete_sem) {
        vSemaphoreDelete(handle->tx_complete_sem);
    }
    if (handle->rx_complete_sem) {
        vSemaphoreDelete(handle->rx_complete_sem);
    }
#endif /* USE_FREERTOS */
    
    /* Reset state */
    memset(handle, 0, sizeof(uart_abstraction_handle_t));
}

uart_abstraction_status_t uart_abstraction_transmit(uart_abstraction_handle_t *handle,
                                                    uint8_t *data,
                                                    uint16_t size,
                                                    uint32_t timeout_ms)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_FREERTOS
    /* Take mutex with timeout */
    if (xSemaphoreTake(handle->tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return UART_ABSTRACTION_TIMEOUT;
    }
#endif
    
    uart_abstraction_status_t result;
    
    /* HAL transmit */
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit(handle->huart, data, size, timeout_ms);
    result = hal_to_abstraction_status(hal_result);
    
cleanup:
#if USE_FREERTOS
    xSemaphoreGive(handle->tx_mutex);
#endif /* USE_FREERTOS */
    
    return result;
}

uart_abstraction_status_t uart_abstraction_receive(uart_abstraction_handle_t *handle,
                                                   uint8_t *data,
                                                   uint16_t size,
                                                   uint32_t timeout_ms)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_FREERTOS
    /* Take mutex with timeout */
    if (xSemaphoreTake(handle->rx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return UART_ABSTRACTION_TIMEOUT;
    }
#endif
    
    uart_abstraction_status_t result;
    
    /* HAL receive */
    HAL_StatusTypeDef hal_result = HAL_UART_Receive(handle->huart, data, size, timeout_ms);
    result = hal_to_abstraction_status(hal_result);
    
cleanup:
#if USE_FREERTOS
    xSemaphoreGive(handle->rx_mutex);
#endif /* USE_FREERTOS */
    
    return result;
}

uart_abstraction_status_t uart_abstraction_transmit_it(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit_IT(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
}

uart_abstraction_status_t uart_abstraction_receive_it(uart_abstraction_handle_t *handle,
                                                      uint8_t *data,
                                                      uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
    HAL_StatusTypeDef hal_result = HAL_UART_Receive_IT(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
}

uart_abstraction_status_t uart_abstraction_transmit_dma(uart_abstraction_handle_t *handle,
                                                        uint8_t *data,
                                                        uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
    if (handle->hdma_tx == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit_DMA(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
}

uart_abstraction_status_t uart_abstraction_receive_dma(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
    if (handle->hdma_rx == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    HAL_StatusTypeDef hal_result = HAL_UART_Receive_DMA(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
}

uint16_t uart_abstraction_get_rx_count(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
    if (handle->hdma_rx) {
        return handle->hdma_rx->Instance->NDTR;
    }
    return 0;
}

void uart_abstraction_flush_rx(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    __HAL_UART_CLEAR_OREFLAG(handle->huart);
}

void uart_abstraction_flush_tx(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
    __HAL_UART_CLEAR_FLAG(handle->huart, UART_FLAG_TC);
}

uint32_t uart_abstraction_is_tx_complete(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
    return (handle->huart->gState == HAL_UART_STATE_READY) ? 1 : 0;
}

uint32_t uart_abstraction_is_rx_complete(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
    return (handle->huart->RxState == HAL_UART_STATE_READY) ? 1 : 0;
}

/*******************************************************************************
 * CMSIS-RTOS2 Integration
 ******************************************************************************/
#if USE_FREERTOS && USE_CMSIS_RTOS

uart_abstraction_status_t uart_abstraction_transmit_rtos(uart_abstraction_handle_t *handle,
                                                         uint8_t *data,
                                                         uint16_t size,
                                                         uint32_t timeout_ms)
{
    /* Use standard FreeRTOS implementation, compatible with CMSIS-RTOS2 */
    return uart_abstraction_transmit(handle, data, size, timeout_ms);
}

uart_abstraction_status_t uart_abstraction_receive_rtos(uart_abstraction_handle_t *handle,
                                                        uint8_t *data,
                                                        uint16_t size,
                                                        uint32_t timeout_ms)
{
    /* Use standard FreeRTOS implementation, compatible with CMSIS-RTOS2 */
    return uart_abstraction_receive(handle, data, size, timeout_ms);
}

#endif /* USE_FREERTOS && USE_CMSIS_RTOS */
