/**
 * @file uart_driver_abstraction.c
 * @brief Implementation of UART abstraction layer for HAL/LL compatibility
 *
 * This file implements the unified API that works with both STM32 HAL
 * and LL drivers, providing seamless integration with CMSIS-RTOS.
 */

#include "uart_driver_abstraction.h"
#include <string.h>

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

#if USE_STM32_LL_DRIVERS

/**
 * @brief LL-specific initialization
 */
static uart_abstraction_status_t uart_ll_init(uart_abstraction_handle_t *handle)
{
    /* Initialize LL UART peripheral */
    if (!LL_USART_IsEnabled(handle->uart_instance)) {
        LL_USART_Enable(handle->uart_instance);
    }
    
    /* Enable UART interrupts if needed */
    LL_USART_EnableIT_RXNE(handle->uart_instance);
    LL_USART_EnableIT_TC(handle->uart_instance);
    
    handle->tx_state = 0;
    handle->rx_state = 0;
    
    return UART_ABSTRACTION_OK;
}

/**
 * @brief LL-specific transmit implementation
 */
static uart_abstraction_status_t uart_ll_transmit(uart_abstraction_handle_t *handle,
                                                  uint8_t *data,
                                                  uint16_t size)
{
    if (handle->tx_state != 0) {
        return UART_ABSTRACTION_BUSY;
    }
    
    handle->tx_buffer = data;
    handle->tx_size = size;
    handle->tx_count = 0;
    handle->tx_state = 1;
    
    /* Send first byte */
    if (size > 0) {
        LL_USART_TransmitData8(handle->uart_instance, data[0]);
        handle->tx_count = 1;
    }
    
    return UART_ABSTRACTION_OK;
}

/**
 * @brief LL-specific receive implementation
 */
static uart_abstraction_status_t uart_ll_receive(uart_abstraction_handle_t *handle,
                                                 uint8_t *data,
                                                 uint16_t size)
{
    if (handle->rx_state != 0) {
        return UART_ABSTRACTION_BUSY;
    }
    
    handle->rx_buffer = data;
    handle->rx_size = size;
    handle->rx_count = 0;
    handle->rx_state = 1;
    
    return UART_ABSTRACTION_OK;
}

#else /* HAL Implementation */

/**
 * @brief HAL-specific initialization
 */
static uart_abstraction_status_t uart_hal_init(uart_abstraction_handle_t *handle)
{
    /* HAL UART should already be initialized by user */
    if (handle->huart == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    
    handle->tx_state = 0;
    handle->rx_state = 0;
    
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

#endif /* USE_STM32_LL_DRIVERS */

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

uart_abstraction_status_t uart_abstraction_init(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    
    /* Initialize state */
    handle->tx_state = 0;
    handle->rx_state = 0;
    handle->tx_buffer = NULL;
    handle->rx_buffer = NULL;
    handle->tx_size = 0;
    handle->rx_size = 0;
    handle->tx_count = 0;
    handle->rx_count = 0;
    
#ifdef USE_FREERTOS
    /* Create FreeRTOS synchronization objects */
    handle->tx_mutex = xSemaphoreCreateMutex();
    handle->rx_mutex = xSemaphoreCreateMutex();
    handle->tx_complete_sem = xSemaphoreCreateBinary();
    handle->rx_complete_sem = xSemaphoreCreateBinary();
    
    if (!handle->tx_mutex || !handle->rx_mutex || 
        !handle->tx_complete_sem || !handle->rx_complete_sem) {
        return UART_ABSTRACTION_ERROR;
    }
#endif
    
#if USE_STM32_LL_DRIVERS
    return uart_ll_init(handle);
#else
    return uart_hal_init(handle);
#endif
}

void uart_abstraction_deinit(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
#ifdef USE_FREERTOS
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
#endif
    
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
    
#ifdef USE_FREERTOS
    /* Take mutex with timeout */
    if (xSemaphoreTake(handle->tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return UART_ABSTRACTION_TIMEOUT;
    }
#endif
    
    uart_abstraction_status_t result;
    
#if USE_STM32_LL_DRIVERS
    /* LL blocking transmit implementation */
    for (uint16_t i = 0; i < size; i++) {
        uint32_t start_tick = GET_TICKS();
        
        /* Wait for TX register to be empty */
        while (!LL_USART_IsActiveFlag_TXE(handle->uart_instance)) {
            if ((GET_TICKS() - start_tick) >= timeout_ms) {
                result = UART_ABSTRACTION_TIMEOUT;
                goto cleanup;
            }
        }
        
        /* Send byte */
        LL_USART_TransmitData8(handle->uart_instance, data[i]);
    }
    
    /* Wait for transmission complete */
    uint32_t start_tick = GET_TICKS();
    while (!LL_USART_IsActiveFlag_TC(handle->uart_instance)) {
        if ((GET_TICKS() - start_tick) >= timeout_ms) {
            result = UART_ABSTRACTION_TIMEOUT;
            goto cleanup;
        }
    }
    
    result = UART_ABSTRACTION_OK;
    
#else
    /* HAL transmit */
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit(handle->huart, data, size, timeout_ms);
    result = hal_to_abstraction_status(hal_result);
#endif
    
cleanup:
#ifdef USE_FREERTOS
    xSemaphoreGive(handle->tx_mutex);
#endif
    
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
    
#ifdef USE_FREERTOS
    /* Take mutex with timeout */
    if (xSemaphoreTake(handle->rx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return UART_ABSTRACTION_TIMEOUT;
    }
#endif
    
    uart_abstraction_status_t result;
    
#if USE_STM32_LL_DRIVERS
    /* LL blocking receive implementation */
    for (uint16_t i = 0; i < size; i++) {
        uint32_t start_tick = GET_TICKS();
        
        /* Wait for data to be received */
        while (!LL_USART_IsActiveFlag_RXNE(handle->uart_instance)) {
            if ((GET_TICKS() - start_tick) >= timeout_ms) {
                result = UART_ABSTRACTION_TIMEOUT;
                goto cleanup;
            }
        }
        
        /* Read byte */
        data[i] = LL_USART_ReceiveData8(handle->uart_instance);
    }
    
    result = UART_ABSTRACTION_OK;
    
#else
    /* HAL receive */
    HAL_StatusTypeDef hal_result = HAL_UART_Receive(handle->huart, data, size, timeout_ms);
    result = hal_to_abstraction_status(hal_result);
#endif
    
cleanup:
#ifdef USE_FREERTOS
    xSemaphoreGive(handle->rx_mutex);
#endif
    
    return result;
}

uart_abstraction_status_t uart_abstraction_transmit_it(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_STM32_LL_DRIVERS
    return uart_ll_transmit(handle, data, size);
#else
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit_IT(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
#endif
}

uart_abstraction_status_t uart_abstraction_receive_it(uart_abstraction_handle_t *handle,
                                                      uint8_t *data,
                                                      uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_STM32_LL_DRIVERS
    return uart_ll_receive(handle, data, size);
#else
    HAL_StatusTypeDef hal_result = HAL_UART_Receive_IT(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
#endif
}

uart_abstraction_status_t uart_abstraction_transmit_dma(uart_abstraction_handle_t *handle,
                                                        uint8_t *data,
                                                        uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_STM32_LL_DRIVERS
    /* LL DMA implementation would go here */
    /* This is more complex and requires DMA LL configuration */
    return UART_ABSTRACTION_ERROR; /* Not implemented yet */
#else
    if (handle->hdma_tx == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    HAL_StatusTypeDef hal_result = HAL_UART_Transmit_DMA(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
#endif
}

uart_abstraction_status_t uart_abstraction_receive_dma(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size)
{
    if (handle == NULL || data == NULL || size == 0) {
        return UART_ABSTRACTION_ERROR;
    }
    
#if USE_STM32_LL_DRIVERS
    /* LL DMA implementation would go here */
    /* This is more complex and requires DMA LL configuration */
    return UART_ABSTRACTION_ERROR; /* Not implemented yet */
#else
    if (handle->hdma_rx == NULL) {
        return UART_ABSTRACTION_ERROR;
    }
    HAL_StatusTypeDef hal_result = HAL_UART_Receive_DMA(handle->huart, data, size);
    return hal_to_abstraction_status(hal_result);
#endif
}

uint16_t uart_abstraction_get_rx_count(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
#if USE_STM32_LL_DRIVERS
    return handle->rx_count;
#else
    if (handle->hdma_rx) {
        return handle->hdma_rx->Instance->NDTR;
    }
    return 0;
#endif
}

void uart_abstraction_flush_rx(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
#if USE_STM32_LL_DRIVERS
    /* Clear RX flag and read data register */
    if (LL_USART_IsActiveFlag_RXNE(handle->uart_instance)) {
        LL_USART_ReceiveData8(handle->uart_instance);
    }
    LL_USART_ClearFlag_ORE(handle->uart_instance);
#else
    __HAL_UART_CLEAR_OREFLAG(handle->huart);
#endif
}

void uart_abstraction_flush_tx(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return;
    }
    
#if USE_STM32_LL_DRIVERS
    LL_USART_ClearFlag_TC(handle->uart_instance);
#else
    __HAL_UART_CLEAR_FLAG(handle->huart, UART_FLAG_TC);
#endif
}

uint32_t uart_abstraction_is_tx_complete(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
#if USE_STM32_LL_DRIVERS
    return (handle->tx_state == 0) ? 1 : 0;
#else
    return (handle->huart->gState == HAL_UART_STATE_READY) ? 1 : 0;
#endif
}

uint32_t uart_abstraction_is_rx_complete(uart_abstraction_handle_t *handle)
{
    if (handle == NULL) {
        return 0;
    }
    
#if USE_STM32_LL_DRIVERS
    return (handle->rx_state == 0) ? 1 : 0;
#else
    return (handle->huart->RxState == HAL_UART_STATE_READY) ? 1 : 0;
#endif
}

/*******************************************************************************
 * CMSIS-RTOS2 Integration
 ******************************************************************************/
#if defined(USE_FREERTOS) && defined(USE_CMSIS_RTOS)

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
