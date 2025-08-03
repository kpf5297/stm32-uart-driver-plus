/**
 * @file uart_driver_abstraction.h
 * @brief Abstraction layer for STM32 HAL vs LL drivers
 *
 * This header provides a unified interface that works with both
 * STM32 HAL and LL (Low Layer) drivers, allowing the same UART
 * driver code to work with either underlying implementation.
 */

#ifndef UART_DRIVER_ABSTRACTION_H
#define UART_DRIVER_ABSTRACTION_H

#include "uart_driver_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Include appropriate headers based on configuration
 ******************************************************************************/
#if USE_STM32_LL_DRIVERS
  /* LL Driver includes */
  #include "stm32f4xx_ll_usart.h"
  #include "stm32f4xx_ll_dma.h"
  #include "stm32f4xx_ll_gpio.h"
  #include "stm32f4xx_ll_rcc.h"
  
  /* LL-specific type definitions */
  typedef USART_TypeDef* UART_Instance_t;
  typedef DMA_TypeDef*   DMA_Instance_t;
  typedef uint32_t       DMA_Stream_t;
  
#else
  /* HAL Driver includes */
  #include "stm32f4xx_hal.h"
  
  /* HAL-specific type definitions */
  typedef UART_HandleTypeDef* UART_Instance_t;
  typedef DMA_HandleTypeDef*  DMA_Instance_t;
  typedef uint32_t           DMA_Stream_t;
  
#endif

/*******************************************************************************
 * Unified Status Codes
 ******************************************************************************/
typedef enum {
    UART_ABSTRACTION_OK = 0,
    UART_ABSTRACTION_ERROR,
    UART_ABSTRACTION_BUSY,
    UART_ABSTRACTION_TIMEOUT
} uart_abstraction_status_t;

/*******************************************************************************
 * Unified Handle Structure
 ******************************************************************************/
typedef struct {
#if USE_STM32_LL_DRIVERS
    USART_TypeDef *uart_instance;
    DMA_TypeDef   *dma_tx_instance;
    DMA_TypeDef   *dma_rx_instance;
    uint32_t       dma_tx_stream;
    uint32_t       dma_rx_stream;
    uint32_t       dma_tx_channel;
    uint32_t       dma_rx_channel;
#else
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_tx;
    DMA_HandleTypeDef  *hdma_rx;
#endif
    
    /* Common fields for both HAL and LL */
    volatile uint32_t tx_state;
    volatile uint32_t rx_state;
    uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint16_t tx_size;
    uint16_t rx_size;
    uint16_t tx_count;
    uint16_t rx_count;
    
#ifdef USE_FREERTOS
    SemaphoreHandle_t tx_mutex;
    SemaphoreHandle_t rx_mutex;
    SemaphoreHandle_t tx_complete_sem;
    SemaphoreHandle_t rx_complete_sem;
#endif
} uart_abstraction_handle_t;

/*******************************************************************************
 * Function Prototypes - Unified API
 ******************************************************************************/

/**
 * @brief Initialize UART abstraction layer
 */
uart_abstraction_status_t uart_abstraction_init(uart_abstraction_handle_t *handle);

/**
 * @brief Deinitialize UART abstraction layer
 */
void uart_abstraction_deinit(uart_abstraction_handle_t *handle);

/**
 * @brief Transmit data (blocking)
 */
uart_abstraction_status_t uart_abstraction_transmit(uart_abstraction_handle_t *handle,
                                                    uint8_t *data,
                                                    uint16_t size,
                                                    uint32_t timeout_ms);

/**
 * @brief Receive data (blocking)
 */
uart_abstraction_status_t uart_abstraction_receive(uart_abstraction_handle_t *handle,
                                                   uint8_t *data,
                                                   uint16_t size,
                                                   uint32_t timeout_ms);

/**
 * @brief Transmit data (non-blocking with interrupt)
 */
uart_abstraction_status_t uart_abstraction_transmit_it(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size);

/**
 * @brief Receive data (non-blocking with interrupt)
 */
uart_abstraction_status_t uart_abstraction_receive_it(uart_abstraction_handle_t *handle,
                                                      uint8_t *data,
                                                      uint16_t size);

/**
 * @brief Transmit data using DMA
 */
uart_abstraction_status_t uart_abstraction_transmit_dma(uart_abstraction_handle_t *handle,
                                                        uint8_t *data,
                                                        uint16_t size);

/**
 * @brief Receive data using DMA
 */
uart_abstraction_status_t uart_abstraction_receive_dma(uart_abstraction_handle_t *handle,
                                                       uint8_t *data,
                                                       uint16_t size);

/**
 * @brief Get number of bytes available in RX buffer (for DMA)
 */
uint16_t uart_abstraction_get_rx_count(uart_abstraction_handle_t *handle);

/**
 * @brief Flush RX buffer
 */
void uart_abstraction_flush_rx(uart_abstraction_handle_t *handle);

/**
 * @brief Flush TX buffer
 */
void uart_abstraction_flush_tx(uart_abstraction_handle_t *handle);

/**
 * @brief Check if transmission is complete
 */
uint32_t uart_abstraction_is_tx_complete(uart_abstraction_handle_t *handle);

/**
 * @brief Check if reception is complete
 */
uint32_t uart_abstraction_is_rx_complete(uart_abstraction_handle_t *handle);

/*******************************************************************************
 * CMSIS-RTOS2 Integration Functions
 ******************************************************************************/
#if defined(USE_FREERTOS) && defined(USE_CMSIS_RTOS)

/**
 * @brief CMSIS-RTOS2 compatible transmit with timeout
 */
uart_abstraction_status_t uart_abstraction_transmit_rtos(uart_abstraction_handle_t *handle,
                                                         uint8_t *data,
                                                         uint16_t size,
                                                         uint32_t timeout_ms);

/**
 * @brief CMSIS-RTOS2 compatible receive with timeout
 */
uart_abstraction_status_t uart_abstraction_receive_rtos(uart_abstraction_handle_t *handle,
                                                        uint8_t *data,
                                                        uint16_t size,
                                                        uint32_t timeout_ms);

#endif /* USE_FREERTOS && USE_CMSIS_RTOS */

#ifdef __cplusplus
}
#endif

#endif /* UART_DRIVER_ABSTRACTION_H */
