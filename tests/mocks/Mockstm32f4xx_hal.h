#ifndef MOCK_STM32F4XX_HAL_H
#define MOCK_STM32F4XX_HAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t CR;
    uint32_t NDTR;
} DMA_Stream_TypeDef;

typedef struct {
    uint32_t SR;
    uint32_t DR;
} USART_TypeDef;

typedef struct DMA_HandleTypeDef {
    DMA_Stream_TypeDef *Instance;
} DMA_HandleTypeDef;

typedef struct UART_HandleTypeDef {
    USART_TypeDef *Instance;
    DMA_HandleTypeDef *hdmatx;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

typedef enum {
    HAL_OK = 0,
    HAL_ERROR = 1
} HAL_StatusTypeDef;

#define UART_IT_IDLE (0x10U)
#define UART_FLAG_IDLE (0x20U)
#define UART_FLAG_TC (0x40U)
#define DMA_SxCR_CIRC (0x01U)

#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)

#define __HAL_UART_ENABLE_IT(hu, it) ((void)(hu), (void)(it))
#define __HAL_UART_DISABLE_IT(hu, it) ((void)(hu), (void)(it))
#define __HAL_UART_CLEAR_OREFLAG(hu) ((void)(hu))
#define __HAL_UART_CLEAR_FLAG(hu, flag) ((void)(hu), (void)(flag))
#define __HAL_UART_GET_FLAG(hu, flag) ((((hu) != NULL) && ((hu)->Instance != NULL) && (((hu)->Instance->SR & (flag)) != 0U)) ? 1U : 0U)

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *data, size_t len);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, size_t len);

void mock_hal_reset(void);
void mock_hal_set_tx_return(HAL_StatusTypeDef status);
void mock_hal_set_rx_return(HAL_StatusTypeDef status);
size_t mock_hal_get_tx_call_count(void);
size_t mock_hal_get_rx_call_count(void);
size_t mock_hal_get_tx_len_at(size_t index);
uint8_t mock_hal_get_tx_first_byte_at(size_t index);

#endif
