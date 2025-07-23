#ifndef CMSIS_UART_ADAPTER_H
#define CMSIS_UART_ADAPTER_H

#include "uart_driver_config.h"

#if UART_BACKEND == UART_BACKEND_CMSIS
#include "Driver_USART.h"

#ifndef HAL_StatusTypeDef
#define HAL_StatusTypeDef int32_t
#endif

#ifndef HAL_OK
#define HAL_OK ARM_DRIVER_OK
#endif

static inline HAL_StatusTypeDef CMSIS_UART_Transmit(ARM_DRIVER_USART *drv,
                                                   uint8_t *data,
                                                   size_t len,
                                                   uint32_t timeout)
{
    (void)timeout; /* CMSIS driver handles blocking internally */
    return drv->Send(data, len);
}

static inline HAL_StatusTypeDef CMSIS_UART_Receive(ARM_DRIVER_USART *drv,
                                                  uint8_t *data,
                                                  size_t len,
                                                  uint32_t timeout)
{
    (void)timeout;
    return drv->Receive(data, len);
}

#define HAL_UART_Transmit(huart, data, len, timeout) \
        CMSIS_UART_Transmit(huart, data, len, timeout)
#define HAL_UART_Receive(huart, data, len, timeout) \
        CMSIS_UART_Receive(huart, data, len, timeout)
#define HAL_UART_Transmit_IT(huart, data, len) CMSIS_UART_Transmit_IT(huart, data, len)
#define HAL_UART_Receive_IT(huart, data, len)  CMSIS_UART_Receive_IT(huart, data, len)
#define HAL_UART_Transmit_DMA(huart, data, len) CMSIS_UART_Transmit_DMA(huart, data, len)
#define HAL_UART_Receive_DMA(huart, data, len)  CMSIS_UART_Receive_DMA(huart, data, len)

#define __HAL_UART_CLEAR_OREFLAG(huart) do { (void)(huart); } while(0)
#define __HAL_UART_CLEAR_FLAG(huart, flag) do { (void)(huart); (void)(flag); } while(0)

typedef ARM_DRIVER_USART CMSIS_UART_HandleTypeDef;
typedef struct DMA_HandleTypeDef DMA_HandleTypeDef;

#endif /* UART_BACKEND == UART_BACKEND_CMSIS */

#endif /* CMSIS_UART_ADAPTER_H */
