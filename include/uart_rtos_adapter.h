#ifndef UART_RTOS_ADAPTER_H
#define UART_RTOS_ADAPTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_RTOS_OP_OK = 0,
    UART_RTOS_OP_TIMEOUT,
    UART_RTOS_OP_ERROR
} uart_rtos_op_status_t;

typedef struct {
    void *(*create_mutex)(void *ctx);
    void (*delete_mutex)(void *ctx, void *mutex);
    uart_rtos_op_status_t (*mutex_lock)(void *ctx, void *mutex, uint32_t timeout_ms);
    void (*mutex_unlock)(void *ctx, void *mutex);
    void (*delay_ms)(void *ctx, uint32_t delay_ms);
    uint32_t (*get_ticks)(void *ctx);
} uart_rtos_adapter_t;

const uart_rtos_adapter_t *uart_rtos_get_default_adapter(void);

#ifdef __cplusplus
}
#endif

#endif
