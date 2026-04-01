#ifndef UART_DETERMINISTIC_QUEUE_H
#define UART_DETERMINISTIC_QUEUE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_QUEUE_OK = 0,
    UART_QUEUE_FULL,
    UART_QUEUE_EMPTY,
    UART_QUEUE_INVALID
} uart_queue_status_t;

typedef struct {
    uint8_t *storage;
    size_t element_size;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
} uart_deterministic_queue_t;

uart_queue_status_t uart_queue_init(uart_deterministic_queue_t *queue,
                                    uint8_t *storage,
                                    size_t element_size,
                                    size_t capacity);

uart_queue_status_t uart_queue_push(uart_deterministic_queue_t *queue,
                                    const void *element);

uart_queue_status_t uart_queue_pop(uart_deterministic_queue_t *queue,
                                   void *element_out);

void uart_queue_reset(uart_deterministic_queue_t *queue);

size_t uart_queue_count(const uart_deterministic_queue_t *queue);
size_t uart_queue_capacity(const uart_deterministic_queue_t *queue);
bool uart_queue_is_full(const uart_deterministic_queue_t *queue);
bool uart_queue_is_empty(const uart_deterministic_queue_t *queue);

#ifdef __cplusplus
}
#endif

#endif
