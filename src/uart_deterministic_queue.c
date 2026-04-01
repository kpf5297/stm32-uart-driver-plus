#include "uart_deterministic_queue.h"

#include <string.h>

static bool is_valid(const uart_deterministic_queue_t *queue) {
    return queue != NULL && queue->storage != NULL && queue->element_size > 0 && queue->capacity > 0;
}

uart_queue_status_t uart_queue_init(uart_deterministic_queue_t *queue,
                                    uint8_t *storage,
                                    size_t element_size,
                                    size_t capacity) {
    if (!queue || !storage || element_size == 0 || capacity == 0) {
        return UART_QUEUE_INVALID;
    }

    queue->storage = storage;
    queue->element_size = element_size;
    queue->capacity = capacity;
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    return UART_QUEUE_OK;
}

uart_queue_status_t uart_queue_push(uart_deterministic_queue_t *queue,
                                    const void *element) {
    if (!is_valid(queue) || !element) {
        return UART_QUEUE_INVALID;
    }
    if (queue->count == queue->capacity) {
        return UART_QUEUE_FULL;
    }

    memcpy(&queue->storage[queue->tail * queue->element_size], element, queue->element_size);
    queue->tail = (queue->tail + 1U) % queue->capacity;
    queue->count++;
    return UART_QUEUE_OK;
}

uart_queue_status_t uart_queue_pop(uart_deterministic_queue_t *queue,
                                   void *element_out) {
    if (!is_valid(queue) || !element_out) {
        return UART_QUEUE_INVALID;
    }
    if (queue->count == 0) {
        return UART_QUEUE_EMPTY;
    }

    memcpy(element_out, &queue->storage[queue->head * queue->element_size], queue->element_size);
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    return UART_QUEUE_OK;
}

void uart_queue_reset(uart_deterministic_queue_t *queue) {
    if (!is_valid(queue)) {
        return;
    }
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}

size_t uart_queue_count(const uart_deterministic_queue_t *queue) {
    if (!is_valid(queue)) {
        return 0;
    }
    return queue->count;
}

size_t uart_queue_capacity(const uart_deterministic_queue_t *queue) {
    if (!is_valid(queue)) {
        return 0;
    }
    return queue->capacity;
}

bool uart_queue_is_full(const uart_deterministic_queue_t *queue) {
    return is_valid(queue) && queue->count == queue->capacity;
}

bool uart_queue_is_empty(const uart_deterministic_queue_t *queue) {
    return !is_valid(queue) || queue->count == 0;
}
