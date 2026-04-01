#include "uart_rtos_adapter.h"

#ifndef UART_RTOS_USE_CMSIS
#define UART_RTOS_USE_CMSIS 1
#endif

#if UART_RTOS_USE_CMSIS
#include "cmsis_os2.h"

static void *cmsis_create_mutex(void *ctx) {
    (void)ctx;
    return osMutexNew(NULL);
}

static void cmsis_delete_mutex(void *ctx, void *mutex) {
    (void)ctx;
    if (mutex) {
        (void)osMutexDelete((osMutexId_t)mutex);
    }
}

static uart_rtos_op_status_t cmsis_mutex_lock(void *ctx, void *mutex, uint32_t timeout_ms) {
    (void)ctx;
    if (!mutex) {
        return UART_RTOS_OP_ERROR;
    }
    uint32_t timeout_ticks = (uint32_t)(((uint64_t)timeout_ms * osKernelGetTickFreq()) / 1000U);
    if (timeout_ms > 0U && timeout_ticks == 0U) {
        timeout_ticks = 1U;
    }
    osStatus_t status = osMutexAcquire((osMutexId_t)mutex, timeout_ticks);
    if (status == osOK) {
        return UART_RTOS_OP_OK;
    }
    if (status == osErrorTimeout) {
        return UART_RTOS_OP_TIMEOUT;
    }
    return UART_RTOS_OP_ERROR;
}

static void cmsis_mutex_unlock(void *ctx, void *mutex) {
    (void)ctx;
    if (mutex) {
        (void)osMutexRelease((osMutexId_t)mutex);
    }
}

static void cmsis_delay_ms(void *ctx, uint32_t delay_ms) {
    (void)ctx;
    uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t delay_ticks = (uint32_t)(((uint64_t)delay_ms * tick_freq) / 1000U);
    if (delay_ms > 0U && delay_ticks == 0U) {
        delay_ticks = 1U;
    }
    osDelay(delay_ticks);
}

static uint32_t cmsis_get_ticks(void *ctx) {
    (void)ctx;
    uint32_t tick_freq = osKernelGetTickFreq();
    uint32_t ticks = osKernelGetTickCount();
    if (tick_freq == 0U) {
        return 0U;
    }
    return (uint32_t)(((uint64_t)ticks * 1000U) / tick_freq);
}

static const uart_rtos_adapter_t g_default_adapter = {
    .create_mutex = cmsis_create_mutex,
    .delete_mutex = cmsis_delete_mutex,
    .mutex_lock = cmsis_mutex_lock,
    .mutex_unlock = cmsis_mutex_unlock,
    .delay_ms = cmsis_delay_ms,
    .get_ticks = cmsis_get_ticks,
};

#else

static void *noop_create_mutex(void *ctx) {
    (void)ctx;
    return (void *)1;
}

static void noop_delete_mutex(void *ctx, void *mutex) {
    (void)ctx;
    (void)mutex;
}

static uart_rtos_op_status_t noop_mutex_lock(void *ctx, void *mutex, uint32_t timeout_ms) {
    (void)ctx;
    (void)mutex;
    (void)timeout_ms;
    return UART_RTOS_OP_OK;
}

static void noop_mutex_unlock(void *ctx, void *mutex) {
    (void)ctx;
    (void)mutex;
}

static void noop_delay_ms(void *ctx, uint32_t delay_ms) {
    (void)ctx;
    (void)delay_ms;
}

static uint32_t noop_get_ticks(void *ctx) {
    (void)ctx;
    return 0;
}

static const uart_rtos_adapter_t g_default_adapter = {
    .create_mutex = noop_create_mutex,
    .delete_mutex = noop_delete_mutex,
    .mutex_lock = noop_mutex_lock,
    .mutex_unlock = noop_mutex_unlock,
    .delay_ms = noop_delay_ms,
    .get_ticks = noop_get_ticks,
};

#endif

const uart_rtos_adapter_t *uart_rtos_get_default_adapter(void) {
    return &g_default_adapter;
}
