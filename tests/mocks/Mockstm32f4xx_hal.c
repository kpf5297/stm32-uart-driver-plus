#include "Mockstm32f4xx_hal.h"

#define MOCK_HAL_HISTORY_MAX 32

static HAL_StatusTypeDef g_tx_return = HAL_OK;
static HAL_StatusTypeDef g_rx_return = HAL_OK;
static size_t g_tx_call_count = 0;
static size_t g_rx_call_count = 0;
static size_t g_tx_len_history[MOCK_HAL_HISTORY_MAX];
static uint8_t g_tx_first_byte_history[MOCK_HAL_HISTORY_MAX];

void mock_hal_reset(void) {
    g_tx_return = HAL_OK;
    g_rx_return = HAL_OK;
    g_tx_call_count = 0;
    g_rx_call_count = 0;
    for (size_t i = 0; i < MOCK_HAL_HISTORY_MAX; ++i) {
        g_tx_len_history[i] = 0;
        g_tx_first_byte_history[i] = 0;
    }
}

void mock_hal_set_tx_return(HAL_StatusTypeDef status) {
    g_tx_return = status;
}

void mock_hal_set_rx_return(HAL_StatusTypeDef status) {
    g_rx_return = status;
}

size_t mock_hal_get_tx_call_count(void) {
    return g_tx_call_count;
}

size_t mock_hal_get_rx_call_count(void) {
    return g_rx_call_count;
}

size_t mock_hal_get_tx_len_at(size_t index) {
    if (index >= g_tx_call_count || index >= MOCK_HAL_HISTORY_MAX) {
        return 0;
    }
    return g_tx_len_history[index];
}

uint8_t mock_hal_get_tx_first_byte_at(size_t index) {
    if (index >= g_tx_call_count || index >= MOCK_HAL_HISTORY_MAX) {
        return 0;
    }
    return g_tx_first_byte_history[index];
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *data, size_t len) {
    (void)huart;
    size_t index = g_tx_call_count;
    if (index < MOCK_HAL_HISTORY_MAX) {
        g_tx_len_history[index] = len;
        g_tx_first_byte_history[index] = (len > 0U && data != NULL) ? data[0] : 0U;
    }
    g_tx_call_count++;
    return g_tx_return;
}

HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *data, size_t len) {
    (void)huart;
    (void)data;
    (void)len;
    g_rx_call_count++;
    return g_rx_return;
}
