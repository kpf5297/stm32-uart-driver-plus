#include "unity.h"

#include "uart_driver.h"
#include "Mockstm32f4xx_hal.h"

#include <string.h>

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *hu);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *hu);

typedef struct {
    uint32_t now_ms;
    uint32_t delay_calls;
} fake_rtos_ctx_t;

static void *fake_create_mutex(void *ctx) {
    (void)ctx;
    static int token;
    return &token;
}

static void fake_delete_mutex(void *ctx, void *mutex) {
    (void)ctx;
    (void)mutex;
}

static uart_rtos_op_status_t fake_mutex_lock(void *ctx, void *mutex, uint32_t timeout_ms) {
    (void)ctx;
    (void)mutex;
    (void)timeout_ms;
    return UART_RTOS_OP_OK;
}

static void fake_mutex_unlock(void *ctx, void *mutex) {
    (void)ctx;
    (void)mutex;
}

static void fake_delay_ms(void *ctx, uint32_t delay_ms) {
    fake_rtos_ctx_t *rtos = (fake_rtos_ctx_t *)ctx;
    rtos->now_ms += delay_ms;
    rtos->delay_calls++;
}

static uint32_t fake_get_ticks(void *ctx) {
    fake_rtos_ctx_t *rtos = (fake_rtos_ctx_t *)ctx;
    return rtos->now_ms;
}

static const uart_rtos_adapter_t g_fake_rtos = {
    .create_mutex = fake_create_mutex,
    .delete_mutex = fake_delete_mutex,
    .mutex_lock = fake_mutex_lock,
    .mutex_unlock = fake_mutex_unlock,
    .delay_ms = fake_delay_ms,
    .get_ticks = fake_get_ticks,
};

static UART_HandleTypeDef g_huart;
static DMA_HandleTypeDef g_dma_tx;
static DMA_HandleTypeDef g_dma_rx;
static USART_TypeDef g_usart;
static DMA_Stream_TypeDef g_dma_tx_stream;
static DMA_Stream_TypeDef g_dma_rx_stream;
static fake_rtos_ctx_t g_rtos;
static uart_drv_t g_drv;

static int g_rx_available_cb_count = 0;

static void test_callback(uart_event_t evt, void *ctx) {
    (void)ctx;
    if (evt == UART_EVT_RX_AVAILABLE) {
        g_rx_available_cb_count++;
    }
}

void setUp(void) {
    memset(&g_huart, 0, sizeof(g_huart));
    memset(&g_dma_tx, 0, sizeof(g_dma_tx));
    memset(&g_dma_rx, 0, sizeof(g_dma_rx));
    memset(&g_usart, 0, sizeof(g_usart));
    memset(&g_dma_tx_stream, 0, sizeof(g_dma_tx_stream));
    memset(&g_dma_rx_stream, 0, sizeof(g_dma_rx_stream));
    memset(&g_rtos, 0, sizeof(g_rtos));
    memset(&g_drv, 0, sizeof(g_drv));

    g_dma_tx.Instance = &g_dma_tx_stream;
    g_dma_rx.Instance = &g_dma_rx_stream;
    g_huart.Instance = &g_usart;

    mock_hal_reset();
    g_rx_available_cb_count = 0;

    uart_set_rtos_adapter(&g_drv, &g_fake_rtos, &g_rtos);
    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_init(&g_drv, &g_huart, &g_dma_tx, &g_dma_rx));
}

void tearDown(void) {
    uart_deinit(&g_drv);
}

static void test_dma_blocking_timeout_is_deterministic(void) {
    uint8_t payload[] = {0xA5};

    mock_hal_set_tx_return(HAL_OK);
    TEST_ASSERT_EQUAL_UINT(UART_ERROR, uart_send_dma_blocking(&g_drv, payload, sizeof(payload), 5U));
    TEST_ASSERT_EQUAL_UINT(1U, mock_hal_get_tx_call_count());
    TEST_ASSERT_EQUAL_UINT(5U, g_rtos.delay_calls);
}

static void test_tx_completion_callback_drains_fifo_in_order(void) {
    uint8_t p1[] = {1U};
    uint8_t p2[] = {2U};
    uint8_t p3[] = {3U};

    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_send_nb(&g_drv, p1, sizeof(p1)));
    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_send_nb(&g_drv, p2, sizeof(p2)));
    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_send_nb(&g_drv, p3, sizeof(p3)));

    TEST_ASSERT_EQUAL_UINT(1U, mock_hal_get_tx_call_count());
    TEST_ASSERT_EQUAL_UINT(1U, mock_hal_get_tx_first_byte_at(0));
    TEST_ASSERT_EQUAL_UINT(2U, uart_get_tx_queue_count(&g_drv));

    HAL_UART_TxCpltCallback(&g_huart);
    TEST_ASSERT_EQUAL_UINT(2U, mock_hal_get_tx_call_count());
    TEST_ASSERT_EQUAL_UINT(2U, mock_hal_get_tx_first_byte_at(1));
    TEST_ASSERT_EQUAL_UINT(1U, uart_get_tx_queue_count(&g_drv));

    HAL_UART_TxCpltCallback(&g_huart);
    TEST_ASSERT_EQUAL_UINT(3U, mock_hal_get_tx_call_count());
    TEST_ASSERT_EQUAL_UINT(3U, mock_hal_get_tx_first_byte_at(2));
    TEST_ASSERT_EQUAL_UINT(0U, uart_get_tx_queue_count(&g_drv));

    HAL_UART_TxCpltCallback(&g_huart);
    TEST_ASSERT_EQUAL_UINT(3U, mock_hal_get_tx_call_count());
    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_get_status(&g_drv));
}

static void test_idle_event_callback_is_triggered_once_per_idle_check(void) {
    uint8_t circular[16] = {0};

    TEST_ASSERT_EQUAL_UINT(UART_OK, uart_start_circular_rx(&g_drv, circular, sizeof(circular)));
    uart_register_callback(&g_drv, test_callback, NULL);

    g_huart.Instance->SR |= UART_FLAG_IDLE;
    HAL_UART_RxCpltCallback(&g_huart);
    TEST_ASSERT_EQUAL_UINT(0U, g_rx_available_cb_count);

    uart_process_idle(&g_huart);
    TEST_ASSERT_EQUAL_UINT(1U, g_rx_available_cb_count);
}

int main(void) {
    UnityBegin("test_uart_driver_determinism");
    RUN_TEST(test_dma_blocking_timeout_is_deterministic);
    RUN_TEST(test_tx_completion_callback_drains_fifo_in_order);
    RUN_TEST(test_idle_event_callback_is_triggered_once_per_idle_check);
    return UnityEnd();
}
