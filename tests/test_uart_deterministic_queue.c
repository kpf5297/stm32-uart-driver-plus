#include "uart_deterministic_queue.h"
#include "unity.h"

#include <stdint.h>

void setUp(void) {}
void tearDown(void) {}

static void test_fifo_order_and_wraparound(void) {
    uart_deterministic_queue_t q;
    uint8_t storage[4 * sizeof(uint32_t)] = {0};

    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_init(&q, storage, sizeof(uint32_t), 4));

    uint32_t a = 10, b = 20, c = 30, d = 40;
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &a));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &b));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &c));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &d));
    TEST_ASSERT_TRUE(uart_queue_is_full(&q));

    uint32_t out = 0;
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(10, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(20, out);

    uint32_t e = 50, f = 60;
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &e));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &f));
    TEST_ASSERT_TRUE(uart_queue_is_full(&q));

    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(30, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(40, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(50, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(60, out);
    TEST_ASSERT_TRUE(uart_queue_is_empty(&q));
}

static void test_full_and_empty_statuses(void) {
    uart_deterministic_queue_t q;
    uint8_t storage[2 * sizeof(uint32_t)] = {0};

    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_init(&q, storage, sizeof(uint32_t), 2));

    uint32_t x = 1, y = 2, z = 3;
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &x));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_push(&q, &y));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_FULL, uart_queue_push(&q, &z));

    uint32_t out = 0;
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(1, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_pop(&q, &out));
    TEST_ASSERT_EQUAL_UINT(2, out);
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_EMPTY, uart_queue_pop(&q, &out));
}

static void test_invalid_inputs(void) {
    uart_deterministic_queue_t q;
    uint8_t storage[4] = {0};
    uint8_t v = 0;

    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_init(NULL, storage, 1, 4));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_init(&q, NULL, 1, 4));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_init(&q, storage, 0, 4));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_init(&q, storage, 1, 0));

    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_OK, uart_queue_init(&q, storage, 1, 4));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_push(NULL, &v));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_push(&q, NULL));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_pop(NULL, &v));
    TEST_ASSERT_EQUAL_UINT(UART_QUEUE_INVALID, uart_queue_pop(&q, NULL));
}

int main(void) {
    UnityBegin("test_uart_deterministic_queue");
    RUN_TEST(test_fifo_order_and_wraparound);
    RUN_TEST(test_full_and_empty_statuses);
    RUN_TEST(test_invalid_inputs);
    return UnityEnd();
}
