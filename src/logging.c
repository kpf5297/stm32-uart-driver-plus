/**
 * @file logging.c
 * @brief Logging and telemetry utilities.
 */
#include "logging.h"
#include "fault_module.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define DEFAULT_UART_TIMEOUT_MS 100
#define LOG_TASK_DELAY_MS 10
#define FAULT_CHECK_INTERVAL_MS 500
#define TASK_DELAY_MS 1
#define TIMESTAMP_SCALE_FACTOR 1000

// Module state
static uart_drv_t *log_uart = NULL;
static QueueHandle_t log_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;
static LogLevel current_level = LOG_LEVEL_INFO;

static void log_tx(const uint8_t *data, size_t len, bool telemetry)
{
#if UART_TX_MODE == UART_MODE_DMA
    if (log_uart && log_uart->hdma_tx) {
        uart_send_dma_blocking(log_uart, data, len, DEFAULT_UART_TIMEOUT_MS);
        return;
    }
#elif UART_TX_MODE == UART_MODE_INTERRUPT
    if (log_uart) {
        if (uart_send_nb(log_uart, data, len) == UART_OK) {
            while (uart_get_status(log_uart) == UART_BUSY) {
                vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
            }
        }
        return;
    }
#endif
    if (log_uart) {
        uart_send_blocking(log_uart, data, len, DEFAULT_UART_TIMEOUT_MS);
    }
}

// Internal task prototypes
static void log_task(void *arg);
static void process_log_queue(char *out_buf, size_t len);
static void process_telemetry_queue(char *out_buf, size_t len);
static void perform_fault_check(char *out_buf, size_t len);


// Fully contained timestamp stub
static Timestamp get_current_timestamp(void)
{
    TickType_t ticks = xTaskGetTickCount();
    Timestamp ts;
    ts.seconds = ticks / TIMESTAMP_SCALE_FACTOR;
    ts.subseconds = ticks % TIMESTAMP_SCALE_FACTOR;
    return ts;
}

// Public initialization
void log_init(uart_drv_t *drv)
{
#if LOGGING_ENABLED
    log_uart = drv;
    log_queue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(LogEntry));
    telemetry_queue = xQueueCreate(16, sizeof(TelemetryPacket));
    fault_init();

    if (log_queue) {
        xTaskCreate(log_task, "Logger", LOG_TASK_STACK, NULL, LOG_TASK_PRIO, NULL);
    }
#endif
}

// Log write API (non-blocking enqueue)
void log_write(LogLevel level, const char *fmt, ...)
{
#if LOGGING_ENABLED
    if (level < current_level) {
        return;
    }
    if (!log_queue) {
        return;
    }

    LogEntry entry = {0};
    entry.ts = get_current_timestamp();
    entry.level = level;

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.payload, sizeof(entry.payload), fmt, args);
    va_end(args);

    xQueueSend(log_queue, &entry, 0);  // No block if queue full
#endif
}

void telemetry_send(const TelemetryPacket *pkt)
{
#if LOGGING_ENABLED
    if (telemetry_queue && pkt) {
        xQueueSend(telemetry_queue, pkt, 0);  // No blocking
    }
#endif
}

void log_set_level(LogLevel level)
{
#if LOGGING_ENABLED
    current_level = level;
#endif
}


// Logging task: drain queue and transmit
static void log_task(void *arg)
{
    char out_buf[256];

    for (;;) {
        process_log_queue(out_buf, sizeof(out_buf));
        process_telemetry_queue(out_buf, sizeof(out_buf));
        perform_fault_check(out_buf, sizeof(out_buf));
    }
}

static void process_log_queue(char *out_buf, size_t len)
{
    LogEntry entry;
    if (xQueueReceive(log_queue, &entry, pdMS_TO_TICKS(LOG_TASK_DELAY_MS)) == pdPASS) {
        int out_len = snprintf(out_buf, len,
                               "[%lu.%03lu] [%d] %s\r\n",
                               entry.ts.seconds,
                               entry.ts.subseconds,
                               entry.level,
                               entry.payload);
        log_tx((const uint8_t *)out_buf, out_len, false);
    }
}

static void process_telemetry_queue(char *out_buf, size_t len)
{
    TelemetryPacket pkt;
    if (xQueueReceive(telemetry_queue, &pkt, 0) == pdPASS) {
        int out_len = snprintf(out_buf, len,
                               "[%lu.%03lu] TLM sensor1=%lu sensor2=%.2f\r\n",
                               get_current_timestamp().seconds,
                               get_current_timestamp().subseconds,
                               pkt.sensor1, pkt.sensor2);
        log_tx((const uint8_t *)out_buf, out_len, true);
    }
}

static void perform_fault_check(char *out_buf, size_t len)
{
    static TickType_t last_fault_check = 0;
    TickType_t now = xTaskGetTickCount();
    if (now - last_fault_check >= pdMS_TO_TICKS(FAULT_CHECK_INTERVAL_MS)) {
        last_fault_check = now;
        if (fault_state.active_mask) {
            Timestamp ts = get_current_timestamp();
            int out_len = snprintf(out_buf, len,
                                   "FLT,ACTIVE=0x%08lX,TIME=%lu.%03lus\r\n",
                                   fault_state.active_mask,
                                   ts.seconds, ts.subseconds);
            log_tx((const uint8_t *)out_buf, out_len, false);
        }
    }
}

