/**
 * @file logging.c
 * @brief Logging and telemetry utilities implementation.
 * 
 * Provides structured logging with configurable levels and telemetry
 * packet transmission over UART. Includes fault monitoring integration.
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
static osMessageQueueId_t log_queue = NULL;
static osMessageQueueId_t telemetry_queue = NULL;
static LogLevel current_level = LOG_LEVEL_INFO;

/**
 * @brief Transmit data using the configured UART mode.
 * @param data Pointer to data to transmit
 * @param len Length of data
 * @param telemetry Whether this is telemetry data (unused currently)
 */
static void log_tx(const uint8_t *data, size_t len, bool telemetry)
{
    /* DMA-only operation */
    if (log_uart && log_uart->hdma_tx) {
        uart_send_nb(log_uart, (uint8_t *)data, len);
        return;
    }
    /* Enforce DMA-only: if no HDMA, silently ignore */
    (void)data; (void)len; (void)telemetry;
}

// Internal task prototypes
static void log_task(void *arg);
static void process_log_queue(char *out_buf, size_t len);
static void process_telemetry_queue(char *out_buf, size_t len);
static void perform_fault_check(char *out_buf, size_t len);


/**
 * @brief Get current timestamp in standard format.
 * @return Timestamp structure with seconds and subseconds
 */
static Timestamp get_current_timestamp(void)
{
    uint32_t ticks = (uint32_t)GET_TICKS();
    Timestamp ts;
    ts.seconds = ticks / TIMESTAMP_SCALE_FACTOR;
    ts.subseconds = ticks % TIMESTAMP_SCALE_FACTOR;
    return ts;
}

/**
 * @brief Initialize logging system with UART driver.
 * @param drv Pointer to UART driver instance to use for output
 */
void log_init(uart_drv_t *drv)
{
#if LOGGING_ENABLED
    log_uart = drv;
    log_queue = osMessageQueueNew(LOG_QUEUE_DEPTH, sizeof(LogEntry), NULL);
    telemetry_queue = osMessageQueueNew(TELEMETRY_QUEUE_DEPTH, sizeof(TelemetryPacket), NULL);
    fault_init();

    if (log_queue) {
        osThreadAttr_t attr = {0};
        attr.priority = LOG_TASK_PRIO;
        attr.stack_size = LOG_TASK_STACK;
        osThreadNew(log_task, NULL, &attr);
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

    osMessageQueuePut(log_queue, &entry, 0, 0);  // No block if queue full
#endif
}

void telemetry_send(const TelemetryPacket *pkt)
{
#if LOGGING_ENABLED
    if (telemetry_queue && pkt) {
        osMessageQueuePut(telemetry_queue, pkt, 0, 0);  // No blocking
    }
#endif
}

void log_set_level(LogLevel level)
{
#if LOGGING_ENABLED
    current_level = level;
#endif
}


/**
 * @brief Background logging task - processes queues and monitors faults.
 * @param arg Unused task parameter
 */
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
    if (osMessageQueueGet(log_queue, &entry, NULL, MS_TO_TICKS(LOG_TASK_DELAY_MS)) == osOK) {
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
    if (osMessageQueueGet(telemetry_queue, &pkt, NULL, 0) == osOK) {
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
    static uint32_t last_fault_check = 0;
    uint32_t now = GET_TICKS();
    if ((now - last_fault_check) >= MS_TO_TICKS(FAULT_CHECK_INTERVAL_MS)) {
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

