/*
 * logging.c
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#include "logging.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// Module state
static uart_drv_t *log_uart = NULL;
static QueueHandle_t log_queue = NULL;
static QueueHandle_t telemetry_queue = NULL;
static LogLevel current_level = LOG_LEVEL_INFO;

// Internal task prototype
static void log_task(void *arg);


// Fully contained timestamp stub
static Timestamp get_current_timestamp(void)
{
    TickType_t ticks = xTaskGetTickCount();
    Timestamp ts;
    ts.seconds = ticks / 1000;
    ts.subseconds = ticks % 1000;
    return ts;
}

// Public initialization
void log_init(uart_drv_t *drv)
{
#if LOGGING_ENABLED
    log_uart = drv;
    log_queue = xQueueCreate(LOG_QUEUE_DEPTH, sizeof(LogEntry));
    telemetry_queue = xQueueCreate(16, sizeof(TelemetryPacket));

    if (log_queue) {
        xTaskCreate(log_task, "Logger", LOG_TASK_STACK, NULL, LOG_TASK_PRIO, NULL);
    }
#endif
}

// Log write API (non-blocking enqueue)
void log_write(LogLevel level, const char *fmt, ...)
{
#if LOGGING_ENABLED
    if (level < current_level) return;
    if (!log_queue) return;

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


// Logging task: drain queue and transmit
static void log_task(void *arg)
{
    LogEntry entry;
    char out_buf[256];

    for (;;) {
        // Process log queue first
        if (xQueueReceive(log_queue, &entry, pdMS_TO_TICKS(10)) == pdPASS) {
            int len = snprintf(out_buf, sizeof(out_buf),
                               "[%lu.%03lu] [%d] %s\r\n",
                               entry.ts.seconds,
                               entry.ts.subseconds,
                               entry.level,
                               entry.payload);

            uart_send_blocking(log_uart, (uint8_t *)out_buf, len, 100);
        }

        // Then check telemetry queue
        TelemetryPacket pkt;
        if (xQueueReceive(telemetry_queue, &pkt, 0) == pdPASS) {
            int len = snprintf(out_buf, sizeof(out_buf),
                               "[%lu.%03lu] TLM sensor1=%lu sensor2=%.2f\r\n",
                               get_current_timestamp().seconds,
                               get_current_timestamp().subseconds,
                               pkt.sensor1, pkt.sensor2);

            uart_send_blocking(log_uart, (uint8_t *)out_buf, len, 100);
        }
    }
}

