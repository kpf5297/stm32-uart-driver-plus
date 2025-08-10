#ifndef LOGGING_H
#define LOGGING_H
/**
 * @file logging.h
 * @brief Simple printf style logging and telemetry helpers.
 */

#include <stdint.h>
#include <stdbool.h>
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "uart_driver_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/** Severity levels for log entries. */
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel;

/** Timestamp used in log entries. */
typedef struct {
    uint32_t seconds;
    uint32_t subseconds;
} Timestamp;

/** Single log entry as stored in the queue. */
typedef struct {
    Timestamp ts;
    LogLevel  level;
    char      payload[MAX_LOG_PAYLOAD];
} LogEntry;

/** Example telemetry packet structure. */
typedef struct {
    uint32_t sensor1;
    float sensor2;
} TelemetryPacket;

/**
 * @brief Initialize logging facilities.
 *
 * The logger transmits log lines using the provided UART driver.
 */
void log_init(uart_drv_t *drv);

/**
 * @brief Queue a formatted log message.
 *
 * @param level Log severity level.
 * @param fmt   printf style format string followed by arguments.
 */
void log_write(LogLevel level, const char *fmt, ...);

/** Send a telemetry packet. */
void telemetry_send(const TelemetryPacket *pkt);

/** Set the minimum severity level for log output. */
void log_set_level(LogLevel level);

#ifdef __cplusplus
}
#endif

#endif // LOGGING_H
