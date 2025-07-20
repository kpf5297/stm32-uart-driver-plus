#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>
#include <stdbool.h>
#include "uart_driver.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LOGGING_ENABLED
#define LOGGING_ENABLED 1
#endif

#ifndef MAX_LOG_PAYLOAD
#define MAX_LOG_PAYLOAD 128
#endif

#ifndef LOG_QUEUE_DEPTH
#define LOG_QUEUE_DEPTH 32
#endif

#ifndef LOG_TASK_STACK
#define LOG_TASK_STACK 512
#endif

#ifndef LOG_TASK_PRIO
#define LOG_TASK_PRIO (tskIDLE_PRIORITY + 1)
#endif

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL
} LogLevel;

typedef struct {
    uint32_t seconds;
    uint32_t subseconds;
} Timestamp;

typedef struct {
    Timestamp ts;
    LogLevel  level;
    char      payload[MAX_LOG_PAYLOAD];
} LogEntry;

// API to initialize logger with uart_driver reference
void log_init(uart_drv_t *drv);

// Log message at specified level
void log_write(LogLevel level, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // LOGGING_H
