#ifndef UART_DRIVER_CONFIG_H
#define UART_DRIVER_CONFIG_H

/* Configuration options for UART driver modules */

/* Command interpreter options */
#define USE_CMD_INTERPRETER   1
#define CMD_MAX_LINE_LEN      128
#define CMD_MAX_PARAMS        8
#define CMD_TASK_PRIO         (tskIDLE_PRIORITY + 1)
#define CMD_TASK_STACK        256

/* Logging and telemetry options */
#define LOGGING_ENABLED       1
#define TELEMETRY_ENABLED     1
#define LOG_TX_USE_DMA        0
#define TELEMETRY_TX_USE_DMA  0
#define LOG_QUEUE_DEPTH       32
#define TELEMETRY_QUEUE_DEPTH 16
#define MAX_LOG_PAYLOAD       128
#define MAX_TELEMETRY_PAYLOAD 64
#define DEFAULT_LOG_LEVEL     LOG_LEVEL_INFO
#define LOG_TASK_STACK        512
#define LOG_TASK_PRIO         (tskIDLE_PRIORITY + 1)

#endif /* UART_DRIVER_CONFIG_H */
