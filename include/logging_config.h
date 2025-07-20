#ifndef LOGGING_CONFIG_H
#define LOGGING_CONFIG_H
/**
 * @file logging_config.h
 * @brief Build-time configuration for the logging module.
 */

// Enable/Disable logging and telemetry independently
#define LOGGING_ENABLED       1
#define TELEMETRY_ENABLED     1

// Select transmission mode: 0 = Interrupt, 1 = DMA
#define LOG_TX_USE_DMA        0
#define TELEMETRY_TX_USE_DMA  0

// Queue depths
#define LOG_QUEUE_DEPTH       32
#define TELEMETRY_QUEUE_DEPTH 16

// Payload sizes
#define MAX_LOG_PAYLOAD       128
#define MAX_TELEMETRY_PAYLOAD 64  // Adjust if telemetry struct larger

// Default log level
#define DEFAULT_LOG_LEVEL     LOG_LEVEL_INFO

// FreeRTOS task settings
#define LOG_TASK_STACK_SIZE   512
#define LOG_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)

#endif // LOGGING_CONFIG_H

