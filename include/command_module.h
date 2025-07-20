#ifndef COMMAND_MODULE_H
#define COMMAND_MODULE_H
/**
 * @file command_module.h
 * @brief Lightweight UART command interpreter.
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uart_driver.h"
#include "uart_driver_config.h"
#include <stdbool.h>

#if USE_CMD_INTERPRETER

/** Parsed argument list provided to command handlers. */
typedef struct {
    int   argc;
    char *argv[CMD_MAX_PARAMS];
} Args;

/** Command descriptor used for registration. */
typedef struct {
    const char *name;
    void      (*handler)(Args *args);
} Command;

/** Application-defined command table */
extern const Command cmd_list[];
extern const size_t  cmd_count;

/**
 * @brief Initialize command interpreter.
 *
 * Registers UART callbacks and launches the command processing task.
 */
void cmd_init(uart_drv_t *uart);

/** Write a string using the interpreter's UART. */
void cmd_write(const char *s);
/** printf style helper using the interpreter's UART. */
void cmd_printf(const char *fmt, ...);

#endif // USE_CMD_INTERPRETER
#endif // COMMAND_MODULE_H
