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
#include <stdbool.h>

#ifndef USE_CMD_INTERPRETER
#define USE_CMD_INTERPRETER 1
#endif

#if USE_CMD_INTERPRETER

#ifndef CMD_MAX_LINE_LEN
#define CMD_MAX_LINE_LEN   128
#endif
#ifndef CMD_MAX_PARAMS
#define CMD_MAX_PARAMS     8
#endif
#ifndef CMD_TASK_PRIO
#define CMD_TASK_PRIO      (tskIDLE_PRIORITY + 1)
#endif
#ifndef CMD_TASK_STACK
#define CMD_TASK_STACK     256
#endif

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

#endif // USE_CMD_INTERPRETER
#endif // COMMAND_MODULE_H
