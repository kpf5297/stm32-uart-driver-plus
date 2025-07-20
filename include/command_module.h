/*
 * command_module.h
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#ifndef COMMAND_MODULE_H
#define COMMAND_MODULE_H

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

// Parsed argument list
typedef struct {
    int   argc;
    char *argv[CMD_MAX_PARAMS];
} Args;

// Command descriptor
typedef struct {
    const char *name;
    void      (*handler)(Args *args);
} Command;

// Must be defined by application
extern const Command cmd_list[];
extern const size_t  cmd_count;

// Initialize command interpreter; shares uart driver instance
void cmd_init(uart_drv_t *uart);

#endif // USE_CMD_INTERPRETER
#endif // COMMAND_MODULE_H
