#ifndef COMMAND_MODULE_H
#define COMMAND_MODULE_H
/**
 * @file command_module.h
 * @brief Lightweight UART command interpreter.
 */

#include "uart_driver.h"

/* Module-level command interpreter defaults; application may override. */
#ifndef USE_CMD_INTERPRETER
#define USE_CMD_INTERPRETER 1
#endif
#ifndef CMD_MAX_LINE_LEN
#define CMD_MAX_LINE_LEN 128
#endif
#ifndef CMD_MAX_PARAMS
#define CMD_MAX_PARAMS 8
#endif
#ifndef CMD_TASK_PRIO
#define CMD_TASK_PRIO osPriorityNormal
#endif
#ifndef CMD_TASK_STACK
#define CMD_TASK_STACK 512
#endif

#if USE_CMD_INTERPRETER
#include "cmsis_os2.h"
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

#if USE_CMD_INTERPRETER

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
/** Send a string as a single blocking DMA transfer. */
uart_status_t cmd_send_blocking(const char *s);

#else

static inline void cmd_init(uart_drv_t *uart) {(void)uart;}
static inline void cmd_write(const char *s) {(void)s;}
static inline void cmd_printf(const char *fmt, ...) {(void)fmt;}

#endif // USE_CMD_INTERPRETER
#endif // COMMAND_MODULE_H
