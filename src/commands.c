/**
 * @file commands.c
 * @brief Example command handlers for UART driver CLI.
 * 
 * Implements various demonstration commands including help, echo,
 * arithmetic operations, fault management, and logging control.
 */

#include "commands.h"
#if USE_CMD_INTERPRETER
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fault_module.h"
#include "logging.h"

/**
 * @brief Helper function to send strings via command interface.
 * @param s String to send
 */
static void send_str(const char *s) {
    cmd_write(s);
}

/**
 * @brief 'help' command handler - list all available commands.
 * @param args Command arguments (unused)
 */
void cmd_help(Args *args) {
    send_str("Available commands:\r\n");
    for (size_t i = 0; i < cmd_count; i++) {
        send_str("  ");
        send_str(cmd_list[i].name);
        send_str("\r\n");
    }
}

/**
 * @brief 'echo' command handler - repeat back provided parameters.
 * @param args Command arguments to echo
 */
void cmd_echo(Args *args) {
    for (int i = 1; i < args->argc; i++) {
        send_str(args->argv[i]);
        if (i < args->argc - 1) {
            send_str(" ");
        }
    }
    send_str("\r\n");
}

/**
 * @brief 'add' command handler - add two integers and print result.
 * @param args Command arguments containing two numbers
 */
void cmd_add(Args *args) {
    if (args->argc != 3) {
        send_str("Usage: add <a> <b>\r\n");
        return;
    }
    int a = atoi(args->argv[1]);
    int b = atoi(args->argv[2]);
    char buf[32];
    snprintf(buf, sizeof(buf), "Sum: %d\r\n", a + b);
    send_str(buf);
}

/**
 * @brief 'faults' command handler - list active faults.
 * @param args Command arguments (unused)
 */
void cmd_faults(Args *args) {
    bool any = false;
    for (int i = 1; i < FAULT_COUNT; ++i) {
        if (fault_is_active((FaultCode)i)) {
            any = true;
            Timestamp ts = fault_state.last_set[i];
            char buf[64];
            snprintf(buf, sizeof(buf), "%s at %lu.%03lu s\r\n",
                     fault_to_string((FaultCode)i),
                     ts.seconds, ts.subseconds);
            send_str(buf);
        }
    }
    if (!any) {
        send_str("No active faults\r\n");
    }
}

/**
 * @brief 'fault_clear' command handler - clear specific or all faults.
 * @param args Command arguments containing fault code or "all"
 */
void cmd_fault_clear(Args *args) {
    if (args->argc != 2) {
        send_str("Usage: fault_clear <code>|all\r\n");
        return;
    }
    if (strcmp(args->argv[1], "all") == 0) {
        fault_clear_all();
        send_str("All faults cleared\r\n");
    } else {
        char *endptr;
        long code = strtol(args->argv[1], &endptr, 10);
        if (*endptr != '\0' || code <= 0 || code >= FAULT_COUNT) {
            send_str("Invalid code\r\n");
        } else {
            fault_clear((FaultCode)code);
            send_str("Fault cleared\r\n");
        }
    }
}

/**
 * @brief 'log_level' command handler - adjust logging verbosity.
 * @param args Command arguments containing log level name
 */
void cmd_log_level(Args *args) {
    if (args->argc != 2) {
        send_str("Usage: log_level <debug|info|warn|error|fatal>\r\n");
        return;
    }
    LogLevel level;
    const char *arg = args->argv[1];
    if (strcmp(arg, "debug") == 0) level = LOG_LEVEL_DEBUG;
    else if (strcmp(arg, "info") == 0) level = LOG_LEVEL_INFO;
    else if (strcmp(arg, "warn") == 0) level = LOG_LEVEL_WARN;
    else if (strcmp(arg, "error") == 0) level = LOG_LEVEL_ERROR;
    else if (strcmp(arg, "fatal") == 0) level = LOG_LEVEL_FATAL;
    else {
        send_str("Invalid level\r\n");
        return;
    }
    log_set_level(level);
    send_str("Log level updated\r\n");
}

/*
 * Command table definition and registration
 */
const Command cmd_list[] = {
    { "help", cmd_help },
    { "echo", cmd_echo  },
    { "add",  cmd_add   },
    { "faults", cmd_faults },
    { "fault_clear", cmd_fault_clear },
    { "log_level", cmd_log_level },
};
const size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);

#endif // USE_CMD_INTERPRETER

