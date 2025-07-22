/*
 * sample_commands.c
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#include "sample_commands.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fault_module.h"

// Helper to send strings using command module UART
static void send_str(const char *s) {
    cmd_write(s);
}

// 'help' command: list all available commands
void cmd_help(Args *args) {
    send_str("Available commands:\r\n");
    for (size_t i = 0; i < cmd_count; i++) {
        send_str("  ");
        send_str(cmd_list[i].name);
        send_str("\r\n");
    }
}

// 'echo' command: repeat back provided parameters
void cmd_echo(Args *args) {
    for (int i = 1; i < args->argc; i++) {
        send_str(args->argv[i]);
        if (i < args->argc - 1) send_str(" ");
    }
    send_str("\r\n");
}

// 'add' command: add two integers and print result
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

// 'faults' command: list active faults
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

// 'fault_clear' command
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

// Define command table and expose to interpreter
const Command cmd_list[] = {
    { "help", cmd_help },
    { "echo", cmd_echo  },
    { "add",  cmd_add   },
    { "faults", cmd_faults },
    { "fault_clear", cmd_fault_clear },
};
const size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);

