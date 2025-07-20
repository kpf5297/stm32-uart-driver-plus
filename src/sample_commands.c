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

// Shared UART driver instance provided by application
extern uart_drv_t *shared_uart;

// Helper to send strings over UART (blocking with 100ms timeout)
static void send_str(const char *s) {
    if (shared_uart) {
        uart_send_blocking(shared_uart, (uint8_t *)s, strlen(s), 100);
    }
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

// Define command table and expose to interpreter
const Command cmd_list[] = {
    { "help", cmd_help },
    { "echo", cmd_echo  },
    { "add",  cmd_add   },
};
const size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);

