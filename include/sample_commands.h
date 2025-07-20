/*
 * sample_commands.h
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#ifndef SAMPLE_COMMANDS_H
#define SAMPLE_COMMANDS_H

#include "command_module.h"
#include "uart_driver.h"

// Shared UART driver instance (set by application)
extern uart_drv_t *shared_uart;

// Command handlers
void cmd_help(Args *args);
void cmd_echo(Args *args);
void cmd_add(Args *args);

// Exported command list and count
extern const Command cmd_list[];
extern const size_t  cmd_count;

#endif // SAMPLE_COMMANDS_H
