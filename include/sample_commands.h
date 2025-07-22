#ifndef SAMPLE_COMMANDS_H
#define SAMPLE_COMMANDS_H
/**
 * @file sample_commands.h
 * @brief Example set of CLI command handlers.
 */

#include "command_module.h"


/** Print list of supported commands. */
void cmd_help(Args *args);
/** Echo back arguments to the console. */
void cmd_echo(Args *args);
/** Add two integers and output result. */
void cmd_add(Args *args);
/** List active faults. */
void cmd_faults(Args *args);
/** Clear specific or all faults. */
void cmd_fault_clear(Args *args);

/** Table of available commands. */
extern const Command cmd_list[];
/** Number of commands in ::cmd_list. */
extern const size_t  cmd_count;

#endif // SAMPLE_COMMANDS_H
