#ifndef COMMANDS_H
#define COMMANDS_H
/**
 * @file commands.h
 * @brief CLI command handlers for demo application.
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
/** Control PWM output duty cycle and state. */
void cmd_pwm(Args *args);
/** Read ADC values and control ADC conversion. */
void cmd_adc(Args *args);
/** Manage binary protocol for C# GUI communication. */
void cmd_protocol(Args *args);

/** Table of available commands. */
extern const Command cmd_list[];
/** Number of commands in ::cmd_list. */
extern const size_t  cmd_count;

#endif // COMMANDS_H
