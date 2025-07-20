/*
 * command_module.c
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#include "command_module.h"
#include <string.h>
#include <stdlib.h>

#if USE_CMD_INTERPRETER

static QueueHandle_t rx_queue;
static uart_drv_t   *uart;
static uint8_t       rx_byte;

// ISR-callback: enqueue received byte and re-arm next RX
static void uart_event_cb(uart_event_t evt, void *ctx) {
    if (evt == UART_EVT_RX_COMPLETE) {
        uint8_t *b = ctx;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(rx_queue, b, &xHigherPriorityTaskWoken);
        // re-arm next RX based on driver config
        if (uart->hdma_rx) {
            uart_start_dma_rx(uart, &rx_byte, 1);
        } else {
            uart_receive_nb(uart, &rx_byte, 1);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


// Task: build lines, parse, dispatch commands
static void cmd_task(void *pvParameters) {
    char    line[CMD_MAX_LINE_LEN];
    size_t  idx = 0;
    uint8_t byte;
    Args    args;

    // Print initial prompt
    const char *prompt = "\r\n> ";
    uart_send_blocking(uart, (uint8_t*)prompt, strlen(prompt), 100);

    for (;;) {
        // Wait for next byte
        if (xQueueReceive(rx_queue, &byte, portMAX_DELAY) == pdPASS) {
            // Echo input
            uart_send_blocking(uart, &byte, 1, 100);

            // Check for end-of-line
            if (byte == '\r' || byte == '\n') {
                line[idx] = '\0';
                // Tokenize into args
                int argc = 0;
                char *tok = strtok(line, " ");
                while (tok && argc < CMD_MAX_PARAMS) {
                    args.argv[argc++] = tok;
                    tok = strtok(NULL, " ");
                }
                args.argc = argc;

                // Dispatch
                if (argc > 0) {
                    bool found = false;
                    for (size_t i = 0; i < cmd_count; ++i) {
                        if (strcmp(args.argv[0], cmd_list[i].name) == 0) {
                            cmd_list[i].handler(&args);
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        const char *err = "Error: unknown command\r\n";
                        uart_send_blocking(uart, (uint8_t*)err, strlen(err), 100);
                    }
                }
                idx = 0;
                // Prompt again
                uart_send_blocking(uart, (uint8_t*)"> ", 2, 100);

            } else if ((byte == '\b' || byte == 127) && idx > 0) {
                // Backspace handling
                idx--;
            } else if (idx < CMD_MAX_LINE_LEN - 1) {
                // Accumulate
                line[idx++] = byte;
            }
        }
    }
}

// Public init: register callback, create queue, start first RX and the cmd task
void cmd_init(uart_drv_t *uart_drv) {
    uart = uart_drv;
    rx_queue = xQueueCreate(CMD_MAX_LINE_LEN, sizeof(uint8_t));
    // Register ISR callback
    uart_register_callback(uart, uart_event_cb, &rx_byte);
    // Kick off first RX based on driver config
    if (uart->hdma_rx) {
        uart_start_dma_rx(uart, &rx_byte, 1);
    } else {
        uart_receive_nb(uart, &rx_byte, 1);
    }
    // Launch the processing task
    xTaskCreate(cmd_task, "CmdIf", CMD_TASK_STACK, NULL, CMD_TASK_PRIO, NULL);
}


#endif // USE_CMD_INTERPRETER
