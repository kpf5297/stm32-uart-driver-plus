/**
 * @file command_module.c
 * @brief Simple command interpreter over UART.
 */
#include "command_module.h"
#include "commands.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#if USE_CMD_INTERPRETER

#define CMD_UART_TIMEOUT_MS 100
#define CMD_TASK_DELAY_MS 1
#define CMD_PROMPT_SIZE 2

static QueueHandle_t rx_queue;
static uart_drv_t   *uart;
static uint8_t       rx_byte;

static void cmd_tx_bytes(const uint8_t *buf, size_t len)
{
#if UART_TX_MODE == UART_MODE_DMA
    if (uart && uart->hdma_tx) {
        uart_send_dma_blocking(uart, buf, len, CMD_UART_TIMEOUT_MS);
        return;
    }
#elif UART_TX_MODE == UART_MODE_INTERRUPT
    if (uart) {
        if (uart_send_nb(uart, buf, len) == UART_OK) {
            while (uart_get_status(uart) == UART_BUSY) {
                vTaskDelay(pdMS_TO_TICKS(CMD_TASK_DELAY_MS));
            }
        }
        return;
    }
#endif
    if (uart) {
        uart_send_blocking(uart, buf, len, CMD_UART_TIMEOUT_MS);
    }
}

// Simple helpers for command handlers
void cmd_write(const char *s) {
    if (uart && s) {
        cmd_tx_bytes((const uint8_t *)s, strlen(s));
    }
}

void cmd_printf(const char *fmt, ...) {
    static char formatted_buffer[CMD_MAX_LINE_LEN]; // Static buffer to avoid malloc
    char stream_buf[32]; // Use a smaller buffer for streaming
    va_list args;
    
    va_start(args, fmt);
    int len = vsnprintf(formatted_buffer, sizeof(formatted_buffer), fmt, args);
    va_end(args);

    if (len > 0) {
        // Stream the formatted string in chunks
        int remaining = (len < (int)sizeof(formatted_buffer)) ? len : (int)sizeof(formatted_buffer) - 1;
        for (int i = 0; i < remaining; i += sizeof(stream_buf) - 1) {
            int chunk_size = (remaining - i < (int)sizeof(stream_buf) - 1) ? 
                           remaining - i : (int)sizeof(stream_buf) - 1;
            strncpy(stream_buf, &formatted_buffer[i], chunk_size);
            stream_buf[chunk_size] = '\0';
            cmd_write(stream_buf);
        }
    }
}

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
    cmd_tx_bytes((const uint8_t*)prompt, strlen(prompt));

    for (;;) {
        // Wait for next byte
        if (xQueueReceive(rx_queue, &byte, portMAX_DELAY) == pdPASS) {
            // Echo input
            cmd_tx_bytes(&byte, 1);

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
                        cmd_tx_bytes((const uint8_t*)err, strlen(err));
                    }
                }
                idx = 0;
                // Prompt again
                cmd_tx_bytes((const uint8_t*)"> ", CMD_PROMPT_SIZE);

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
