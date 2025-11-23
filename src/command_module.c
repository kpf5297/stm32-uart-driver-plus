/**
 * @file command_module.c
 * @brief Simple command interpreter over UART implementation.
 * 
 * Provides a lightweight command-line interface that processes commands
 * received via UART and dispatches them to registered handlers. Uses
 * CMSIS-RTOS v2 message queues for thread-safe operation.
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

static osThreadId_t cmd_task_id = NULL;
static uart_drv_t   *uart;
// Circular RX buffer; command parser expects line-length sized buffer
static uint8_t       rx_circ_buf[CMD_MAX_LINE_LEN];

/**
 * @brief Transmit raw bytes using the configured UART mode.
 * @param buf Pointer to data buffer
 * @param len Number of bytes to send
 */
static void cmd_tx_bytes(const uint8_t *buf, size_t len)
{
    /* DMA-only operation (enforced in config) */
    if (uart && uart->hdma_tx) {
        /* Use queued non-blocking TX for command echoes */
        uart_send_nb(uart, buf, len);
        return;
    }
    /* Enforce DMA-only: if HDMA not available, silently return to avoid blocking */
    (void)uart; (void)buf; (void)len;
}

/**
 * @brief Send a string as a single blocking DMA transfer.
 * @param s Null-terminated string to send
 * @return uart_status_t from uart_send_blocking
 */
uart_status_t cmd_send_blocking(const char *s) {
    if (!uart || !s) return UART_ERROR;
    return uart_send_blocking(uart, (const uint8_t*)s, strlen(s), CMD_UART_TIMEOUT_MS);
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

/**
 * @brief UART event callback for handling received bytes.
 * @param evt Event type received
 * @param ctx Context pointer (points to rx_byte)
 */
static void uart_event_cb(uart_event_t evt, void *ctx) {
    (void)ctx;
    if ((evt == UART_EVT_RX_COMPLETE) || (evt == UART_EVT_RX_AVAILABLE)) {
        if (cmd_task_id) osThreadFlagsSet(cmd_task_id, 1u);
    }
}


/**
 * @brief Command processing task - builds lines, parses, and dispatches commands.
 * @param pvParameters Unused task parameter
 */
static void cmd_task(void *pvParameters) {
    char    line[CMD_MAX_LINE_LEN];
    size_t  idx = 0;
    uint8_t byte;
    Args    args;

    // Print initial prompt
    const char *prompt = "\r\n> ";
    cmd_tx_bytes((const uint8_t*)prompt, strlen(prompt));

    for (;;) {
        // Wait until ISR signals data available
        osThreadFlagsWait(1u, osFlagsWaitAny, osWaitForever);
        // Read available bytes from circular DMA into tmpbuf
        uint8_t tmpbuf[CMD_MAX_LINE_LEN];
        size_t read = uart_read_from_circular(uart, tmpbuf, sizeof(tmpbuf));
        for (size_t r = 0; r < read; ++r) {
            byte = tmpbuf[r];
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

/**
 * @brief Initialize command interpreter subsystem.
 * @param uart_drv Pointer to UART driver instance to use
 */
void cmd_init(uart_drv_t *uart_drv) {
    uart = uart_drv;
    // Register ISR callback (ctx not used for circular mode)
    uart_register_callback(uart, uart_event_cb, NULL);
    // Start circular DMA RX into rx_circ_buf and enable IDLE notifications
    uart_start_circular_rx(uart, rx_circ_buf, sizeof(rx_circ_buf));
    // Launch the processing task
    osThreadAttr_t attr = {0};
    attr.priority = CMD_TASK_PRIO;
    attr.stack_size = CMD_TASK_STACK;
    cmd_task_id = osThreadNew(cmd_task, NULL, &attr);
}


#endif // USE_CMD_INTERPRETER
