/*
 * commands.c
 *
 *  Created on: Jul 20, 2025
 *      Author: kevinfox
 */
#include "uart_driver_config.h"
#if USE_CMD_INTERPRETER
#include "commands.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fault_module.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_protocol.h"

// External timer handle for PWM control
extern TIM_HandleTypeDef htim2;
// External ADC handle for analog readings
extern ADC_HandleTypeDef hadc1;

// Global protocol handle for binary communication
static protocol_handle_t g_protocol_handle = {0};

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

// 'pwm' command: control PWM duty cycle
void cmd_pwm(Args *args) {
    if (args->argc < 2) {
        send_str("Usage: pwm <duty_cycle> [start|stop|status]\r\n");
        send_str("  duty_cycle: 0-100 (percentage)\r\n");
        send_str("  start: start PWM output\r\n");
        send_str("  stop: stop PWM output\r\n");
        send_str("  status: show current PWM status\r\n");
        return;
    }

    if (strcmp(args->argv[1], "status") == 0) {
        // Get current PWM settings
        uint32_t period = htim2.Init.Period + 1;  // Period is 0-based
        uint32_t pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
        uint32_t duty_percent = (pulse * 100) / period;
        
        char buf[64];
        snprintf(buf, sizeof(buf), "PWM Status:\r\n  Period: %lu\r\n  Pulse: %lu\r\n  Duty: %lu%%\r\n", 
                 (unsigned long)period, (unsigned long)pulse, (unsigned long)duty_percent);
        send_str(buf);
        return;
    }

    if (strcmp(args->argv[1], "start") == 0) {
        if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) == HAL_OK) {
            send_str("PWM started\r\n");
        } else {
            send_str("Failed to start PWM\r\n");
        }
        return;
    }

    if (strcmp(args->argv[1], "stop") == 0) {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        send_str("PWM stopped\r\n");
        return;
    }

    // Parse duty cycle
    char *endptr;
    long duty = strtol(args->argv[1], &endptr, 10);
    if (*endptr != '\0' || duty < 0 || duty > 100) {
        send_str("Invalid duty cycle. Must be 0-100\r\n");
        return;
    }

    // Calculate pulse value based on duty cycle percentage
    uint32_t period = htim2.Init.Period + 1;  // Period is 0-based (99+1=100)
    uint32_t pulse = (duty * period) / 100;

    // Set new duty cycle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
    
    char buf[48];
    snprintf(buf, sizeof(buf), "PWM duty set to %ld%% (pulse=%lu)\r\n", duty, (unsigned long)pulse);
    send_str(buf);

    // Handle optional start/stop command
    if (args->argc >= 3) {
        if (strcmp(args->argv[2], "start") == 0) {
            if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) == HAL_OK) {
                send_str("PWM started\r\n");
            } else {
                send_str("Failed to start PWM\r\n");
            }
        } else if (strcmp(args->argv[2], "stop") == 0) {
            HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
            send_str("PWM stopped\r\n");
        }
    }
}

// 'adc' command: read ADC values and control ADC
void cmd_adc(Args *args) {
    if (args->argc < 2) {
        send_str("Usage: adc <command>\r\n");
        send_str("  read [samples] - read ADC value(s)\r\n");
        send_str("  start - start continuous ADC conversion\r\n");
        send_str("  stop - stop ADC conversion\r\n");
        send_str("  status - show ADC configuration\r\n");
        send_str("  voltage [samples] - read and convert to voltage\r\n");
        return;
    }

    if (strcmp(args->argv[1], "status") == 0) {
        // Show ADC configuration
        char buf[128];
        snprintf(buf, sizeof(buf), "ADC Status:\r\n  Instance: ADC1\r\n  Channel: %lu\r\n  Resolution: 12-bit\r\n  Max Value: 4095\r\n", 
                 (unsigned long)ADC_CHANNEL_0);
        send_str(buf);
        return;
    }

    if (strcmp(args->argv[1], "start") == 0) {
        if (HAL_ADC_Start(&hadc1) == HAL_OK) {
            send_str("ADC conversion started\r\n");
        } else {
            send_str("Failed to start ADC\r\n");
        }
        return;
    }

    if (strcmp(args->argv[1], "stop") == 0) {
        HAL_ADC_Stop(&hadc1);
        send_str("ADC conversion stopped\r\n");
        return;
    }

    if (strcmp(args->argv[1], "read") == 0) {
        uint32_t samples = 1;
        
        // Parse optional number of samples
        if (args->argc >= 3) {
            char *endptr;
            long parsed_samples = strtol(args->argv[2], &endptr, 10);
            if (*endptr != '\0' || parsed_samples < 1 || parsed_samples > 100) {
                send_str("Invalid sample count. Must be 1-100\r\n");
                return;
            }
            samples = (uint32_t)parsed_samples;
        }

        // Start ADC if not already running
        HAL_ADC_Start(&hadc1);
        
        if (samples == 1) {
            // Single reading
            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
                char buf[32];
                snprintf(buf, sizeof(buf), "ADC: %lu\r\n", (unsigned long)adc_value);
                send_str(buf);
            } else {
                send_str("ADC conversion timeout\r\n");
            }
        } else {
            // Multiple readings
            uint32_t sum = 0;
            uint32_t valid_samples = 0;
            
            for (uint32_t i = 0; i < samples; i++) {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
                    sum += adc_value;
                    valid_samples++;
                }
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between samples
            }
            
            if (valid_samples > 0) {
                uint32_t average = sum / valid_samples;
                char buf[64];
                snprintf(buf, sizeof(buf), "ADC samples: %lu, Average: %lu\r\n", 
                         (unsigned long)valid_samples, (unsigned long)average);
                send_str(buf);
            } else {
                send_str("No valid ADC readings\r\n");
            }
        }
        return;
    }

    if (strcmp(args->argv[1], "voltage") == 0) {
        uint32_t samples = 1;
        
        // Parse optional number of samples
        if (args->argc >= 3) {
            char *endptr;
            long parsed_samples = strtol(args->argv[2], &endptr, 10);
            if (*endptr != '\0' || parsed_samples < 1 || parsed_samples > 100) {
                send_str("Invalid sample count. Must be 1-100\r\n");
                return;
            }
            samples = (uint32_t)parsed_samples;
        }

        // Start ADC if not already running
        HAL_ADC_Start(&hadc1);
        
        if (samples == 1) {
            // Single reading
            if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
                // Convert to voltage (assuming 3.3V reference, 12-bit ADC)
                float voltage = (adc_value * 3.3f) / 4095.0f;
                char buf[48];
                snprintf(buf, sizeof(buf), "ADC: %lu, Voltage: %.3f V\r\n", 
                         (unsigned long)adc_value, voltage);
                send_str(buf);
            } else {
                send_str("ADC conversion timeout\r\n");
            }
        } else {
            // Multiple readings with averaging
            uint32_t sum = 0;
            uint32_t valid_samples = 0;
            
            for (uint32_t i = 0; i < samples; i++) {
                if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
                    uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
                    sum += adc_value;
                    valid_samples++;
                }
                vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between samples
            }
            
            if (valid_samples > 0) {
                uint32_t average = sum / valid_samples;
                float voltage = (average * 3.3f) / 4095.0f;
                char buf[80];
                snprintf(buf, sizeof(buf), "ADC samples: %lu, Average: %lu, Voltage: %.3f V\r\n", 
                         (unsigned long)valid_samples, (unsigned long)average, voltage);
                send_str(buf);
            } else {
                send_str("No valid ADC readings\r\n");
            }
        }
        return;
    }

    send_str("Unknown ADC command\r\n");
}

// 'protocol' command: manage binary protocol for C# GUI
void cmd_protocol(Args *args) {
    if (args->argc < 2) {
        send_str("Usage: protocol <command>\r\n");
        send_str("  init - initialize binary protocol\r\n");
        send_str("  test - send test frame\r\n");
        send_str("  info - show protocol information\r\n");
        return;
    }

    if (strcmp(args->argv[1], "init") == 0) {
        // Get UART driver from command module
        uart_drv_t *cmd_uart = cmd_get_uart_driver();
        
        if (cmd_uart && protocol_init(&g_protocol_handle, cmd_uart)) {
            send_str("Binary protocol initialized\r\n");
        } else {
            send_str("Failed to initialize protocol\r\n");
        }
        return;
    }

    if (strcmp(args->argv[1], "info") == 0) {
        char buf[128];
        snprintf(buf, sizeof(buf), "Protocol Info:\r\n  Start: 0x%02X\r\n  End: 0x%02X\r\n  Max Data: %d bytes\r\n", 
                 PROTOCOL_START_BYTE, PROTOCOL_END_BYTE, PROTOCOL_MAX_DATA_LEN);
        send_str(buf);
        send_str("Commands:\r\n");
        send_str("  0x01: PING\r\n");
        send_str("  0x10-0x13: PWM Control\r\n");
        send_str("  0x20-0x25: ADC Control\r\n");
        send_str("  0x30: System Info\r\n");
        return;
    }

    if (strcmp(args->argv[1], "test") == 0) {
        if (!g_protocol_handle.initialized) {
            send_str("Protocol not initialized. Run 'protocol init' first.\r\n");
            return;
        }
        
        // Create a test ping frame
        protocol_frame_t test_frame = {0};
        test_frame.start_byte = PROTOCOL_START_BYTE;
        test_frame.cmd_id = CMD_PING;
        test_frame.data_len = 0;
        test_frame.checksum = protocol_calculate_checksum(&test_frame);
        test_frame.end_byte = PROTOCOL_END_BYTE;
        
        // Process the test frame
        protocol_process_data(&g_protocol_handle, (uint8_t*)&test_frame, 
                             sizeof(test_frame.start_byte) + sizeof(test_frame.cmd_id) + 
                             sizeof(test_frame.data_len) + test_frame.data_len + 
                             sizeof(test_frame.checksum) + sizeof(test_frame.end_byte));
        
        send_str("Test frame sent and processed\r\n");
        return;
    }

    send_str("Unknown protocol command\r\n");
}

// Define command table and expose to interpreter
const Command cmd_list[] = {
    { "help", cmd_help },
    { "echo", cmd_echo  },
    { "add",  cmd_add   },
    { "faults", cmd_faults },
    { "fault_clear", cmd_fault_clear },
    { "pwm", cmd_pwm },
    { "adc", cmd_adc },
    { "protocol", cmd_protocol },
};
const size_t cmd_count = sizeof(cmd_list) / sizeof(cmd_list[0]);

#endif // USE_CMD_INTERPRETER
