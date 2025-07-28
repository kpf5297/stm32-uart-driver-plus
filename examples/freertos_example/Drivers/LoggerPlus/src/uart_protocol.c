/*
 * uart_protocol.c
 *
 *  Created on: Jul 27, 2025
 *      Author: kevinfox
 */

#include "uart_protocol.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

// External handles
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

// Static protocol buffer for frame assembly
static uint8_t rx_buffer[sizeof(protocol_frame_t)];
static uint16_t rx_index = 0;
static bool frame_in_progress = false;

bool protocol_init(protocol_handle_t *handle, uart_drv_t *uart_drv) {
    if (!handle || !uart_drv) {
        return false;
    }
    
    handle->uart_drv = uart_drv;
    handle->initialized = true;
    
    // Reset receive buffer
    rx_index = 0;
    frame_in_progress = false;
    
    return true;
}

uint8_t protocol_calculate_checksum(const protocol_frame_t *frame) {
    uint8_t checksum = 0;
    checksum ^= frame->cmd_id;
    checksum ^= frame->data_len;
    
    for (int i = 0; i < frame->data_len; i++) {
        checksum ^= frame->data[i];
    }
    
    return checksum;
}

bool protocol_validate_frame(const protocol_frame_t *frame) {
    // Check start and end bytes
    if (frame->start_byte != PROTOCOL_START_BYTE || 
        frame->end_byte != PROTOCOL_END_BYTE) {
        return false;
    }
    
    // Check data length
    if (frame->data_len > PROTOCOL_MAX_DATA_LEN) {
        return false;
    }
    
    // Verify checksum
    uint8_t calculated_checksum = protocol_calculate_checksum(frame);
    if (calculated_checksum != frame->checksum) {
        return false;
    }
    
    return true;
}

void protocol_send_response(protocol_handle_t *handle, uint8_t cmd_id, 
                           protocol_status_t status, uint8_t *data, uint8_t data_len) {
    if (!handle || !handle->initialized) {
        return;
    }
    
    protocol_frame_t response = {0};
    response.start_byte = PROTOCOL_START_BYTE;
    response.cmd_id = cmd_id;
    response.data_len = data_len + 1; // +1 for status byte
    
    // First byte of data is always the status
    response.data[0] = status;
    
    // Copy additional data if provided
    if (data && data_len > 0 && data_len <= (PROTOCOL_MAX_DATA_LEN - 1)) {
        memcpy(&response.data[1], data, data_len);
    }
    
    response.checksum = protocol_calculate_checksum(&response);
    response.end_byte = PROTOCOL_END_BYTE;
    
    // Send the response
    uart_send_blocking(handle->uart_drv, (uint8_t*)&response, 
                      sizeof(response.start_byte) + sizeof(response.cmd_id) + 
                      sizeof(response.data_len) + response.data_len + 
                      sizeof(response.checksum) + sizeof(response.end_byte), 
                      PROTOCOL_TIMEOUT_MS);
}

void protocol_process_data(protocol_handle_t *handle, uint8_t *data, uint16_t len) {
    if (!handle || !handle->initialized || !data || len == 0) {
        return;
    }
    
    for (uint16_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        
        // Look for start byte
        if (!frame_in_progress && byte == PROTOCOL_START_BYTE) {
            rx_buffer[0] = byte;
            rx_index = 1;
            frame_in_progress = true;
            continue;
        }
        
        // If we're in the middle of a frame
        if (frame_in_progress) {
            rx_buffer[rx_index++] = byte;
            
            // Check if we have enough data to determine frame length
            if (rx_index >= 3) { // start_byte + cmd_id + data_len
                uint8_t expected_len = 5 + rx_buffer[2]; // 5 fixed bytes + data_len
                
                // Check for buffer overflow
                if (expected_len > sizeof(protocol_frame_t)) {
                    frame_in_progress = false;
                    rx_index = 0;
                    continue;
                }
                
                // Check if we have a complete frame
                if (rx_index >= expected_len) {
                    protocol_frame_t *frame = (protocol_frame_t*)rx_buffer;
                    
                    // Validate and process the frame
                    if (protocol_validate_frame(frame)) {
                        // Process the command
                        switch (frame->cmd_id) {
                            case CMD_PING:
                                protocol_handle_ping(handle, frame);
                                break;
                            case CMD_PWM_SET_DUTY:
                                protocol_handle_pwm_set_duty(handle, frame);
                                break;
                            case CMD_PWM_START:
                                protocol_handle_pwm_start(handle, frame);
                                break;
                            case CMD_PWM_STOP:
                                protocol_handle_pwm_stop(handle, frame);
                                break;
                            case CMD_PWM_GET_STATUS:
                                protocol_handle_pwm_get_status(handle, frame);
                                break;
                            case CMD_ADC_READ:
                                protocol_handle_adc_read(handle, frame);
                                break;
                            case CMD_ADC_READ_VOLTAGE:
                                protocol_handle_adc_read_voltage(handle, frame);
                                break;
                            case CMD_ADC_START:
                                protocol_handle_adc_start(handle, frame);
                                break;
                            case CMD_ADC_STOP:
                                protocol_handle_adc_stop(handle, frame);
                                break;
                            case CMD_ADC_GET_STATUS:
                                protocol_handle_adc_get_status(handle, frame);
                                break;
                            case CMD_ADC_READ_MULTI:
                                protocol_handle_adc_read_multi(handle, frame);
                                break;
                            case CMD_GET_SYSTEM_INFO:
                                protocol_handle_get_system_info(handle, frame);
                                break;
                            default:
                                protocol_send_response(handle, frame->cmd_id, STATUS_INVALID_CMD, NULL, 0);
                                break;
                        }
                    } else {
                        // Invalid frame - send error response if we can determine cmd_id
                        if (rx_index >= 2) {
                            protocol_send_response(handle, rx_buffer[1], STATUS_ERROR, NULL, 0);
                        }
                    }
                    
                    // Reset for next frame
                    frame_in_progress = false;
                    rx_index = 0;
                }
            }
        }
    }
}

// Command Handlers

void protocol_handle_ping(protocol_handle_t *handle, const protocol_frame_t *frame) {
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, NULL, 0);
}

void protocol_handle_pwm_set_duty(protocol_handle_t *handle, const protocol_frame_t *frame) {
    if (frame->data_len < 1) {
        protocol_send_response(handle, frame->cmd_id, STATUS_INVALID_PARAM, NULL, 0);
        return;
    }
    
    uint8_t duty_cycle = frame->data[0];
    if (duty_cycle > 100) {
        protocol_send_response(handle, frame->cmd_id, STATUS_INVALID_PARAM, NULL, 0);
        return;
    }
    
    // Calculate pulse value
    uint32_t period = htim2.Init.Period + 1;
    uint32_t pulse = (duty_cycle * period) / 100;
    
    // Set new duty cycle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
    
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, &duty_cycle, 1);
}

void protocol_handle_pwm_start(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    protocol_status_t proto_status = (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
    protocol_send_response(handle, frame->cmd_id, proto_status, NULL, 0);
}

void protocol_handle_pwm_stop(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, NULL, 0);
}

void protocol_handle_pwm_get_status(protocol_handle_t *handle, const protocol_frame_t *frame) {
    pwm_status_t status = {0};
    
    // Check if PWM is running (simplified check)
    status.is_running = (__HAL_TIM_GET_COUNTER(&htim2) > 0) ? 1 : 0;
    
    uint32_t period = htim2.Init.Period + 1;
    uint32_t pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
    
    status.duty_cycle = (pulse * 100) / period;
    status.period = (uint16_t)period;
    status.pulse = (uint16_t)pulse;
    
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, (uint8_t*)&status, sizeof(status));
}

void protocol_handle_adc_read(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_ADC_Start(&hadc1);
    
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        uint16_t adc_data = (uint16_t)adc_value;
        protocol_send_response(handle, frame->cmd_id, STATUS_OK, (uint8_t*)&adc_data, sizeof(adc_data));
    } else {
        protocol_send_response(handle, frame->cmd_id, STATUS_TIMEOUT, NULL, 0);
    }
}

void protocol_handle_adc_read_voltage(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_ADC_Start(&hadc1);
    
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        uint16_t voltage_mv = (uint16_t)((adc_value * 3300) / 4095); // Convert to millivolts
        
        uint8_t response_data[4];
        memcpy(&response_data[0], &adc_value, 2); // ADC raw value (16-bit)
        memcpy(&response_data[2], &voltage_mv, 2); // Voltage in mV (16-bit)
        
        protocol_send_response(handle, frame->cmd_id, STATUS_OK, response_data, 4);
    } else {
        protocol_send_response(handle, frame->cmd_id, STATUS_TIMEOUT, NULL, 0);
    }
}

void protocol_handle_adc_start(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
    protocol_status_t proto_status = (status == HAL_OK) ? STATUS_OK : STATUS_ERROR;
    protocol_send_response(handle, frame->cmd_id, proto_status, NULL, 0);
}

void protocol_handle_adc_stop(protocol_handle_t *handle, const protocol_frame_t *frame) {
    HAL_ADC_Stop(&hadc1);
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, NULL, 0);
}

void protocol_handle_adc_get_status(protocol_handle_t *handle, const protocol_frame_t *frame) {
    adc_status_t status = {0};
    
    // Get ADC status
    status.is_running = (hadc1.State == HAL_ADC_STATE_REG_BUSY) ? 1 : 0;
    status.resolution = 12; // 12-bit ADC
    
    // Get last value
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        status.last_value = (uint16_t)adc_value;
        status.voltage_mv = (uint16_t)((adc_value * 3300) / 4095);
    }
    
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, (uint8_t*)&status, sizeof(status));
}

void protocol_handle_adc_read_multi(protocol_handle_t *handle, const protocol_frame_t *frame) {
    if (frame->data_len < 1) {
        protocol_send_response(handle, frame->cmd_id, STATUS_INVALID_PARAM, NULL, 0);
        return;
    }
    
    uint8_t samples = frame->data[0];
    if (samples == 0 || samples > 20) { // Limit to 20 samples to fit in response
        protocol_send_response(handle, frame->cmd_id, STATUS_INVALID_PARAM, NULL, 0);
        return;
    }
    
    HAL_ADC_Start(&hadc1);
    
    uint32_t sum = 0;
    uint8_t valid_samples = 0;
    
    for (uint8_t i = 0; i < samples; i++) {
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
            uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
            sum += adc_value;
            valid_samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(5)); // Small delay between samples
    }
    
    if (valid_samples > 0) {
        uint16_t average = (uint16_t)(sum / valid_samples);
        uint16_t voltage_mv = (uint16_t)((average * 3300) / 4095);
        
        uint8_t response_data[5];
        response_data[0] = valid_samples;
        memcpy(&response_data[1], &average, 2);
        memcpy(&response_data[3], &voltage_mv, 2);
        
        protocol_send_response(handle, frame->cmd_id, STATUS_OK, response_data, 5);
    } else {
        protocol_send_response(handle, frame->cmd_id, STATUS_TIMEOUT, NULL, 0);
    }
}

void protocol_handle_get_system_info(protocol_handle_t *handle, const protocol_frame_t *frame) {
    system_info_t info = {0};
    
    info.uptime_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    info.cpu_usage = 0; // Not implemented
    info.free_heap = (uint16_t)xPortGetFreeHeapSize();
    info.temperature = 25; // Mock temperature
    
    protocol_send_response(handle, frame->cmd_id, STATUS_OK, (uint8_t*)&info, sizeof(info));
}
