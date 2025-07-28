#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

/**
 * @file uart_protocol.h
 * @brief Structured UART communication protocol for C# GUI integration
 */

#include <stdint.h>
#include <stdbool.h>
#include "uart_driver.h"

// Protocol Configuration
#define PROTOCOL_START_BYTE     0xAA
#define PROTOCOL_END_BYTE       0x55
#define PROTOCOL_MAX_DATA_LEN   32
#define PROTOCOL_TIMEOUT_MS     1000

// Command IDs
typedef enum {
    CMD_PING            = 0x01,
    CMD_PWM_SET_DUTY    = 0x10,
    CMD_PWM_START       = 0x11,
    CMD_PWM_STOP        = 0x12,
    CMD_PWM_GET_STATUS  = 0x13,
    CMD_ADC_READ        = 0x20,
    CMD_ADC_READ_VOLTAGE = 0x21,
    CMD_ADC_START       = 0x22,
    CMD_ADC_STOP        = 0x23,
    CMD_ADC_GET_STATUS  = 0x24,
    CMD_ADC_READ_MULTI  = 0x25,
    CMD_GET_SYSTEM_INFO = 0x30,
} protocol_cmd_t;

// Response Status Codes
typedef enum {
    STATUS_OK           = 0x00,
    STATUS_ERROR        = 0x01,
    STATUS_INVALID_CMD  = 0x02,
    STATUS_INVALID_PARAM = 0x03,
    STATUS_TIMEOUT      = 0x04,
    STATUS_BUSY         = 0x05,
} protocol_status_t;

// Protocol Frame Structure
typedef struct __attribute__((packed)) {
    uint8_t start_byte;     // 0xAA
    uint8_t cmd_id;         // Command ID
    uint8_t data_len;       // Length of data payload (0-32)
    uint8_t data[PROTOCOL_MAX_DATA_LEN]; // Data payload
    uint8_t checksum;       // Simple checksum
    uint8_t end_byte;       // 0x55
} protocol_frame_t;

// PWM Status Structure
typedef struct __attribute__((packed)) {
    uint8_t is_running;     // 0=stopped, 1=running
    uint8_t duty_cycle;     // 0-100%
    uint16_t period;        // Timer period
    uint16_t pulse;         // Current pulse value
} pwm_status_t;

// ADC Status Structure
typedef struct __attribute__((packed)) {
    uint8_t is_running;     // 0=stopped, 1=running
    uint8_t resolution;     // ADC resolution in bits
    uint16_t last_value;    // Last ADC reading
    uint16_t voltage_mv;    // Voltage in millivolts
} adc_status_t;

// System Info Structure
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;     // System uptime in milliseconds
    uint8_t  cpu_usage;     // CPU usage percentage (if available)
    uint16_t free_heap;     // Free heap memory in bytes
    uint8_t  temperature;   // MCU temperature (if available)
} system_info_t;

// Protocol Handler Functions
typedef struct {
    uart_drv_t *uart_drv;
    bool initialized;
} protocol_handle_t;

// Function Prototypes

/**
 * @brief Initialize the protocol handler
 * @param handle Protocol handle to initialize
 * @param uart_drv UART driver instance
 * @return true if successful, false otherwise
 */
bool protocol_init(protocol_handle_t *handle, uart_drv_t *uart_drv);

/**
 * @brief Process incoming protocol messages
 * @param handle Protocol handle
 * @param data Received data buffer
 * @param len Length of received data
 */
void protocol_process_data(protocol_handle_t *handle, uint8_t *data, uint16_t len);

/**
 * @brief Send a response frame
 * @param handle Protocol handle
 * @param cmd_id Original command ID
 * @param status Response status
 * @param data Response data (can be NULL)
 * @param data_len Length of response data
 */
void protocol_send_response(protocol_handle_t *handle, uint8_t cmd_id, 
                           protocol_status_t status, uint8_t *data, uint8_t data_len);

/**
 * @brief Calculate frame checksum
 * @param frame Frame to calculate checksum for
 * @return Calculated checksum
 */
uint8_t protocol_calculate_checksum(const protocol_frame_t *frame);

/**
 * @brief Validate frame integrity
 * @param frame Frame to validate
 * @return true if frame is valid, false otherwise
 */
bool protocol_validate_frame(const protocol_frame_t *frame);

// Command Handler Functions
void protocol_handle_ping(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_pwm_set_duty(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_pwm_start(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_pwm_stop(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_pwm_get_status(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_read(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_read_voltage(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_start(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_stop(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_get_status(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_adc_read_multi(protocol_handle_t *handle, const protocol_frame_t *frame);
void protocol_handle_get_system_info(protocol_handle_t *handle, const protocol_frame_t *frame);

#endif // UART_PROTOCOL_H
