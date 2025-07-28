# STM32 UART Protocol for C# GUI Integration

This protocol provides structured binary communication between an STM32 microcontroller and a C# GUI application over UART. It supports PWM control, ADC reading, and system monitoring.

## Protocol Overview

### Frame Structure
```
[START] [CMD_ID] [DATA_LEN] [DATA...] [CHECKSUM] [END]
  0xAA   1 byte    1 byte    0-32 bytes  1 byte   0x55
```

- **START_BYTE**: Always `0xAA`
- **CMD_ID**: Command identifier (see commands below)
- **DATA_LEN**: Number of data bytes (0-32)
- **DATA**: Command-specific data payload
- **CHECKSUM**: XOR of CMD_ID, DATA_LEN, and all DATA bytes
- **END_BYTE**: Always `0x55`

### Response Format
All responses start with a status byte:
- `0x00`: Success (OK)
- `0x01`: General error
- `0x02`: Invalid command
- `0x03`: Invalid parameter
- `0x04`: Timeout
- `0x05`: Busy

## Commands

### System Commands

#### PING (0x01)
Test communication with the device.
- **Request**: No data
- **Response**: Status only

#### GET_SYSTEM_INFO (0x30)
Get system information.
- **Request**: No data
- **Response**: Status + 8 bytes
  - Bytes 1-4: Uptime in milliseconds (uint32)
  - Byte 5: CPU usage percentage (uint8)
  - Bytes 6-7: Free heap memory in bytes (uint16)
  - Byte 8: Temperature in Celsius (uint8)

### PWM Commands

#### PWM_SET_DUTY (0x10)
Set PWM duty cycle.
- **Request**: 1 byte (duty cycle 0-100%)
- **Response**: Status + 1 byte (echo of duty cycle)

#### PWM_START (0x11)
Start PWM output.
- **Request**: No data
- **Response**: Status only

#### PWM_STOP (0x12)
Stop PWM output.
- **Request**: No data
- **Response**: Status only

#### PWM_GET_STATUS (0x13)
Get current PWM status.
- **Request**: No data
- **Response**: Status + 6 bytes
  - Byte 1: Running state (0=stopped, 1=running)
  - Byte 2: Current duty cycle (0-100%)
  - Bytes 3-4: Timer period (uint16)
  - Bytes 5-6: Current pulse value (uint16)

### ADC Commands

#### ADC_READ (0x20)
Read single ADC value.
- **Request**: No data
- **Response**: Status + 2 bytes (ADC value as uint16)

#### ADC_READ_VOLTAGE (0x21)
Read ADC value and convert to voltage.
- **Request**: No data
- **Response**: Status + 4 bytes
  - Bytes 1-2: Raw ADC value (uint16)
  - Bytes 3-4: Voltage in millivolts (uint16)

#### ADC_START (0x22)
Start ADC conversion.
- **Request**: No data
- **Response**: Status only

#### ADC_STOP (0x23)
Stop ADC conversion.
- **Request**: No data
- **Response**: Status only

#### ADC_GET_STATUS (0x24)
Get ADC status and last reading.
- **Request**: No data
- **Response**: Status + 6 bytes
  - Byte 1: Running state (0=stopped, 1=running)
  - Byte 2: Resolution in bits (typically 12)
  - Bytes 3-4: Last ADC value (uint16)
  - Bytes 5-6: Last voltage in millivolts (uint16)

#### ADC_READ_MULTI (0x25)
Read multiple ADC samples and return average.
- **Request**: 1 byte (number of samples 1-20)
- **Response**: Status + 5 bytes
  - Byte 1: Number of valid samples
  - Bytes 2-3: Average ADC value (uint16)
  - Bytes 4-5: Average voltage in millivolts (uint16)

## STM32 Implementation

### Files Added
- `Drivers/LoggerPlus/include/uart_protocol.h` - Protocol definitions and function prototypes
- `Drivers/LoggerPlus/src/uart_protocol.c` - Protocol implementation
- Updated `commands.c` with protocol command for CLI testing

### Usage in STM32

1. Initialize the protocol in your application:
```c
protocol_handle_t protocol;
uart_drv_t *uart_driver = /* your UART driver */;
protocol_init(&protocol, uart_driver);
```

2. Process incoming data in your UART receive callback:
```c
void uart_rx_callback(uint8_t *data, uint16_t len) {
    protocol_process_data(&protocol, data, len);
}
```

3. Test using CLI commands:
```
protocol init    # Initialize binary protocol
protocol info    # Show protocol information
protocol test    # Send test ping frame
```

## C# Implementation

### Files Provided
- `STM32Controller.cs` - Complete protocol implementation and controller class
- `MainForm.cs` - Windows Forms GUI example

### Usage in C#

1. Basic connection and control:
```csharp
using var controller = new STM32Controller("COM3");
controller.Connect();

// Test connection
bool connected = await controller.PingAsync();

// Control PWM
await controller.SetPwmDutyCycleAsync(75);
await controller.StartPwmAsync();

// Read ADC
var adcReading = await controller.ReadAdcVoltageAsync();
if (adcReading.HasValue)
{
    Console.WriteLine($"Voltage: {adcReading.Value.voltageMillivolts / 1000.0:F3}V");
}
```

2. GUI Application:
   - Compile and run `MainForm.cs`
   - Select appropriate COM port
   - Connect to STM32
   - Use sliders and buttons to control PWM
   - Monitor ADC readings and system information

## Configuration

### STM32 Configuration
- Timer: TIM2 Channel 2 for PWM (configured in main.c)
- ADC: ADC1 Channel 0 (configured in main.c)
- UART: Any UART supported by the uart_driver
- Reference voltage: 3.3V (configurable in protocol implementation)

### C# Configuration
- Baud rate: 115200 (default, configurable)
- Timeout: 1000ms
- Data bits: 8, Parity: None, Stop bits: 1

## Error Handling

### STM32 Side
- Invalid frames are ignored
- Timeouts return appropriate status codes
- Hardware errors return STATUS_ERROR

### C# Side
- Serial port exceptions are caught and displayed
- Invalid responses return null values
- Timeouts throw exceptions that can be handled

## Example Communication Flow

1. **C# sends PWM duty cycle command:**
   ```
   TX: AA 10 01 4B 5A 55  (Set duty to 75%)
   RX: AA 10 02 00 4B 59 55  (OK, duty set to 75%)
   ```

2. **C# requests ADC reading:**
   ```
   TX: AA 21 00 21 55  (Read ADC voltage)
   RX: AA 21 05 00 00 08 E6 0C AB 55  (OK, ADC=2048, Voltage=1650mV)
   ```

This protocol provides a robust, efficient way to control and monitor your STM32 from a C# GUI application with real-time feedback and comprehensive error handling.
