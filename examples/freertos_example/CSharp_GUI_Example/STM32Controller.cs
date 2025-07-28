using System;
using System.IO.Ports;
using System.Threading.Tasks;

namespace STM32Controller
{
    // Protocol constants matching the STM32 implementation
    public static class Protocol
    {
        public const byte START_BYTE = 0xAA;
        public const byte END_BYTE = 0x55;
        public const int MAX_DATA_LEN = 32;
        public const int TIMEOUT_MS = 1000;

        // Command IDs
        public enum Commands : byte
        {
            PING = 0x01,
            PWM_SET_DUTY = 0x10,
            PWM_START = 0x11,
            PWM_STOP = 0x12,
            PWM_GET_STATUS = 0x13,
            ADC_READ = 0x20,
            ADC_READ_VOLTAGE = 0x21,
            ADC_START = 0x22,
            ADC_STOP = 0x23,
            ADC_GET_STATUS = 0x24,
            ADC_READ_MULTI = 0x25,
            GET_SYSTEM_INFO = 0x30,
        }

        // Response status codes
        public enum Status : byte
        {
            OK = 0x00,
            ERROR = 0x01,
            INVALID_CMD = 0x02,
            INVALID_PARAM = 0x03,
            TIMEOUT = 0x04,
            BUSY = 0x05,
        }
    }

    // Frame structure
    public class ProtocolFrame
    {
        public byte StartByte { get; set; } = Protocol.START_BYTE;
        public Protocol.Commands CommandId { get; set; }
        public byte DataLength { get; set; }
        public byte[] Data { get; set; } = new byte[Protocol.MAX_DATA_LEN];
        public byte Checksum { get; set; }
        public byte EndByte { get; set; } = Protocol.END_BYTE;

        public byte CalculateChecksum()
        {
            byte checksum = 0;
            checksum ^= (byte)CommandId;
            checksum ^= DataLength;

            for (int i = 0; i < DataLength; i++)
            {
                checksum ^= Data[i];
            }

            return checksum;
        }

        public byte[] ToByteArray()
        {
            Checksum = CalculateChecksum();
            var result = new byte[5 + DataLength];
            result[0] = StartByte;
            result[1] = (byte)CommandId;
            result[2] = DataLength;
            Array.Copy(Data, 0, result, 3, DataLength);
            result[3 + DataLength] = Checksum;
            result[4 + DataLength] = EndByte;
            return result;
        }

        public static ProtocolFrame FromByteArray(byte[] data)
        {
            if (data.Length < 5) return null;

            var frame = new ProtocolFrame
            {
                StartByte = data[0],
                CommandId = (Protocol.Commands)data[1],
                DataLength = data[2]
            };

            if (data.Length < 5 + frame.DataLength) return null;

            Array.Copy(data, 3, frame.Data, 0, frame.DataLength);
            frame.Checksum = data[3 + frame.DataLength];
            frame.EndByte = data[4 + frame.DataLength];

            return frame;
        }

        public bool IsValid()
        {
            return StartByte == Protocol.START_BYTE &&
                   EndByte == Protocol.END_BYTE &&
                   DataLength <= Protocol.MAX_DATA_LEN &&
                   Checksum == CalculateChecksum();
        }
    }

    // PWM Status structure
    public class PwmStatus
    {
        public bool IsRunning { get; set; }
        public byte DutyCycle { get; set; }
        public ushort Period { get; set; }
        public ushort Pulse { get; set; }

        public static PwmStatus FromByteArray(byte[] data)
        {
            if (data.Length < 6) return null;

            return new PwmStatus
            {
                IsRunning = data[0] != 0,
                DutyCycle = data[1],
                Period = BitConverter.ToUInt16(data, 2),
                Pulse = BitConverter.ToUInt16(data, 4)
            };
        }
    }

    // ADC Status structure
    public class AdcStatus
    {
        public bool IsRunning { get; set; }
        public byte Resolution { get; set; }
        public ushort LastValue { get; set; }
        public ushort VoltageMillivolts { get; set; }

        public static AdcStatus FromByteArray(byte[] data)
        {
            if (data.Length < 6) return null;

            return new AdcStatus
            {
                IsRunning = data[0] != 0,
                Resolution = data[1],
                LastValue = BitConverter.ToUInt16(data, 2),
                VoltageMillivolts = BitConverter.ToUInt16(data, 4)
            };
        }
    }

    // System Info structure
    public class SystemInfo
    {
        public uint UptimeMs { get; set; }
        public byte CpuUsage { get; set; }
        public ushort FreeHeap { get; set; }
        public byte Temperature { get; set; }

        public static SystemInfo FromByteArray(byte[] data)
        {
            if (data.Length < 8) return null;

            return new SystemInfo
            {
                UptimeMs = BitConverter.ToUInt32(data, 0),
                CpuUsage = data[4],
                FreeHeap = BitConverter.ToUInt16(data, 5),
                Temperature = data[7]
            };
        }
    }

    // Main STM32 Controller class
    public class STM32Controller : IDisposable
    {
        private SerialPort _serialPort;
        private bool _disposed = false;

        public STM32Controller(string portName, int baudRate = 115200)
        {
            _serialPort = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = Protocol.TIMEOUT_MS,
                WriteTimeout = Protocol.TIMEOUT_MS
            };
        }

        public void Connect()
        {
            if (!_serialPort.IsOpen)
            {
                _serialPort.Open();
            }
        }

        public void Disconnect()
        {
            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
            }
        }

        private async Task<ProtocolFrame> SendCommandAsync(Protocol.Commands command, byte[] data = null)
        {
            var frame = new ProtocolFrame
            {
                CommandId = command,
                DataLength = (byte)(data?.Length ?? 0)
            };

            if (data != null)
            {
                Array.Copy(data, frame.Data, data.Length);
            }

            var frameBytes = frame.ToByteArray();
            _serialPort.Write(frameBytes, 0, frameBytes.Length);

            // Read response
            var responseBuffer = new byte[256];
            var bytesRead = await Task.Run(() => _serialPort.Read(responseBuffer, 0, responseBuffer.Length));
            
            if (bytesRead >= 5)
            {
                var response = ProtocolFrame.FromByteArray(responseBuffer);
                if (response?.IsValid() == true)
                {
                    return response;
                }
            }

            return null;
        }

        // PWM Control Methods
        public async Task<bool> PingAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.PING);
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<bool> SetPwmDutyCycleAsync(byte dutyCycle)
        {
            if (dutyCycle > 100) throw new ArgumentException("Duty cycle must be 0-100");

            var response = await SendCommandAsync(Protocol.Commands.PWM_SET_DUTY, new[] { dutyCycle });
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<bool> StartPwmAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.PWM_START);
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<bool> StopPwmAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.PWM_STOP);
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<PwmStatus> GetPwmStatusAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.PWM_GET_STATUS);
            if (response?.Data[0] == (byte)Protocol.Status.OK)
            {
                var statusData = new byte[response.DataLength - 1];
                Array.Copy(response.Data, 1, statusData, 0, statusData.Length);
                return PwmStatus.FromByteArray(statusData);
            }
            return null;
        }

        // ADC Control Methods
        public async Task<ushort?> ReadAdcAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.ADC_READ);
            if (response?.Data[0] == (byte)Protocol.Status.OK && response.DataLength >= 3)
            {
                return BitConverter.ToUInt16(response.Data, 1);
            }
            return null;
        }

        public async Task<(ushort adcValue, ushort voltageMillivolts)?> ReadAdcVoltageAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.ADC_READ_VOLTAGE);
            if (response?.Data[0] == (byte)Protocol.Status.OK && response.DataLength >= 5)
            {
                var adcValue = BitConverter.ToUInt16(response.Data, 1);
                var voltage = BitConverter.ToUInt16(response.Data, 3);
                return (adcValue, voltage);
            }
            return null;
        }

        public async Task<(byte validSamples, ushort average, ushort voltageMillivolts)?> ReadAdcMultiAsync(byte samples)
        {
            if (samples == 0 || samples > 20) throw new ArgumentException("Samples must be 1-20");

            var response = await SendCommandAsync(Protocol.Commands.ADC_READ_MULTI, new[] { samples });
            if (response?.Data[0] == (byte)Protocol.Status.OK && response.DataLength >= 6)
            {
                var validSamples = response.Data[1];
                var average = BitConverter.ToUInt16(response.Data, 2);
                var voltage = BitConverter.ToUInt16(response.Data, 4);
                return (validSamples, average, voltage);
            }
            return null;
        }

        public async Task<bool> StartAdcAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.ADC_START);
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<bool> StopAdcAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.ADC_STOP);
            return response != null && response.Data[0] == (byte)Protocol.Status.OK;
        }

        public async Task<AdcStatus> GetAdcStatusAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.ADC_GET_STATUS);
            if (response?.Data[0] == (byte)Protocol.Status.OK)
            {
                var statusData = new byte[response.DataLength - 1];
                Array.Copy(response.Data, 1, statusData, 0, statusData.Length);
                return AdcStatus.FromByteArray(statusData);
            }
            return null;
        }

        // System Info
        public async Task<SystemInfo> GetSystemInfoAsync()
        {
            var response = await SendCommandAsync(Protocol.Commands.GET_SYSTEM_INFO);
            if (response?.Data[0] == (byte)Protocol.Status.OK)
            {
                var infoData = new byte[response.DataLength - 1];
                Array.Copy(response.Data, 1, infoData, 0, infoData.Length);
                return SystemInfo.FromByteArray(infoData);
            }
            return null;
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _serialPort?.Dispose();
                _disposed = true;
            }
        }
    }

    // Example usage
    class Program
    {
        static async Task Main(string[] args)
        {
            using var controller = new STM32Controller("COM3"); // Adjust COM port as needed
            
            try
            {
                controller.Connect();
                Console.WriteLine("Connected to STM32");

                // Test ping
                if (await controller.PingAsync())
                {
                    Console.WriteLine("Ping successful!");
                }

                // Set PWM to 75% duty cycle and start
                await controller.SetPwmDutyCycleAsync(75);
                await controller.StartPwmAsync();
                Console.WriteLine("PWM started at 75% duty cycle");

                // Get PWM status
                var pwmStatus = await controller.GetPwmStatusAsync();
                if (pwmStatus != null)
                {
                    Console.WriteLine($"PWM Status: Running={pwmStatus.IsRunning}, Duty={pwmStatus.DutyCycle}%, Period={pwmStatus.Period}, Pulse={pwmStatus.Pulse}");
                }

                // Read ADC voltage
                var adcReading = await controller.ReadAdcVoltageAsync();
                if (adcReading.HasValue)
                {
                    Console.WriteLine($"ADC: {adcReading.Value.adcValue}, Voltage: {adcReading.Value.voltageMillivolts / 1000.0:F3}V");
                }

                // Read multiple ADC samples
                var multiReading = await controller.ReadAdcMultiAsync(5);
                if (multiReading.HasValue)
                {
                    Console.WriteLine($"ADC Multi: {multiReading.Value.validSamples} samples, Average: {multiReading.Value.average}, Voltage: {multiReading.Value.voltageMillivolts / 1000.0:F3}V");
                }

                // Get system info
                var sysInfo = await controller.GetSystemInfoAsync();
                if (sysInfo != null)
                {
                    Console.WriteLine($"System: Uptime={sysInfo.UptimeMs}ms, Free Heap={sysInfo.FreeHeap}B, Temperature={sysInfo.Temperature}°C");
                }

                // Stop PWM
                await controller.StopPwmAsync();
                Console.WriteLine("PWM stopped");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Error: {ex.Message}");
            }
            finally
            {
                controller.Disconnect();
                Console.WriteLine("Disconnected");
            }
        }
    }
}
