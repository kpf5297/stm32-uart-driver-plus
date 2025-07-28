using System;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace STM32Controller
{
    public partial class MainForm : Form
    {
        private STM32Controller _controller;
        private Timer _updateTimer;
        private bool _isConnected = false;

        // Controls
        private ComboBox _portComboBox;
        private Button _connectButton;
        private Button _disconnectButton;
        private GroupBox _pwmGroupBox;
        private TrackBar _pwmTrackBar;
        private Label _pwmValueLabel;
        private Button _pwmStartButton;
        private Button _pwmStopButton;
        private Label _pwmStatusLabel;
        private GroupBox _adcGroupBox;
        private Label _adcValueLabel;
        private Label _adcVoltageLabel;
        private ProgressBar _adcProgressBar;
        private NumericUpDown _adcSamplesNumeric;
        private Button _adcReadButton;
        private Button _adcReadMultiButton;
        private GroupBox _systemGroupBox;
        private Label _systemUptimeLabel;
        private Label _systemHeapLabel;
        private Label _systemTempLabel;
        private Button _pingButton;
        private Label _connectionStatusLabel;

        public MainForm()
        {
            InitializeComponent();
            InitializeTimer();
            PopulateComPorts();
        }

        private void InitializeComponent()
        {
            this.Text = "STM32 Controller";
            this.Size = new Size(600, 500);
            this.FormBorderStyle = FormBorderStyle.FixedDialog;
            this.MaximizeBox = false;
            this.StartPosition = FormStartPosition.CenterScreen;

            // Connection controls
            var connectionPanel = new Panel { Dock = DockStyle.Top, Height = 60 };
            this.Controls.Add(connectionPanel);

            var portLabel = new Label { Text = "COM Port:", Location = new Point(10, 15), AutoSize = true };
            connectionPanel.Controls.Add(portLabel);

            _portComboBox = new ComboBox 
            { 
                Location = new Point(80, 12), 
                Width = 100,
                DropDownStyle = ComboBoxStyle.DropDownList
            };
            connectionPanel.Controls.Add(_portComboBox);

            _connectButton = new Button 
            { 
                Text = "Connect", 
                Location = new Point(190, 10), 
                Width = 80
            };
            _connectButton.Click += ConnectButton_Click;
            connectionPanel.Controls.Add(_connectButton);

            _disconnectButton = new Button 
            { 
                Text = "Disconnect", 
                Location = new Point(280, 10), 
                Width = 80,
                Enabled = false
            };
            _disconnectButton.Click += DisconnectButton_Click;
            connectionPanel.Controls.Add(_disconnectButton);

            _pingButton = new Button 
            { 
                Text = "Ping", 
                Location = new Point(370, 10), 
                Width = 60,
                Enabled = false
            };
            _pingButton.Click += PingButton_Click;
            connectionPanel.Controls.Add(_pingButton);

            _connectionStatusLabel = new Label 
            { 
                Text = "Disconnected", 
                Location = new Point(10, 35), 
                ForeColor = Color.Red,
                AutoSize = true
            };
            connectionPanel.Controls.Add(_connectionStatusLabel);

            // PWM Group
            _pwmGroupBox = new GroupBox 
            { 
                Text = "PWM Control", 
                Location = new Point(10, 70), 
                Size = new Size(270, 150),
                Enabled = false
            };
            this.Controls.Add(_pwmGroupBox);

            var pwmLabel = new Label { Text = "Duty Cycle:", Location = new Point(10, 25), AutoSize = true };
            _pwmGroupBox.Controls.Add(pwmLabel);

            _pwmTrackBar = new TrackBar 
            { 
                Location = new Point(10, 45), 
                Width = 200, 
                Maximum = 100, 
                Minimum = 0, 
                Value = 50,
                TickFrequency = 10
            };
            _pwmTrackBar.ValueChanged += PwmTrackBar_ValueChanged;
            _pwmGroupBox.Controls.Add(_pwmTrackBar);

            _pwmValueLabel = new Label 
            { 
                Text = "50%", 
                Location = new Point(220, 45), 
                AutoSize = true 
            };
            _pwmGroupBox.Controls.Add(_pwmValueLabel);

            _pwmStartButton = new Button 
            { 
                Text = "Start", 
                Location = new Point(10, 85), 
                Width = 80 
            };
            _pwmStartButton.Click += PwmStartButton_Click;
            _pwmGroupBox.Controls.Add(_pwmStartButton);

            _pwmStopButton = new Button 
            { 
                Text = "Stop", 
                Location = new Point(100, 85), 
                Width = 80 
            };
            _pwmStopButton.Click += PwmStopButton_Click;
            _pwmGroupBox.Controls.Add(_pwmStopButton);

            _pwmStatusLabel = new Label 
            { 
                Text = "Status: Unknown", 
                Location = new Point(10, 115), 
                AutoSize = true 
            };
            _pwmGroupBox.Controls.Add(_pwmStatusLabel);

            // ADC Group
            _adcGroupBox = new GroupBox 
            { 
                Text = "ADC Reading", 
                Location = new Point(290, 70), 
                Size = new Size(280, 150),
                Enabled = false
            };
            this.Controls.Add(_adcGroupBox);

            _adcValueLabel = new Label 
            { 
                Text = "ADC Value: 0", 
                Location = new Point(10, 25), 
                AutoSize = true 
            };
            _adcGroupBox.Controls.Add(_adcValueLabel);

            _adcVoltageLabel = new Label 
            { 
                Text = "Voltage: 0.000V", 
                Location = new Point(10, 45), 
                AutoSize = true 
            };
            _adcGroupBox.Controls.Add(_adcVoltageLabel);

            _adcProgressBar = new ProgressBar 
            { 
                Location = new Point(10, 70), 
                Width = 200, 
                Maximum = 4095, 
                Minimum = 0 
            };
            _adcGroupBox.Controls.Add(_adcProgressBar);

            var samplesLabel = new Label { Text = "Samples:", Location = new Point(10, 100), AutoSize = true };
            _adcGroupBox.Controls.Add(samplesLabel);

            _adcSamplesNumeric = new NumericUpDown 
            { 
                Location = new Point(70, 98), 
                Width = 60, 
                Minimum = 1, 
                Maximum = 20, 
                Value = 5 
            };
            _adcGroupBox.Controls.Add(_adcSamplesNumeric);

            _adcReadButton = new Button 
            { 
                Text = "Read", 
                Location = new Point(140, 95), 
                Width = 60 
            };
            _adcReadButton.Click += AdcReadButton_Click;
            _adcGroupBox.Controls.Add(_adcReadButton);

            _adcReadMultiButton = new Button 
            { 
                Text = "Multi", 
                Location = new Point(210, 95), 
                Width = 60 
            };
            _adcReadMultiButton.Click += AdcReadMultiButton_Click;
            _adcGroupBox.Controls.Add(_adcReadMultiButton);

            // System Info Group
            _systemGroupBox = new GroupBox 
            { 
                Text = "System Information", 
                Location = new Point(10, 230), 
                Size = new Size(560, 120),
                Enabled = false
            };
            this.Controls.Add(_systemGroupBox);

            _systemUptimeLabel = new Label 
            { 
                Text = "Uptime: Unknown", 
                Location = new Point(10, 25), 
                AutoSize = true 
            };
            _systemGroupBox.Controls.Add(_systemUptimeLabel);

            _systemHeapLabel = new Label 
            { 
                Text = "Free Heap: Unknown", 
                Location = new Point(10, 50), 
                AutoSize = true 
            };
            _systemGroupBox.Controls.Add(_systemHeapLabel);

            _systemTempLabel = new Label 
            { 
                Text = "Temperature: Unknown", 
                Location = new Point(10, 75), 
                AutoSize = true 
            };
            _systemGroupBox.Controls.Add(_systemTempLabel);
        }

        private void InitializeTimer()
        {
            _updateTimer = new Timer { Interval = 1000 }; // Update every second
            _updateTimer.Tick += UpdateTimer_Tick;
        }

        private void PopulateComPorts()
        {
            _portComboBox.Items.Clear();
            foreach (string port in System.IO.Ports.SerialPort.GetPortNames())
            {
                _portComboBox.Items.Add(port);
            }
            if (_portComboBox.Items.Count > 0)
                _portComboBox.SelectedIndex = 0;
        }

        private async void ConnectButton_Click(object sender, EventArgs e)
        {
            if (_portComboBox.SelectedItem == null)
            {
                MessageBox.Show("Please select a COM port.", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                return;
            }

            try
            {
                _controller = new STM32Controller(_portComboBox.SelectedItem.ToString());
                _controller.Connect();

                // Test connection with ping
                if (await _controller.PingAsync())
                {
                    _isConnected = true;
                    UpdateConnectionStatus(true);
                    _updateTimer.Start();
                }
                else
                {
                    MessageBox.Show("Failed to ping STM32. Check connection.", "Connection Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    _controller.Disconnect();
                    _controller.Dispose();
                    _controller = null;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"Connection failed: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                _controller?.Dispose();
                _controller = null;
            }
        }

        private void DisconnectButton_Click(object sender, EventArgs e)
        {
            _updateTimer.Stop();
            _isConnected = false;
            UpdateConnectionStatus(false);
            _controller?.Disconnect();
            _controller?.Dispose();
            _controller = null;
        }

        private async void PingButton_Click(object sender, EventArgs e)
        {
            if (_controller != null)
            {
                try
                {
                    bool success = await _controller.PingAsync();
                    MessageBox.Show(success ? "Ping successful!" : "Ping failed!", "Ping Result", 
                                  MessageBoxButtons.OK, success ? MessageBoxIcon.Information : MessageBoxIcon.Warning);
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"Ping error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void PwmTrackBar_ValueChanged(object sender, EventArgs e)
        {
            _pwmValueLabel.Text = $"{_pwmTrackBar.Value}%";
            if (_controller != null)
            {
                try
                {
                    await _controller.SetPwmDutyCycleAsync((byte)_pwmTrackBar.Value);
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"PWM set error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void PwmStartButton_Click(object sender, EventArgs e)
        {
            if (_controller != null)
            {
                try
                {
                    await _controller.StartPwmAsync();
                    await UpdatePwmStatus();
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"PWM start error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void PwmStopButton_Click(object sender, EventArgs e)
        {
            if (_controller != null)
            {
                try
                {
                    await _controller.StopPwmAsync();
                    await UpdatePwmStatus();
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"PWM stop error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void AdcReadButton_Click(object sender, EventArgs e)
        {
            if (_controller != null)
            {
                try
                {
                    var reading = await _controller.ReadAdcVoltageAsync();
                    if (reading.HasValue)
                    {
                        _adcValueLabel.Text = $"ADC Value: {reading.Value.adcValue}";
                        _adcVoltageLabel.Text = $"Voltage: {reading.Value.voltageMillivolts / 1000.0:F3}V";
                        _adcProgressBar.Value = Math.Min(reading.Value.adcValue, _adcProgressBar.Maximum);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"ADC read error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void AdcReadMultiButton_Click(object sender, EventArgs e)
        {
            if (_controller != null)
            {
                try
                {
                    var reading = await _controller.ReadAdcMultiAsync((byte)_adcSamplesNumeric.Value);
                    if (reading.HasValue)
                    {
                        _adcValueLabel.Text = $"ADC Average ({reading.Value.validSamples} samples): {reading.Value.average}";
                        _adcVoltageLabel.Text = $"Voltage: {reading.Value.voltageMillivolts / 1000.0:F3}V";
                        _adcProgressBar.Value = Math.Min(reading.Value.average, _adcProgressBar.Maximum);
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"ADC multi read error: {ex.Message}", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        private async void UpdateTimer_Tick(object sender, EventArgs e)
        {
            if (_controller != null && _isConnected)
            {
                try
                {
                    await UpdatePwmStatus();
                    await UpdateSystemInfo();
                }
                catch
                {
                    // Ignore errors during periodic updates
                }
            }
        }

        private async Task UpdatePwmStatus()
        {
            try
            {
                var status = await _controller.GetPwmStatusAsync();
                if (status != null)
                {
                    _pwmStatusLabel.Text = $"Status: {(status.IsRunning ? "Running" : "Stopped")}, " +
                                         $"Duty: {status.DutyCycle}%, Period: {status.Period}, Pulse: {status.Pulse}";
                }
            }
            catch
            {
                _pwmStatusLabel.Text = "Status: Error";
            }
        }

        private async Task UpdateSystemInfo()
        {
            try
            {
                var info = await _controller.GetSystemInfoAsync();
                if (info != null)
                {
                    _systemUptimeLabel.Text = $"Uptime: {info.UptimeMs / 1000.0:F1} seconds";
                    _systemHeapLabel.Text = $"Free Heap: {info.FreeHeap} bytes";
                    _systemTempLabel.Text = $"Temperature: {info.Temperature}°C";
                }
            }
            catch
            {
                _systemUptimeLabel.Text = "Uptime: Error";
                _systemHeapLabel.Text = "Free Heap: Error";
                _systemTempLabel.Text = "Temperature: Error";
            }
        }

        private void UpdateConnectionStatus(bool connected)
        {
            _isConnected = connected;
            _connectButton.Enabled = !connected;
            _disconnectButton.Enabled = connected;
            _pingButton.Enabled = connected;
            _pwmGroupBox.Enabled = connected;
            _adcGroupBox.Enabled = connected;
            _systemGroupBox.Enabled = connected;

            _connectionStatusLabel.Text = connected ? "Connected" : "Disconnected";
            _connectionStatusLabel.ForeColor = connected ? Color.Green : Color.Red;

            if (!connected)
            {
                _pwmStatusLabel.Text = "Status: Unknown";
                _adcValueLabel.Text = "ADC Value: 0";
                _adcVoltageLabel.Text = "Voltage: 0.000V";
                _adcProgressBar.Value = 0;
                _systemUptimeLabel.Text = "Uptime: Unknown";
                _systemHeapLabel.Text = "Free Heap: Unknown";
                _systemTempLabel.Text = "Temperature: Unknown";
            }
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            _updateTimer.Stop();
            _controller?.Disconnect();
            _controller?.Dispose();
            base.OnFormClosing(e);
        }
    }

    // Program entry point
    public static class Program
    {
        [STAThread]
        public static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
