#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "uart_driver_config.h"
#include "uart_driver.h"
#include "logging.h"
#include "fault_module.h"

ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;

uart_drv_t shared_uart;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
static void TestTask(void *arg);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  uart_system_init(&shared_uart, &huart2, &hdma_usart2_tx, &hdma_usart2_rx);
  log_write(LOG_LEVEL_INFO, "Started");

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(testTask, TestTask, osPriorityAboveNormal, 0, 256);
  osThreadCreate(osThread(testTask), NULL);

  osKernelStart();

  while (1)
  {

    
  }

}



/* USER CODE BEGIN 4 */
static void TestTask(void *arg) {
    TelemetryPacket pkt = {0};

    for (;;) {
        // Raise a test fault
        static uint32_t faultCode = FAULT_OVERCURRENT;
        fault_raise((FaultCode)faultCode);
        log_write(LOG_LEVEL_INFO, "Raised fault: %s", fault_to_string((FaultCode)faultCode));

        // Send a telemetry packet
        pkt.sensor1++;
        pkt.sensor2 += 0.5f;
        telemetry_send(&pkt);

        // Clear the fault after 50 ms
        vTaskDelay(pdMS_TO_TICKS(50));
        fault_clear((FaultCode)faultCode);
        log_write(LOG_LEVEL_INFO, "Cleared fault: %s", fault_to_string((FaultCode)faultCode));

        // Cycle fault code for next iteration
        faultCode++;
        if (faultCode >= FAULT_COUNT) faultCode = FAULT_OVERCURRENT;

        // Wait 100 ms before next cycle
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
