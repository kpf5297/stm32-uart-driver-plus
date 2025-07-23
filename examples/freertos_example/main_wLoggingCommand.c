#include "main.h"
#include "cmsis_os.h"
#include "uart_driver.h"
#include <string.h>
#include "sample_commands.h"
#include "command_module.h"
#include "logging.h"

/**
 * @file main_wLoggingCommand.c
 * @brief Example using UART driver with CLI and logging modules.
 *
 * A terminal on USART2 can issue commands. The logging module outputs
 * periodic messages and telemetry packets using the same UART driver.
*/

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

osThreadId defaultTaskHandle;

static uart_drv_t uart2_drv;  // UART driver instance for USART2
static SemaphoreHandle_t rx_done_sem;
static void uart_evt_cb(uart_event_t evt, void *user_ctx);
static void log_test_task(void *pv);
static void telemetry_test_task(void *pv);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void const * argument);

static void uart_sender_task(void *pv);
static void uart_receiver_task(void *pv);
static void uart_echo_task(void *pv);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  rx_done_sem = xSemaphoreCreateBinary();

  // Initialize the UART driver
  if (uart_init(&uart2_drv, &huart2, &hdma_usart2_tx, &hdma_usart2_rx) != UART_OK) {
      Error_Handler();
  }

  // Initialize the command interpreter
  // This registers the UART callback, creates the RTOS queue & task
  cmd_init(&uart2_drv);

  log_init(&uart2_drv);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//  xTaskCreate(uart_sender_task,  "UART_SND",  256, NULL, tskIDLE_PRIORITY+1, NULL);
//  xTaskCreate(uart_receiver_task,"UART_RCV",  256, NULL, tskIDLE_PRIORITY+1, NULL);
  xTaskCreate(uart_echo_task, "UART_ECHO", 256, NULL, tskIDLE_PRIORITY+1, NULL);

  xTaskCreate(log_test_task, "LogTest", 256, NULL, tskIDLE_PRIORITY+1, NULL);

  xTaskCreate(telemetry_test_task, "TlmTest", 256, NULL, tskIDLE_PRIORITY + 1, NULL);

  log_write(LOG_LEVEL_INFO, "System initialized.");

  /* Start scheduler */
//  osKernelStart();
  vTaskStartScheduler();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */

  while (1)
  {

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

	HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

static void uart_echo_task(void *pv)
{
    uint8_t c;
    for (;;) {
        // wait for exactly 1 byte
//        if (uart_receive_nb(&uart2_drv, &c, 1) == UART_OK) {
        if (uart_start_dma_rx(&uart2_drv, &c, 1) == UART_OK) {
            // block until that one byte arrives
            if (xSemaphoreTake(rx_done_sem, portMAX_DELAY) == pdTRUE) {
                // immediately echo it back
//                uart_send_nb(&uart2_drv, &c, 1);
            	uart_start_dma_tx(&uart2_drv, &c, 1);
            }
            log_write(LOG_LEVEL_INFO, "Test info");
              vTaskDelay(pdMS_TO_TICKS(500));
              log_write(LOG_LEVEL_WARN, "Test warn");
              vTaskDelay(pdMS_TO_TICKS(500));
              log_write(LOG_LEVEL_ERROR, "Test error");
        }
        // no vTaskDelay needed here – we re-arm as soon as we finish
    }
}

static void log_test_task(void *pv)
{
    vTaskDelay(pdMS_TO_TICKS(1000));  // Optional delay to wait for CLI ready
    log_write(LOG_LEVEL_INFO, "Test info");
    vTaskDelay(pdMS_TO_TICKS(1000));
    log_write(LOG_LEVEL_WARN, "Test warn");
    vTaskDelay(pdMS_TO_TICKS(1000));
    log_write(LOG_LEVEL_ERROR, "Test error");
    vTaskDelay(pdMS_TO_TICKS(1000));
    log_write(LOG_LEVEL_FATAL, "Test fatal");
    vTaskDelete(NULL);
}

static void telemetry_test_task(void *pv)
{
    TelemetryPacket pkt = {0};
    for (;;) {
        pkt.sensor1++;
        pkt.sensor2 = 3.14f * pkt.sensor1;

        telemetry_send(&pkt);

        vTaskDelay(pdMS_TO_TICKS(2000));  // Every 2 seconds
    }
}

// UART event callback — called from ISR context
static void uart_evt_cb(uart_event_t evt, void *user_ctx)
{
    BaseType_t higher_woken = pdFALSE;
    if (evt == UART_EVT_RX_COMPLETE) {
        // give the semaphore to wake the RX task
        xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &higher_woken);
    }
    // optionally handle TX_COMPLETE or ERROR events here …
    portYIELD_FROM_ISR(higher_woken);
}

//----------------------------------------------------------------
// Sender task — periodically transmit a message
static void uart_sender_task(void *pv)
{
    const char *msg = "Hello from FreeRTOS UART!\r\n";
    for (;;) {
        if (uart_send_nb(&uart2_drv, (uint8_t*)msg, strlen(msg)) == UART_OK) {
            // optionally wait for TX_COMPLETE event or just delay
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//----------------------------------------------------------------
// Receiver task — issue a receive and then block until data arrives
#define RX_BUF_LEN  32
static void uart_receiver_task(void *pv)
{
    uint8_t rx_buf[RX_BUF_LEN];

    for (;;) {
        // kick off a non-blocking receive of up to RX_BUF_LEN bytes
        if (uart_receive_nb(&uart2_drv, rx_buf, RX_BUF_LEN) == UART_OK) {
            // wait until the callback gives us the semaphore
            if (xSemaphoreTake(rx_done_sem, portMAX_DELAY) == pdTRUE) {
                // rx_buf now contains RX_BUF_LEN bytes (or fewer if your app tracks count)
                // process your data here…
            }
        } else {
            // driver was busy; back off a bit
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */

void StartDefaultTask(void const * argument)
{

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{

  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }

}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{

  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

}
#endif /* USE_FULL_ASSERT */
