/**
 * @file freertos_ll_example.c
 * @brief FreeRTOS + LL drivers example for STM32 UART Driver Plus
 * 
 * This example demonstrates how to use the UART driver with:
 * - STM32 LL (Low Layer) drivers
 * - FreeRTOS RTOS
 * - CMSIS-RTOS wrapper (optional)
 * 
 * Performance benefits of LL:
 * - Lower memory footprint
 * - Faster execution (direct register access)
 * - More control over peripheral configuration
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* LL Driver includes */
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"

/* UART Driver Plus with LL support */
#include "uart_driver.h"
#include "uart_driver_config.h"
#include "logging.h"
#include "command_module.h"

/* Private variables */
static uart_drv_t main_uart;
static TaskHandle_t uart_task_handle;
static uint8_t rx_buffer[256];

/* Private function prototypes */
static void SystemClock_Config(void);
static void MX_GPIO_Init_LL(void);
static void MX_USART2_UART_Init_LL(void);
static void MX_DMA_Init_LL(void);
static void UART_Communication_Task(void *pvParameters);

/**
 * @brief Main function - LL + FreeRTOS example
 */
int main(void)
{
    /* MCU Configuration */
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize LL peripherals */
    MX_GPIO_Init_LL();
    MX_DMA_Init_LL();
    MX_USART2_UART_Init_LL();
    
    /* Initialize UART driver with LL backend and optional modules */
    uart_status_t uart_result = uart_system_init_ll(&main_uart,
                                                   USART2,        // LL UART instance
                                                   DMA1,          // TX DMA instance
                                                   DMA1,          // RX DMA instance
                                                   LL_DMA_STREAM_6, // TX stream
                                                   LL_DMA_STREAM_5); // RX stream

    if (uart_result != UART_OK) {
        Error_Handler();
    }

#if LOGGING_ENABLED
    LOG_INFO("STM32 UART Driver Plus - LL Example Started");
    LOG_INFO("Configuration: LL drivers + FreeRTOS");
#endif
    
    /* Create UART communication task */
    BaseType_t task_result = xTaskCreate(
        UART_Communication_Task,
        "UART_Task",
        512,                    // Stack size
        &main_uart,            // Task parameter
        tskIDLE_PRIORITY + 2,  // Priority
        &uart_task_handle
    );
    
    if (task_result != pdPASS) {
        Error_Handler();
    }
    
    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    /* Should never reach here */
    while (1) {
        Error_Handler();
    }
}

/**
 * @brief UART Communication Task
 * Demonstrates thread-safe UART operations with LL backend
 */
static void UART_Communication_Task(void *pvParameters)
{
    uart_drv_t *uart = (uart_drv_t*)pvParameters;
    uint8_t welcome_msg[] = "STM32 UART Driver Plus - LL Example Ready\r\n";
    uint32_t message_counter = 0;
    
    /* Send welcome message */
    uart_send_blocking(uart, welcome_msg, sizeof(welcome_msg) - 1, 1000);
    
    while (1) {
        /* Demonstrate blocking receive with timeout */
        uart_status_t rx_status = uart_receive_blocking(uart, rx_buffer, 1, 100);
        
        if (rx_status == UART_OK) {
            /* Echo received character */
            uart_send_blocking(uart, rx_buffer, 1, 100);
            
            /* Process command if newline received */
            if (rx_buffer[0] == '\r' || rx_buffer[0] == '\n') {
                /* Command interpreter will automatically handle the command */

                /* Send periodic status message */
                char status_msg[64];
                snprintf(status_msg, sizeof(status_msg), 
                        "\r\nMsg #%lu - LL UART Status: OK\r\n", 
                        ++message_counter);
                uart_send_blocking(uart, (uint8_t*)status_msg, strlen(status_msg), 1000);
                
#if LOGGING_ENABLED
                LOG_DEBUG("Message %lu processed", message_counter);
#endif
            }
        }
        
        /* Yield to other tasks */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/**
 * @brief Initialize GPIO with LL drivers
 */
static void MX_GPIO_Init_LL(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    
    /* Configure LED pin (PC13) */
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_13, LL_GPIO_OUTPUT_PUSHPULL);
    LL_GPIO_SetPinSpeed(GPIOC, LL_GPIO_PIN_13, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_NO);
    
    /* Configure UART pins (PA2 = TX, PA3 = RX) */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Initialize USART2 with LL drivers
 */
static void MX_USART2_UART_Init_LL(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};
    
    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    
    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(USART2_IRQn);
    
    /* Configure USART2 */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    
    LL_USART_Init(USART2, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(USART2);
    
    /* Enable USART2 */
    LL_USART_Enable(USART2);
    
    /* Enable interrupts */
    LL_USART_EnableIT_RXNE(USART2);
    LL_USART_EnableIT_TC(USART2);
    LL_USART_EnableIT_ERROR(USART2);
}

/**
 * @brief Initialize DMA with LL drivers
 */
static void MX_DMA_Init_LL(void)
{
    LL_DMA_InitTypeDef DMA_InitStruct = {0};
    
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    
    /* Configure DMA for USART2 TX (DMA1 Stream6 Channel4) */
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART2->DR;
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    DMA_InitStruct.Mode = LL_DMA_MODE_NORMAL;
    DMA_InitStruct.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
    DMA_InitStruct.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
    DMA_InitStruct.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
    DMA_InitStruct.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
    DMA_InitStruct.NbData = 0;
    DMA_InitStruct.Channel = LL_DMA_CHANNEL_4;
    DMA_InitStruct.Priority = LL_DMA_PRIORITY_LOW;
    DMA_InitStruct.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
    
    LL_DMA_Init(DMA1, LL_DMA_STREAM_6, &DMA_InitStruct);
    
    /* Configure DMA for USART2 RX (DMA1 Stream5 Channel4) */
    DMA_InitStruct.PeriphOrM2MSrcAddress = (uint32_t)&USART2->DR;
    DMA_InitStruct.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    LL_DMA_Init(DMA1, LL_DMA_STREAM_5, &DMA_InitStruct);
    
    /* DMA interrupt configuration */
    NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    
    NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    
    /* Enable DMA interrupts */
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_5);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_6);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_6);
}

/**
 * @brief USART2 Interrupt Handler (LL version)
 */
void USART2_IRQHandler(void)
{
    /* RX data available */
    if (LL_USART_IsActiveFlag_RXNE(USART2)) {
        /* Handle RX in abstraction layer */
        uart_abstraction_handle_ll_rx_irq(&main_uart.abstraction);
    }
    
    /* Transmission complete */
    if (LL_USART_IsActiveFlag_TC(USART2)) {
        LL_USART_ClearFlag_TC(USART2);
        /* Handle TX complete in abstraction layer */
        uart_abstraction_handle_ll_tx_irq(&main_uart.abstraction);
    }
    
    /* Error handling */
    if (LL_USART_IsActiveFlag_ORE(USART2)) {
        LL_USART_ClearFlag_ORE(USART2);
    }
    if (LL_USART_IsActiveFlag_NE(USART2)) {
        LL_USART_ClearFlag_NE(USART2);
    }
    if (LL_USART_IsActiveFlag_FE(USART2)) {
        LL_USART_ClearFlag_FE(USART2);
    }
    if (LL_USART_IsActiveFlag_PE(USART2)) {
        LL_USART_ClearFlag_PE(USART2);
    }
}

/**
 * @brief DMA Stream 5 IRQ Handler (USART2 RX)
 */
void DMA1_Stream5_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC5(DMA1)) {
        LL_DMA_ClearFlag_TC5(DMA1);
        /* Handle DMA RX complete */
        uart_abstraction_handle_ll_dma_rx_irq(&main_uart.abstraction);
    }
    
    if (LL_DMA_IsActiveFlag_TE5(DMA1)) {
        LL_DMA_ClearFlag_TE5(DMA1);
        /* Handle DMA error */
    }
}

/**
 * @brief DMA Stream 6 IRQ Handler (USART2 TX)
 */
void DMA1_Stream6_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TC6(DMA1)) {
        LL_DMA_ClearFlag_TC6(DMA1);
        /* Handle DMA TX complete */
        uart_abstraction_handle_ll_dma_tx_irq(&main_uart.abstraction);
    }
    
    if (LL_DMA_IsActiveFlag_TE6(DMA1)) {
        LL_DMA_ClearFlag_TE6(DMA1);
        /* Handle DMA error */
    }
}

/**
 * @brief System Clock Configuration
 */
static void SystemClock_Config(void)
{
    /* Configure system clock for 84MHz (adjust as needed) */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        Error_Handler();
    }
    
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();
    
    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1) {
    }
    
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
    LL_RCC_PLL_Enable();
    
    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    
    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    
    LL_SetSystemCoreClock(84000000);
    
    /* Configure SysTick */
    LL_Init1msTick(84000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();
    
    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
}

/**
 * @brief Error Handler
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Toggle LED to indicate error */
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 1000000; i++);
    }
}
