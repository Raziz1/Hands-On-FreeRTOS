/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "semphr.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define DWT_CTRL	(*(volatile uint32_t*)0xE0001000)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
//global space for some variable
char usr_msg[250]={0};

TaskHandle_t xTaskHandleM;
TaskHandle_t xTaskHandleE;

/* The tasks to be created. */
static void vManagerTask( void *pvParameters );
static void vEmployeeTask( void *pvParameters );

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize both manager and employee task */
SemaphoreHandle_t xWork;

/* this is the queue which manager uses to put the work ticket id */
QueueHandle_t xWorkQueue;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  // Enable the CYCCNT counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Start the SEGGER recorder
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  /* Before a semaphore is used it must be explicitly created.
   * In this example a binary semaphore is created
   */
   vSemaphoreCreateBinary( xWork );

   /* The queue is created to hold a maximum of 1 Element. */
   xWorkQueue = xQueueCreate( 1, sizeof( unsigned int ) );

   /* Check the semaphore and queue was created successfully. */
   if( (xWork != NULL) && (xWorkQueue != NULL) )
   {
	   /*
		* Both tasks need to have the same priority (to ensure that yielding works),
		* take/give the semaphore in turns and yield the execution.
		*/
	   /* Create the 'Manager' task.  This is the task that will be synchronized with the Employee task*/
	   xTaskCreate( vManagerTask, "Manager", 500, NULL, 3, &xTaskHandleM );
	   xTaskCreate( vEmployeeTask, "Employee", 500, NULL, 3, &xTaskHandleE );
	   /* Start the scheduler so the created tasks start executing. */
	   vTaskStartScheduler();
   }

  //if the control comes here, then the launch of the scheduler has failed due to insufficient memory in heap
  printf("INSUFFUCIENT MEMORY IN HEAP!\n");
  SEGGER_SYSVIEW_PrintfTarget("Queue/Sema create failed.. \r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void vManagerTask( void *pvParameters )
{

	 unsigned int xWorkTicketId;
	 portBASE_TYPE xStatus;
	 portBASE_TYPE xSemaStatus;
	 unsigned int upper = 50;
	 unsigned int lower = 10;

   /* The semaphore is created in the 'empty' state, meaning the semaphore must
	 first be given using the xSemaphoreGive() API function before it
	 can subsequently be taken (obtained) */
   xSemaphoreGive( xWork);

   for( ;; )
   {
	   /*
	    * If a semaphore is available take one so the manager can assign its employee a task
	    * If a semaphore isn't available put this task in the blocking state indefinitely
	    */
	   xSemaStatus = xSemaphoreTake(xWork, portMAX_DELAY);
	   if (xSemaStatus == pdTRUE)
	   {
		   /* get a work ticket id(some random number between 10 - 50) */
		   xWorkTicketId = (rand() % (upper + 1 - lower) + lower);

		   /* Sends work ticket id to the work queue */
		   xStatus = xQueueSend( xWorkQueue, &xWorkTicketId , portMAX_DELAY ); //Post an item on back of the queue

		   if( xStatus != pdPASS )
		   {
			   sprintf(usr_msg,"Could not send to the queue.\r\n");
			   SEGGER_SYSVIEW_PrintfTarget(usr_msg);
		   }
		   else
		   {
			   sprintf(usr_msg,"Manager Task: Sending Ticket id : %d.\r\n", xWorkTicketId);
			   SEGGER_SYSVIEW_PrintfTarget(usr_msg);
			   /* Manager notifying the employee by "Giving" semaphore */
			   xSemaphoreGive( xWork);
			   /* after assigning the work , just yield the processor because nothing to do */
			   taskYIELD();
		   }
	   }
	   else
	   {
		   sprintf(usr_msg,"Employee Busy!!!\r\n");
		   SEGGER_SYSVIEW_PrintfTarget(usr_msg);
	   }
   }
}
/*-----------------------------------------------------------*/

void EmployeeDoWork(unsigned char TicketId)
{
	/* implement the work according to TickedID */
	sprintf(usr_msg,"Employee task : Working on Ticket id : %d\r\n",TicketId);
	SEGGER_SYSVIEW_PrintfTarget(usr_msg);
	vTaskDelay(pdMS_TO_TICKS(TicketId));
}

static void vEmployeeTask( void *pvParameters )
{
	unsigned char xWorkTicketId;
	portBASE_TYPE xStatus;
	portBASE_TYPE xSemaStatus;
   /* As per most tasks, this task is implemented within an infinite loop. */
   for( ;; )
   {
	   /* First Employee tries to take the semaphore,
	    * if it is available that means there is a task assigned by manager
	    * otherwise employee task will be blocked indefinitely
	    * */
	   xSemaStatus = xSemaphoreTake( xWork, portMAX_DELAY);
	   if (xSemaStatus == pdTRUE)
	   {
		   sprintf(usr_msg,"Employee task : Semaphore Taken!\r\n");
		   SEGGER_SYSVIEW_PrintfTarget(usr_msg);

		   /*
		    * Get the ticket id from the work queue
		    * If the queue timeout is set to portMAX_DELAY then this task is blocked indefinitely
		    * and the QueueReceive and Queuesend calls are used to perform context switching.
		    * In this example we want the semaphores to trigger the context switching
		    * */
		   xStatus = xQueueReceive( xWorkQueue, &xWorkTicketId, 0);
		   if( xStatus == pdPASS )
		   {
			   /* employee may decode the xWorkTicketId in this function to do the work*/
			   EmployeeDoWork(xWorkTicketId);
			   sprintf(usr_msg,"Employee task : Employee completed ticket\r\n");
			   SEGGER_SYSVIEW_PrintfTarget(usr_msg);

			   /*
			    * Return the semaphore to the manager and perform a context switch to the manager task
			    */
			   xSemaphoreGive(xWork);
			   taskYIELD();
		   }
		   else
		   {
			   /* We did not receive anything from the queue.  This must be an error as this task should only run when the manager assigns at least one work. */
			   sprintf(usr_msg,"Employee task : Queue is empty , nothing to do.\r\n");
			   SEGGER_SYSVIEW_PrintfTarget(usr_msg);
			   taskYIELD();
		   }
	   }
	   else
	   {
		   sprintf(usr_msg,"Employee task : No semaphore available!\r\n");
		   SEGGER_SYSVIEW_PrintfTarget(usr_msg);
	   }
   }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
