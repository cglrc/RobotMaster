/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_motor.h"
#include "linewalking.h"
#include "LiquidCrystal_I2C.h"
#include "buzzer.h"
#include "blue.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
LiquidCrystal_I2C lcd;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LineWalking_Tas */
osThreadId_t LineWalking_TasHandle;
const osThreadAttr_t LineWalking_Tas_attributes = {
  .name = "LineWalking_Tas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BUZZER_TASK */
osThreadId_t BUZZER_TASKHandle;
const osThreadAttr_t BUZZER_TASK_attributes = {
  .name = "BUZZER_TASK",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTimer */
osTimerId_t EncoderTimerHandle;
const osTimerAttr_t EncoderTimer_attributes = {
  .name = "EncoderTimer"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void EncoderTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  LineWalking_Task_Init();
  Buzzer_Task_Init();
  BLUE_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of EncoderTimer */
  EncoderTimerHandle = osTimerNew(EncoderTimerCallback, osTimerPeriodic, NULL, &EncoderTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(EncoderTimerHandle, 10);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LineWalking_Tas */
  LineWalking_TasHandle = osThreadNew(StartTask02, NULL, &LineWalking_Tas_attributes);

  /* creation of BUZZER_TASK */
  BUZZER_TASKHandle = osThreadNew(StartTask03, NULL, &BUZZER_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  char buffer[32];
  
  LCD_Init(&lcd, 0x27, 16, 2);
  LCD_Backlight(&lcd);
  LCD_Clear(&lcd);
  //LCD_Print(&lcd, "+:ten l:left r:right");
  LCD_SetCursor(&lcd, 0, 1);
  //LCD_Print(&lcd, "time: all:");
  
  /* Infinite loop */
  for(;;)
  {
    BLUE_Process();
    
    //LCD_SetCursor(&lcd, 0, 0);
		//LCD_Print(&lcd, "                ");
    // ��һ�У�+:ten l:left r:right
    LCD_SetCursor(&lcd, 0, 0);
    sprintf(buffer,"+:%d Le:%d Ri:%d  ", crossroad_count, left_branch_count, right_branch_count);
    LCD_Print(&lcd, buffer);
    
    // �ڶ��У�time:debug_counter all:all
		all = left_branch_count + right_branch_count + crossroad_count;
    LCD_SetCursor(&lcd, 0, 1);
		
    sprintf(buffer,"time:%lu all:%d   ", debug_counter, all);
    LCD_Print(&lcd, buffer);
    
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the LineWalking_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the BUZZER_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* EncoderTimerCallback function */
void EncoderTimerCallback(void *argument)
{
  /* USER CODE BEGIN EncoderTimerCallback */
Encoder_Update_Count();
  /* USER CODE END EncoderTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */