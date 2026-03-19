/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP/Motor/Dji/DjiMotor.hpp"
#include "Alg/PID/pid.hpp"
#include "../core/BSP/RemoteControl/yaokongqi.h"
#include "math.h"
#include "/BSP/IMU/HI12_imu.hpp"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//BSP::Motor::DjiMotor::GM6020<2> gimbal_motor;
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
//BSP::Motor::DjiMotor<2> gimbal_motor;
// GM6020 反馈ID = 0x204 + 拨码值，拨码1→0x205，拨码2→0x206
BSP::Motor::Dji::GM6020<2> gimbal_motor{0x204, {1, 2}, 0x1FF};
BSP::IMU::HI12_float imu;
ALG::PID::PID pos_pid[2]={
	{5.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f},
  {5.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f}
};//5.0,0.01,0.5
ALG::PID::PID speed_pid[2] = {
    {5.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},
    {5.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},
    
};//10.0,0.1,0.5
CAN_RxHeaderTypeDef rxHeader0, rxHeader1;
float yaw=0;
float target_angle[2] = {0, 0};
uint8_t send_str2[sizeof(float) * 8];
float target_speed[2]={0,0};
float yaw_increment=0;
float pitch_increment=0;
int16_t current[2]={0,0};
float current_angle[2]={0,0};
extern uint8_t receivedata[50];
extern RC_Ctl_t data;
uint8_t imu_rx_buffer[64];
uint8_t txDataBuffer[8], rxDataBuffer0[8], rxDataBuffer1[8];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 角度差过零归一化，将差值限制在 [-180, 180]
static inline float wrap_angle(float target, float current)
{
    float delta = target - current;
    delta -= 360.0f * floorf((delta + 180.0f) / 360.0f);
    return delta;
}

// 接收缓冲区（HI12 数据帧较长）


// UART 接收回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)  // 假设 IMU 接在 UART3
    {
        // 更新 IMU 数据
        imu.DataUpdate(imu_rx_buffer);

        // 继续接收
        HAL_UART_Receive_DMA(&huart6, imu_rx_buffer, sizeof(imu_rx_buffer));
    }
}
// CAN1 ���ջص�����
void CAN1_RxCallback(HAL::CAN::Frame& frame)
{
    // frame.id ���� CAN ֡�� ID
    // 3508 ����� ID ��Χ�� 0x201-0x208

    
    // GM6020 拨码1→0x205，拨码2→0x206
    if (frame.id >= 0x205 && frame.id <= 0x206)
    {
        // ������̨��������ݣ�6020��
        gimbal_motor.Parse(frame);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame frame;
    if (HAL::CAN::get_can_bus_instance().get_can1().receive(frame))
    {
        CAN1_RxCallback(frame);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  // 1ms
{
    if (htim == &htim3) {
			//yaw  
    float rc_yaw =data.rc.ch0-1024.0f;
		if (abs((int) rc_yaw)> 15)   //abs为取绝对值
     {      
            yaw_increment+=rc_yaw/660.0f;
     }	
			if(yaw_increment>45.0f){
					yaw_increment=45.0f;
				}else if(yaw_increment<-45.0f){
         yaw_increment=-45.0f;
				} 
		target_angle[0]=yaw_increment	;
		
		//pitch		
		float rc_pitch =data.rc.ch1-1024.0f;
			if (abs((int) rc_pitch)> 15)   //abs为取绝对值
            {       
							pitch_increment+=rc_pitch/660.0f;
              
            }
        if(pitch_increment>45.0f){
					pitch_increment=45.0f;
				}else if(pitch_increment<-45.0f){
         pitch_increment=-45.0f;
				} 
		target_angle[1]=pitch_increment	;
			
		 //yaw = imu.GetAngle(2);				
    for (int i = 0; i < 2; i++)
    {
        // ��ȡ��ǰ�ٶ�
			  current_angle[i] = gimbal_motor.getAngleDeg(i+1);
        float current_speed = gimbal_motor.getVelocityRpm(i + 1);

        // PID ����
			  
			  float angle_error = wrap_angle(target_angle[i], current_angle[i]);
			  target_speed[i] = pos_pid[i].UpDate(current_angle[i] + angle_error, current_angle[i]);
        float output = speed_pid[i].UpDate(target_speed[i], current_speed);

        // �޷�
        if (output > 10000) output = 10000;
        if (output < -10000) output = -10000;

        current[i] = (int16_t)output;
    }

    // ���Ϳ���ָ��
    gimbal_motor.setCAN(current[0],1);
		gimbal_motor.setCAN(current[1],2);
		gimbal_motor.sendCAN();
		vofa_send(current_angle[0],target_angle[0],current_angle[1],target_angle[1],0,0);
		}
	}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
HAL::CAN::get_can_bus_instance(); // 触发 CAN bus 初始化：HAL_CAN_Start + 激活中断通知
HAL_TIM_Base_Start_IT(&htim3);
HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)receivedata, 18);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

/* USER CODE BEGIN 4 */
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
  { const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
    HAL_UART_Transmit_DMA(&huart1, send_str2, sizeof(send_str2));
}
/* USER CODE END 4 */

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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
