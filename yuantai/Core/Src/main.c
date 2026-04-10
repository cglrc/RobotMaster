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
//BSP::Motor::DjiMotor<2> gimbal_motor;
// GM6020 反馈ID = 0x204 + 拨码值，拨码1→0x205，拨码2→0x206
BSP::Motor::Dji::GM6020<2> gimbal_motor{0x204, {1, 2}, 0x1FF};
BSP::IMU::HI12_float imu;
ALG::PID::PID pos_pid[2]={
	{43.0f, 0.03f, 0.005f, 5000.0f, 1000.0f, 100.0f},
  {42.0f, 0.015f, 0.0f, 5000.0f, 1000.0f, 100.0f}
};//5.0,0.01,0.5
ALG::PID::PID speed_pid[2] = {
    {4.0f, 0.016f, 0.012f, 10000.0f, 5000.0f, 500.0f},
    {12.0f, 0.015f, 0.0f, 10000.0f, 5000.0f, 500.0f},
    
};//10.0,0.1,0.5
void VisionSend(void);
CAN_RxHeaderTypeDef rxHeader0, rxHeader1;
float target_angle[2] = {0, 0};
float target_speed[2]={0,0};
int32_t pitch_raw=0,yaw_raw=0;
float yaw_increment=0;
float pitch_increment=0;
int16_t current[2]={0,0};
float current_angle[2]={0,0};
uint8_t imu_rx_buffer[82];
uint8_t txDataBuffer[8], rxDataBuffer0[8], rxDataBuffer1[8];
uint8_t send_str2[sizeof(float) * 8];
extern RC_Ctl_t data;
extern uint8_t receivedata[18];
extern uint32_t rc_last_recv_time;
extern uint8_t rc_disconnect_flag;
float w, x, y, z;

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

/** 两路电机均在线后采样一次编码器角，作为相对 0°（上电姿态） */
static float gimbal_angle_zero[2];
static uint8_t gimbal_angle_zero_latched;

/** 视觉接收缓冲：0x39 0x39 + pitch(int32大端) + yaw(int32大端) + 10字节其他 = 18字节 */
 uint8_t vision_rx_buf[18];
/** 视觉模式标志：1=视觉控制，0=遥控器控制 */
 uint8_t vision_mode = 0;
/** 帧头 0x39 0x39 + w,x,y,z（float，小端，与 STM32 一致），经 USART6 DMA 发出 */
 uint8_t vision_tx_buf[2u + 4u * sizeof(float)];
/** 1ms 节拍计数，每 VISION_SEND_PERIOD_MS 发一帧视觉串口 */
#define VISION_SEND_PERIOD_MS 40u
static uint8_t vision_send_1ms_cnt;
//void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
//{
//  
//    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

//    // 将6个浮点数据写入缓冲区（小端模式）
//    *((float*)&send_str2[sendSize * 0]) = x1;
//    *((float*)&send_str2[sendSize * 1]) = x2;
//    *((float*)&send_str2[sendSize * 2]) = x3;
//    *((float*)&send_str2[sendSize * 3]) = x4;
//    *((float*)&send_str2[sendSize * 4]) = x5;
//    *((float*)&send_str2[sendSize * 5]) = x6;

//    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
//    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

//    // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
//    HAL_UART_Transmit_DMA(&huart6, send_str2, sizeof(send_str2));
//}

void VisionSend(void)
{
    w = imu.GetQuaternion(0);
    x = imu.GetQuaternion(1);
    y = imu.GetQuaternion(2);
    z = imu.GetQuaternion(3);
    vision_tx_buf[0] = 0x39u;
    vision_tx_buf[1] = 0x39u;

    // 大端序写入：取 float 的字节逆序
    uint32_t tmp;
    memcpy(&tmp, &w, 4); tmp = __builtin_bswap32(tmp); memcpy(&vision_tx_buf[2],  &tmp, 4);
    memcpy(&tmp, &x, 4); tmp = __builtin_bswap32(tmp); memcpy(&vision_tx_buf[6],  &tmp, 4);
    memcpy(&tmp, &y, 4); tmp = __builtin_bswap32(tmp); memcpy(&vision_tx_buf[10], &tmp, 4);
    memcpy(&tmp, &z, 4); tmp = __builtin_bswap32(tmp); memcpy(&vision_tx_buf[14], &tmp, 4);

    /* 只检查 TX 是否忙，避免 RX 状态干扰 */
    HAL_UART_Transmit_DMA(&huart6, vision_tx_buf, (uint16_t)sizeof(vision_tx_buf));
}

// 接收缓冲区（HI12 数据帧较长）

// UART 接收回调
	//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
      if (huart == &huart3)
    {
//        
         rc_last_recv_time = HAL_GetTick();
        rc_disconnect_flag = 0;
        remoteDataProcess(receivedata);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t *)receivedata, 18);
	}	
    else if (huart == &huart1)  // 假设 IMU 接在 UART3
    {
        // 更新 IMU 数据
        imu.DataUpdate(imu_rx_buffer);

        // 继续接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_rx_buffer, 82);
    }else if(huart == &huart6)
    {
        // 仅视觉模式下处理数据
        if (vision_mode && Size == 18 && vision_rx_buf[0] == 0x39 && vision_rx_buf[1] == 0x39)
        {
             pitch_raw = (int32_t)((uint32_t)vision_rx_buf[2] << 24 |
                                          (uint32_t)vision_rx_buf[3] << 16 |
                                          (uint32_t)vision_rx_buf[4] << 8  |
                                          (uint32_t)vision_rx_buf[5]);
             yaw_raw   = (int32_t)((uint32_t)vision_rx_buf[6] << 24 |
                                          (uint32_t)vision_rx_buf[7] << 16 |
                                          (uint32_t)vision_rx_buf[8] << 8  |
                                          (uint32_t)vision_rx_buf[9]);
          target_angle[0] = yaw_raw* 0.01f;
            target_angle[1] =  pitch_raw* 0.01f;  
//					target_angle[0] = current_angle[0] + yaw_raw* 0.01f;
//            target_angle[1] = current_angle[1] + pitch_raw* 0.01f;
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, vision_rx_buf, sizeof(vision_rx_buf));
    }
	}

// UART 接收回调

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
			gimbal_motor.refreshConnectionStatus();

			if (++vision_send_1ms_cnt >= VISION_SEND_PERIOD_MS)
			{
				vision_send_1ms_cnt = 0;
				VisionSend();
			}
			
			// s1==2（拨下）进视觉模式，其他为遥控器模式
        vision_mode = (data.rc.s1 == 2) ? 1 : 0;
        uint8_t rc_connected = (HAL_GetTick() - rc_last_recv_time <= 500);

        
       

        if (!vision_mode )
        {		
            // 遥控器模式：摇杆控制增量
            float rc_yaw = data.rc.ch0 - 1024.0f;
            if (abs((int)rc_yaw) > 15) yaw_increment += rc_yaw / 660.0f;
            if (yaw_increment >  45.0f) yaw_increment =  45.0f;
            if (yaw_increment < -45.0f) yaw_increment = -45.0f;
            target_angle[0] = yaw_increment;

            float rc_pitch = data.rc.ch1 - 1024.0f;
            if (abs((int)rc_pitch) > 15) pitch_increment += rc_pitch / 660.0f;
            if (pitch_increment >  45.0f) pitch_increment =  45.0f;
            if (pitch_increment < -45.0f) pitch_increment = -45.0f;
            target_angle[1] = pitch_increment;
        }
        // 视觉模式下 target_angle 由 UART6 回调写入，此处不覆盖
			
		 

    if (!gimbal_angle_zero_latched && gimbal_motor.getOfflineStatus() == 0)
    {
        for (int j = 0; j < 2; j++)
        {
            float r = gimbal_motor.getAngleDeg(j + 1);
            if (r > 180.0f) r -= 360.0f;
            gimbal_angle_zero[j] = r;
        }
        gimbal_angle_zero_latched = 1;
    }

    /* 未完成零点锁存前不发电流，避免用「绝对编码角」对「目标 0」做 PID 导致猛扭 */
    if (!gimbal_angle_zero_latched)
    {
        gimbal_motor.setCAN(0, 1);
        gimbal_motor.setCAN(0, 2);
        gimbal_motor.sendCAN();
        return;
    }

    for (int i = 0; i < 2; i++)
    {
        float raw = gimbal_motor.getAngleDeg(i + 1);
        if (raw > 180.0f) raw -= 360.0f;
        if (gimbal_angle_zero_latched)
            current_angle[i] = wrap_angle(raw, gimbal_angle_zero[i]);
        else
            current_angle[i] = raw;
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
		//vofa_send(current_angle[0],current_angle[1],target_angle[0],target_angle[1],0,0);
	
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
HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *)receivedata, sizeof(receivedata));
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *)imu_rx_buffer,82);
HAL_UARTEx_ReceiveToIdle_DMA(&huart6, vision_rx_buf, sizeof(vision_rx_buf));
//HAL_UART_Receive_DMA(&huart1, imu_rx_buffer, 64);  // 假设 IMU 数据长度为 64 字节
rc_last_recv_time = HAL_GetTick();
 rc_disconnect_flag = 0;
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
