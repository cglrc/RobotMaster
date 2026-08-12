#ifndef __InHpp_Hpp
#define __InHpp_Hpp
/* C++代码的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <arm_math.h>
#include <math.h>
#include <algorithm>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"

#include "InH.h"
#include "MathUtils.hpp"
#include "PID.hpp"
#include "Filter.hpp"
#include "Init.hpp"

#include "CanCallBack.hpp"
#include "RmMotor.hpp"

#include "UartCallBack.hpp"
#include "DT7.hpp"
#include "../../user/core/BSP/RemoteControl/DT7.hpp"
#include "../../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../../user/core/HAL/CAN/can_hal.hpp"
#include "VisionCommunication.hpp"
#include "VofaCallBack.hpp"

#include "Dart.hpp"
#include "auto.hpp"
#include "MotorControl.hpp"
#include "UartSend.hpp"
#include "RMRefereeSystem.hpp"
#include "StringUtil.hpp"
#include "SystemMonitor.hpp"
#include "Buzzer.hpp"
#include "MotorOfflineDetector.hpp"
#include "ServoControl.hpp"
#include "HX711.hpp"

// 全局遥控器对象 (使用 user/core 库)
extern BSP::REMOTE_CONTROL::RemoteController remote;
extern unsigned char DT7_ReceiveArr[18];

// 全局电机对象 (使用 user/core 库)
namespace BSP::Motor::Dji {
    extern GM3508<4> motors_3508;   // 0x201, 0x203, 0x204, 0x207
    extern GM2006<2> motors_2006;   // 0x202, 0x206
    extern GM6020<1> motors_6020;   // 0x205
}

/* USER CODE END Includes */

/* C++代码的声明 ----------------------------------------------------------*/
#endif