#include "InHpp.hpp"
#include "../../user/core/HAL/CAN/can_hal.hpp"

/* 里程计的初始化 --------------------------------------------------*/
Mileage_Temp Mileage = 
{
    {
      false //CircleCount
    , true //Temp
    }//YawM6020
    ,
    {
      false //CircleCount
    , true //Temp
    }//PitchM3508
     ,
    {
      false //CircleCount
    , true //Temp
    }//HI14Yaw  
     ,
    {
      false //CircleCount
    , true //Temp
    }//DialM3508
};
/* 里程计的初始化 --------------------------------------------------*/

/* 电机速度环的初始化 --------------------------------------------------*/
#define Temp_SpeedKP 10
#define Temp_SpeedKPCoefficient 0
#define Temp_SpeedKPGainMini 150
#define Temp_SpeedKPGainMax 300
#define Temp_SpeedKI 0.7
#define Temp_SpeedKILimit 3000
#define Temp_SpeedKITime 1
#define Temp_SpeedFinalLimit 16000

// Yaw 外环角度PID（独立于 Motors[] 数组）
AngleSensorPID_t SpeedPID_AngleSensorM3508(250.0f, 0.0f, 0.1f, 16000.0f, 0.0f, 0.0f);
/* 电机速度环的初始化 --------------------------------------------------*/

// 电机实例定义 (使用 user/core 库)
namespace BSP::Motor::Dji {
    // M3508 电机: 0x201(保留), 0x203(右上膛), 0x204(左上膛), 0x207(拉簧)
    const uint8_t recv_idxs_3508[4] = {1, 3, 4, 7};
    GM3508<4> motors_3508(0x200, recv_idxs_3508, 0x200);
    
    // M2006 电机: 0x202(左升降), 0x206(右升降)
    const uint8_t recv_idxs_2006[2] = {2, 6};
    GM2006<2> motors_2006(0x200, recv_idxs_2006, 0x200);
    
    // M6020 电机: 0x205(Yaw轴)
    const uint8_t recv_idxs_6020[1] = {5};
    GM6020<1> motors_6020(0x200, recv_idxs_6020, 0x1FF);
}

void PID_Init()
{
	// 飞镖系统 PID 初始化 (当前使用统一的 Motors[] 数组管理速度环)
}