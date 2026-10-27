#include "Inhpp.hpp"
#include "MotorOfflineDetector.hpp"
#include "timers.h"

/*  =========================== 全局变量的初始化 ===========================  */

/*  ==============================进程的变量===============================  */
// TickType_t SystemTick; // 系统滴答计数

// 裁判系统连接检测（只依赖 RMRefereeSystemData，不修改其模块）
static uint8_t RMRefereeSystem_LastSeq     = 0;
static uint32_t RMRefereeSystem_LastRxTick = 0;

/*  =========================== 函数的声明 ===========================  */
void CheckSystemStatus_TrafficLight();
void SyncYawAngle();

void SystemMonitor_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();

    // 初始化电机掉线检测器
    MotorOfflineDetector_Init();
    Buzzer_Timers_Init();
    HX711.Init();
    HX711.Offset = 8420300; // 固定零点值 (无需每次开机去皮)【TODO：注意这个值是不好的，温漂和蠕变影响比较大，当前有待使用的去皮函数HX711.Tare()】
    vTaskDelay(3000);       // 延时，确保定时器初始化完成
    /* Infinite loop */
    for (;;) {

        uint32_t current_tick = HAL_GetTick();
        
        SyncYawAngle(); // 同步Yaw轴角度

        static uint32_t last_traffic_time = 0;
        if (current_tick - last_traffic_time >= 300) {
            CheckSystemStatus_TrafficLight();
            last_traffic_time = current_tick;
        }

        MotorOfflineDetector_Update(); // 更新电机掉线检测器状态
        VisionUartReceive.CheckConnection(); // 检查视觉连接状态
        Handle_Buzzer();               // 处理蜂鸣器播放

        // HX711 非阻塞持续轮询（HX711数据率为10Hz，每100ms更新一次）
        static uint32_t last_hx711_time = 0;
        if (current_tick - last_hx711_time >= 100) {
            HX711.ReadFiltered();
            last_hx711_time = current_tick;
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1)); // 每1毫秒执行一次
    }
    /* USER CODE END LED_Flashing */
}

/* Private application code --------------------------------------------------*/
void CheckSystemStatus_TrafficLight()
{
    if (remote.get_s1() == SW_DOWN && remote.get_s2() == SW_DOWN)
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14); // 绿灯闪烁
    else if (remote.isConnected())
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // 绿灯常亮
    else
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); // 绿灯关闭

    // 裁判系统连接状态指示：1s 内 seq 有变化则认为连接正常
    uint32_t now = HAL_GetTick();
    if (RMRefereeSystemData.seq != RMRefereeSystem_LastSeq) {
        RMRefereeSystem_LastSeq    = RMRefereeSystemData.seq;
        RMRefereeSystem_LastRxTick = now;
    }

    if ((RMRefereeSystem_LastRxTick != 0U) && (now - RMRefereeSystem_LastRxTick <= 1000U)) {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // 红灯常亮
    } else {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11); // 红灯闪烁
    }
}

void SyncYawAngle()
{
    static bool was_moving = false;

    if ((remote.get_s1() == SW_MID) && remote.isConnected() && std::abs(remote.get_right_stick_x()) > 20) {
        // yaw轴的值加等于摇杆的值
        DartBot.AddTarget(&DartBot.YawAngle, remote.get_right_stick_x() * 0.01, YAW_MIN, YAW_MAX);
        was_moving = true;
    } else {
        
        if (was_moving) {
            DartBot.YawAngle = SpeedPID_AngleSensorM3508.Current;
            was_moving     = false;
        }
    }
}


uint32_t TIM_GetCounterFreq(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock;
    // 高级定时器是APB2
    if (htim->Instance == TIM1) {
        timer_clock = HAL_RCC_GetPCLK2Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK2Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    } else {
        // 其他定时器是APB1
        timer_clock = HAL_RCC_GetPCLK1Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK1Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    }

    uint32_t prescaler = htim->Instance->PSC;
    return timer_clock / (prescaler + 1);
}