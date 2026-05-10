#include "bsp_encoder.h"

/* 全局编码器累计计数值 */
int g_Encoder_M1_Now = 0;
int g_Encoder_M2_Now = 0;
int g_Encoder_M3_Now = 0;
int g_Encoder_M4_Now = 0;

/**
 * @brief 编码器初始化（在main函数中调用）
 */
void Encoder_Init(void)
{
    // 初始化所有编码器定时器
    // CubeMX已经生成了MX_TIM2_Init()等函数
    
    // 设置初始计数器值
    __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CNT_INIT);
    __HAL_TIM_SET_COUNTER(&htim3, ENCODER_CNT_INIT);
    __HAL_TIM_SET_COUNTER(&htim4, ENCODER_CNT_INIT);
    __HAL_TIM_SET_COUNTER(&htim5, ENCODER_CNT_INIT);
    
    // 启动编码器模式
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

/**
 * @brief 读取编码器计数（每10ms调用一次）
 * @param Motor_id: MOTOR_ID_M1~M4
 * @return 10ms内的脉冲变化量
 */
int16_t Encoder_Read_CNT(uint8_t Motor_id)
{
    int16_t Encoder_TIM = 0;
    uint16_t cnt;
    
    switch(Motor_id)
    {
    case MOTOR_ID_M1:
        cnt = __HAL_TIM_GET_COUNTER(&htim2);
        Encoder_TIM = ENCODER_CNT_INIT - (int16_t)cnt;
        __HAL_TIM_SET_COUNTER(&htim2, ENCODER_CNT_INIT);
        break;
        
    case MOTOR_ID_M2:
        cnt = __HAL_TIM_GET_COUNTER(&htim4);
        Encoder_TIM = ENCODER_CNT_INIT - (int16_t)cnt;
        __HAL_TIM_SET_COUNTER(&htim4, ENCODER_CNT_INIT);
        break;
        
    case MOTOR_ID_M3:
        cnt = __HAL_TIM_GET_COUNTER(&htim5);
        Encoder_TIM = ENCODER_CNT_INIT - (int16_t)cnt;
        __HAL_TIM_SET_COUNTER(&htim5, ENCODER_CNT_INIT);
        break;
        
    case MOTOR_ID_M4:
        cnt = __HAL_TIM_GET_COUNTER(&htim3);
        Encoder_TIM = ENCODER_CNT_INIT - (int16_t)cnt;
        __HAL_TIM_SET_COUNTER(&htim3, ENCODER_CNT_INIT);
        break;
        
    default:
        break;
    }
    return Encoder_TIM;
}

/**
 * @brief 获取累计编码器计数
 */
int Encoder_Get_Count_Now(uint8_t Motor_id)
{
    if (Motor_id == MOTOR_ID_M1) return g_Encoder_M1_Now;
    if (Motor_id == MOTOR_ID_M2) return g_Encoder_M2_Now;
    if (Motor_id == MOTOR_ID_M3) return g_Encoder_M3_Now;
    if (Motor_id == MOTOR_ID_M4) return g_Encoder_M4_Now;
    return 0;
}

/**
 * @brief 获取所有编码器累计计数
 */
void Encoder_Get_ALL(int* Encoder_all)
{
    Encoder_all[0] = g_Encoder_M1_Now;
    Encoder_all[1] = g_Encoder_M2_Now;
    Encoder_all[2] = g_Encoder_M3_Now;
    Encoder_all[3] = g_Encoder_M4_Now;
}

/**
 * @brief 更新编码器累计计数（每10ms调用一次）
 */
void Encoder_Update_Count(void)
{
    // 注意：正负号根据实际安装方向调整
    g_Encoder_M1_Now -= Encoder_Read_CNT(MOTOR_ID_M1);  // M1反向
    g_Encoder_M2_Now += Encoder_Read_CNT(MOTOR_ID_M2);  // M2正向
    g_Encoder_M3_Now += Encoder_Read_CNT(MOTOR_ID_M3);  // M3正向
    g_Encoder_M4_Now -= Encoder_Read_CNT(MOTOR_ID_M4);  // M4反向
}
