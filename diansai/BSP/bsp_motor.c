/* bsp_motor.c */
#include "bsp_motor.h"

uint8_t motor_enable = 0;

static int16_t Motor_Ignore_Dead_Zone(int16_t pulse)
{
    // 死区补偿：所有输入都加上死区偏移
    if (pulse > 0) return pulse + MOTOR_IGNORE_PULSE;
    if (pulse < 0) return pulse - MOTOR_IGNORE_PULSE;
    return 0;
}

void Motor_PWM_Init(void)
{
    // 启动TIM8所有PWM通道
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    
    // 启动TIM1所有PWM通道（注意：这里不使用互补输出）
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    // 使能主输出（MOE位）- 高级定时器必须设置才能输出PWM
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim8);
    
    motor_enable = MOTOR_ENABLE_A | MOTOR_ENABLE_B | MOTOR_ENABLE_C | MOTOR_ENABLE_D;
    
    // 测试：直接设置PWM寄存器
    
    
}

void Motor_Stop(uint8_t brake)
{
    uint16_t brake_val = (brake != 0) ? MOTOR_MAX_PULSE : 0;
    
    PWM_M1_A = brake_val;
    PWM_M1_B = brake_val;
    PWM_M2_A = brake_val;
    PWM_M2_B = brake_val;
    PWM_M3_A = brake_val;
    PWM_M3_B = brake_val;
    PWM_M4_A = brake_val;
    PWM_M4_B = brake_val;
}

void Motor_Set_Pwm(uint8_t id, int16_t speed)
{
    int16_t pulse = Motor_Ignore_Dead_Zone(speed);
    
    if (pulse > MOTOR_MAX_PULSE) pulse = MOTOR_MAX_PULSE;
    if (pulse < -MOTOR_MAX_PULSE) pulse = -MOTOR_MAX_PULSE;

    switch (id)
    {
    case MOTOR_ID_M1:
        pulse = -pulse;
        if (pulse >= 0) { PWM_M1_A = pulse; PWM_M1_B = 0; }
        else { PWM_M1_A = 0; PWM_M1_B = -pulse; }
        break;
    case MOTOR_ID_M2:
        pulse = -pulse;
        if (pulse >= 0) { PWM_M2_A = pulse; PWM_M2_B = 0; }
        else { PWM_M2_A = 0; PWM_M2_B = -pulse; }
        break;
    case MOTOR_ID_M3:
        if (pulse >= 0) { PWM_M3_A = pulse; PWM_M3_B = 0; }
        else { PWM_M3_A = 0; PWM_M3_B = -pulse; }
        break;
    case MOTOR_ID_M4:
        if (pulse >= 0) { PWM_M4_A = pulse; PWM_M4_B = 0; }
        else { PWM_M4_A = 0; PWM_M4_B = -pulse; }
        break;
    default:
        break;
    }
}

uint8_t Motor_Get_Enable_State(uint8_t id)
{
    return motor_enable & (1 << id);
}