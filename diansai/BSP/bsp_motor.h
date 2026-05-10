/* bsp_motor.h */
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include "main.h"
#include "tim.h"

/* 电机ID定义 */
typedef enum {
    MOTOR_ID_M1 = 0,
    MOTOR_ID_M2,
    MOTOR_ID_M3,
    MOTOR_ID_M4,
    MAX_MOTOR
} Motor_ID;

/* 电机使能位 */
#define MOTOR_ENABLE_A      (0x01)
#define MOTOR_ENABLE_B      (0x02)
#define MOTOR_ENABLE_C      (0x04)
#define MOTOR_ENABLE_D      (0x08)

/* 电机参数 */
#define MOTOR_IGNORE_PULSE  (10)
#define MOTOR_MAX_PULSE     (99)
#define MOTOR_FREQ_DIVIDE   (0)

/* PWM通道宏定义 */
#define PWM_M1_A  TIM8->CCR1  // PC6
#define PWM_M1_B  TIM8->CCR2  // PC7
#define PWM_M2_A  TIM8->CCR3  // PC8
#define PWM_M2_B  TIM8->CCR4  // PC9
#define PWM_M3_A  TIM1->CCR4  // PA11
#define PWM_M3_B  TIM1->CCR1  // PA8
#define PWM_M4_A  TIM1->CCR2  // PA9
#define PWM_M4_B  TIM1->CCR3  // PA10

extern uint8_t motor_enable;

void Motor_PWM_Init(void);
void Motor_Set_Pwm(uint8_t id, int16_t speed);
void Motor_Stop(uint8_t brake);
uint8_t Motor_Get_Enable_State(uint8_t id);

#endif