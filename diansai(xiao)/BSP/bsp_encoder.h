#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "main.h"
#include "tim.h"
#include "bsp_motor.h"
/* 编码器定时器周期 */
#define ENCODER_TIM_PERIOD  65535
#define ENCODER_CNT_INIT    0x7fff  // 初始值32767

/* 电机编码器ID */
#define ENCODER_M1  0  // 右前轮 TIM2
#define ENCODER_M2  1  // 左前轮 TIM4
#define ENCODER_M3  2  // 右后轮 TIM5
#define ENCODER_M4  3  // 左后轮 TIM3

/* 函数声明 */
void Encoder_Init(void);
void Encoder_Update_Count(void);
int Encoder_Get_Count_Now(uint8_t Motor_id);
void Encoder_Get_ALL(int* Encoder_all);
int16_t Encoder_Read_CNT(uint8_t Motor_id);

extern int g_Encoder_M1_Now;
extern int g_Encoder_M2_Now;
extern int g_Encoder_M3_Now;
extern int g_Encoder_M4_Now;

#endif
