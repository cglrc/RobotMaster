/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   MS42CG 磁编码器 A/B 增量信号接口（硬件 QEI）
  ******************************************************************************
  * 硬件连接：
  *   A = PA1 / TIMG8_CCP0 (IOMUX_PINCM2 PF=7)
  *   B = PA0 / TIMG8_CCP1 (IOMUX_PINCM1 PF=6)
  *   使用 TIMG8 硬件 QEI 模式，4 倍频后 4000 counts/圈
  *
  * 使用方法：
  *   1. 调用 Encoder_Init() 初始化
  *   2. 在 5ms 定时中断中调用 Encoder_Tick() 更新计数
  *   3. 调用 Encoder_GetAngle() 获取当前角度 (deg)
  *   4. 调用 Encoder_SetZero() 标定零点
  ******************************************************************************
  */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* MS42CG: 1000 CPR × 4 倍频 = 4000 counts/圈 */
#define ENCODER_COUNTS_PER_REV  4000U

/* 初始化 QEI 硬件，设置 PA0/PA1 为 QEI 输入，清零计数 */
void Encoder_Init(void);

/* 在固定周期(5ms)调用，读取硬件计数器并扩展为 32 位多圈计数 */
void Encoder_Tick(void);

/* 获取当前累计计数（已减去零点偏移） */
int32_t Encoder_GetCount(void);

/* 获取当前角度 (deg)，基于 4000 counts/圈 */
float Encoder_GetAngle(void);

/* 计算角速度 (deg/s)，基于两次调用间的计数差 */
float Encoder_CalcSpeedDps(void);

/* 将当前位置设为零点 */
void Encoder_SetZero(void);

#endif /* ENCODER_H */
