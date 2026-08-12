#ifndef _VISION_CONTROL_H_
#define _VISION_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>
#include "zf_common_headfile.h"

// 视觉控制模块初始化
void vision_control_init(void);

// 解析并处理来自视觉模块的数据
void vision_process_data(void);

// 获取解析出的X轴误差值
float vision_get_error_x(void);

// 获取解析出的Y轴误差值
float vision_get_error_y(void);

// 获取计算出的目标角度
float vision_get_target_angle(void);

// 设置PID参数
void vision_pid_set(float kp, float ki, float kd);

// 设置自稳角度（基础角度）
void vision_set_stable_angle(float angle);

// 设置Y轴系数
void vision_set_y_coefficient(float coefficient);

void vision_set_y_dead_zone(float dead_zone);  // 添加Y轴死区设置函数声明
extern int pwm_value;

/* ==================================================================
 *  钢珠平衡任务新增：钢珠位置接口（视觉协议已定，节点5实现）
 *  - 协议格式：6字节固定帧
 *      偏移0: uint8  0x39      固定帧头
 *      偏移1: uint8  0x01/0x00 1=检测到球, 0=丢球
 *      偏移2: float32 LE       位置值(cm, 小端)
 *  - ball_pos_err: 钢珠距水管中点的位置误差(cm，正负代表方向)
 *  - ball_pos_vel : 钢珠速度估计（MCU 内部差分 + 低通滤波得到）
 * ================================================================== */
extern float ball_pos_err;       /* 钢珠位置误差（像素差，-240~240，0=居中） */
extern float ball_pos_vel;      /* 钢珠速度估计 (像素/s) */
extern uint8_t ball_detected;    /* 球检测状态：1=检测到，0=丢球 */

/* 获取钢珠位置误差 */
float vision_get_ball_error(void);

/* 获取钢珠速度估计（差分） */
float vision_get_ball_velocity(void);

/* 获取球检测状态 */
uint8_t vision_get_ball_detected(void);

/* 设置钢珠位置数据后处理参数
 *   pos_scale     : 视觉原始值到 mm 的缩放系数（如像素→mm）
 *   filter_alpha  : 速度差分低通滤波系数 (0~1，越小越平滑，延迟越大)
 *   sample_period : 视觉定频周期 (s)，用于差分计算速度
 */
void  vision_set_ball_params(float pos_scale, float filter_alpha, float sample_period);

/* 手动注入钢珠位置（用于无视觉时的测试，节点5联调用） */
void  vision_inject_ball_error(float err);

/* ==================================================================
 *  单视觉环 PD 控制（速度控制，输出 PWM 频率）
 *  - 取代串级PID，直接用钢珠位置误差控制电机转速
 *  - 控制律：pwm = Kp * pos_err + Kd * pos_vel
 *  - 不使用 IMU pitch 反馈
 *  - 调用 Set_PWM_Pitch(output) 驱动电机
 * ================================================================== */

/* 初始化单环 PD 参数
 *   kp         : 比例系数
 *   kd         : 微分系数
 *   direction  : 方向符号 +1 或 -1（联调时调整） */
void  vision_beam_init(float kp, float kd, int direction);

/* 设置输出限幅（PWM 频率最大值，Hz） */
void  vision_beam_set_limit(int max_pwm);

/* 设置前馈参数（直接位置→PWM）
 *   kff : 位置→PWM增益 (PWM/pixel) */
void  vision_beam_set_feedforward(float kff);

/* 设置积分增益（消除稳态误差）
 *   ki : 积分增益 (Hz/(pixel·s))，调用时会清空积分累计 */
void  vision_beam_set_ki(float ki);

/* 单环 PD 控制：根据 ball_pos_err/vel 计算 PWM 并驱动电机
 * 应在主循环中调用（与视觉数据更新同步） */
void  vision_beam_run(void);

/* 急停：停止电机输出 */
void  vision_beam_stop(void);

/* 重置角度估算（归零当前位置） */
void  vision_beam_reset_angle(void);

/* 获取当前估算角度 (deg) */
float vision_beam_get_angle(void);

/* 获取上次 PD 输出值（调试用） */
int   vision_beam_get_output(void);

/* 新帧到达标志（供外部模块判断是否打印调试信息） */
extern uint8_t s_new_frame_arrived;
#endif