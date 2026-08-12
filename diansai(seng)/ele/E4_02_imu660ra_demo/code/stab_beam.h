#ifndef _STAB_BEAM_H_
#define _STAB_BEAM_H_

/* ==================================================================
 *  钢珠平衡三环串级控制器
 *  - 位置环（P）：钢珠位置误差 → 目标球速
 *  - 速度环（PI）：目标球速 - 实际球速 → 目标水管倾角
 *  - 倾角环（PD）：目标倾角 - 编码器倾角 → PWM频率（5ms中断, 200Hz）
 * ================================================================== */

#include <stdint.h>
#include <stdbool.h>
#include "zf_common_headfile.h"

/* ==================== 位置环（P） ====================*/
/* 输入：钢珠位置误差 x (像素差)
 * 输出：目标球速 v_ref (px/s)
 * 控制律：v_ref = -Kp_pos * x
 */
void  stab_beam_outer_init(float kp);
void  stab_beam_outer_set_pos(float pos_err);
float stab_beam_outer_get_target_vel(void);
void  stab_beam_outer_reset(void);
void  stab_beam_outer_set_limit(float max_vel);

extern float stab_beam_outer_target_vel;  /* 位置环输出：目标球速 (px/s) */
extern float stab_beam_outer_pos_err;     /* 当前钢珠位置误差 (px) */

/* ==================== 速度环（PI） ====================*/
/* 输入：目标球速 v_ref (来自位置环)，实际球速 v (来自视觉差分)
 * 输出：目标水管倾角 θ_target (deg)
 * 控制律：θ = Kp_vel * (v_ref - v) + Ki_vel * ∫(v_ref - v)dt
 *         Ki 在速度环上积分速度误差，球受摩擦卡顿时速度误差单向偏置，
 *         积分可累积消除稳态偏差（解决原位置环 Ki 被振荡抵消的问题）
 */
void  stab_beam_velocity_init(float kp, float ki);
void  stab_beam_velocity_set_vel(float vel_ref, float vel_actual);
float stab_beam_velocity_get_target(void);
float stab_beam_velocity_get_integral(void);   /* 速度环积分项 (px)，VOFA监控用 */
void  stab_beam_velocity_reset(void);
void  stab_beam_velocity_set_limit(float max_angle_deg);
void  stab_beam_velocity_set_dt(float dt_sec);
void  stab_beam_velocity_set_integral_sep(float threshold);  /* 积分分离阈值 (px/s) */
void  stab_beam_velocity_set_integral_limit(float limit);   /* 积分限幅 (px) */

extern float stab_beam_velocity_target_pitch;  /* 速度环输出：目标倾角 (deg) */

/* ==================== 内环（编码器倾角环 PD → PWM 速度输出） ==================== */
/* 输入：目标倾角 θ_target (来自外环)，当前倾角 θ_cur (来自编码器)
 * 输出：PWM 频率 (Hz)，直接驱动步进电机
 * 调用：5ms 中断（200Hz，与 imu_5ms_task 同步）
 * 控制律：pwm = Kp_inner * (θ_target - θ_cur) - Kd_inner * ω_cur
 *         Kp_inner 单位 PWM/deg，Kd_inner 单位 PWM/(deg/s)
 */
void  stab_beam_inner_init(float kp, float kd);
void  stab_beam_inner_set_target(float target_pitch_deg);
void  stab_beam_inner_run_200Hz(void);   /* 在 5ms 中断中调用 */
void  stab_beam_inner_reset(void);       /* 重置内部状态 */

extern float stab_beam_inner_target;        /* 内环目标倾角 (deg) */
extern float stab_beam_inner_current;       /* 内环当前倾角 (来自编码器, deg) */
extern float stab_beam_inner_motor_target;  /* 内环输出：PWM频率 (Hz) */

/* ==================== 通用辅助 ==================== */
/* 设置内环 PWM 限幅 (Hz) */
void  stab_beam_inner_set_limit(int max_pwm);
/* 设置内环位置容差 (deg)，小于此值停止输出 */
void  stab_beam_inner_set_tolerance(float tol_deg);
/* 设置内环最小 PWM 频率 (Hz)，低于此值步进电机无法启动，强制提升 */
void  stab_beam_inner_set_min_pwm(float min_hz);

#endif
