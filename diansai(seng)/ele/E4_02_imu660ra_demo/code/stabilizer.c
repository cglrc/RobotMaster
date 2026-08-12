#include "stabilizer.h"
#include "attitude_solution.h"   /* 提供 T_N，单位 deg */
#include "motor.h"               /* 提供 Set_PWM() */

/* ---------- PID 参数 ---------- */
static float  kp_ = 0.0f;
static float  ki_ = 0.0f;
static float  kd_ = 0.0f;

/* ---------- 运行期变量 ---------- */
float stab_yaw_target_deg   = 0.0f;   /* 目标 */
float stab_yaw_current_deg  = 0.0f;   /* 当前 */
float stab_yaw_error_deg    = 0.0f;   /* 误差 */
float stab_yaw_pid_out      = 0.0f;   /* PID 输出 */

static float  error_sum_    = 0.0f;   /* 积分 */
static float  error_last_   = 0.0f;   /* 上次误差 */

/* 抗积分饱和：输出限幅 */
#define PID_OUT_MAX   6400   /* 与 Set_PWM 频率值范围匹配，自行调整 */
#define PID_OUT_MIN  (-PID_OUT_MAX)
#define I_WINDUP_MAX  2000.0f

/* ---------- 初始化 ---------- */
void stab_yaw_init(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    stab_yaw_target_deg  = 0.0f;
    stab_yaw_current_deg = 0.0f;
    stab_yaw_error_deg   = 0.0f;
    stab_yaw_pid_out     = 0.0f;

    error_sum_  = 0.0f;
    error_last_ = 0.0f;
}

/* 外部可随时改目标 */
void stab_yaw_set_target(float target_deg)
{
    stab_yaw_target_deg = target_deg;
}

/* ---------- 2 ms 周期调用 ---------- */
void stab_yaw_run_200Hz(void)
{
    /* 1. 获取当前角度（T_M 就是解算后的角度，单位 deg） */
    stab_yaw_current_deg = T_M;

    /* 2. 计算误差 */
    float error = stab_yaw_target_deg - stab_yaw_current_deg;

    /* 3. 积分项 */
    error_sum_ += error;
    /* 抗积分饱和 */
    if (error_sum_ >  I_WINDUP_MAX) error_sum_ =  I_WINDUP_MAX;
    if (error_sum_ < -I_WINDUP_MAX) error_sum_ = -I_WINDUP_MAX;

    /* 4. 微分项 */
    float error_diff = error - error_last_;
    error_last_ = error;

    /* 5. PID 计算 */
    float out = kp_ * error
              + ki_ * error_sum_
              + kd_ * error_diff;

    /* 6. 输出限幅 */
    if (out > PID_OUT_MAX) out = PID_OUT_MAX;
    if (out < PID_OUT_MIN) out = PID_OUT_MIN;

    stab_yaw_error_deg = error;
    stab_yaw_pid_out   = out;

    /* 7. 只控制电机 Yaw，Pitch电机给 0 */
    Set_PWM_Yaw((int)stab_yaw_pid_out);
}