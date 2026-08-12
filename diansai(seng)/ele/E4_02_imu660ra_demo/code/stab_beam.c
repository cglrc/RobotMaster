/* ==================================================================
 *  stab_beam.c - 钢珠平衡三环串级控制器实现
 *  - 位置环（P）：钢珠位置误差 → 目标球速
 *  - 速度环（PI）：目标球速 - 实际球速 → 目标水管倾角
 *  - 内环（编码器倾角环 PD）：目标倾角 - 当前倾角 → PWM频率
 *  - 内环在 5ms 中断中调用（与 cascade_tick 同步）
 * ================================================================== */

#include "stab_beam.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>

/* ==================== 外部变量定义（供调试与监控） ==================== */
/* 位置环输出 */
float stab_beam_outer_target_vel = 0.0f;    /* 位置环输出：目标球速 (px/s) */
float stab_beam_outer_pos_err    = 0.0f;    /* 当前钢珠位置误差 (px) */

/* 速度环输出 */
float stab_beam_velocity_target_pitch = 0.0f; /* 速度环输出：目标倾角 (deg) */

/* 内环状态 */
float stab_beam_inner_target       = 0.0f;   /* 内环目标倾角 (deg) */
float stab_beam_inner_current      = 0.0f;   /* 内环当前倾角 (来自编码器, deg) */
float stab_beam_inner_motor_target = 0.0f;   /* 内环输出：电机目标转角 (deg) */

/* ==================== 位置环（P）参数 ==================== */
static float s_pos_kp    = 0.0f;
static float s_pos_limit = 500.0f;  /* 目标速度限幅 (px/s) */

void stab_beam_outer_init(float kp)
{
    s_pos_kp = kp;
    stab_beam_outer_target_vel = 0.0f;
    stab_beam_outer_pos_err   = 0.0f;
}

void stab_beam_outer_set_pos(float pos_err)
{
    stab_beam_outer_pos_err = pos_err;

    /* 控制律：v_ref = -Kp * x
     * 钢珠偏右(x>0) → 目标速度为负(向左) */
    float error = -pos_err;
    float output = s_pos_kp * error;
    if (output >  s_pos_limit) output =  s_pos_limit;
    if (output < -s_pos_limit) output = -s_pos_limit;
    stab_beam_outer_target_vel = output;
}

float stab_beam_outer_get_target_vel(void)
{
    return stab_beam_outer_target_vel;
}

void stab_beam_outer_reset(void)
{
    stab_beam_outer_target_vel = 0.0f;
    stab_beam_outer_pos_err   = 0.0f;
}

void stab_beam_outer_set_limit(float max_vel)
{
    if (max_vel > 0.0f)
        s_pos_limit = max_vel;
}

/* ==================== 速度环（PI）参数 ==================== */
static float s_vel_kp    = 0.0f;
static float s_vel_ki    = 0.0f;
static float s_vel_limit = 10.0f;           /* 速度环输出限幅 (deg) */
static float s_vel_integral      = 0.0f;    /* 速度误差积分 (px) */
static float s_vel_integral_limit = 500.0f;  /* 积分限幅 (px) */
static float s_vel_dt    = 0.016f;          /* 速度环调用周期 (s) */
static float s_vel_integral_sep = 10.0f;    /* 积分分离阈值 (px/s)，速度误差大于此值时禁用积分 */

void stab_beam_velocity_init(float kp, float ki)
{
    s_vel_kp = kp;
    s_vel_ki = ki;
    stab_beam_velocity_target_pitch = 0.0f;
    s_vel_integral = 0.0f;
}

void stab_beam_velocity_set_vel(float vel_ref, float vel_actual)
{
    /* 控制律：θ = Kp * (v_ref - v) + Ki * ∫(v_ref - v)dt
     * 速度误差 = 目标球速 - 实际球速
     * Ki 积分速度误差，球受摩擦卡顿时速度误差持续偏置，
     * 积分单向累积消除稳态偏差 */
    float error = vel_ref - vel_actual;
    float p_term = s_vel_kp * error;

    /* 积分分离（integral separation）：
     * 大速度误差时（系统振荡或远端制动）禁用积分，避免正负抵消；
     * 小速度误差时（接近稳态）才启用积分，消除摩擦导致的稳态偏差。
     * 注意：不积分时保留已有积分值（不清零），让 I 项平滑过渡 */
    if (fabsf(error) <= s_vel_integral_sep)
    {
        /* 抗积分饱和（conditional integration）：
         * 基于总输出(p+i)判断饱和，而非仅 P 项，
         * 这样 P 项饱和但 I 项反向制动时仍可继续积分 */
        float temp_output = p_term + s_vel_ki * s_vel_integral;
        int saturating_pos = (temp_output >  s_vel_limit);
        int saturating_neg = (temp_output < -s_vel_limit);
        if (!((saturating_pos && error > 0.0f) || (saturating_neg && error < 0.0f)))
        {
            s_vel_integral += error * s_vel_dt;
            if (s_vel_integral >  s_vel_integral_limit) s_vel_integral =  s_vel_integral_limit;
            if (s_vel_integral < -s_vel_integral_limit) s_vel_integral = -s_vel_integral_limit;
        }
    }
    float i_term = s_vel_ki * s_vel_integral;

    float output = p_term + i_term;
    if (output >  s_vel_limit) output =  s_vel_limit;
    if (output < -s_vel_limit) output = -s_vel_limit;

    stab_beam_velocity_target_pitch = output;
}

float stab_beam_velocity_get_target(void)
{
    return stab_beam_velocity_target_pitch;
}

float stab_beam_velocity_get_integral(void)
{
    return s_vel_integral;
}

void stab_beam_velocity_reset(void)
{
    stab_beam_velocity_target_pitch = 0.0f;
    s_vel_integral = 0.0f;
}

void stab_beam_velocity_set_limit(float max_angle_deg)
{
    if (max_angle_deg > 0.0f)
        s_vel_limit = max_angle_deg;
}

void stab_beam_velocity_set_dt(float dt_sec)
{
    if (dt_sec > 0.0f)
        s_vel_dt = dt_sec;
}

void stab_beam_velocity_set_integral_sep(float threshold)
{
    if (threshold >= 0.0f)
        s_vel_integral_sep = threshold;
}

void stab_beam_velocity_set_integral_limit(float limit)
{
    if (limit > 0.0f)
        s_vel_integral_limit = limit;
}

/* ==================== 内环（编码器位置闭环） ==================== */
/* 参考 WHEELTEC closed_loop.c 架构：
 *   读取编码器角度 → 计算与目标的误差 → 选择PWM频率 → 输出步进脉冲
 *   与 WHEELTEC 的区别：我们用连续 PWM 驱动（而非发送固定步数脉冲）
 *
 * 内环参数：
 *   s_inner_kp  — 位置误差到 PWM 频率的增益 (Hz/deg)
 *   s_inner_kd  — 速度阻尼增益 (Hz/(deg/s))，用编码器角速度反馈
 *   s_inner_limit — PWM 限幅 (Hz)
 */
static float s_inner_kp = 0.0f;       /* 角度误差增益 (Hz/deg) */
static float s_inner_kd = 0.0f;       /* 角速度阻尼增益 (Hz/(deg/s)) */
static int   s_inner_limit = 7000;    /* PWM 限幅 (Hz) */
static float s_inner_tolerance = 0.02f; /* 位置容差 (deg)，原0.1≈1个编码器count太大导致停机过早 */
static float s_inner_min_pwm = 20.0f;   /* 最小 PWM 频率 (Hz)，低于此步进电机无法克服静摩擦启动 */

void stab_beam_inner_init(float kp, float kd)
{
    s_inner_kp = kp;
    s_inner_kd = kd;
    stab_beam_inner_target       = 0.0f;
    stab_beam_inner_current      = 0.0f;
    stab_beam_inner_motor_target = 0.0f;
}

void stab_beam_inner_set_target(float target_pitch_deg)
{
    stab_beam_inner_target = target_pitch_deg;
}

void stab_beam_inner_run_200Hz(void)
{
    /* 1. 获取编码器当前角度和角速度 */
    stab_beam_inner_current = Encoder_GetAngle();
    float enc_speed = Encoder_CalcSpeedDps();

    /* 2. 计算位置误差 */
    float error = stab_beam_inner_target - stab_beam_inner_current;

    /* 3. 在容差范围内停止输出 */
    if (fabsf(error) < s_inner_tolerance)
    {
        stab_beam_inner_motor_target = 0.0f;
        Set_PWM_Pitch(0);
        return;
    }

    /* 4. PD 控制律：pwm = -(Kp * error - Kd * ω)
     *    取反：编码器取反后角度方向与电机PWM方向相反，需取反输出
     *    Kp 项：误差越大→PWM频率越高→电机转得越快
     *    Kd 项：速度阻尼→抑制超调和振荡
     *    输出为 PWM 频率 (Hz)，正负号表示方向 */
    float output = -(s_inner_kp * error - s_inner_kd * enc_speed);

    /* 5. 最小 PWM 频率兜底：小误差时 P 算出的频率太低（如 30Hz）
     *    步进电机带载无法启动，强制提升到 s_inner_min_pwm，保证连续转动 */
    if (output > 0.0f && output < s_inner_min_pwm)
        output = s_inner_min_pwm;
    else if (output < 0.0f && output > -s_inner_min_pwm)
        output = -s_inner_min_pwm;

    /* 6. PWM 限幅 */
    if (output > (float)s_inner_limit)
        output = (float)s_inner_limit;
    else if (output < -(float)s_inner_limit)
        output = -(float)s_inner_limit;

    stab_beam_inner_motor_target = output;

    /* 6. 输出 PWM 频率驱动步进电机 */
    Set_PWM_Pitch((int)output);
}

void stab_beam_inner_reset(void)
{
    stab_beam_inner_target       = 0.0f;
    stab_beam_inner_current      = 0.0f;
    stab_beam_inner_motor_target = 0.0f;
    Set_PWM_Pitch(0);
}

void stab_beam_inner_set_limit(int max_pwm)
{
    s_inner_limit = max_pwm;
}

void stab_beam_inner_set_tolerance(float tol_deg)
{
    if (tol_deg > 0.0f)
        s_inner_tolerance = tol_deg;
}

void stab_beam_inner_set_min_pwm(float min_hz)
{
    if (min_hz >= 0.0f)
        s_inner_min_pwm = min_hz;
}
