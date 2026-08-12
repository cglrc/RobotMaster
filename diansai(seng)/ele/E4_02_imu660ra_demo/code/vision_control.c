#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "vision_control.h"
#include "uart.h"
#include "stabilizer.h"
#include "stab_beam.h"          // 双环外环接口
#include "motor.h"

// PID控制结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float last_error;
    float integral;
    float derivative;
    float output;
} pid_struct;

// 视觉控制结构体
typedef struct {
    float target_angle;         // 目标角度
    float stable_angle;         // 自稳角度（基础角度）
    float pixel_error_x;        // X轴像素误差
    float pixel_error_y;        // Y轴像素误差
    float angle_offset;         // 角度偏移量
    pid_struct pid;             // PID控制器
} vision_control_struct;

// 全局变量
static vision_control_struct vision_ctrl;
static char vision_data_buffer[32];  // 存储解析前的数据（旧文本协议用）
static uint8_t buffer_index = 0;
static float y_axis_coefficient = -4.0f;  // Y轴误差系数
static float y_axis_dead_zone = 0.0f;      // Y轴死区值
int pwm_value = 0;

/* ==================================================================
 *  钢珠平衡任务：二进制协议解析（节点5）
 *  协议：6字节固定帧 [0x39][status][pos_b0][pos_b1][pos_b2][pos_b3]
 * ================================================================== */
#define VISION_FRAME_HEAD       0x39
#define VISION_FRAME_LEN        6

float    ball_pos_err  = 0.0f;   /* 钢珠位置误差 (像素差，-240~240，0=居中) */
float    ball_pos_vel  = 0.0f;   /* 钢珠速度估计 (像素/s) */
uint8_t  ball_detected = 0;       /* 球检测状态：1=检测到，0=丢球 */

static uint8_t s_rx_buf[VISION_FRAME_LEN];   /* 接收缓冲区 */
static uint8_t s_rx_idx = 0;                  /* 接收索引 */

/* 速度差分滤波参数 */
static float s_pos_scale     = 1.0f;     /* 位置缩放系数（协议已是像素差，默认1.0） */
static float s_filter_alpha  = 0.15f;    /* 低通滤波系数（0~1，越小越平滑；0.3噪声大，0.15适合速度环输入） */
static float s_sample_period = 0.016f;   /* 视觉采样周期 (s)，实测帧率~60Hz(16ms) */
static float s_last_pos     = 0.0f;     /* 上一次位置（用于差分） */
static uint8_t s_pos_init   = 0;        /* 位置差分初始化标志 */
uint8_t s_new_frame_arrived = 0; /* 新帧到达标志（调试打印用） */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     视觉控制模块初始化
// 参数说明     void
// 返回参数     void
// 使用示例     vision_control_init();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void vision_control_init(void)
{
    // 初始化PID参数
    vision_ctrl.pid.kp = 0.1f;
    vision_ctrl.pid.ki = 0.0f;
    vision_ctrl.pid.kd = 0.0f;
    vision_ctrl.pid.error = 0.0f;
    vision_ctrl.pid.last_error = 0.0f;
    vision_ctrl.pid.integral = 0.0f;
    vision_ctrl.pid.derivative = 0.0f;
    vision_ctrl.pid.output = 0.0f;
    
    // 初始化角度值
    vision_ctrl.target_angle = 0.0f;
    vision_ctrl.stable_angle = 0.0f;
    vision_ctrl.pixel_error_x = 0.0f;
    vision_ctrl.pixel_error_y = 0.0f;
    vision_ctrl.angle_offset = 0.0f;
    
    // 清空数据缓冲区
    memset(vision_data_buffer, 0, sizeof(vision_data_buffer));
    buffer_index = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析视觉数据
// 参数说明     *data 数据字符串
// 返回参数     解析出的数值
// 使用示例     int value = parse_vision_data("E15");
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static float parse_vision_data(char *data)
{
    // 检查数据格式是否正确（以E或F开头）
    if (data[0] != 'E' && data[0] != 'F') {
        return 0.0f; // 格式错误，返回0
    }
    
    // 使用atoi转换E/F后面的数值部分
    return (float)atoi(&data[1]);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新PID控制器
// 参数说明     error 误差值
// 返回参数     void
// 使用示例     update_pid(15.0f);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static void update_pid(float error)
{
    // 添加死区处理，当误差绝对值小于等于1时不处理
    if (fabs(error) <= 1.0f) {
        return;
    }
    
    // 更新PID各项值
    vision_ctrl.pid.error = error;
    vision_ctrl.pid.integral += vision_ctrl.pid.error;
    vision_ctrl.pid.derivative = vision_ctrl.pid.error - vision_ctrl.pid.last_error;
    vision_ctrl.pid.last_error = vision_ctrl.pid.error;
    
    // 计算PID输出
    vision_ctrl.pid.output = vision_ctrl.pid.kp * vision_ctrl.pid.error +
                           vision_ctrl.pid.ki * vision_ctrl.pid.integral +
                           vision_ctrl.pid.kd * vision_ctrl.pid.derivative;
    
    // 限制积分项，防止积分饱和
    if (vision_ctrl.pid.integral > 100.0f) {
        vision_ctrl.pid.integral = 100.0f;
    } else if (vision_ctrl.pid.integral < -100.0f) {
        vision_ctrl.pid.integral = -100.0f;
    }
    
    // 保存计算出的角度偏移量
    vision_ctrl.angle_offset = vision_ctrl.pid.output;
    
    // 计算最终目标角度 = 当前目标角度 + 角度偏移量（持续叠加）
    // 只有当误差绝对值大于1时才进行叠加
    vision_ctrl.target_angle += vision_ctrl.angle_offset;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     解析并处理来自视觉模块的数据
// 参数说明     void
// 返回参数     void
// 使用示例     vision_process_data();
// 备注信息     需要在主循环中定期调用
//             VISION_PROTO_BINARY=1 使用新二进制协议(0x39帧头)
//             VISION_PROTO_BINARY=0 使用旧文本协议(E/F开头)
//-------------------------------------------------------------------------------------------------------------------
#define VISION_PROTO_BINARY  1   /* 1=新二进制协议，0=旧文本协议 */

#if VISION_PROTO_BINARY
/* 二进制协议解析：状态机逐字节处理 */
static void vision_parse_byte(uint8 ch)
{
    if (s_rx_idx == 0)
    {
        /* 等待帧头 */
        if (ch == VISION_FRAME_HEAD)
        {
            s_rx_buf[s_rx_idx++] = ch;
        }
    }
    else
    {
        /* 接收剩余字节 */
        s_rx_buf[s_rx_idx++] = ch;

        if (s_rx_idx >= VISION_FRAME_LEN)
        {
            /* 完整帧接收完毕，解析 */
            s_rx_idx = 0;

            uint8_t status = s_rx_buf[1];
            /* 小端 float32 拼接（原始数据为像素差，范围 -240~240） */
            float pos_raw;
            memcpy(&pos_raw, &s_rx_buf[2], 4);

            ball_detected = (status == 0x01) ? 1 : 0;

            if (ball_detected)
            {
                float pos_scaled = pos_raw * s_pos_scale;

                /* 速度差分 + 低通滤波 */
                if (s_pos_init)
                {
                    float raw_vel = (pos_scaled - s_last_pos) / s_sample_period;
                    ball_pos_vel = s_filter_alpha * raw_vel + (1.0f - s_filter_alpha) * ball_pos_vel;
                }
                else
                {
                    ball_pos_vel = 0.0f;
                    s_pos_init = 1;
                }
                s_last_pos = pos_scaled;
                ball_pos_err = pos_scaled;
            }
            else
            {
                /* 丢球：保持位置但速度清零 */
                ball_pos_vel = 0.0f;
                s_pos_init = 0;
            }

            /* 打印摄像头发来的原始数据（VOFA 二进制输出模式下禁用，避免文本干扰数据流） */
            /* printf("[CAM] status=%d pos_raw=%.1f\r\n", status, pos_raw); */

            /* 收到完整帧：置位新帧标志，供 vision_beam_run 打印调试信息 */
            s_new_frame_arrived = 1;
        }
    }
}

void vision_process_data(void)
{
    uint32 data_count = fifo_used(&uart_data_fifo);

    if (data_count > 0)
    {
        uint8 temp_data[64];
        uint32 read_count = data_count;
        if (read_count > sizeof(temp_data))
        {
            read_count = sizeof(temp_data);
        }

        fifo_read_buffer(&uart_data_fifo, temp_data, &read_count, FIFO_READ_AND_CLEAN);

        for (uint32 i = 0; i < read_count; i++)
        {
            vision_parse_byte(temp_data[i]);
        }
    }
}

#else  /* 旧文本协议 */
void vision_process_data(void)
{
    // 检查是否有新数据
    uint32 data_count = fifo_used(&uart_data_fifo);
    
    if (data_count > 0) {
        // 读取FIFO中的数据
        uint8 temp_data[64];
        uint32 read_count = data_count;
        if (read_count > sizeof(temp_data)) {
            read_count = sizeof(temp_data);
        }
        
        fifo_read_buffer(&uart_data_fifo, temp_data, &read_count, FIFO_READ_AND_CLEAN);
        
        // 处理每个字节的数据
        for (uint32 i = 0; i < read_count; i++) {
            uint8 ch = temp_data[i];
            
            // 如果遇到换行符，则处理整条数据
            if ((ch == '\n' || ch == '\r') && buffer_index > 0) {
                vision_data_buffer[buffer_index] = '\0';  // 字符串结束符
                
                // 检查数据是否以'E'开头，确保是有效的视觉数据(X轴)
                if (vision_data_buffer[0] == 'E') {
                    // 解析X轴数据
                    vision_ctrl.pixel_error_x = parse_vision_data(vision_data_buffer);
                    
                    // 使用PID控制器处理误差
                    update_pid(vision_ctrl.pixel_error_x);
                    
                    // 立即更新目标角度
                    stab_yaw_set_target(vision_ctrl.target_angle);
                }
                 // 检查数据是否以'F'开头，确保是有效的视觉数据(Y轴)
                 else if (vision_data_buffer[0] == 'F') {
                     // 解析Y轴数据
                     vision_ctrl.pixel_error_y = parse_vision_data(vision_data_buffer);
                    
                     // 添加死区控制
                     if (fabs(vision_ctrl.pixel_error_y) <= y_axis_dead_zone) {
                         pwm_value = 0;
                         Set_PWM_Pitch(0);
                     } else {
                         pwm_value = (int)(vision_ctrl.pixel_error_y * y_axis_coefficient);
                         Set_PWM_Pitch(pwm_value);
                     }
                 }
                
                // 重置缓冲区
                buffer_index = 0;
                memset(vision_data_buffer, 0, sizeof(vision_data_buffer));
            } 
            else if (ch != '\n' && ch != '\r') {
                if (buffer_index < sizeof(vision_data_buffer) - 1) {
                    vision_data_buffer[buffer_index++] = ch;
                }
            }
        }
    }
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取解析出的X轴误差值
// 参数说明     void
// 返回参数     float X轴像素误差值
// 使用示例     float error = vision_get_error_x();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
float vision_get_error_x(void)
{
    return vision_ctrl.pixel_error_x;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取解析出的Y轴误差值
// 参数说明     void
// 返回参数     float Y轴像素误差值
// 使用示例     float error = vision_get_error_y();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
float vision_get_error_y(void)
{
    return vision_ctrl.pixel_error_y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取计算出的目标角度
// 参数说明     void
// 返回参数     float 目标角度
// 使用示例     float target = vision_get_target_angle();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
float vision_get_target_angle(void)
{
    return vision_ctrl.target_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置Y轴系数
// 参数说明     coefficient Y轴系数
// 返回参数     void
// 使用示例     vision_set_y_coefficient(15.0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void vision_set_y_coefficient(float coefficient)
{
    y_axis_coefficient = coefficient;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置Y轴死区值
// 参数说明     dead_zone Y轴死区值
// 返回参数     void
// 使用示例     vision_set_y_dead_zone(2.0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void vision_set_y_dead_zone(float dead_zone)
{
    y_axis_dead_zone = fabs(dead_zone);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID参数
// 参数说明     kp 比例系数
// 参数说明     ki 积分系数
// 参数说明     kd 微分系数
// 返回参数     void
// 使用示例     vision_pid_set(0.1, 0.01, 0.05);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void vision_pid_set(float kp, float ki, float kd)
{
    vision_ctrl.pid.kp = kp;
    vision_ctrl.pid.ki = ki;
    vision_ctrl.pid.kd = kd;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置自稳角度（基础角度）
// 参数说明     angle 自稳角度
// 返回参数     void
// 使用示例     vision_set_stable_angle(45.0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void vision_set_stable_angle(float angle)
{
    vision_ctrl.stable_angle = angle;
}

/* ==================================================================
 *  钢珠位置接口实现（节点5）
 * ================================================================== */
float vision_get_ball_error(void)
{
    return ball_pos_err;
}

float vision_get_ball_velocity(void)
{
    return ball_pos_vel;
}

uint8_t vision_get_ball_detected(void)
{
    return ball_detected;
}

void vision_set_ball_params(float pos_scale, float filter_alpha, float sample_period)
{
    s_pos_scale     = pos_scale;
    s_filter_alpha  = filter_alpha;
    s_sample_period = sample_period;
}

void vision_inject_ball_error(float err)
{
    /* 手动注入位置（用于无视觉时的测试） */
    if (s_pos_init)
    {
        float raw_vel = (err - s_last_pos) / s_sample_period;
        ball_pos_vel = s_filter_alpha * raw_vel + (1.0f - s_filter_alpha) * ball_pos_vel;
    }
    else
    {
        ball_pos_vel = 0.0f;
        s_pos_init = 1;
    }
    s_last_pos = err;
    ball_pos_err = err;
    ball_detected = 1;
}

/* ==================================================================
 *  单视觉环 PD 控制（速度控制）
 *  - 控制律：pwm = direction * (Kp * pos_err + Kd * pos_vel)
 *  - 输出：PWM 频率（Hz），带方向符号
 *  - 不使用 IMU pitch 反馈
 *  - 软件角度限位：基于 PWM 估算角度，超限停止输出
 * ================================================================== */

/* 电机参数（与 motor.c 一致，用于角度估算）
 * 注意：BEAM_MICROSTEPS = 32 为最终校准值，请勿修改 */
#define BEAM_STEPS_PER_REV   200
#define BEAM_MICROSTEPS      32
#define BEAM_PULSES_PER_DEG  ((float)(BEAM_STEPS_PER_REV * BEAM_MICROSTEPS) / 360.0f)
/* 双边沿触发：实际微步率 = 2 × PWM频率 */
#define BEAM_DT_SEC          0.004f   /* 主循环周期 4ms */

static float s_beam_kp = 0.0f;
static float s_beam_kd = 0.0f;
static float s_beam_ki = 0.0f;          /* PID积分增益 */
static float s_beam_kff = 0.0f;         /* 前馈增益：位置→PWM (PWM/pixel) */
static int   s_beam_direction = 1;       /* 方向符号 +1 或 -1 */
static int   s_beam_limit = 100;         /* PWM 限幅 (Hz) */
static float s_beam_cur_angle = 0.0f;    /* 当前估算角度 (deg) */
static int   s_beam_last_output = 0;     /* 上次输出值（调试用） */
static float s_beam_vel_integral = 0.0f; /* 误差积分（用于I项） */
static float s_beam_vel_integral_limit = 500.0f; /* 积分限幅 */
static float s_beam_last_error = 0.0f;   /* 上次误差（用于D项微分） */

void vision_beam_init(float kp, float kd, int direction)
{
    s_beam_kp = kp;
    s_beam_kd = kd;
    s_beam_ki = 0.0f;
    s_beam_direction = (direction >= 0) ? 1 : -1;
    s_beam_last_output = 0;
    s_beam_cur_angle = 0.0f;
    s_beam_kff = 0.0f;
    s_beam_vel_integral = 0.0f;
    s_beam_last_error = 0.0f;
    Set_PWM_Pitch(0);
}

void vision_beam_set_limit(int max_pwm)
{
    s_beam_limit = max_pwm;
}

void vision_beam_set_feedforward(float kff)
{
    s_beam_kff = kff;   /* 位置→PWM增益 (PWM/pixel) */
}

void vision_beam_set_ki(float ki)
{
    s_beam_ki = ki;     /* 积分增益 (Hz/(pixel·s)) */
    s_beam_vel_integral = 0.0f;  /* 改参数时清空积分，避免旧值冲击 */
}

void vision_beam_run(void)
{
    int pwm_out = 0;
    float raw_pd = 0.0f;
    int  limit_triggered = 0;

    if (ball_detected)
    {
        /* --- PID 控制 → 输出 PWM 频率 ---
         * target_pwm = (Kp × error + Ki × ∫error + Kd × velocity) × direction
         * Kp: 位置偏差→PWM (Hz/pixel)
         * Ki: 位置误差积分→PWM，消除稳态偏差
         * Kd: 速度→PWM (Hz/(pixel/s))
         */
        float position_error = ball_pos_err;  /* 正=偏右，负=偏左，目标=0 */
        float dir = (float)s_beam_direction;
        /* 将方向并入误差，统一到输出空间，便于抗饱和判断 */
        float err = position_error * dir;

        float p_term = s_beam_kp * err;
        float d_term = s_beam_kd * ball_pos_vel * dir;

        /* 抗积分饱和（conditional integration）：
         * PD 输出已饱和且误差会加剧饱和时停止积分，防止 windup */
        float pd_out = p_term + d_term;
        int saturating_pos = (pd_out >  (float)s_beam_limit);
        int saturating_neg = (pd_out < -(float)s_beam_limit);
        if (!((saturating_pos && err > 0.0f) || (saturating_neg && err < 0.0f)))
        {
            s_beam_vel_integral += err * BEAM_DT_SEC;
            if (s_beam_vel_integral >  s_beam_vel_integral_limit) s_beam_vel_integral =  s_beam_vel_integral_limit;
            if (s_beam_vel_integral < -s_beam_vel_integral_limit) s_beam_vel_integral = -s_beam_vel_integral_limit;
        }
        float i_term = s_beam_ki * s_beam_vel_integral;

        float output = p_term + d_term + i_term;

        raw_pd = output;

        /* PWM 限幅 */
        if (output > (float)s_beam_limit)
        {
            output = (float)s_beam_limit;
            limit_triggered = 1;
        }
        else if (output < -(float)s_beam_limit)
        {
            output = -(float)s_beam_limit;
            limit_triggered = 1;
        }

        pwm_out = (int)output;
    }
    else
    {
        /* 丢球：PWM=0 */
        pwm_out = 0;
        limit_triggered = -1;
    }

    s_beam_last_output = pwm_out;
    Set_PWM_Pitch(pwm_out);

    /* 调试信息 */
    if (s_new_frame_arrived)
    {
        s_new_frame_arrived = 0;
        printf("[PD] det=%d pos=%.1fpx vel=%.1fpx/s | raw=%.1f pwm=%d lim=%d\r\n",
               ball_detected,
               ball_pos_err,
               ball_pos_vel,
               raw_pd,
               pwm_out,
               limit_triggered);
    }
}

void vision_beam_stop(void)
{
    s_beam_last_output = 0;
    Set_PWM_Pitch(0);
}

void vision_beam_reset_angle(void)
{
    s_beam_cur_angle = 0.0f;
}

float vision_beam_get_angle(void)
{
    return s_beam_cur_angle;
}

int vision_beam_get_output(void)
{
    return s_beam_last_output;
}
