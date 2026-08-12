#ifndef _MOTOR_H
#define _MOTOR_H

#include "zf_common_headfile.h"

void Set_PWM_Yaw(int L_Target);
void Set_PWM_Pitch(int R_Target);
int Calculate_target(int Target) ;
int Calculate_Angle_PWM(float target_angle);
int get_fixed_speed_pwm_period(void);
int Calculate_MA_Angle_PWM(float target_angle);
int Calculate_MB_Angle_PWM(float target_angle);
void Motor_Control_Init(void);
int Calculate_MA_Angle_PWM(float target_angle);
int Calculate_MB_Angle_PWM(float target_angle);
void Motor_init(void);

/* ==================================================================
 *  钢珠平衡任务新增：Pitch 轴位置控制接口（基于原 Calculate_MA_Angle_PWM 改造）
 *  - 使用 Pitch 轴 PWM (PWM_TIM_G7_CH1_A7) 控制升降电机
 *  - 由节点3 实现具体逻辑
 * ================================================================== */

/* 曲柄连杆几何参数结构体（节点3使用） */
typedef struct {
    float crank_length;        /* 曲柄长度 (mm)                          */
    float rod_length;          /* 连杆长度 (mm)                          */
    float pivot_to_crank;      /* 水管合页到曲柄旋转中心水平距离 (mm)    */
    float pivot_height;        /* 水管合页安装高度 (mm)                  */
    float motor_zero_angle;    /* 电机零位转角 (deg)，对应水管水平位置   */
} crank_rod_params_t;

/* 配置曲柄连杆几何参数 */
void Beam_Set_CrankRod_Params(const crank_rod_params_t *params);

/* 获取当前曲柄连杆参数 */
const crank_rod_params_t* Beam_Get_CrankRod_Params(void);

/* 水管倾角(deg) → 电机目标转角(deg) 映射（解析公式或查表，节点3定） */
float Beam_BeamAngle_To_MotorAngle(float beam_angle_deg);

/* 电机目标转角(deg) → PWM 控制量（位置控制核心，启用注释代码改造） */
int   Calculate_Beam_Angle_PWM(float target_motor_angle_deg);

/* 位置控制状态初始化（重置步数累加器、加减速计数器等）
 * 注：原 Motor_Control_Init 已在上方声明，此处不再重复声明 */

/* 获取电机当前估算转角（基于步数累加的开环估算，deg） */
float Beam_Get_Motor_CurrentAngle(void);

/* 位置控制急停（立即停转，清空剩余步数） */
void  Beam_Motor_Stop(void);
#endif