#ifndef __ATTITUDE_H
#define __ATTITUDE_H

#include "zf_common_headfile.h"
#define ANGLE_TO_RAD(x)    ((x) * PI / 180.0)                                   // 角度转换为弧度
#define RAD_TO_ANGLE(x)    ((x) * 180.0 / PI)                                   // 弧度转换为角度
#define PI                 (3.1415926535898)
//结构体声明
typedef struct{
    float Xdata;   //零飘参数X
    float Ydata;   //零飘参数Y
    float Zdata;   //零飘参数Z
}gyro_param_t ;

typedef struct{
    float acc_x;   //x轴加速度
    float acc_y;   //y轴加速度
    float acc_z;   //z轴加速度

    float gyro_x;  //x轴角速度
    float gyro_y;  //y轴角速度
    float gyro_z;  //z轴角速度
}IMU_param_t ;

//全局变量声明
extern float gyro_Offset_flag;
extern float T_M;
extern float T_N;
extern int   GL_IMU_Flag;
extern float Daty_Y;     // 俯仰角  单位：度
extern float Daty_Z;     // 偏航角  单位：度

//函数声明

float IMU_gyro_Offset_Init();//陀螺仪零漂初始化
void  IMU_GetValues();//将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
void  IMU_YAW_integral(void);//对角速度进行积分
void  IMU_init(void);//IMU初始化
void  IMU_Handle_180(void);
void  IMU_Handle_360(void);
void  IMU_Handle_0  (void);
void  imu_pit_cb(uint8_t index, void *param);
void  imu_5ms_task(uint32 event, void *user_ptr);

/* ==================================================================
 *  钢珠平衡任务新增：Pitch 角解算接口（IMU 装在水管上）
 *  - 由节点2 实现，采用纯陀螺仪积分方式（参考原 yaw 积分风格）
 *  - pitch_angle: 水管俯仰角 (deg)，正值表示右端升高
 *  - pitch_gyro : 俯仰角速度 (deg/s)，内环 PD 微分项使用
 * ================================================================== */
extern float pitch_angle;        /* 积分得到的水管倾角 (deg) */
extern float pitch_gyro;         /* 当前俯仰角速度 (deg/s) */
extern float pitch_angle_acc;    /* 保留字段（纯积分模式下不更新，始终为 0） */

/* 在 5ms 中断中调用，更新 pitch_angle 与 pitch_gyro */
void  IMU_pitch_update(void);

/* 设置 pitch 角零点偏置（标定用，deg），调用后当前角度归零到该偏置 */
void  IMU_pitch_set_bias(float bias_deg);

/* 获取当前 pitch 角零点偏置 */
float IMU_pitch_get_bias(void);

/* 内环使能标志：1=运行双环内环，0=仅更新角度不控制电机 */
extern uint8_t s_inner_loop_enabled;
#endif
