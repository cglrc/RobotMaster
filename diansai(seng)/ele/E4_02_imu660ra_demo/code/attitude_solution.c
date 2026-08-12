#include "attitude_solution.h"
#include "stab_beam.h"
#include <math.h>

//gyro_Offset_Init()--1.当陀螺仪静止的时候，会产生零飘，那么我们将静止时刻的零飘值进行均值滤波，采集一千次累加之后求其平均值，得到零飘的平均值，在后续的处理中减去零飘平均值即可
//IMU_GetValues()-----2，函数中对加速度计进行一阶低通滤波，但实际没有用到加速度计/陀螺仪减去零飘平均值并转换为弧度制
//IMU_YAW_integral()--3.对转换为弧度制后的角速度求积分，然后利用GPS函数里的弧度制转角度制函数并累积，放在定时器里，2ms中断一次，IMU刷新频率200HZ,5ms一次

gyro_param_t Gyro_Offset;//陀螺仪零飘结构体
IMU_param_t  IMU_Data;   //减去零飘数据结构体

float gyro_Offset_flag;
float Daty_X=0;//积分后的角度
float Daty_Y=0;
float Daty_Z=0;
float IMU_Daty_Z=0;
float X;
float T_M=0;
float T_N=0;
float C_Direction_2=0;
int   Get = 2;  //ms  2ms提取一次数据 要记得去.h文件里也更新数据

/* ==================================================================
 *  钢珠平衡任务：Pitch 角解算全局变量
 *  - IMU660ra 安装方向：水管 pitch 运动对应 IMU 的 X 轴（实测确认）
 *  - 如安装方向变化，可调整 IMU_pitch_update 内的轴选择
 * ================================================================== */
float pitch_angle      = 0.0f;    /* 积分得到的水管倾角 (deg)，正=右端升高 */
float pitch_gyro       = 0.0f;    /* 当前俯仰角速度 (deg/s) */
float pitch_angle_acc  = 0.0f;    /* 保留字段（纯积分模式下不更新） */

uint8_t s_inner_loop_enabled = 0;  /* 内环使能标志，默认关闭 */

static float  pitch_angle_raw_   = 0.0f;   /* 积分原始角度，未减偏置 (deg) */
static float  pitch_bias_        = 0.0f;   /* 零点偏置 (deg) */
/**
 * @brief 陀螺仪零漂初始化
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后续 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */
float IMU_gyro_Offset_Init()
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++)
    {
       Gyro_Offset.Xdata += imu660ra_gyro_x;
       Gyro_Offset.Ydata += imu660ra_gyro_y;
       Gyro_Offset.Zdata += imu660ra_gyro_z;
        system_delay_ms(Get);   // 最大 1Khz
    }

    Gyro_Offset.Xdata /= 1000;
    Gyro_Offset.Ydata /= 1000;
    Gyro_Offset.Zdata /= 1000;

    return gyro_Offset_flag=1;
}

/**
 * @brief 将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
 * 加速度计初始化配置 -> 测量范围: ±8g        对应灵敏度: 4096 LSB/g
 * 陀螺仪初始化配置   -> 测量范围: ±2000 dps  对应灵敏度: 16.4 LSB/dps   (degree per second)
 * @tips: gyro = (gyro_val / 16.4) °/s = ((gyro_val / 16.4) * PI / 180) rad/s
 */
void IMU_GetValues()//将采集的数值转化为实际物理值, 并对陀螺仪进行去零漂处理
{
//2000dps:IMU660--16.4
//2000dps:IMU963--14.3

    //! 陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    //! 注：pitch (现用 X 轴) 已恢复零漂校正，与 yaw 处理一致
    //!     X 轴灵敏度已校准：16.10（实测微调）
    IMU_Data.gyro_x = ((float) imu660ra_gyro_x - Gyro_Offset.Xdata) * PI / 180 / 16.10f;
    IMU_Data.gyro_y = (float) imu660ra_gyro_y * PI / 180 / 16.4f;
    IMU_Data.gyro_z = ((float) imu660ra_gyro_z - Gyro_Offset.Zdata) * PI / 180 / 16.4f;
}


void IMU_YAW_integral()//对角速度进行积分
{


        IMU_GetValues();

    //    Daty_X+=RAD_TO_ANGLE(IMU_Data.gyro_x*0.005);
    //    Daty_Y+=RAD_TO_ANGLE(IMU_Data.gyro_y*0.005);
    //    if(IMU_Data.gyro_z<0.0045&&IMU_Data.gyro_z>-0.0045)
        if(IMU_Data.gyro_z<0.015&&IMU_Data.gyro_z>-0.015)//滤波
        {
            Daty_Z-=0;
        }
        else
        {
//             IMU_Handle_180();//规划为0-180和0-(-180)   Daty_Z
             IMU_Handle_360();//规划为0-360和0-(-360)   T_M
//             IMU_Handle_0();  //规划为0-正无穷和0-负无穷  T_N
         }
}



void IMU_init()//IMU初始化
{
    // 初始化陀螺仪
    while(1)
    {
        if(imu660ra_init())
        {
			printf("\r\n imu660ra init error.");                              	// imu660ra 初始化失败
        }
        else
        {
			printf("\r\n imu660ra init success.");                            	// imu660ra 初始化成功
			break;
        }

    }
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_ms_init(PIT_TIM_A0, Get, imu_5ms_task, NULL); //imu660ra定时器中断  5ms积分一次角速度
    IMU_gyro_Offset_Init();// 陀螺仪零漂初始化
}



void IMU_Handle_180()
{
    Daty_Z-=RAD_TO_ANGLE(IMU_Data.gyro_z*Get/1000);//(积分过程)本来是逆时针为正,现在改为顺时针为正

    if((Daty_Z>0&&Daty_Z<=180)  ||   (Daty_Z<0&&Daty_Z>=(-180)))//顺时针
    {
        Daty_Z= +Daty_Z;
    }
    else if(Daty_Z>180 && Daty_Z<=360)
    {
        Daty_Z-=360;
    }
    else if(Daty_Z<(-180) && Daty_Z>=(-360))
    {
        Daty_Z+=360;
    }

}


void IMU_Handle_360()
{
    T_M-=RAD_TO_ANGLE(IMU_Data.gyro_z*Get/1000);//(积分过程)本来是逆时针为正,现在改为顺时针为正

    if(T_M>360||T_M<-360)//限制IMU的数值在0-360之间
    {
        T_M=0;
    }
}

void IMU_Handle_0()
{
    T_N-=RAD_TO_ANGLE(IMU_Data.gyro_z*Get/1000);//(积分过程)本来是逆时针为正,现在改为顺时针为正
}

/* ==================================================================
 *  钢珠平衡任务：Pitch 角解算实现
 *  - 采用纯陀螺仪积分方式（参考原 yaw 积分 IMU_Handle_0 风格）
 *  - IMU_GetValues 已将 gyro_y 转为 rad/s 并去零漂
 *  - pitch_angle 累积减去角速度×时间步长（与原 T_N 风格一致）
 * ================================================================== */
void IMU_pitch_update(void)
{
    /* 1. 获取 pitch 角速度 (deg/s)，来自陀螺仪 X 轴（已去零漂、转 rad/s）
     *    注：因 IMU 安装方向，水管 pitch 运动对应 IMU 的 X 轴 */
    pitch_gyro = RAD_TO_ANGLE(IMU_Data.gyro_x);

    /* 2. 死区滤波 + 直接积分得到 pitch 角 (deg)
     *    模仿 yaw 轴处理：角速度绝对值小于阈值时视为静止，不积分
     *    阈值 0.005 rad/s ≈ 0.29 deg/s（比 yaw 的 0.015 小，适配 pitch 缓慢倾斜） */
    if (IMU_Data.gyro_x < 0.005f && IMU_Data.gyro_x > -0.005f)
    {
        /* 死区内：不积分 */
    }
    else
    {
        pitch_angle_raw_ -= pitch_gyro * (float)Get / 1000.0f;
    }

    /* 3. 直接输出原始积分值（已按用户要求去除零点偏置） */
    pitch_angle = pitch_angle_raw_;
}

/* 设置 pitch 角零点偏置 (deg)
 * 标定流程（程序启动后 pitch_bias_=0，此时 pitch_angle == pitch_angle_raw_）：
 *   1. 水管水平稳定后多次读取 pitch_angle 取平均（即 raw 角度）
 *   2. 调用 IMU_pitch_set_bias(该平均值)
 *   3. 之后 pitch_angle 输出归零 */
void IMU_pitch_set_bias(float bias_deg)
{
    pitch_bias_ = bias_deg;
}

/* 获取当前 pitch 角零点偏置 (deg) */
float IMU_pitch_get_bias(void)
{
    return pitch_bias_;
}

void imu_5ms_task(uint32 event, void *user_ptr)
{
    (void)event;
    (void)user_ptr;
    imu660ra_get_gyro();   // 取原始角速度
    if (gyro_Offset_flag)
    {
        IMU_YAW_integral();    // 积分得角度（内部调用 IMU_GetValues 更新 IMU_Data.gyro_y）
        IMU_pitch_update();    // 钢珠平衡任务：更新 pitch_angle / pitch_gyro（依赖 IMU_Data.gyro_y）
        if (s_inner_loop_enabled)
        {
            stab_beam_inner_run_200Hz();  // 双环内环：目标角度 vs 当前角度 → PWM
        }
    }
}