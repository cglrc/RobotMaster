/*********************************************************************************************************************
* MSPM0G3507 Opensource Library 即（MSPM0G3507 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MSPM0G3507 开源库的一部分
* 
* MSPM0G3507 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          mian
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK 5.37
* 适用平台          MSPM0G3507
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2025-06-1        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "../code/motor.h"
#include "../code/attitude_solution.h"
#include "../code/vision_control.h"
#include "../code/stab_beam.h"
#include "../code/stabilizer.h"
#include "../code/encoder.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源
// 接入 imu660ra
//      模块管脚            单片机管脚
//      SCL/SPC             查看 zf_device_imu660ra.h 中 IMU6660RA_SPC_PIN 宏定义
//      SDA/DSI             查看 zf_device_imu660ra.h 中 IMU6660RA_SDI_PIN 宏定义
//      SA0/SDO             查看 zf_device_imu660ra.h 中 IMU6660RA_SDO_PIN 宏定义
//      CS                  查看 zf_device_imu660ra.h 中 IMU6660RA_CS_PIN  宏定义
//      GND                 电源地 GND
//      3V3                 电源 3V3

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好编码器，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑 完成上电 正常 H2 LED 会闪烁
// 3.电脑上使用 逐飞助手 打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在 逐飞助手 上看到如下串口信息：
//      imu660ra acc data: x-..., y-..., z-...
//      imu660ra gyro data: x-..., y-..., z-...
// 5.移动旋转 imu660ra 就会看到数值变化
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

/* ==================================================================
 *  节点2 测试用：普通串口输出（带标签可读格式）
 *  - 通过 debug UART (UART_0) 发送
 *  - 测试完成后可删除该函数与主循环中的调用
 * ================================================================== */
#define VOFA_TEST_ENABLE    (0)     /* 1=启用 pitch 测试输出，0=禁用 */

/* ==================================================================
 *  IMU 测试模式：只初始化IMU并打印pitch角度，不运行控制环
 *  - 用于验证IMU660ra软件IIC连接是否正常
 *  - 设为0后启用双环控制
 * ================================================================== */
#define IMU_TEST_ENABLE     (0)     /* 1=仅测试IMU数据，0=双环控制 */

#if VOFA_TEST_ENABLE
/* 输出 pitch_angle, pitch_gyro, T_M 三个数据 */
static void vofa_send_pitch_data(void)
{
    /* 普通串口格式：带标签可读 */
    printf("pitch=%.2f, gyro=%.2f, yaw=%.2f\r\n", pitch_angle, pitch_gyro, T_M);
}
#endif

/* ==================================================================
 *  节点5 测试用：摄像头通信测试 + 单视觉环 PD 控制
 *  - 初始化 UART1 接收视觉数据
 *  - 主循环调用 vision_process_data() 解析
 *  - 调用 vision_beam_run() 单环 PD 控制电机
 *  - 串口输出 det/pos/vel/pwm 调试信息
 * ================================================================== */
#define VISION_TEST_ENABLE  (0)     /* 1=启用单环PD测试，0=禁用 */

/* ==================================================================
 *  串级 PID 双环控制模式
 *  - 外环（视觉环PD）：钢珠位置误差 → 目标水管倾角
 *  - 内环（编码器倾角环PD）：目标倾角 - 当前倾角 → PWM频率
 *  - 参考 WHEELTEC 球杆系统架构
 *  - 使用 MS42CG 编码器反馈（不再需要 IMU）
 * ================================================================== */
#define CASCADE_TEST_ENABLE (1)     /* 1=启用双环串级控制，0=禁用 */

/* ==================================================================
 *  编码器测试模式：验证 MS42CG 编码器硬件是否正常
 *  - 每 200ms 打印编码器计数、角度、角速度
 *  - 用于确认 A/B 信号接线正确、QEI 计数方向正确
 * ================================================================== */
#define ENCODER_TEST_ENABLE (0)     /* 1=启用编码器测试，0=禁用 */

/* ==================================================================
 *  VOFA+ JustFloat 二进制协议输出（11通道）
 *  - 供上位机 vofa_tuner.py 直接解析
 *  - 帧结构：11×float32 数据 + 1×float32 帧尾(+inf 0x7F800000) = 48字节
 *  - 通道映射详见 vofa_send_justfloat()
 *  - 设为0则恢复为下方 FireWater 文本输出（5通道）
 * ================================================================== */
#define VOFA_JUSTFLOAT_ENABLE   (1)     /* 1=JustFloat二进制输出，0=FireWater文本输出 */

/* ==================================================================
 *  位置阶跃自动测试模式
 *  - 让 positionTarget 周期在 0 / ±STEP_TEST_STEP_PX 之间跳变，
 *    供 vofa_tuner.py 的 StepDetector 触发阶跃响应捕获并整定PID
 *  - 每阶段保持时长必须 ≥ 脚本 --capture 参数（默认 8s），否则捕获不完整
 *  - 设为0则钢珠始终以中心为目标（传统抗扰模式，不产生阶跃）
 * ================================================================== */
#define STEP_TEST_ENABLE        (1)     /* 1=自动位置阶跃测试，0=固定居中 */
#define STEP_TEST_HOLD_MS       10000   /* 每阶段保持时长 (ms)，需≥脚本capture */
#define STEP_TEST_STEP_PX       80      /* 阶跃幅度 (px) */

/* ==================================================================
 *  对象辨识模式（供 cascade_tuner.py 使用）
 *  - 旁路位置环/速度环，直接给内环（编码器倾角环）周期性倾角阶跃
 *  - 被测对象：x'' = a·θ（双积分），每段倾角恒定 → 球速线性爬升
 *  - 脚本按段拟合 v(t) 斜率 → 每段 a_i = 斜率/θ，求均值得对象增益 a (px/s²/deg)
 *  - 分段序列：0°→±3°→0°→±6°→0°→±9°→0° 循环，正负交替使球在中心往返不滚出视野
 *  - 开启时 STEP_TEST_ENABLE 建议置 0（s_pos_target 不使用）
 * ================================================================== */
#define IDENTIFY_TEST_ENABLE    (1)     /* 1=对象辨识模式，0=正常三环串级 */
#define IDENTIFY_TEST_HOLD_MS   1000    /* 每档倾角保持时长 (ms) */
#define IDENTIFY_TEST_SETTLE_MS 500     /* 归零档保持时长 (ms) */

/* ==================== 位置阶跃状态机（state 编号与 vofa_tuner.py 一致）
 *  0=IDLE, 1=SELECTING, 2=HOLD_CENTER, 3=MOVE_TO_PLUS,
 *  4=MOVE_TO_MINUS, 5=HOLD_MINUS
 * ================================================================ */
static float    s_pos_target = 0.0f;    /* 钢珠目标位置 (px) */
static uint8_t  s_step_state = 2;       /* 当前状态 */
static uint32_t s_step_timer = 0;       /* 当前状态已持续 ms */

static void step_test_run(uint32 elapsed_ms)
{
#if STEP_TEST_ENABLE
    s_step_timer += elapsed_ms;
    if (s_step_timer >= STEP_TEST_HOLD_MS)
    {
        s_step_timer = 0;
        switch (s_step_state)
        {
        case 2:                 /* HOLD_CENTER → 目标 +STEP_TEST_STEP_PX */
            s_step_state = 3;
            s_pos_target =  STEP_TEST_STEP_PX;
            break;
        case 3:                 /* MOVE_TO_PLUS → 目标 -STEP_TEST_STEP_PX */
            s_step_state = 4;
            s_pos_target = -STEP_TEST_STEP_PX;
            break;
        case 4:                 /* MOVE_TO_MINUS → HOLD_MINUS */
            s_step_state = 5;
            s_pos_target = -STEP_TEST_STEP_PX;
            break;
        default:                /* 5 HOLD_MINUS → 回到中心 */
            s_step_state = 2;
            s_pos_target = 0.0f;
            break;
        }
    }
#else
    (void)elapsed_ms;
    s_step_state = 2;
    s_pos_target = 0.0f;
#endif
}

/* ==================== 对象辨识状态机（IDENTIFY_TEST_ENABLE）
 *  0°→±3°→0°→±6°→0°→±9°→0° 循环，直接驱动内环目标倾角
 *  段号 (s_identify_idx) 经 VOFA I10 输出，供 cascade_tuner.py 分段拟合
 * ================================================================ */
#if IDENTIFY_TEST_ENABLE
#define IDENTIFY_SEG_COUNT      13
static const float s_identify_tilt_seq[IDENTIFY_SEG_COUNT] =
{
     0.0f,  3.0f,  0.0f, -3.0f,  0.0f,
     6.0f,  0.0f, -6.0f,  0.0f,
     9.0f,  0.0f, -9.0f,  0.0f,
};
static uint32_t s_identify_idx   = 0;   /* 当前段号 */
static uint32_t s_identify_timer = 0;   /* 当前段已持续 ms */
static float    s_identify_tilt  = 0.0f; /* 当前目标倾角 (deg)，VOFA I3 输出 */

/* 自适应零位补偿：编码器0°可能≠水平（球向右漂），
 * 用完整循环内所有归零段的球平均位置漂移，每循环微调一次 */
static float    s_zero_offset      = 0.0f; /* 零位偏置 (deg)，VOFA I8 输出 */
static float    s_cycle_settle_sum = 0.0f; /* 本循环归零段球位置累积 */
static uint32_t s_cycle_settle_cnt = 0;    /* 本循环归零段样本数 */

static void identify_test_run(uint32 elapsed_ms)
{
    uint32 hold;
    float tilt = s_identify_tilt_seq[s_identify_idx];

    /* 有效倾角 = 序列倾角 + 零位补偿 */
    float tilt_eff = tilt + s_zero_offset;
    s_identify_tilt = tilt_eff;

    /* 直接驱动内环目标（200Hz 内环在 cascade_tick 中自动执行） */
    stab_beam_inner_set_target(tilt_eff);

    /* 归零段采集球位置漂移（球需在自由区，贴墙数据弃用） */
    if (tilt == 0.0f && ball_detected && ball_pos_err > -200.0f && ball_pos_err < 200.0f)
    {
        s_cycle_settle_sum += ball_pos_err;
        s_cycle_settle_cnt++;
    }

    s_identify_timer += elapsed_ms;
    hold = (tilt == 0.0f) ? IDENTIFY_TEST_SETTLE_MS : IDENTIFY_TEST_HOLD_MS;
    if (s_identify_timer >= hold)
    {
        s_identify_timer = 0;
        s_identify_idx = (s_identify_idx + 1) % IDENTIFY_SEG_COUNT;

        /* 段号回绕到0 = 完成一个完整循环 → 用本循环归零段漂移更新零位
         * 球偏右(pos>0) → 右端偏低 → offset 应更负 */
        if (s_identify_idx == 0 && s_cycle_settle_cnt >= 10)
        {
            float avg_pos = s_cycle_settle_sum / (float)s_cycle_settle_cnt;
            float adj = -0.05f * avg_pos;
            if (adj >  1.5f) adj =  1.5f;
            if (adj < -1.5f) adj = -1.5f;
            s_zero_offset += adj;
            if (s_zero_offset >  6.0f) s_zero_offset =  6.0f;
            if (s_zero_offset < -6.0f) s_zero_offset = -6.0f;
        }
        s_cycle_settle_sum = 0.0f;
        s_cycle_settle_cnt = 0;
    }
}
#endif  /* IDENTIFY_TEST_ENABLE */

/* ==================== VOFA+ JustFloat 帧发送 ==================== */
#if VOFA_JUSTFLOAT_ENABLE
#define VOFA_FRAME_CHANNELS    11
#define VOFA_FRAME_TAIL        0x7F800000u    /* float32 +inf 帧尾 */

/* 小端 float32 写入 4 字节（MSPM0 为小端） */
static void vofa_pack_float(uint8 *buf, float val)
{
    uint32 tmp;
    memcpy(&tmp, &val, sizeof(tmp));
    buf[0] = (uint8)(tmp & 0xFF);
    buf[1] = (uint8)((tmp >>  8) & 0xFF);
    buf[2] = (uint8)((tmp >> 16) & 0xFF);
    buf[3] = (uint8)((tmp >> 24) & 0xFF);
}

static void vofa_send_justfloat(void)
{
    uint8  frame[48];
    float  ch[VOFA_FRAME_CHANNELS];

    /* 通道映射（对应 vofa_tuner.py 头部注释的 gimbal.cpp vofa_send） */
#if IDENTIFY_TEST_ENABLE
    ch[0]  = (float)ball_detected;                /* 球检测状态 (1=检测到)，脚本过滤丢球样本 */
    ch[1]  = ball_pos_err;                        /* filteredPosition 滤波后钢珠位置 (px) */
    ch[2]  = ball_pos_vel;                        /* filteredVelocity 滤波后钢珠速度 (px/s) */
    ch[3]  = s_identify_tilt;                     /* 目标倾角 (deg) —— 辨识模式当前档位 */
    ch[4]  = (float)stab_beam_inner_motor_target; /* lastMotorCommand 电机PWM指令 (Hz) */
    ch[5]  = stab_beam_inner_current;             /* currentMotorAngle 编码器实际倾角 (deg) */
    ch[6]  = s_identify_tilt - stab_beam_inner_current;  /* angleError */
    ch[7]  = Encoder_CalcSpeedDps();              /* currentMotorSpeed 编码器角速度 (deg/s) */
    ch[8]  = s_zero_offset;                       /* 自适应零位偏置 (deg)，脚本/监视用 */
    ch[9]  = s_identify_tilt;                     /* 档位倾角 (deg)，便于脚本分段 */
    ch[10] = (float)s_identify_idx;               /* 段号 (0..12)，脚本按此分段 */
#else
    ch[0]  = ball_pos_err;                        /* raw_offset       原始像素偏移 (px) */
    ch[1]  = ball_pos_err;                        /* filteredPosition 滤波后钢珠位置 (px) */
    ch[2]  = ball_pos_vel;                        /* filteredVelocity 滤波后钢珠速度 (px/s) */
    ch[3]  = stab_beam_velocity_target_pitch;     /* targetMotorAngle 目标倾角 (deg) */
    ch[4]  = (float)stab_beam_inner_motor_target; /* lastMotorCommand 电机PWM指令 (Hz) */
    ch[5]  = stab_beam_inner_current;             /* currentMotorAngle 编码器实际倾角 (deg) */
    ch[6]  = stab_beam_velocity_target_pitch - stab_beam_inner_current;  /* angleError */
    ch[7]  = Encoder_CalcSpeedDps();              /* currentMotorSpeed 编码器角速度 (deg/s) */
    ch[8]  = stab_beam_velocity_get_integral();   /* integralOffset   速度环积分项 */
    ch[9]  = s_pos_target;                        /* positionTarget   钢珠目标位置 (px) */
    ch[10] = (float)s_step_state;                 /* state            状态机 (0..5) */
#endif

    for (uint32 i = 0; i < VOFA_FRAME_CHANNELS; i++)
    {
        vofa_pack_float(&frame[i * 4], ch[i]);
    }

    /* 帧尾：float32 +inf = 0x7F800000（小端） */
    uint32 tail = VOFA_FRAME_TAIL;
    frame[44] = (uint8)(tail & 0xFF);
    frame[45] = (uint8)((tail >>  8) & 0xFF);
    frame[46] = (uint8)((tail >> 16) & 0xFF);
    frame[47] = (uint8)((tail >> 24) & 0xFF);

    debug_send_buffer(frame, sizeof(frame));
}
#endif  /* VOFA_JUSTFLOAT_ENABLE */

#if ENCODER_TEST_ENABLE
/* 编码器 5ms 定时回调：更新编码器计数 */
static void encoder_test_tick(uint32_t index, void *param)
{
    (void)index;
    (void)param;
    Encoder_Tick();
}
#endif

#if CASCADE_TEST_ENABLE
/* 双环 5ms 定时回调：更新编码器计数 + 运行内环 */
static void cascade_tick(uint32_t index, void *param)
{
    (void)index;
    (void)param;
    Encoder_Tick();
    stab_beam_inner_run_200Hz();
}
#endif

#if VISION_TEST_ENABLE
#define VISION_PRINT_MS     100     /* 每 100ms 输出一次 */

/* 注意：详细调试信息已移至 vision_beam_run 内部，每收到一帧摄像头数据时打印
 *      此处 vision_test_print 已禁用，避免重复输出 */
static void vision_test_print(void)
{
    /* 调试信息现由 vision_beam_run 中的 [PD] 行输出，此处保留空函数以兼容主循环调用 */
}
#endif

/* ==================================================================
 *  节点3 测试用：步进电机位置控制测试
 *  - 测试 Calculate_Beam_Angle_PWM 位置控制功能
 *  - 循环执行 0°→90°→0°→-90°→0° 的阶梯运动
 *  - 调用周期 5ms（与 motor.c 的 ISR_INTERVAL_S 一致）
 *  - 串口输出当前目标角度和估算角度
 * ================================================================== */
#define MOTOR_TEST_ENABLE   (0)     /* 1=启用电机测试，0=禁用 */

#if MOTOR_TEST_ENABLE
#define MOTOR_TEST_PERIOD_MS   5       /* 调用周期 5ms */
#define MOTOR_TEST_PHASE_MS    3000     /* 每阶段持续 3 秒 */
#define MOTOR_TEST_PRINT_MS    500      /* 每 500ms 输出一次状态 */

/* ===== 直接 PWM 测试：绕过位置控制，直接输出固定频率 =====
 * 阶段序列：停→正转→停→反转→停 循环
 * 用于验证电机硬件和驱动是否正常
 */
static void motor_test_run(void)
{
    static int phase = 0;
    static int timer_ms = 0;
    static int print_ms = 0;

    /* 直接 PWM 频率测试序列 (Hz) */
    static const int s_pwm_test_seq[] = {0, 500, 0, -500, 0};
    static const int s_pwm_test_cnt = sizeof(s_pwm_test_seq) / sizeof(s_pwm_test_seq[0]);

    int pwm = s_pwm_test_seq[phase];
    Set_PWM_Pitch(pwm);

    /* 每 500ms 输出一次状态 */
    print_ms += MOTOR_TEST_PERIOD_MS;
    if (print_ms >= MOTOR_TEST_PRINT_MS)
    {
        print_ms = 0;
        printf("phase=%d, pwm=%d\r\n", phase, pwm);
    }

    /* 阶段切换 */
    timer_ms += MOTOR_TEST_PERIOD_MS;
    if (timer_ms >= MOTOR_TEST_PHASE_MS)
    {
        timer_ms = 0;
        phase = (phase + 1) % s_pwm_test_cnt;
        printf("=== Phase %d: pwm=%d ===\r\n", phase, s_pwm_test_seq[phase]);
    }
}
#endif

int main (void)
{
    clock_init(SYSTEM_CLOCK_80M);   // 时钟配置及系统初始化<务必保留>
    debug_init();					// 调试串口信息初始化
    system_delay_ms(50);

    #if ENCODER_TEST_ENABLE
    /* ===== 编码器测试模式 ===== */
    Motor_init();
    Encoder_Init();
    printf("Encoder test start (PA0=B, PA1=A, TIMG8 QEI)\r\n");

    /* 5ms 定时器用于编码器 Tick */
    pit_ms_init(PIT_TIM_A0, 5, encoder_test_tick, NULL);

    while(true)
    {
        system_delay_ms(200);
        printf("[ENC] count=%ld angle=%.2fdeg speed=%.1fdeg/s\r\n",
               (long)Encoder_GetCount(),
               Encoder_GetAngle(),
               Encoder_CalcSpeedDps());
    }

    #elif MOTOR_TEST_ENABLE
    /* ===== 电机直接 PWM 测试模式 ===== */
    Motor_init();
    printf("Motor direct PWM test start\r\n");

    while(true)
    {
        motor_test_run();
        system_delay_ms(MOTOR_TEST_PERIOD_MS);
    }

    #else
    /* ===== 正常运行模式 ===== */
    Motor_init();
    system_delay_ms(500);

    // 设置中断优先级，确保UART1接收具有最高优先级
    interrupt_set_priority(UART_PRIORITY, 0);        // UART1优先级最高
    interrupt_set_priority(TIMA0_INT_IRQn, 1);       // IMU定时器优先级次之
    interrupt_set_priority(DEBUG_UART_PRIORITY, 0);   // Debug UART优先级最低

    #if VISION_TEST_ENABLE
    /* ===== 单环 PD 控制（参考balance_controller架构） ===== */
    vision_control_init();
    uart1_init();

    vision_set_ball_params(1.0f, 0.3f, 0.05f);
    vision_beam_init(5.0f, 5.0f, 1);             /* Kp=5(位置Hz/px), Kd=5(速度Hz/(px/s)) */
    vision_beam_set_limit(7000);
    vision_beam_set_feedforward(0.0f);           /* Kff=0(无前馈) */

    printf("Position PID control start (no IMU)\r\n");

    #elif CASCADE_TEST_ENABLE
    /* ===== 双环串级 PID 控制（编码器反馈 + 视觉） ===== */
    /* 1. 初始化编码器（提供角度反馈给内环） */
    Encoder_Init();
    Encoder_SetZero();    /* 将当前位置设为零点 */

    /* 2. 初始化视觉数据接收 */
    vision_control_init();
    uart1_init();
    vision_set_ball_params(1.0f, 0.15f, 0.016f); /* 位置缩放1.0, 滤波0.15(加强降噪), 采样16ms(实测~60Hz帧率,原0.02偏大导致速度低估) */

    /* 3. 初始化位置环（P）：位置误差 → 目标球速
     *   Kp_pos=0.6 (1/s): 20px误差→12px/s目标速度
     *   限幅 300px/s: 防止目标速度过大导致速度环饱和 */
    stab_beam_outer_init(0.6f);
    stab_beam_outer_set_limit(300.0f);

    /* 4. 初始化速度环（PI）：目标球速 - 实际球速 → 目标倾角
     *   Kp_vel=0.3 deg/(px/s): 降低P项增益，减少目标倾角频繁翻转
     *   Ki_vel=1.0 deg/px: 积分速度误差消除稳态偏差
     *   限幅 ±10deg: 恢复限幅解除P项饱和死锁(±5度时P项=0.3×19.2=5.76饱和导致Ki被抗积分饱和锁死，稳态偏差卡在32px)
     *   内环Kp=400已能跟踪±10度，无滞后问题
     *   积分分离阈值=30px/s: 系统误差常>10px/s，原10太小导致Ki永远被禁用；加大到30让Ki更早介入
     *   积分限幅=20px: i_term最大=Ki×20=20deg(超过输出限幅10deg，保证I项能主导消稳态偏差) */
    stab_beam_velocity_init(0.3f, 1.0f);
    stab_beam_velocity_set_limit(10.0f);
    stab_beam_velocity_set_dt(0.016f);
    stab_beam_velocity_set_integral_sep(30.0f);
    stab_beam_velocity_set_integral_limit(20.0f);

    /* 5. 初始化内环（编码器倾角环PD）
     *   Kp=400 Hz/deg: 提高响应速度(原200太小，目标±10deg需0.5秒才能跟上导致跟踪滞后)
     *   Kd=3 Hz/(deg/s): 速度阻尼（原10太大，放大编码器噪声导致PWM符号翻转） */
    stab_beam_inner_init(400.0f, 3.0f);
    stab_beam_inner_set_limit(7000);             /* 内环PWM限幅: 4000Hz */
    stab_beam_inner_set_min_pwm(150.0f);         /* 最小PWM频率: 150Hz（原20Hz太低导致stick-slip抖动） */
    stab_beam_inner_set_tolerance(0.1f);         /* 位置容差: 0.1deg（原0.5太大，Ki积分缓慢变化时内环过早停机导致PWM=0） */

    /* 6. 启动5ms定时器（编码器tick + 内环运行，200Hz）
     *    原2ms(500Hz)太快，编码器每count噪声=45deg/s；
     *    5ms(200Hz)下每count噪声=18deg/s，更平稳 */
    pit_ms_init(PIT_TIM_A0, 5, cascade_tick, NULL);

    printf("Cascade PID control start (Encoder + Vision)\r\n");

    #else
    /* 正常模式：需要 IMU */
    IMU_init();
    system_delay_ms(500);
    vision_control_init();
    uart1_init ();
    vision_pid_set(0.06f, 0.0f, 0.0f);
    stab_yaw_init(-600, -0, -0);
    #endif

    while(true)
    {
        // 处理视觉模块数据
        vision_process_data();

        /* 位置阶跃测试状态机（更新钢珠目标位置，周期约4ms） */
        step_test_run(4);

        #if VISION_TEST_ENABLE
        /* 单环 PD 控制 */
        vision_beam_run();
        vision_test_print();

        #elif CASCADE_TEST_ENABLE
#if IDENTIFY_TEST_ENABLE
        /* 对象辨识模式：旁路位置环/速度环，倾角阶跃直接驱动内环
         * 内环（编码器→PWM）仍在 cascade_tick 的 5ms 中断中运行 */
        identify_test_run(4);
#else
        /* 三环串级控制：
         *   位置环 + 速度环：仅在新视觉帧到达时运行（dt=16ms 与视觉帧率匹配）
         *   倾角环：在5ms中断中自动运行（200Hz），将目标倾角与编码器反馈对比 → PWM */
        if (ball_detected)
        {
            /* 位置环 + 速度环：仅在新帧到达时计算，避免 dt 不匹配 */
            if (s_new_frame_arrived)
            {
                s_new_frame_arrived = 0;
                /* 位置环(P)：位置误差 = 当前位置 - 目标位置 → 目标球速 */
                stab_beam_outer_set_pos(ball_pos_err - s_pos_target);
                /* 速度环(PI)：目标球速 - 实际球速 → 目标倾角 */
                stab_beam_velocity_set_vel(stab_beam_outer_get_target_vel(), ball_pos_vel);
            }
            /* 将速度环输出传给内环目标（每帧刷新） */
            stab_beam_inner_set_target(stab_beam_velocity_get_target());
        }
        else
        {
            /* 丢球：目标倾角归零，让管子回到水平 */
            stab_beam_inner_set_target(0.0f);
        }
#endif  /* IDENTIFY_TEST_ENABLE */

        #if VOFA_JUSTFLOAT_ENABLE
        /* VOFA+ JustFloat 二进制协议输出（11通道，供 vofa_tuner.py 解析）：
         *   I0  原始像素偏移        I1  滤波后钢珠位置 (px)
         *   I2  滤波后钢珠速度 (px/s)  I3  目标倾角 (deg)
         *   I4  电机PWM指令 (Hz)     I5  编码器实际倾角 (deg)
         *   I6  角度误差 (deg)       I7  编码器角速度 (deg/s)
         *   I8  速度环积分项          I9  钢珠目标位置 (px)
         *   I10 状态机 (0..5)
         * 在 VOFA+ 中选择 JustFloat 协议即可实时绘图；
         * 也可用 vofa_tuner.py 监听并自动整定 PID */
        static uint32 vofa_send_counter = 0;
        vofa_send_counter++;
        if (vofa_send_counter >= 5)      /* ~4ms主循环 × 5 = 20ms = 50Hz */
        {
            vofa_send_counter = 0;
            vofa_send_justfloat();
        }
        #else
        /* VOFA+ FireWater 格式输出（文本，逗号分隔，\n 结尾）：
         *   通道1: ball_pos_err              小球当前位置 (像素)
         *   通道2: outer_target_vel          位置环输出：目标球速 (px/s)
         *   通道3: velocity_target_pitch     速度环输出：目标倾角 (deg)  ← 内环目标
         *   通道4: inner_motor_target        内环输出：PWM频率 (Hz)
         *   通道5: inner_current             编码器实际倾角 (deg)        ← 内环反馈
         * 对比通道3和通道5可判断内环是否跟踪到位（若通道5远小于通道3说明内环跟不上）
         * 在 VOFA+ 中选择 FireWater 协议即可实时绘图 */
        printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",
               ball_pos_err,
               stab_beam_outer_target_vel,
               stab_beam_velocity_target_pitch,
               stab_beam_inner_motor_target,
               stab_beam_inner_current);
        #endif

        #else
        // 正常模式：目标角度由视觉数据设置
        float target_angle = vision_get_target_angle();
        stab_yaw_set_target(target_angle);

        /* 节点2 测试：普通串口输出 pitch_angle, pitch_gyro, T_M */
        #if VOFA_TEST_ENABLE
        vofa_send_pitch_data();
        #endif
        #endif

        system_delay_ms(4);
    }
    #endif
}
void uart1_rx_interrupt_handler (uint32 state, void *ptr);


// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看 逐飞助手 打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看 逐飞助手 设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：串口输出 imu660ra init error.
//      检查imu660ra的接线是否正确
//      检查imu660ra的模块是不是坏了
//      给信号线加上拉看看
// 问题4：imu660ra 数值异常
//      看看是不是线松了 或者信号线被短路了
//      可能模块部分受损