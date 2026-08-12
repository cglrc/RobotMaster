#include "motor.h"    
#include <math.h>     
#include <stdlib.h>   

// --- Ӳ������
#define STEPS_PER_REVOLUTION 200    // �����������: ���תһȦ(360��)��Ҫ�Ĳ�����1.8�Ȳ����� = 360/1.8 = 200����
#define MICROSTEPS 32               // ����������: ���������õ�ϸ������ϸ��Խ�ߣ�ת��Խƽ�������Ե�Ƭ������Ҫ��ҲԽ�ߡ�
#define PWM_CLOCK_FREQ 10000000    

// --- ϵͳ���� ---
#define ISR_INTERVAL_S 0.005f       // �жϵ�������(��)��

// --- �㶨ת�ٲ��� (�޸Ĳ���) ---
#define CONSTANT_SPEED_RPM 10       // Ŀ��㶨ת�٣����ԭ���Ŀ������л�
#define START_SPEED_RPM 3           // �����ٶȣ����ⶪ��
#define ACCELERATION_TIME_MS 200    // ����ʱ��200ms
#define ACCELERATION_STEPS ((int)(ACCELERATION_TIME_MS / (ISR_INTERVAL_S * 1000))) // ������Ҫ���жϴ���
    

#define EN_MA_PIN    A13
#define EN_MB_PIN    A14
#define DIR_A_PIN    A16
#define DIR_B_PIN    A17
#define PWM_Yaw_CHAN   PWM_TIM_G6_CH1_B3   // Yaw��
#define PWM_Pitch_CHAN   PWM_TIM_G7_CH1_A7   // Pitch��

/* ռ�ձȹ̶� 50 %������ɵ����ٴ��� */
#define MOTOR_DUTY   (PWM_DUTY_MAX / 2)

// ==============================================================================
//  5. �ײ�Ӳ����������
// ==============================================================================

/**
 * @brief ��������ײ��������� - �Ľ��棨����Ƶ��ʹ��/ʧ�ܣ�
 * @param L_Target ����(A)�Ŀ���ָ���Calculate_MA_Angle_PWM()���ɡ�
 * @param R_Target �ҵ��(B)�Ŀ���ָ���Calculate_MB_Angle_PWM()���ɡ�
 * @details
 *  - ����ָ��ֵ�����������õ������
 *  - ����ָ��ֵ�ľ���ֵ����PWMƵ�ʣ�������ٶȣ���
 *  - ָ��ֵΪ0ʱ��ֹͣPWM���������ʹ�ܣ�ͨ������ռ�ձ�Ϊ0����ȫֹͣ���塣
 */

void Motor_init(void)
{   
    gpio_init(A13, GPO, 1, GPO_PUSH_PULL);  //EN_MA_PIN ��A13��
    gpio_init(A14, GPO, 1, GPO_PUSH_PULL);  //EN_MB_PIN ��A14��
    gpio_init(A16, GPO, 1, GPO_PUSH_PULL);  //DIR_A_PIN ��A16��
    gpio_init(A17, GPO, 1, GPO_PUSH_PULL);  //DIR_B_PIN ��A17��
    pwm_init (PWM_TIM_G6_CH1_B3, 1, 50);    //PWM_0_INST��B3��
    pwm_init (PWM_TIM_G7_CH1_A7, 1, 50);    //PWM_1_INST��A7��
}


void Set_PWM_Yaw(int L_Target)
{
    /* -------- Yaw��� -------- */
    if (L_Target > 0) {                // ��ת
        gpio_set_level(EN_MA_PIN, 1);  // ʹ��
        gpio_set_level(DIR_A_PIN, 1);  // ����
        pwm_set_frequency(PWM_Yaw_CHAN, (uint32)L_Target);
        pwm_set_duty(PWM_Yaw_CHAN, MOTOR_DUTY);
    }
    else if (L_Target < 0) {           // ��ת
        gpio_set_level(EN_MA_PIN, 1);
        gpio_set_level(DIR_A_PIN, 0);
        pwm_set_frequency(PWM_Yaw_CHAN, (uint32)(-L_Target));
        pwm_set_duty(PWM_Yaw_CHAN, MOTOR_DUTY);
    }
    else {                             // ֹͣ
        gpio_set_level(EN_MA_PIN, 1);
        pwm_set_duty(PWM_Yaw_CHAN, 50);   // ռ�ձ� 0 �������
        // Ƶ�ʿɱ��֣�Ҳ�������һ��ֵ�����ﲻ�ٵ���
    }
}

void Set_PWM_Pitch(int R_Target)    
{    /* -------- Pitch��� -------- */
    if (R_Target > 0) {
        gpio_set_level(EN_MB_PIN, 1);
        gpio_set_level(DIR_B_PIN, 1);
        pwm_set_frequency(PWM_Pitch_CHAN, (uint32)R_Target);
        pwm_set_duty(PWM_Pitch_CHAN, MOTOR_DUTY);
    }
    else if (R_Target < 0) {
        gpio_set_level(EN_MB_PIN, 1);
        gpio_set_level(DIR_B_PIN, 0);
        pwm_set_frequency(PWM_Pitch_CHAN, (uint32)(-R_Target));
        pwm_set_duty(PWM_Pitch_CHAN, MOTOR_DUTY);
    }
    else {
        gpio_set_level(EN_MB_PIN, 1);
        pwm_set_duty(PWM_Pitch_CHAN, 50);
    }
}


//// ȫ�ֳ�����Ԥ�ȼ����"ÿתһ����Ҫ���ٸ�����"���������ֱ��ʹ�á�
//// ��ʽ: (�ܲ���/Ȧ * ϸ����) / 360��/Ȧ
//const float PULSES_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360.0f;

///**
// * @brief �� RPM (ÿ����ת��) ת��ΪPWM��ʱ����Ҫ�����ڼ���ֵ(LoadValue)
// * @param rpm Ŀ��ת��
// * @return int ���ظ�PWM��ʱ�����õ�����ֵ����ֵԽ��PWMƵ��Խ�ͣ����ת��Խ����
// */
//static int rpm_to_pwm_period(int rpm)
//{
//    // ����ٶ�Ϊ0�����������壬ֱ�ӷ���0
//    if (rpm <= 0) return 0;

//    // 1. �� RPM (ת/����) ת��Ϊ RPS (ת/��)
//    float rps = (float)rpm / 60.0f;
//    // 2. �� RPS (ת/��) ת��Ϊ Steps-per-second (������/��)
//    float steps_per_second = rps * STEPS_PER_REVOLUTION * MICROSTEPS;
//    // 3. ������/�� ����PWM��Ƶ��(Hz)
//    // 4. ����PWMƵ�ʺ�ʱ��Ƶ�ʣ����㶨ʱ������ֵ����ʽ: Period = ClockFreq / PwmFreq - 1
//    return (int)((PWM_CLOCK_FREQ / steps_per_second) - 1);
//}

///**
// * @brief ������A(��)Ϊ����Ŀ��Ƕ�����Ҫ��PWM����ֵ - �㶨ת�ٰ汾�������٣�
// * @param target_angle ���A��Ŀ����ԽǶ� (��λ: ��)
// * @return int �����ŵ�PWM����ֵ��
// *         - ����: ��ʾ��ת����ֵΪPWM����ֵ��
// *         - ����: ��ʾ��ת����ֵ�ľ���ֵΪPWM����ֵ��
// *         - 0:    ��ʾֹͣ��
// */
//int Calculate_MA_Angle_PWM(float target_angle)
//{
//    // ���A��˽��״̬������ʹ��'static'�ؼ��֣�ʹ�����ǵ�ֵ�ں�����ε���֮����Ա��֡�
//    static float s_ma_current_target_angle = -999.0f; // ��ʼ��Ϊ����ֵ��������������
//    static float s_ma_current_assumed_angle = 0.0f;   // �ڲ�׷�ٵĵ����ǰ�ٶ�λ�ã�α�ջ���
//    static long  s_ma_pulses_remaining = 0;           // �����˶���ʣ�����������Ҫ����
//    static int   s_ma_direction = 0;                  // �˶�����: 1=��, -1=��, 0=ͣ
//    static float s_ma_pulse_accumulator = 0.0f;       // �����ۼ��������ڲ�����������ľ�����ʧ
//    static int   s_ma_acceleration_counter = 0;       // ���ټ�����
//    static int   s_ma_first_call = 1;                 // �״ε��ñ�־

//    // �״ε������⴦�� - ����������
//    if (s_ma_first_call)
//    {
//        s_ma_first_call = 0;
//        s_ma_current_target_angle = target_angle;
//        s_ma_current_assumed_angle = target_angle;  // ��������ʱ�Ѿ���Ŀ��λ��
//        return 0;  // �״ε��ò������κ��˶�
//    }

//    // ���� 1: ���Ŀ��Ƕ��Ƿ����仯��������ˣ��͹滮һ���µ��˶���
//    if (target_angle != s_ma_current_target_angle)
//    {
//        s_ma_current_target_angle = target_angle; // ������Ŀ��
//        // ����һ���˶����յ㣬���㵽��Ŀ����Ҫ�ƶ��ĽǶ�
//        float angle_to_move = target_angle - s_ma_current_assumed_angle;
//        
//        // ����ƶ��Ƕ�̫С��С��0.1�ȣ���ֱ����ת����ִ���˶�
//        if (fabs(angle_to_move) < 0.1f)
//        {
//            s_ma_current_assumed_angle = target_angle;
//            s_ma_pulses_remaining = 0;
//            s_ma_direction = 0;
//            return 0;
//        }
//        
//        // ������������
//        s_ma_pulses_remaining = (long)(fabs(angle_to_move) * PULSES_PER_DEGREE);
//        // ���������ۼ����ͼ��ټ�����
//        s_ma_pulse_accumulator = 0.0f;
//        s_ma_acceleration_counter = 0;
//        // �����ƶ��Ƕȵ�������ȷ������
//        s_ma_direction = (angle_to_move > 0) ? 1 : ((angle_to_move < 0) ? -1 : 0);
//    }

//    // ���� 2: �������������Ҫ���ͣ���ִ���˶������߼���
//    if (s_ma_pulses_remaining > 0)
//    {
//        // ���㵱ǰӦ��ʹ�õ�ת�٣�ƽ�����٣�
//        int current_speed_rpm;
//        if (s_ma_acceleration_counter < ACCELERATION_STEPS)
//        {
//            // ���ٽ׶Σ��������ٶ����Լ��ٵ�Ŀ���ٶ�
//            float acceleration_progress = (float)s_ma_acceleration_counter / ACCELERATION_STEPS;
//            current_speed_rpm = (int)(START_SPEED_RPM + (CONSTANT_SPEED_RPM - START_SPEED_RPM) * acceleration_progress);
//            s_ma_acceleration_counter++;
//        }
//        else
//        {
//            // ���ٽ׶�
//            current_speed_rpm = CONSTANT_SPEED_RPM;
//        }
//        
//        // ���ݵ�ǰ�ٶȣ�����������ж�������(5ms)��������Ӧ���Ͷ��ٸ����壨������С����
//        float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
//        float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
//        
//        // ʹ���ۼ�������ÿ�μ������С�������ۼ�����
//        s_ma_pulse_accumulator += pulses_to_send_float;
//        // ���ۼ�����ȡ������������Ϊ�����ж�ʵ��Ҫ���͵�������
//        long pulses_this_interval = (long)s_ma_pulse_accumulator;
//        
//        if (pulses_this_interval > 0) {
//            s_ma_pulse_accumulator -= pulses_this_interval; // �ۼ�����ȥ�ѷ��͵Ĳ��֣�����С��
//            s_ma_pulses_remaining -= pulses_this_interval;  // ��ʣ��������Ҳ��ȥ��Ӧ����
//        }

//        // ���� 3: ����˶��Ƿ����
//        if (s_ma_pulses_remaining <= 0)
//        {
//            s_ma_pulses_remaining = 0; // ���㣬��ֹ����
//            s_ma_direction = 0;        // ֹͣ
//            s_ma_acceleration_counter = 0; // ���ü��ټ�����
//            // ����Ŀ�꣡���µ���ļٶ�λ��Ϊ��ǰĿ��λ�á�����α�ջ��Ĺؼ���
//            s_ma_current_assumed_angle = s_ma_current_target_angle; 
//            return 0; // ����0������Set_PWM����ֹͣ���
//        }
//        
//        // ����˶�δ��ɣ����ش�������ٶ�ָ��
//        return s_ma_direction * rpm_to_pwm_period(current_speed_rpm);
//    }
//    
//    // ���û���˶����ѵ��������ָ�������0
//    return 0;
//}

///**
// * @brief ������B(��)Ϊ����Ŀ��Ƕ�����Ҫ��PWM����ֵ - �㶨ת�ٰ汾�������٣�
// * @note  �˺����߼���Calculate_MA_Angle_PWM��ȫ��ͬ����ʹ�ö�����static������
// *        �Ӷ�ʵ�ֶԵ��B�Ķ���״̬׷�ٺͿ��ơ�
// */
//int Calculate_MB_Angle_PWM(float target_angle)
//{
//    // ���B��˽��״̬���� (����A����ȫ����)
//    static float s_mb_current_target_angle = -999.0f; // ��ʼ��Ϊ����ֵ
//    static float s_mb_current_assumed_angle = 0.0f;
//    static long s_mb_pulses_remaining = 0;
//    static int s_mb_direction = 0;
//    static float s_mb_pulse_accumulator = 0.0f;
//    static int s_mb_acceleration_counter = 0;
//    static int s_mb_first_call = 1;                    // �״ε��ñ�־

//    // �״ε������⴦�� - ����������
//    if (s_mb_first_call)
//    {
//        s_mb_first_call = 0;
//        s_mb_current_target_angle = target_angle;
//        s_mb_current_assumed_angle = target_angle;  // ��������ʱ�Ѿ���Ŀ��λ��
//        return 0;  // �״ε��ò������κ��˶�
//    }

//    if (target_angle != s_mb_current_target_angle)
//    {
//        s_mb_current_target_angle = target_angle;
//        float angle_to_move = target_angle - s_mb_current_assumed_angle;
//        
//        // ����ƶ��Ƕ�̫С��ֱ����ת
//        if (fabs(angle_to_move) < 0.1f)
//        {
//            s_mb_current_assumed_angle = target_angle;
//            s_mb_pulses_remaining = 0;
//            s_mb_direction = 0;
//            return 0;
//        }
//        
//        s_mb_pulses_remaining = (long)(fabs(angle_to_move) * PULSES_PER_DEGREE);
//        s_mb_pulse_accumulator = 0.0f;
//        s_mb_acceleration_counter = 0;
//        s_mb_direction = (angle_to_move > 0) ? 1 : ((angle_to_move < 0) ? -1 : 0);
//    }

//    if (s_mb_pulses_remaining > 0)
//    {
//        // ���㵱ǰӦ��ʹ�õ�ת�٣�ƽ�����٣�
//        int current_speed_rpm;
//        if (s_mb_acceleration_counter < ACCELERATION_STEPS)
//        {
//            // ���ٽ׶Σ��������ٶ����Լ��ٵ�Ŀ���ٶ�
//            float acceleration_progress = (float)s_mb_acceleration_counter / ACCELERATION_STEPS;
//            current_speed_rpm = (int)(START_SPEED_RPM + (CONSTANT_SPEED_RPM - START_SPEED_RPM) * acceleration_progress);
//            s_mb_acceleration_counter++;
//        }
//        else
//        {
//            // ���ٽ׶�
//            current_speed_rpm = CONSTANT_SPEED_RPM;
//        }
//        
//        float steps_per_second = (current_speed_rpm * (STEPS_PER_REVOLUTION * MICROSTEPS)) / 60.0f;
//        float pulses_to_send_float = steps_per_second * ISR_INTERVAL_S;
//        
//        s_mb_pulse_accumulator += pulses_to_send_float;
//        long pulses_this_interval = (long)s_mb_pulse_accumulator;
//        
//        if (pulses_this_interval > 0) {
//            s_mb_pulse_accumulator -= pulses_this_interval;
//            s_mb_pulses_remaining -= pulses_this_interval;
//        }

//        if (s_mb_pulses_remaining <= 0)
//        {
//            s_mb_pulses_remaining = 0;
//            s_mb_direction = 0;
//            s_mb_acceleration_counter = 0; // ���ü��ټ�����
//            s_mb_current_assumed_angle = s_mb_current_target_angle;
//            return 0;
//        }
//        
//        return s_mb_direction * rpm_to_pwm_period(current_speed_rpm);
//    }
//    
//    return 0;
//}

/* ==================================================================
 *  钢珠平衡任务新增：Pitch 轴位置控制实现
 *  - 使用 Pitch 轴 PWM (PWM_TIM_G7_CH1_A7) 控制升降电机
 *  - 输入：电机目标转角 (deg)
 *  - 内部基于步数累加估算当前转角（开环步进）
 *  - 不做曲柄连杆几何映射，直接恒等映射
 * ================================================================== */

/* 每度对应的脉冲数（步进电机 200 步/圈 × 32 细分 / 360 度） */
static const float PULSES_PER_DEGREE = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360.0f;

/* 曲柄连杆几何参数（仅存储，节点3 不使用映射，机械定型后补充） */
static crank_rod_params_t s_crank_rod_params = {0};

/* 位置控制内部状态 */
static float s_beam_target_angle   = -999.0f;   /* 当前目标转角 (deg) */
static float s_beam_assumed_angle  = 0.0f;       /* 当前估算转角 (deg) */
static long  s_beam_pulses_remain = 0;           /* 剩余脉冲数 */
static int   s_beam_direction     = 0;           /* 运动方向：1=正，-1=反，0=停 */
static float s_beam_pulse_acc     = 0.0f;        /* 小步累加器 */
static int   s_beam_accel_cnt     = 0;           /* 加减速计数器 */
static int   s_beam_first_call    = 1;           /* 首次调用标志 */

/* RPM 转 PWM 频率值（Hz）
 * 注：驱动器双边沿触发，每个 PWM 周期产生 2 个微步，
 *     所以 PWM 频率 = 目标微步率 / 2 */
static int rpm_to_pwm_freq(int rpm)
{
    if (rpm <= 0) return 0;
    float rps = (float)rpm / 60.0f;
    float steps_per_second = rps * STEPS_PER_REVOLUTION * MICROSTEPS;
    return (int)(steps_per_second / 2.0f);   /* 双边沿补偿 */
}

/* 配置曲柄连杆几何参数（仅存储，节点3 不使用映射） */
void Beam_Set_CrankRod_Params(const crank_rod_params_t *params)
{
    if (params) s_crank_rod_params = *params;
}

const crank_rod_params_t* Beam_Get_CrankRod_Params(void)
{
    return &s_crank_rod_params;
}

/* 不做几何映射：直接返回输入值（机械定型后可改为解析公式或查表） */
float Beam_BeamAngle_To_MotorAngle(float beam_angle_deg)
{
    return beam_angle_deg;
}

/* 位置控制状态初始化（重置步数累加器等） */
void Motor_Control_Init(void)
{
    s_beam_target_angle   = -999.0f;
    s_beam_assumed_angle  = 0.0f;
    s_beam_pulses_remain  = 0;
    s_beam_direction      = 0;
    s_beam_pulse_acc      = 0.0f;
    s_beam_accel_cnt      = 0;
    s_beam_first_call     = 1;
    Set_PWM_Pitch(0);
}

/* 获取电机当前估算转角（基于步数累加的开环估算，deg） */
float Beam_Get_Motor_CurrentAngle(void)
{
    return s_beam_assumed_angle;
}

/* 位置控制急停：立即停转并清空剩余步数 */
void Beam_Motor_Stop(void)
{
    s_beam_pulses_remain = 0;
    s_beam_direction     = 0;
    s_beam_accel_cnt     = 0;
    Set_PWM_Pitch(0);
}

/**
 * @brief 位置控制核心：根据目标电机转角计算并设置 PWM
 * @param target_motor_angle_deg 目标电机转角 (deg)
 * @return int 当前实际发送的 PWM 频率值（带方向符号），0 表示已停转
 * @note 应在 5ms 中断中调用（与 stab_beam_inner_run_200Hz 同周期）
 */
int Calculate_Beam_Angle_PWM(float target_motor_angle_deg)
{
    /* 首次调用：初始化当前角度为目标值，不运动 */
    if (s_beam_first_call)
    {
        s_beam_first_call    = 0;
        s_beam_target_angle  = target_motor_angle_deg;
        s_beam_assumed_angle = target_motor_angle_deg;
        Set_PWM_Pitch(0);
        return 0;
    }

    /* 目标变化时，规划新运动 */
    if (target_motor_angle_deg != s_beam_target_angle)
    {
        s_beam_target_angle = target_motor_angle_deg;
        float angle_to_move = target_motor_angle_deg - s_beam_assumed_angle;

        /* 移动量过小：直接到位 */
        if (fabsf(angle_to_move) < 0.1f)
        {
            s_beam_assumed_angle = target_motor_angle_deg;
            s_beam_pulses_remain = 0;
            s_beam_direction    = 0;
            Set_PWM_Pitch(0);
            return 0;
        }

        /* 计算剩余脉冲数与方向 */
        s_beam_pulses_remain = (long)(fabsf(angle_to_move) * PULSES_PER_DEGREE);
        s_beam_pulse_acc     = 0.0f;
        s_beam_accel_cnt     = 0;
        s_beam_direction     = (angle_to_move > 0.0f) ? 1 : -1;
    }

    /* 执行运动 */
    if (s_beam_pulses_remain > 0)
    {
        /* 加减速曲线 */
        int current_rpm;
        if (s_beam_accel_cnt < ACCELERATION_STEPS)
        {
            float progress = (float)s_beam_accel_cnt / (float)ACCELERATION_STEPS;
            current_rpm = (int)(START_SPEED_RPM + (CONSTANT_SPEED_RPM - START_SPEED_RPM) * progress);
            s_beam_accel_cnt++;
        }
        else
        {
            current_rpm = CONSTANT_SPEED_RPM;
        }

        /* 本周期应发步数（小数累加） */
        float steps_per_sec = (current_rpm * STEPS_PER_REVOLUTION * MICROSTEPS) / 60.0f;
        float pulses_this   = steps_per_sec * ISR_INTERVAL_S;
        s_beam_pulse_acc   += pulses_this;
        long pulses_send    = (long)s_beam_pulse_acc;

        if (pulses_send > 0)
        {
            s_beam_pulse_acc     -= (float)pulses_send;
            s_beam_pulses_remain -= pulses_send;
            /* 同步更新估算角度 */
            s_beam_assumed_angle += (float)pulses_send / PULSES_PER_DEGREE * (float)s_beam_direction;
        }

        /* 运动完成检查 */
        if (s_beam_pulses_remain <= 0)
        {
            s_beam_pulses_remain = 0;
            s_beam_direction     = 0;
            s_beam_accel_cnt     = 0;
            s_beam_assumed_angle = s_beam_target_angle;
            Set_PWM_Pitch(0);
            return 0;
        }

        /* 设置 PWM（方向 × 频率） */
        int pwm_freq = rpm_to_pwm_freq(current_rpm);
        int pwm_out  = s_beam_direction * pwm_freq;
        Set_PWM_Pitch(pwm_out);
        return pwm_out;
    }

    /* 无运动任务：保持停转 */
    Set_PWM_Pitch(0);
    return 0;
}


