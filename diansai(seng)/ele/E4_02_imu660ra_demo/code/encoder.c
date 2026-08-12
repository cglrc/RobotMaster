/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   MS42CG 磁编码器 A/B 增量信号驱动（TIMG8 硬件 QEI）
  ******************************************************************************
  * 移植自 WHEELTEC MSPM0G3507 编码器闭环例程，适配逐飞库框架
  * 硬件 QEI 4 倍频 → 4000 counts/圈，5ms 周期读取扩展为 32 位多圈计数
  ******************************************************************************
  */
#include "encoder.h"
#include "zf_common_headfile.h"

/* ---- 硬件寄存器 ---- */
#define ENCODER_QEI_INST   TIMG8

/* IOMUX 引脚功能选择：使用 SDK 头文件 mspm0g350x.h 中已定义的
 * IOMUX_PINCM1_PF_TIMG8_CCP1 (PA0→B相) 和 IOMUX_PINCM2_PF_TIMG8_CCP0 (PA1→A相)，
 * 此处不再重复定义，避免 -Wmacro-redefined 警告 */

/* ---- 编码器状态 ---- */
typedef struct {
    volatile int32_t accumulated;   /* 32位多圈累计计数 */
    int32_t zero;                   /* 零点偏移 */
    uint16_t last_hw_count;         /* 上次读取的硬件16位计数 */
    int32_t speed_last;             /* 上次计数（用于速度计算） */
    uint8_t initialized;            /* 初始化标志 */
} EncoderState_t;

static EncoderState_t s_enc;

/* 速度低通滤波系数 (0~1，越小越平滑，延迟越大)
 * 编码器分辨率 0.09deg/count，5ms 周期下每count噪声=18deg/s
 * 滤波系数 0.3 可有效抑制量化噪声 */
static float s_speed_filter_alpha = 0.3f;
static float s_speed_filtered = 0.0f;

/* ---- 底层初始化（不依赖 SysConfig） ---- */
static void Encoder_InitQEI(void)
{
    /* 1. 先复位 TIMG8，再使能电源（顺序很重要！reset会清除PWREN） */
    DL_TimerG_reset(ENCODER_QEI_INST);
    DL_TimerG_enablePower(ENCODER_QEI_INST);
    system_delay_ms(5);

    /* 2. 配置引脚：PA0 = TIMG8_CCP1, PA1 = TIMG8_CCP0 */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM2,
        IOMUX_PINCM2_PF_TIMG8_CCP0);  /* PA1 = encoder A */
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM1,
        IOMUX_PINCM1_PF_TIMG8_CCP1);  /* PA0 = encoder B */

    /* 3. 配置时钟：BUSCLK，不分频 */
    DL_TimerG_ClockConfig clock_config = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0U
    };
    DL_TimerG_setClockConfig(ENCODER_QEI_INST, &clock_config);

    /* 4. 配置 QEI 模式：2 输入（A/B），不反相 */
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(ENCODER_QEI_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);

    /* 5. 设置计数范围和初始值 */
    DL_TimerG_setLoadValue(ENCODER_QEI_INST, 0xFFFFU);
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);

    /* 6. 使能时钟并启动 */
    DL_TimerG_enableClock(ENCODER_QEI_INST);
    DL_TimerG_startCounter(ENCODER_QEI_INST);
}

void Encoder_Init(void)
{
    Encoder_InitQEI();

    /* 二次确保计数器启动（参考WHEELTEC例程） */
    DL_TimerG_setTimerCount(ENCODER_QEI_INST, 0U);
    DL_TimerG_startCounter(ENCODER_QEI_INST);

    s_enc.accumulated   = 0;
    s_enc.zero          = 0;
    /* 读取当前硬件计数作为初始值，避免首次tick的delta计算错误 */
    s_enc.last_hw_count = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    s_enc.speed_last    = 0;
    s_speed_filtered    = 0.0f;   /* 重置速度滤波器 */
    s_enc.initialized   = 1;
}

/**
  * 5ms 周期调用：
  * 读取 TIMG8 的 16 位硬件计数器，将差值扩展累加到 32 位多圈计数。
  *
  * 原理：
  *   硬件 QEI 计数器是 16 位无符号（0~65535），UP/DOWN 计数模式。
  *   当计数器从 0 减到 65535 时实际是 -1（反转 1 步），
  *   所以 (int16_t)(new - old) 能正确算出 ±N 步的差值。
  *   累加到 32 位 accumulated 即可支持多圈计数。
  */
void Encoder_Tick(void)
{
    if (!s_enc.initialized) return;

    uint16_t now = (uint16_t)DL_TimerG_getTimerCount(ENCODER_QEI_INST);
    int16_t delta = (int16_t)(now - s_enc.last_hw_count);
    s_enc.last_hw_count = now;
    s_enc.accumulated += (int32_t)delta;
}

int32_t Encoder_GetCount(void)
{
    return -(s_enc.accumulated - s_enc.zero);   /* 取反：使电机正转时角度为正 */
}

float Encoder_GetAngle(void)
{
    return (float)Encoder_GetCount() * 360.0f / (float)ENCODER_COUNTS_PER_REV;
}

float Encoder_CalcSpeedDps(void)
{
    int32_t now = Encoder_GetCount();
    int32_t delta = now - s_enc.speed_last;
    s_enc.speed_last = now;
    /* 调用周期 5ms = 0.005s（与 main.c 中 pit_ms_init(PIT_TIM_A0, 5, ...) 一致） */
    float raw_speed = (float)delta * 360.0f / ((float)ENCODER_COUNTS_PER_REV * 0.005f);
    /* 一阶低通滤波：抑制编码器量化噪声，避免 Kd 项放大抖动 */
    s_speed_filtered = s_speed_filter_alpha * raw_speed + (1.0f - s_speed_filter_alpha) * s_speed_filtered;
    return s_speed_filtered;
}

void Encoder_SetZero(void)
{
    s_enc.zero = s_enc.accumulated;
    s_enc.speed_last = 0;
    s_speed_filtered = 0.0f;   /* 重置速度滤波器 */
}
