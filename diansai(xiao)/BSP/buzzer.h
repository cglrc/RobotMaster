#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"
#include "cmsis_os.h"

#ifndef BUZZER_GPIO_Port
#define BUZZER_GPIO_Port  GPIOB
#define BUZZER_Pin        GPIO_PIN_0
#endif

typedef enum {
    BUZZER_OFF = 0,
    BUZZER_ON = 1
} BuzzerState_t;

typedef enum {
    BUZZER_MODE_SHORT,      // 短叫
    BUZZER_MODE_LONG,       // 长鸣
    BUZZER_MODE_OFF         // 关闭
} BuzzerMode_t;

void Buzzer_Init(void);
void Buzzer_SetState(BuzzerState_t state);
void Buzzer_ShortBeep(void);
void Buzzer_LongBeep(uint32_t duration_ms);
void Buzzer_Task_Init(void);
void StartBuzzerTask(void *argument);

// 蜂鸣器触发标志（由其他任务设置）
extern volatile uint8_t g_buzzer_left_branch_trigger;
extern volatile uint8_t g_buzzer_right_branch_trigger;
extern volatile uint8_t g_buzzer_crossroad_trigger;
extern volatile uint8_t g_buzzer_stop_trigger;

#endif