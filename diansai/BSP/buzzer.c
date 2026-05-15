#include "buzzer.h"

static BuzzerMode_t g_buzzerMode = BUZZER_MODE_OFF;
static uint32_t g_buzzerDuration = 0;
static uint32_t g_buzzerStartTime = 0;

// 蜂鸣器触发标志
volatile uint8_t g_buzzer_left_branch_trigger = 0;
volatile uint8_t g_buzzer_right_branch_trigger = 0;
volatile uint8_t g_buzzer_crossroad_trigger = 0;
volatile uint8_t g_buzzer_stop_trigger = 0;

void Buzzer_Init(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

void Buzzer_SetState(BuzzerState_t state)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Buzzer_ShortBeep(void)
{
    g_buzzerMode = BUZZER_MODE_SHORT;
    g_buzzerDuration = 20;
    g_buzzerStartTime = HAL_GetTick();
    Buzzer_SetState(BUZZER_ON);
}

void Buzzer_LongBeep(uint32_t duration_ms)
{
    g_buzzerMode = BUZZER_MODE_LONG;
    g_buzzerDuration = duration_ms;
    g_buzzerStartTime = HAL_GetTick();
    Buzzer_SetState(BUZZER_ON);
}

void StartBuzzerTask(void *argument)
{
		// 启动时清除所有触发标志，避免上电误触发
    g_buzzer_left_branch_trigger = 0;
    g_buzzer_right_branch_trigger = 0;
    g_buzzer_crossroad_trigger = 0;
    g_buzzer_stop_trigger = 0;
    
    // 清除蜂鸣器状态
    g_buzzerMode = BUZZER_MODE_OFF;
    Buzzer_SetState(BUZZER_OFF);
	
    Buzzer_Init();
    
    for(;;)
    {
        // 检测触发标志
        if(g_buzzer_left_branch_trigger)
        {
            Buzzer_ShortBeep();
            g_buzzer_left_branch_trigger = 0;
        }
        
        if(g_buzzer_right_branch_trigger)
        {
            Buzzer_ShortBeep();
            g_buzzer_right_branch_trigger = 0;
        }
        
        if(g_buzzer_crossroad_trigger)
        {
            Buzzer_ShortBeep();
            g_buzzer_crossroad_trigger = 0;
        }
        
        if(g_buzzer_stop_trigger)
        {
            Buzzer_LongBeep(1000);
            g_buzzer_stop_trigger = 0;
        }
        
        // 处理蜂鸣器计时
        if(g_buzzerMode != BUZZER_MODE_OFF)
        {
            uint32_t elapsed = HAL_GetTick() - g_buzzerStartTime;
            
            if(elapsed >= g_buzzerDuration)
            {
                Buzzer_SetState(BUZZER_OFF);
                g_buzzerMode = BUZZER_MODE_OFF;
            }
        }
        
        osDelay(10);
    }
}

void Buzzer_Task_Init(void)
{
    const osThreadAttr_t BuzzerTask_attributes = {
        .name = "BuzzerTask",
        .stack_size = 128 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    osThreadNew(StartBuzzerTask, NULL, &BuzzerTask_attributes);
}