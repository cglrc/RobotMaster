#include "linewalking.h"
#include "bsp_motor.h"  // 包含 Motion_Set_Pwm 函数
#include "buzzer.h"
#define V_MAX 99
#define V_MIN 60
// 任务句柄
osThreadId_t LineWalkingTaskHandle;

// 巡线状态变量
static LineWalking_State_t g_lineState = {1, 1, 0};

// 路口检测计数器（用于防抖）
static uint8_t junction_counter = 0;
#define JUNCTION_THRESHOLD 2

// 系统运行状态
static SystemState_t g_systemState = SYS_STOPPED;
// 启动按钮防抖变量
static uint8_t button_last_state = BUTTON_RELEASED;
static uint8_t button_stable_state = BUTTON_RELEASED;
static uint8_t button_debounce_counter = 0;
#define BUTTON_DEBOUNCE_DELAY 5
// 分支计数全局变量
volatile int left_branch_count = 0;
volatile int right_branch_count = 0;

// 分支防抖相关变量
static uint8_t left_branch_counter = 0;
static uint8_t right_branch_counter = 0;
#define BRANCH_THRESHOLD 2
static uint8_t left_branch_detected = 0;
static uint8_t right_branch_detected = 0;
// 十字计数全局变量
volatile int crossroad_count =-1;

//所有分支总和
volatile int all = 0;
// 十字路口防抖计数器
static uint8_t crossroad_counter = 0;
#define CROSSROAD_THRESHOLD 2
// 十字路口检测标志（防止重复计数）
static uint8_t crossroad_detected_flag = 0;

// 计时变量
static uint32_t button_press_time = 0;
volatile uint32_t debug_counter = 0;

// 停止蜂鸣器标志
static uint8_t stop_beep_flag = 0;
/**
 * @brief 初始化巡线任务
 */
void LineWalking_Task_Init(void)
{
		g_systemState = SYS_STOPPED;
    ResetAllState();
    // 创建巡线任务
    const osThreadAttr_t LineWalkingTask_attributes = {
        .name = "LineWalkingTask",
        .stack_size = 256 * 4,
        .priority = (osPriority_t) osPriorityNormal,
    };
    LineWalkingTaskHandle = osThreadNew(StartLineWalkingTask, NULL, &LineWalkingTask_attributes);
}

/**
 * @brief 获取巡线传感器状态
 */
void GetLineWalking(LineWalking_State_t *pState)
{
    if(pState == NULL) return;
    
    // 读取中间两路传感器状态（L2和R1）
    pState->L2 = HAL_GPIO_ReadPin(LINE_L2_GPIO_Port, LINE_L2_Pin);
    pState->R1 = HAL_GPIO_ReadPin(LINE_R1_GPIO_Port, LINE_R1_Pin);
	
		// 读取分支传感器状态
    pState->branch_left = HAL_GPIO_ReadPin(LINE_L1_GPIO_Port, LINE_L1_Pin);
    pState->branch_right = HAL_GPIO_ReadPin(LINE_R2_GPIO_Port, LINE_R2_Pin);
}

/**
 * @brief 处理巡线逻辑，返回运动模式
 */
MotionMode_t ProcessLineWalking(LineWalking_State_t *pState)
{
		if(pState == NULL) return MOVE_STOP;
    if (g_systemState == SYS_STOPPED) return MOVE_STOP;
    
    if(pState == NULL) return MOVE_STOP;
    // 检测十字路口
    if(pState->branch_left == LOW && pState->L2 == LOW && 
       pState->R1 == LOW && pState->branch_right == LOW)
    {
        crossroad_counter++;
        if(crossroad_counter >= CROSSROAD_THRESHOLD && !crossroad_detected_flag)
        {
            crossroad_count++;                    // 计数+1
            crossroad_detected_flag = 1;          // 防止重复计数
            crossroad_counter = 0;
            g_buzzer_crossroad_trigger = 1;       // 触发蜂鸣器短叫
            return MOVE_CROSSROAD;
        }
    }
		
    else
    {
        crossroad_counter = 0;
        crossroad_detected_flag = 0;              // 离开十字路口后重置标志
			
				// 检测分支计数
    if(pState->branch_left == LOW && pState->branch_right == HIGH && !left_branch_detected)
    {
        left_branch_counter++;
        if(left_branch_counter >= BRANCH_THRESHOLD)
        {
            left_branch_count++;
            left_branch_detected = 1;
            g_buzzer_left_branch_trigger = 1;    // 触发蜂鸣器短叫
        }
    }
    else if(pState->branch_left == HIGH)
    {
        left_branch_counter = 0;
        left_branch_detected = 0;
    }
    
    if(pState->branch_right == LOW && pState->branch_left == HIGH && !right_branch_detected)
    {
        right_branch_counter++;
        if(right_branch_counter >= BRANCH_THRESHOLD)
        {
            right_branch_count++;
            right_branch_detected = 1;
            g_buzzer_right_branch_trigger = 1;   // 触发蜂鸣器短叫
        }
    }
    else if(pState->branch_right == HIGH)
    {
        right_branch_counter = 0;
        right_branch_detected = 0;
    }
		
    }
		
    // 检测分支路口（中间两路传感器同时检测到黑线）
    if(pState->L2 == LOW && pState->R1 == LOW)
    {
        pState->junction_detected = 1;
        junction_counter++;
        
        if(junction_counter >= JUNCTION_THRESHOLD)
        {
            // 确认是路口，可以选择直行、左转或右转
            // 这里选择直行，保持前进
            junction_counter = 0;
            return MOVE_FORWARD;
        }
    }
    else
    {
        junction_counter = 0;
        pState->junction_detected = 0;
    }
    
    // 中间左侧检测到黑线，需要左转修正
    if(pState->L2 == LOW && pState->R1 == HIGH)
    {   
        return MOVE_LEFT_TURN;
    }
    // 中间右侧检测到黑线，需要右转修正
    else if(pState->L2 == HIGH && pState->R1 == LOW)
    {   
        return MOVE_RIGHT_TURN;
    }
    // 都在黑线上，加速前进
    else if(pState->L2 == LOW && pState->R1 == LOW)
    {  
        return MOVE_FORWARD;
    }
    
    // 默认前进
    return MOVE_STOP;
}

/**
 * @brief 执行对应的运动模式
 */
void ExecuteMotion(MotionMode_t mode)
{
    switch(mode)
    {
        case MOVE_FORWARD:
            Motor_Set_Pwm(MOTOR_ID_M1, V_MAX * 0.995);
            Motor_Set_Pwm(MOTOR_ID_M2, V_MAX);
            Motor_Set_Pwm(MOTOR_ID_M3, V_MAX * 0.995);
            Motor_Set_Pwm(MOTOR_ID_M4, V_MAX);
            stop_beep_flag = 0;  // 重新开始运动时清除标志
            break;
            
        case MOVE_LEFT_TURN:
            Motor_Set_Pwm(MOTOR_ID_M1, V_MAX); //左转微调
            Motor_Set_Pwm(MOTOR_ID_M2, V_MIN);
            Motor_Set_Pwm(MOTOR_ID_M3, V_MAX);
            Motor_Set_Pwm(MOTOR_ID_M4, V_MIN);
            stop_beep_flag = 0;
            break;
            
        case MOVE_RIGHT_TURN:
            Motor_Set_Pwm(MOTOR_ID_M1, V_MIN); //右转微调
            Motor_Set_Pwm(MOTOR_ID_M2, V_MAX);
            Motor_Set_Pwm(MOTOR_ID_M3, V_MIN);
            Motor_Set_Pwm(MOTOR_ID_M4, V_MAX);
            stop_beep_flag = 0;
            break;
            
        case MOVE_STOP:
        default:
            Motor_Set_Pwm(MOTOR_ID_M1, 0);
            Motor_Set_Pwm(MOTOR_ID_M2, 0);
            Motor_Set_Pwm(MOTOR_ID_M3, 0);
            Motor_Set_Pwm(MOTOR_ID_M4, 0);
            
            // 只有在运行状态下停车才触发蜂鸣器（到达终点）
            if(g_systemState == SYS_RUNNING && !stop_beep_flag)
            {
                g_buzzer_stop_trigger = 1;
                stop_beep_flag = 1;
            }
            break;
    
					}
    if (g_systemState == SYS_RUNNING && button_press_time != 0 && mode != MOVE_STOP)
    {
        debug_counter = HAL_GetTick() - button_press_time;
    }

}
uint8_t GetLeftBranchCount(void)
{
    return left_branch_count;
}

uint8_t GetRightBranchCount(void)
{
    return right_branch_count;
}

void ResetBranchCount(void)
{
    left_branch_count = 0;
    right_branch_count = 0;
}
void ResetAllState(void)
{
    left_branch_count = 0;
    right_branch_count = 0;
    crossroad_count = -1;
    all = 0;
    left_branch_counter = 0;
    right_branch_counter = 0;
    left_branch_detected = 0;
    right_branch_detected = 0;
    junction_counter = 0;
    crossroad_counter = 0;
    crossroad_detected_flag = 0;
    stop_beep_flag = 0;  // 重置停止蜂鸣器标志
    
    Motor_Set_Pwm(MOTOR_ID_M1, 0);
    Motor_Set_Pwm(MOTOR_ID_M2, 0);
    Motor_Set_Pwm(MOTOR_ID_M3, 0);
    Motor_Set_Pwm(MOTOR_ID_M4, 0);
}

SystemState_t GetSystemState(void)
{
    return g_systemState;
}

static uint8_t ReadStartButton(void)
{
    uint8_t current_state = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
    uint8_t button_pressed = 0;
    
    if (current_state != button_stable_state)
    {
        button_debounce_counter++;
        if (button_debounce_counter >= BUTTON_DEBOUNCE_DELAY)
        {
            button_stable_state = current_state;
            button_debounce_counter = 0;
        }
    }
    else
    {
        button_debounce_counter = 0;
    }
    
    if (button_last_state == BUTTON_RELEASED && button_stable_state == BUTTON_PRESSED)
    {
        button_pressed = 1;
    }
    
    button_last_state = button_stable_state;
    return button_pressed;
}

static void HandleStartButton(void)
{
    if (ReadStartButton())
    {
        if (g_systemState == SYS_STOPPED)
        {
            ResetAllState();
            g_systemState = SYS_RUNNING;
            button_press_time = HAL_GetTick();
        }
        else
        {
            g_systemState = SYS_STOPPED;
            Motor_Set_Pwm(MOTOR_ID_M1, 0);
            Motor_Set_Pwm(MOTOR_ID_M2, 0);
            Motor_Set_Pwm(MOTOR_ID_M3, 0);
            Motor_Set_Pwm(MOTOR_ID_M4, 0);
        }
    }
}
uint8_t GetCrossroadCount(void)
{
    return crossroad_count;
}

void ResetCrossroadCount(void)
{
    crossroad_count = 0;
}
/**
 * @brief FreeRTOS巡线任务入口函数
 */

void StartLineWalkingTask(void *argument)
{
    LineWalking_State_t localState;
    MotionMode_t currentMode;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriod = pdMS_TO_TICKS(5);  
    
    for(;;)
    {
				HandleStartButton();
        // 1. 读取传感器状态
        GetLineWalking(&localState);
        
        // 2. 处理巡线逻辑
        currentMode = ProcessLineWalking(&localState);
        
        // 3. 执行运动控制
        ExecuteMotion(currentMode);
        
        // 可选：打印调试信息
        #ifdef DEBUG_LINE_WALKING
        printf("L1:%d L2:%d R1:%d R2:%d Mode:%d\r\n", 
               localState.L1, localState.L2, localState.R1, localState.R2, currentMode);
        #endif
        
        // 精确延时，保证固定周期运行
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}