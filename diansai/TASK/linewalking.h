#ifndef __LINEWALKING_H
#define __LINEWALKING_H

#include "main.h"
#include "gpio.h"
#include "cmsis_os.h"


// GPIO引脚定义（需要在CubeMX中配置）
// 如果CubeMX中未定义，需要手动添加

//巡线
#ifndef LINE_L2_GPIO_Port
#define LINE_L2_GPIO_Port  GPIOA
#define LINE_L2_Pin        GPIO_PIN_5  
#endif

#ifndef LINE_R1_GPIO_Port
#define LINE_R1_GPIO_Port  GPIOA
#define LINE_R1_Pin        GPIO_PIN_3  
#endif

//读分支
#ifndef LINE_R2_GPIO_Port
#define LINE_R2_GPIO_Port  GPIOA
#define LINE_R2_Pin        GPIO_PIN_2  
#endif

#ifndef LINE_L1_GPIO_Port
#define LINE_L1_GPIO_Port  GPIOA
#define LINE_L1_Pin        GPIO_PIN_4  
#endif

//启动按钮
#ifndef START_BUTTON_GPIO_Port
#define START_BUTTON_GPIO_Port  GPIOB
#define START_BUTTON_Pin        GPIO_PIN_11
#endif

// 按钮按下电平（按下为高电平）
#define BUTTON_PRESSED  1
#define BUTTON_RELEASED 0

// 传感器状态定义
#define LOW     0   // 检测到黑线
#define HIGH    1   // 未检测到黑线

// 运动模式定义
typedef enum {
    MOVE_FORWARD,
    MOVE_LEFT_TURN,
    MOVE_RIGHT_TURN,
    MOVE_STOP,
		MOVE_CROSSROAD
} MotionMode_t;
// 系统运行状态
typedef enum {
    SYS_STOPPED,
    SYS_RUNNING
} SystemState_t;
// 巡线状态结构体
typedef struct {
    int L2;  // 左2（内侧）
    int R1;  // 右1（内侧）
    uint8_t junction_detected;  // 路口检测标志
	
		int branch_left;   // 左分支传感器
    int branch_right;  // 右分支传感器
	
} LineWalking_State_t;

// 函数声明
void ResetAllState(void);
SystemState_t GetSystemState(void);
void LineWalking_Task_Init(void);
void GetLineWalking(LineWalking_State_t *pState);
MotionMode_t ProcessLineWalking(LineWalking_State_t *pState);
void ExecuteMotion(MotionMode_t mode);
void StartLineWalkingTask(void *argument);
uint8_t GetLeftBranchCount(void);
uint8_t GetRightBranchCount(void);
void ResetBranchCount(void);
uint8_t GetCrossroadCount(void);
void ResetCrossroadCount(void);
// 分支计数全局变量
extern volatile int left_branch_count;
extern volatile int right_branch_count;
extern volatile int crossroad_count;
extern volatile int all;
extern volatile uint32_t debug_counter;

void Buzzer_ShortBeep(void);
void Buzzer_LongBeep(uint32_t duration_ms);

// 蜂鸣器触发标志（由蜂鸣器任务管理）
extern volatile uint8_t g_buzzer_left_branch_trigger;
extern volatile uint8_t g_buzzer_right_branch_trigger;
extern volatile uint8_t g_buzzer_crossroad_trigger;
extern volatile uint8_t g_buzzer_stop_trigger;

#endif