#ifndef __MOTOR_OFFLINE_DETECTOR_HPP
#define __MOTOR_OFFLINE_DETECTOR_HPP

#include <stdint.h>
#include "main.h"
#include "tim.h"

#define MOTOR_COUNT 8
#define OFFLINE_THRESHOLD_MS 100
#define BUZZER_ON_TIME_MS 50
#define BUZZER_OFF_TIME_MS 50
#define BUZZER_CYCLE_WAIT_MS 750

typedef enum {
    MOTOR_ID_1 = 0,
    MOTOR_ID_2 = 1,
    MOTOR_ID_3 = 2,
    MOTOR_ID_4 = 3,
    MOTOR_ID_5 = 4,
    MOTOR_ID_6 = 5,
    MOTOR_ID_7 = 6,
    MOTOR_ID_8 = 7
} MotorID_t;

typedef struct {
    uint32_t last_update_time;
    bool is_enabled;
    bool is_offline;
    bool is_alarming;
    uint8_t alarm_count;
    uint32_t alarm_start_time;
    uint32_t cycle_start_time;
} MotorStatus_t;

typedef enum {
    BUZZER_IDLE,
    BUZZER_ON,
    BUZZER_OFF,
    BUZZER_CYCLE_WAIT,
    BUZZER_MOTOR_WAIT
} BuzzerState_t;

typedef struct {
    BuzzerState_t state;
    uint32_t state_start_time;
    uint8_t current_motor_id;
    uint8_t current_alarm_count;
    uint8_t motor_queue[MOTOR_COUNT];
    uint8_t queue_head;
    uint8_t queue_tail;
    uint8_t queue_count;
} BuzzerControl_t;

void MotorOfflineDetector_Init(void);
void MotorOfflineDetector_Update(void);
void MotorOfflineDetector_UpdateMotorStatus(MotorID_t motor_id);
void MotorOfflineDetector_CheckOffline(void);
void MotorOfflineDetector_HandleBuzzer(void);
void MotorOfflineDetector_StartAlarm(MotorID_t motor_id);
void MotorOfflineDetector_StopAlarm(void);
void MotorOfflineDetector_AddToQueue(MotorID_t motor_id);
void MotorOfflineDetector_RemoveFromQueue(MotorID_t motor_id);
uint8_t MotorOfflineDetector_GetNextMotor(void);
bool MotorOfflineDetector_IsQueueEmpty(void);

extern MotorStatus_t motor_status[MOTOR_COUNT];
extern BuzzerControl_t buzzer_control;
extern bool all_motors_connected;

#endif // __MOTOR_OFFLINE_DETECTOR_HPP
