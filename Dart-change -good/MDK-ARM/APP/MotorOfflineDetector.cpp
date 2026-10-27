#include "InHpp.hpp"
#include "MotorOfflineDetector.hpp"
#include "SystemMonitor.hpp"
#include "Buzzer.hpp"
#include "Dart.hpp"

MotorStatus_t motor_status[MOTOR_COUNT] = {0};
BuzzerControl_t buzzer_control = {BUZZER_IDLE, 0, 0, 0, {0}, 0, 0, 0};
bool all_motors_connected = true;

static void MotorOfflineDetector_HandleLEDs(void);

void MotorOfflineDetector_Init(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_status[i].last_update_time = HAL_GetTick();
        motor_status[i].is_enabled = true;
        motor_status[i].is_offline = false;
        motor_status[i].is_alarming = false;
        motor_status[i].alarm_count = 0;
        motor_status[i].alarm_start_time = 0;
        motor_status[i].cycle_start_time = 0;
    }
    motor_status[MOTOR_ID_1].is_enabled = false;
    buzzer_control.state = BUZZER_IDLE;
    buzzer_control.state_start_time = 0;
    buzzer_control.current_motor_id = 0;
    buzzer_control.current_alarm_count = 0;
    buzzer_control.queue_head = 0;
    buzzer_control.queue_tail = 0;
    buzzer_control.queue_count = 0;
}

void MotorOfflineDetector_Update(void)
{
    MotorOfflineDetector_CheckOffline();
    MotorOfflineDetector_HandleBuzzer();
    MotorOfflineDetector_HandleLEDs();
    static bool prev_all_motors_connected = true;
    all_motors_connected = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motor_status[i].is_enabled && motor_status[i].is_offline) {
            all_motors_connected = false;
            break;
        }
    }
    if (all_motors_connected && !prev_all_motors_connected) {
        playState = 1;
    } else if (!all_motors_connected && prev_all_motors_connected) {
        playState = 0;
    }
    prev_all_motors_connected = all_motors_connected;
}

void MotorOfflineDetector_UpdateMotorStatus(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT) {
        motor_status[motor_id].last_update_time = HAL_GetTick();
        motor_status[motor_id].is_offline = false;
        if (motor_status[motor_id].is_alarming) {
            motor_status[motor_id].is_alarming = false;
            motor_status[motor_id].alarm_count = 0;
            MotorOfflineDetector_RemoveFromQueue(motor_id);
            if (buzzer_control.current_motor_id == motor_id) {
                MotorOfflineDetector_StopAlarm();
            }
        }
    }
}

void MotorOfflineDetector_CheckOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motor_status[i].is_enabled) {
            continue;
        }
        if (current_time - motor_status[i].last_update_time > OFFLINE_THRESHOLD_MS) {
            if (!motor_status[i].is_offline) {
                motor_status[i].is_offline = true;
                motor_status[i].is_alarming = true;
                motor_status[i].alarm_start_time = current_time;
                motor_status[i].cycle_start_time = current_time;
                motor_status[i].alarm_count = 0;
                MotorOfflineDetector_AddToQueue((MotorID_t)i);
            }
        }
    }
}

void MotorOfflineDetector_HandleBuzzer(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - buzzer_control.state_start_time;
    switch (buzzer_control.state) {
        case BUZZER_IDLE:
            if (!MotorOfflineDetector_IsQueueEmpty()) {
                uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                buzzer_control.current_motor_id = next_motor;
                buzzer_control.current_alarm_count = 0;
                buzzer_control.state = BUZZER_ON;
                buzzer_control.state_start_time = HAL_GetTick();
                uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                uint32_t arr = timer_freq / 523;
                __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
                __HAL_TIM_SetCounter(&htim12, 0);
            }
            break;
        case BUZZER_ON:
            if (elapsed_time >= BUZZER_ON_TIME_MS) {
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                buzzer_control.state = BUZZER_OFF;
                buzzer_control.state_start_time = current_time;
            }
            break;
        case BUZZER_OFF:
            if (elapsed_time >= BUZZER_OFF_TIME_MS) {
                buzzer_control.current_alarm_count++;
                uint8_t target_alarm_count = buzzer_control.current_motor_id + 1;
                if (buzzer_control.current_alarm_count >= target_alarm_count) {
                    if (buzzer_control.queue_count > 0) {
                        buzzer_control.state = BUZZER_MOTOR_WAIT;
                        buzzer_control.state_start_time = current_time;
                    } else {
                        buzzer_control.state = BUZZER_CYCLE_WAIT;
                        buzzer_control.state_start_time = current_time;
                    }
                    if (buzzer_control.current_motor_id < MOTOR_COUNT) {
                        motor_status[buzzer_control.current_motor_id].alarm_count++;
                    }
                } else {
                    uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                    uint32_t arr = timer_freq / 523;
                    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
                    __HAL_TIM_SetCounter(&htim12, 0);
                    buzzer_control.state = BUZZER_ON;
                    buzzer_control.state_start_time = current_time;
                }
            }
            break;
        case BUZZER_CYCLE_WAIT:
            if (elapsed_time >= BUZZER_CYCLE_WAIT_MS) {
                if (buzzer_control.current_motor_id < MOTOR_COUNT) {
                    if (motor_status[buzzer_control.current_motor_id].is_offline) {
                        buzzer_control.queue_head = 0;
                        buzzer_control.queue_tail = 0;
                        buzzer_control.queue_count = 0;
                        for (int i = 0; i < MOTOR_COUNT; i++) {
                            if (motor_status[i].is_offline && motor_status[i].is_alarming) {
                                MotorOfflineDetector_AddToQueue((MotorID_t)i);
                            }
                        }
                        if (!MotorOfflineDetector_IsQueueEmpty()) {
                            uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                            buzzer_control.current_motor_id = next_motor;
                            buzzer_control.current_alarm_count = 0;
                            buzzer_control.state = BUZZER_ON;
                            buzzer_control.state_start_time = current_time;
                            uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                            uint32_t arr = timer_freq / 523;
                            __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
                            __HAL_TIM_SetCounter(&htim12, 0);
                        } else {
                            buzzer_control.state = BUZZER_IDLE;
                        }
                    } else {
                        MotorOfflineDetector_RemoveFromQueue((MotorID_t)buzzer_control.current_motor_id);
                        motor_status[buzzer_control.current_motor_id].is_alarming = false;
                        if (!MotorOfflineDetector_IsQueueEmpty()) {
                            buzzer_control.state = BUZZER_MOTOR_WAIT;
                            buzzer_control.state_start_time = current_time;
                        } else {
                            buzzer_control.state = BUZZER_IDLE;
                        }
                    }
                } else {
                    buzzer_control.state = BUZZER_IDLE;
                }
            }
            break;
        case BUZZER_MOTOR_WAIT:
            if (elapsed_time >= BUZZER_CYCLE_WAIT_MS) {
                if (!MotorOfflineDetector_IsQueueEmpty()) {
                    uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                    buzzer_control.current_motor_id = next_motor;
                    buzzer_control.current_alarm_count = 0;
                    buzzer_control.state = BUZZER_ON;
                    buzzer_control.state_start_time = current_time;
                    uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                    uint32_t arr = timer_freq / 523;
                    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
                    __HAL_TIM_SetCounter(&htim12, 0);
                } else {
                    buzzer_control.state = BUZZER_IDLE;
                }
            }
            break;
    }
}

void MotorOfflineDetector_StartAlarm(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT && buzzer_control.state == BUZZER_IDLE) {
        buzzer_control.current_motor_id = motor_id;
        buzzer_control.current_alarm_count = 0;
        buzzer_control.state = BUZZER_ON;
        buzzer_control.state_start_time = HAL_GetTick();
        uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
        uint32_t arr = timer_freq / 523;
        __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
        __HAL_TIM_SetCounter(&htim12, 0);
    }
}

void MotorOfflineDetector_StopAlarm(void)
{
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    buzzer_control.state = BUZZER_IDLE;
    buzzer_control.current_motor_id = 0;
    buzzer_control.current_alarm_count = 0;
}

void MotorOfflineDetector_AddToQueue(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT && buzzer_control.queue_count < MOTOR_COUNT) {
        for (int i = 0; i < buzzer_control.queue_count; i++) {
            uint8_t index = (buzzer_control.queue_head + i) % MOTOR_COUNT;
            if (buzzer_control.motor_queue[index] == motor_id) {
                return;
            }
        }
        buzzer_control.motor_queue[buzzer_control.queue_tail] = motor_id;
        buzzer_control.queue_tail = (buzzer_control.queue_tail + 1) % MOTOR_COUNT;
        buzzer_control.queue_count++;
    }
}

void MotorOfflineDetector_RemoveFromQueue(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT && buzzer_control.queue_count > 0) {
        for (int i = 0; i < buzzer_control.queue_count; i++) {
            uint8_t index = (buzzer_control.queue_head + i) % MOTOR_COUNT;
            if (buzzer_control.motor_queue[index] == motor_id) {
                for (int j = i; j < buzzer_control.queue_count - 1; j++) {
                    uint8_t current_index = (buzzer_control.queue_head + j) % MOTOR_COUNT;
                    uint8_t next_index = (buzzer_control.queue_head + j + 1) % MOTOR_COUNT;
                    buzzer_control.motor_queue[current_index] = buzzer_control.motor_queue[next_index];
                }
                buzzer_control.queue_count--;
                buzzer_control.queue_tail = (buzzer_control.queue_tail - 1 + MOTOR_COUNT) % MOTOR_COUNT;
                break;
            }
        }
    }
}

uint8_t MotorOfflineDetector_GetNextMotor(void)
{
    if (buzzer_control.queue_count > 0) {
        uint8_t motor_id = buzzer_control.motor_queue[buzzer_control.queue_head];
        buzzer_control.queue_head = (buzzer_control.queue_head + 1) % MOTOR_COUNT;
        buzzer_control.queue_count--;
        return motor_id;
    }
    return 0xFF;
}

bool MotorOfflineDetector_IsQueueEmpty(void)
{
    return buzzer_control.queue_count == 0;
}

static void MotorOfflineDetector_HandleLEDs(void)
{
    static uint32_t last_led_tick = 0;
    if (SystemTick - last_led_tick < 50) {
        return;
    }
    last_led_tick = SystemTick;
    const uint16_t led_pins[8] = {
        GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4,
        GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8
    };
    static uint16_t led_tick_counters[8] = {0};
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (!motor_status[i].is_enabled || !motor_status[i].is_offline) {
            HAL_GPIO_WritePin(GPIOG, led_pins[i], GPIO_PIN_SET);
            led_tick_counters[i] = 0;
            continue;
        }
        uint16_t motor_id = i + 1;
        uint16_t total_cycle_ticks = (motor_id * 10) + 30;
        led_tick_counters[i]++;
        if (led_tick_counters[i] > total_cycle_ticks) {
            led_tick_counters[i] = 1;
        }
        uint16_t t = led_tick_counters[i];
        if (t <= (motor_id * 10)) {
            if (((t - 1) / 5) % 2 == 0) {
                HAL_GPIO_WritePin(GPIOG, led_pins[i], GPIO_PIN_RESET);
            } else {
                HAL_GPIO_WritePin(GPIOG, led_pins[i], GPIO_PIN_SET);
            }
        } else {
            HAL_GPIO_WritePin(GPIOG, led_pins[i], GPIO_PIN_SET);
        }
    }
}
