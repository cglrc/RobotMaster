#include "blue.h"
#include "linewalking.h"
#include "bsp_motor.h"
#include "linewalking.h"

uint8_t blue_rx_buffer[BLUE_RX_BUFFER_SIZE];
uint8_t blue_rx_data = 0;
volatile uint8_t blue_data_received = 0;

void BLUE_Init(void)
{
    HAL_UART_Receive_DMA(&huart3, &blue_rx_data, 1);
}

void BLUE_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (blue_rx_data == '1')
        {
            blue_data_received = 1;
        }
        HAL_UART_Receive_DMA(&huart3, &blue_rx_data, 1);
    }
}

void BLUE_Process(void)
{
    if (blue_data_received)
    {
        blue_data_received = 0;
        
        if (g_systemState == SYS_STOPPED)
        {
            ResetAllState();
            g_systemState = SYS_RUNNING;
            button_press_time = HAL_GetTick();
            g_startup_blind_run = 1;
            g_blind_run_start_time = HAL_GetTick();
            g_reached_end = 0;
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