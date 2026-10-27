#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 =========================== */

/*  =========================== 进程的变量 =========================== */

/*  =========================== 函数的声明 =========================== */
// void GimbalToChassisUartSend();
void VofaUartSend();

/* Private application code --------------------------------------------------*/
void Uart_Task_Entry(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        // GimbalToChassisUartSend();
        VofaUartSend();

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(5));
    }
    /* USER CODE END LED_Flashing */
}


void Vision_Task_Entry(void *argument)
{
    TickType_t Lasttick = xTaskGetTickCount();
    VisionUartSend.Head = 0x39;

    
    VisionUartSend.AimingFinishCount = 255;

    for (;;) {
        Arr_VisionSend[0] = VisionUartSend.Head;
        Arr_VisionSend[1] = VisionUartSend.Head;

        VisionUartSend.YawAngle = (SpeedPID_AngleSensorM3508.Current - YAW_CENTER)
                                / 22.755556f * 100.0f;
        VisionUartSend.PullingForce = HX711.Filter_WeightInt;
        VisionUartSend.DartIndex = AutoLaunch.currentDartIndex;

        Arr_VisionSend[2] = VisionUartSend.YawAngle >> 8;
        Arr_VisionSend[3] = VisionUartSend.YawAngle & 0xFF;
        Arr_VisionSend[4] = VisionUartSend.PullingForce >> 8;
        Arr_VisionSend[5] = VisionUartSend.PullingForce & 0xFF;
        Arr_VisionSend[6] = VisionUartSend.AimingFinishCount;
        Arr_VisionSend[7] = VisionUartSend.DartIndex;
        Arr_VisionSend[8] = VisionUartSend.Reserved;

        SendDelayCount++;
        if (SendDelayCount >= SendDelay) {
            HAL_UART_Transmit_DMA(&VisionUartHandle, Arr_VisionSend, sizeof(Arr_VisionSend));
            SendDelayCount = 0;
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
}

/* Private application code --------------------------------------------------*/

/************************************************************************
 * @brief:      	void
 * @param[in]: 	void
 * @retval:      void
 * @details:    	void
 *************************************************************************/
unsigned char VofaUartSendArr[28] = {0}; // 修改数组大小为28字节，以容纳4个float和1个uint32_t

void VofaUartSend()
{
    float Ch1 = Motors[RIGHT_LOAD].Feedback.RPM;
    float Ch2 = Motors[LEFT_LOAD].Feedback.RPM;
    float Ch3 = (float)Motors[LEFT_LOAD].Output;
    float Ch4 = (float)Motors[RIGHT_LOAD].Output;

    uint32_t special_value = 0x7f800000;

    memcpy(&VofaUartSendArr[0], &Ch1, sizeof(float));
    memcpy(&VofaUartSendArr[4], &Ch2, sizeof(float));
    memcpy(&VofaUartSendArr[8], &Ch3, sizeof(float));
    memcpy(&VofaUartSendArr[12], &Ch4, sizeof(float));
    memcpy(&VofaUartSendArr[sizeof(float) * 4], &special_value, sizeof(uint32_t));

    HAL_UART_Transmit_DMA(&VofaUartHandle, VofaUartSendArr, sizeof(float) * 5); // 发送的数据长度相应调整
}