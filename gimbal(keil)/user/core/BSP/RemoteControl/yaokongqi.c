#include "../core/BSP/RemoteControl/yaokongqi.h"

uint8_t receivedata[18];
uint32_t rc_last_recv_time = 0;  // 遥控器最后接收时间（单位：ms）
uint8_t rc_disconnect_flag = 0;  // 遥控器断联标志（0=正常，1=断联）


 
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
//{
    //if (huart == &huart3)
   // {
//        
        // rc_last_recv_time = HAL_GetTick();
        //rc_disconnect_flag = 0;
        //remoteDataProcess(receivedata);
        //HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(uint8_t *)receivedata, 18);
	//}	  
//               
		
    
//}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
//{
//	if (huart == &huart3)
//	{
//		rc_last_recv_time = HAL_GetTick();  // 更新最后接收时间（HAL_GetTick()返回系统运行ms数）
//    rc_disconnect_flag = 0;  // 清除断联标志
//		remoteDataProcess(receivedata);
//		
//		HAL_UARTEx_ReceiveToIdle_DMA (&huart3 ,(uint8_t *)receivedata ,50);  
//	}
//}

RC_Ctl_t data;//��yaokongqi.h�ļ�������extern RC_Ctl_t data����������һ��.c�ļ��ж��壨RC_Ctl_t data������ֻ�ܶ���һ�Σ���������һ���ļ��ٴζ���
void remoteDataProcess(uint8_t* pData)
{
	if (pData == NULL)
	{
		return;
	}
	data.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	data.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	data.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
	data.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5] << 7)) & 0x07FF;

	data.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	data.rc.s2 = ((pData[5] >> 4) & 0x0003);

	data.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	data.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	data.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

	data.mouse.press_l = pData[12];
	data.mouse.press_r = pData[13];

	data.key.v = ((int16_t)pData[14]);
}

