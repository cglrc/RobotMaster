#include "InHpp.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DT7UartInstance) {
         //DT7 遥控器未接时注释掉，避免垃圾数据反复触发回调
         remote.parseData(DT7UartCom.ReceiveArr);
         DT7UartCom.GetMessage();
    }
    if (huart->Instance == VisionUartInstance)
        VisionUartReceive.GetData();
    if (huart->Instance == HuartDistance_RMRefereeSystem)
        RMRefereeSystemParse();
}

// 空闲中断回调函数
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == VofaUartInstance) {
        // 处理接收到的数据
        VofaCallBack.ProcessReceivedData();
    }
    
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (huart->Instance == USART6) { // 裁判系统
        
        HAL_UART_Receive_IT(huart, &MyRefereeSys8Data, sizeof(MyRefereeSys8Data));
    } else if (huart->Instance == USART1) { // 遥控器
        HAL_UART_Receive_DMA(huart, DT7UartCom.ReceiveArr, DT7UartReceiveLength);
    } else if (huart->Instance == UART8) { // 视觉通信
        VisionUartReceive.GetData(); // 内部已包含重新启动 DMA 接收
    } else if (huart->Instance == UART7) {
        // 其他处理
    }
}