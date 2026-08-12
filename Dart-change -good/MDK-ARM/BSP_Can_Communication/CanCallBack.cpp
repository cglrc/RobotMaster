#include "InHpp.hpp"
#include "../../user/core/HAL/CAN/can_hal.hpp"
/*  =========================== 全局变量的初始化 =========================== */ 
Can_RX_T Can_RX = {0};
CAN_HandleTypeDef *Can1_HandleTypedef = &hcan1;

/**
**********************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: Hal库的Can的通道0接收中断函数
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于接收机甲大师电机的Can数据,主要接收Yaw轴、升降、上膛和拉簧电机的数据
***********************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Can_RX.Header, Can_RX.Data);
	
	if(hcan->Instance == Can1_HandleTypedef->Instance) {
		// 使用 user/core 库解析数据
		HAL::CAN::Frame frame;
		frame.id = Can_RX.Header.StdId;
		frame.dlc = Can_RX.Header.DLC;
		frame.is_extended_id = (Can_RX.Header.IDE == CAN_ID_EXT);
		frame.is_remote_frame = (Can_RX.Header.RTR == CAN_RTR_REMOTE);
		memcpy(frame.data, Can_RX.Data, frame.dlc);
		
		// 解析各类电机数据
		BSP::Motor::Dji::motors_3508.Parse(frame);
		BSP::Motor::Dji::motors_2006.Parse(frame);
		BSP::Motor::Dji::motors_6020.Parse(frame);
		
		// 刷新电机连接状态
		BSP::Motor::Dji::motors_3508.refreshConnectionStatus();
		BSP::Motor::Dji::motors_2006.refreshConnectionStatus();
		BSP::Motor::Dji::motors_6020.refreshConnectionStatus();
		
		// 保留原有逻辑以兼容现有代码
		RmMotorGetCanData();
	}
}


// /**
// **********************************************************************
// * @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: Hal库的Can的通道1接收中断函数
// * @param[in]: 	void
// * @retval:      void
// * @details:    	该函数用于接收机甲大师电机的Can数据,主要接收6个FrM3508的电机数据
// ***********************************************************************
// **/
// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
// 	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &Can_RX.Header, Can_RX.Data);
// 	CAN_HandleTypeDef *Can1_HandleTypedef = &hcan2;

// 	if(hcan->Instance == Can1_HandleTypedef->Instance)
// 	RmMotorGetCanData();
// }

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE; //使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);//配置过滤器
    HAL_CAN_Start(&hcan1);//启动CAN
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//启动接收中断


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
//    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
//    HAL_CAN_Start(&hcan2);
//    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}