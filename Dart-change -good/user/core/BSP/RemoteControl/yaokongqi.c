#include "../core/BSP/RemoteControl/yaokongqi.h"

uint8_t receivedata[18];
uint32_t rc_last_recv_time = 0;
uint8_t rc_disconnect_flag = 0;


RC_Ctl_t data;
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
	data.rc.scroll = ((int16_t)pData[16]) | ((int16_t)pData[17] << 8);

	data.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	data.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	data.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);

	data.mouse.press_l = pData[12];
	data.mouse.press_r = pData[13];

	data.key.v = ((int16_t)pData[14]);
}
