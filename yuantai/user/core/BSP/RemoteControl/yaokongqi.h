//#include "struct_typedef.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;

	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;

extern RC_Ctl_t data; // ����Ϊ�ⲿ�������������ļ�ʹ��
void remoteDataProcess(uint8_t* pData);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size);
