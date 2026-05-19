#ifndef __CanCallBack_Hpp
#define __CanCallBack_Hpp
#include "can.h"
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    unsigned char Data[8];
}Can_RX_T;
extern Can_RX_T Can_RX;

void can_filter_init(void);

#endif
