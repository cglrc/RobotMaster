#ifndef __BLUE_H
#define __BLUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BLUE_RX_BUFFER_SIZE 32

extern UART_HandleTypeDef huart3;

void BLUE_Init(void);
void BLUE_Process(void);
void BLUE_RxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif