#ifndef __LED_TASK_H
#define __LED_TASK_H
 
#ifdef __cplusplus
extern "C" {
#endif
 
#include "cmsis_os.h"
#include "main.h" 
 
typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
  osThreadId_t handle;
} LED_t;
 
void LED_Task_Init(void);
void LED_On(LED_t* led);
void LED_Off(LED_t* led);
void LED_Toggle(LED_t* led);
 
#ifdef __cplusplus
}
#endif
#endif