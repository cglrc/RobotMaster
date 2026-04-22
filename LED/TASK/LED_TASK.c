#include "LED_TASK.h"
#include "main.h"           
#include "cmsis_os.h"

LED_t leds[3] = {
  {LED_0_GPIO_Port, GPIO_PIN_0, NULL},
  {LED_1_GPIO_Port, GPIO_PIN_1, NULL},
  {LED_2_GPIO_Port, GPIO_PIN_2, NULL}
};

void LED_On(LED_t* led) {
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}
void LED_Off(LED_t* led) {
  HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}
void LED_Toggle(LED_t* led) {
  HAL_GPIO_TogglePin(led->port, led->pin);
}

void LED_Task(void* argument) {
  for(;;) {
    for(int i = 0; i < 3; i++) {
        LED_Toggle(&leds[i]);  
      }
    osDelay(1000);
  }
}
void LED_Task_Init(void) {
  const osThreadAttr_t ledTask_attributes = {
    .name = "ledTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityNormal,
  };
  osThreadNew(LED_Task, NULL, &ledTask_attributes);
}