#ifndef _uart_H_
#define _uart_H_
#include <stdint.h>
#include <stdbool.h>
#include "zf_common_headfile.h"
#include "vision_control.h"

#define UART_INDEX              ( UART_1 )                                // UART_1
#define UART_BAUDRATE           (115200)                                  // 115200
#define UART_TX_PIN             (UART1_TX_A8  )                           // UART1_TX_A8
#define UART_RX_PIN             (UART1_RX_A9  )                           // UART1_RX_A9

#define UART_PRIORITY           (UART1_INT_IRQn)                                // Ӧжϵжϱ  MIMXMSPM0G3507.h ͷļв鿴 IRQn_Type ö

extern uint8 uart1_get_data[64];                                                        // ڽݻ
extern uint8 fifo1_get_data[64];                                                        // fifo 

extern uint8 gpio_status;
extern uint8 get1_data;                                                             // ݱ
extern uint32 fifo1_data_count;                                                     // fifo ݸ

extern fifo_struct uart_data_fifo;

void uart1_rx_interrupt_handler (uint32 state, void *ptr);
void uart1_init (void);
void fifo1_print (void);
#endif