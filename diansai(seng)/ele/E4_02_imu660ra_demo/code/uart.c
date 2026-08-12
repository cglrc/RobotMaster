#include "uart.h"

uint8 uart1_get_data[64];                                                        // 用于接收数据的缓冲区
uint8 fifo1_get_data[64];                                                        // fifo 读取数据的缓冲区

uint8 gpio_status;
uint8 get1_data = 0;                                                             // 接收数据的变量
uint32 fifo1_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;


void uart1_rx_interrupt_handler (uint32 state, void *ptr)
{ 
    if(uart_query_byte(UART_INDEX, &get1_data))                                  // 读取数据 查询式 会返回 TRUE 没有数据会返回 FALSE
	{
	    fifo_write_buffer(&uart_data_fifo, &get1_data, 1);                       // 将数据写入 fifo 中
	}
}


void uart1_init (void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart1_get_data, 64);              // 初始化 fifo 数据缓冲区
    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
    uart_set_interrupt_config(UART_INDEX, UART_INTERRUPT_CONFIG_RX_ENABLE);		// 使能串口接收中断
    // 注意：中断优先级已在main函数中统一设置，此处不再设置
	uart_set_callback(UART_INDEX, uart1_rx_interrupt_handler, NULL);			    // 设置中断接收函数
}


void fifo1_print (void)
{
    fifo1_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
    if(fifo1_data_count != 0)                                                // 读取到数据了
    {
        fifo_read_buffer(&uart_data_fifo, fifo1_get_data, &fifo1_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
        
        // 注释掉原来的回显代码，避免与视觉控制模块冲突
        /*
        // 先原样回传接收到的数据
        uart_write_buffer(UART_INDEX, fifo1_get_data, fifo1_data_count);

        // 打印接收到的数据长度和内容（作为字符串）
        printf("\r\nUART1 Received %d bytes: ", fifo1_data_count);
        for(uint32 i = 0; i < fifo1_data_count; i++)
        {
            putchar(fifo1_get_data[i]);
        }
        printf("\r\n");
        */
    }
}