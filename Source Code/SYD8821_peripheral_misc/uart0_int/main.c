/*
UART例程main.c

作者：北京盛源达科技有限公司
日期：2018/3/19
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "uart.h"
#include "queue.h"

int main()
{	
	uint8_t *buff;
	uint16_t buff_size=0;
	__disable_irq();
	//GPO
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
  gpo_config(LED4,1);
	
	//GPI
	pad_mux_write(KEY1, 0);
	pad_mux_write(KEY2, 0);
	pad_mux_write(KEY3, 0);
	pad_mux_write(KEY4, 0);
	gpi_config(KEY1, PULL_UP);
	gpi_config(KEY2, PULL_UP);
	gpi_config(KEY3, PULL_UP);
	gpi_config(KEY4, PULL_UP);
//	//uart 1
//	pad_mux_write(4, 7);
//	pad_mux_write(5, 7);
//	uart_1_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
//	uart_write(1,"SYD8821 UART TEST", 18);

 //uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
	uart_write(0,"SYD8821 UART TEST", 18);
	
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
		buff_size=uart_queue_size(0,buff);
		if (buff_size){
			uint8_t temp=0; 
			uint16_t i=0;
			for(i=0;i<buff_size;i++) {
			    uart_read(0,&temp);
				  uart_write(0,&temp,1);
			}
		    gpo_toggle(LED1);
		}
	}		
}
