/*
SPI例程main.c

作者：北京盛源达科技有限公司
日期：2018/3/8
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "uart.h"
#include "queue.h"
#include "timer.h"
#include "ble_slave.h"

static uint8_t timer1s_inting = 0,timer4s_inting = 0;


void timer1_callback(void);
void timer4_callback(void);


void LED_KEY_Module_Init(void)
{
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
}


void UART_Module_Init(void)
{
	//uart 1
	pad_mux_write(4, 7);
	pad_mux_write(5, 7);
	uart_1_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
	NVIC_EnableIRQ(UART1_IRQn); 
	uart_write(1,"SYD8821 TIMER4 1s TEST\r\n", 24);

	// uart 0
	// pad_mux_write(20, 7);
	// pad_mux_write(21, 7);
	// uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
	// uart_write(0,"SYD8821 UART TEST", 18);
}


void Timer_Module_Init(void)
{
	timer_disable(TIMER_1); 
	timer_enable(TIMER_1, timer1_callback, 32768*2, 1);//32768 = 1S  16384 = 500ms
  NVIC_EnableIRQ(TIMER1_IRQn);
	timer_disable(TIMER_4); 
	timer_enable(TIMER_4, timer4_callback, 32768*2, 1);//32768 = 1S  16384 = 500ms
  NVIC_EnableIRQ(TIMER4_IRQn);
}


void timer1_callback(void)
{
	timer1s_inting = 1;
}
void timer4_callback(void)
{
	timer4s_inting = 1;
}

int main()
{	
	uint8_t *buff;
	uint16_t buff_size=0;
	__disable_irq();

	gap_s_ble_init();
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// Set MCU Clock 64M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	// RC bumping
	sys_mcu_rc_calibration();
	
	
	LED_KEY_Module_Init();
	UART_Module_Init();
	Timer_Module_Init();
	
	__enable_irq();
	
	
	while(1)
	{
		gpo_toggle(LED4);
		
		if(timer1s_inting)
		{
			timer1s_inting = 0;
			gpo_toggle(LED1);
			//uart_write(1,"SYD8821 TIMER1 1s TEST\r\n", 24);
		}
		if(timer4s_inting)
		{
			timer4s_inting = 0;
			gpo_toggle(LED2);
			//uart_write(1,"SYD8821 TIMER4 1s TEST\r\n", 24);
		}
		
		buff_size = uart_queue_size(1,buff);
		if (buff_size){
			uint8_t temp=0; 
			uint16_t i=0;
			for(i=0;i<buff_size;i++) {
			    uart_read(1,&temp);
				  uart_write(1,&temp,1);
			}
		  gpo_toggle(LED3);
		}
	}		
}
