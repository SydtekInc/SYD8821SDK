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
#include "debug.h"
#include "wdt.h"
#include "ble_slave.h"

uint16_t delay_num=0;

void wdt_callback(void)
{
  dbg_printf("wdt_callback\r\n");
  pmu_system_reset();
}

int main()
{	
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
  //uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	dbg_printf("SYD8821 WDT TEST\r\n");
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	//sys_32k_clock_set(SYSTEM_32K_CLOCK_LPO);
	
	wdt_set_crv(32768*15);  //1S
	wdt_set_RR_enable(WDT_RR_ALL);
	wdt_set_reset_type(0);
	//wdt_set_reset_type(WDT_RESET_ALL);
	wdt_start(0);
	
	wdt_set_interrupt_callback(wdt_callback);
	wdt_int_enable();
	delay_ms(100);
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
//		if(gpi_get_val(KEY1)){
//		    wdt_reset_counter();
//			  gpo_toggle(LED1);
//		}
//		if(!gpi_get_val(KEY2)){
//		    wdt_stop();
//			  gpo_toggle(LED2);
//		}
    delay_ms(100);
	}		
}
