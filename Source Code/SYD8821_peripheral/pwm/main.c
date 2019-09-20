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
#include "pwm_led.h"
#include "ble_slave.h"

int main()
{	
	__disable_irq();
	
	gap_s_ble_init();
	
  // Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// Set MCU Clock 64M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	
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
  dbg_init();
	dbg_printf("SYD8821 PWM TEST\r\n");
	
	pad_mux_write(0, 10);
	pad_mux_write(1, 10);
	pad_mux_write(2, 10);
	pwm_led_set_pwm(0,50,100);
	pwm_led_set_flash(1, 4,10,10,8,127);
	pwm_led_set_breath(2,0,100,100,10);
	pwm_led_start(0);
	pwm_led_start(1);
	pwm_led_start(2);
	
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
	  if(!gpi_get_val(KEY1)){
			pwm_led_stop(0);
			pwm_led_stop(1);
			pwm_led_stop(2);
			dbg_printf("pwm_led_stop 0 1 2\r\n");
		  gpo_toggle(LED1);
		}
		if(!gpi_get_val(KEY2)){
			pwm_led_resume(0);
			pwm_led_resume(1);
			pwm_led_resume(2);
			dbg_printf("pwm_led_resume 0 1 2\r\n");
		  gpo_toggle(LED2);
		}
		if(!gpi_get_val(KEY3)){
			pwm_led_pause(0);
			pwm_led_pause(1);
			pwm_led_pause(2);
			dbg_printf("pwm_led_pause 0 1 2\r\n");
		  gpo_toggle(LED3);
		}
    delay_ms(100);
	}		
}
