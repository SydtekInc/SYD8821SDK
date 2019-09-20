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
#include "ble_slave.h"
#include "gpadc.h"



int main()
{	
	uint16_t adc_value = 0;
	float voltage_value = 0;
	
	__disable_irq();
	
	gap_s_ble_init();
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// Set MCU Clock 96M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	// RC bumping
	sys_mcu_rc_calibration();	

//uart 0
	pad_mux_write(GPIO_20, 7);
	pad_mux_write(GPIO_21, 7);
	dbg_init();
	dbg_printf("uart 0 init ok!!!\r\n"); //GPIO20 GPIO21

//ADC GPIO
	pad_mux_write(GPIO_4, 1);

	adc_init();
	__enable_irq();	
	
	//get VBAT 
	
	
	while(1)
	{	
	 	adc_open(VBAT_CHANNEL); //select VBAT_CHANNEL	
		adc_value = gpadc_get_value();
		voltage_value  = (float)adc_value * 3.6 / 1023;//adc采集的电压
		voltage_value = voltage_value * 4.3/1.7;			 //adc_voltage分压倍数*4.3/1.7
		dbg_printf("CHANNEL  VBAT adc:%x voltage:%04f \t",  adc_value, voltage_value);
		
		delay_ms(10);

		//get GPIO4_CHANNEL	
		adc_open(GPIO4_CHANNEL); //select GPIO4_CHANNEL
		adc_value = gpadc_get_value();
		voltage_value = ((float)adc_value * 3.6) / 1023 ;
		dbg_printf("GPIO4 adc:%x voltage:%04f\r\n", adc_value,voltage_value);
		
		delay_ms(100);
	}		
}
