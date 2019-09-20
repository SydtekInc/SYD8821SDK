/*
GPIO例程main.c

作者：北京盛源达科技有限公司
日期：2018/3/8
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "ble_slave.h"

#define    PAD_CTRL    ((PAD_CTRL_TYPE *) PAD_CTRL_BASE)

int main()
{	

	uint8_t i = 0;
	
	gap_s_ble_init();
	
	sys_mcu_rc_calibration();
	delay_ms(100);
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	// RC bumping
	
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	
	
	__disable_irq();
    PAD_CTRL->DEBUG_BUS_SEL=0x30;   //syd_ctrl 0x5c=0X30
	__enable_irq();
	pad_mux_write(23, 14); 
	while(1)
	{
	}		
}
