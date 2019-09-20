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
#include "i2c_master.h"
#include "oled_9632.h"

int main()
{	
	uint8_t iic_buff[32]={0};
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
	dbg_printf("SYD8821 IIC DEMO\r\n");

	// IIC
	pad_mux_write(8, 6); //i2c 0 scl
	pad_mux_write(9, 6); //i2c 0 sda

	/* Masters Init M0*/
	//set i2c clk rate param 0, (48/8)*50K=300k 
	i2c_master_set_speed(1, 0);
	i2c_master_enable(1, true);
	i2c_master_enable_int(1, true);
	i2c_master_set_address_mode(1, I2C_MASTER_1BYTE_ADDRESS);
	NVIC_EnableIRQ(I2CM0_IRQn);
	
	oled_init();
	
	oled_printf(0,0,"SYD Inc."); 
	oled_printf(0,2,"SYD8821 EVB"); 
	oled_printf(0,4,"oled demo"); 
	oled_printf(0,6,"2018-3-20"); 
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);

		delay_ms(100);
	}		
}
