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

void key_handler(GPIO_BANK_TYPE type, uint32_t bitmask)
{
//    if(!gpi_get_val(KEY3))
//    {
//			delay_ms(20);
//			if(!gpi_get_val(KEY3)) gpo_toggle(LED7);
//    }
	if(type==GPIO_BANK_0)  // GPIO  0 ~ 31
	{
	    if(bitmask==U32BIT(KEY3))
			{
					delay_ms(20);
					if(!gpi_get_val(KEY3)) gpo_toggle(LED3);
			}
	}else{   // GPIO 32 ~ 38
	
	}
}

int main()
{	

	uint8_t i = 0;
	
	__disable_irq();
	//GPIO
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
	gpo_config(LED4,1);
	
	//GPIO
	pad_mux_write(KEY1, 0);
	pad_mux_write(KEY2, 0);
	pad_mux_write(KEY3, 0);
	pad_mux_write(KEY4, 0);
	gpi_config(KEY1, PULL_UP);
	gpi_config(KEY2, PULL_UP);
	gpi_config(KEY3, PULL_UP);
	gpi_config(KEY4, PULL_UP);
	gpi_enable_int(KEY3, EDGE_TRIGGER, POL_FALLING_LOW);
	gpi_irq_set_cb(key_handler);
	
	__enable_irq();

	for(i = 0; i < 4; i ++)
	{
		gpo_toggle(LED1);
		gpo_toggle(LED2);
		gpo_toggle(LED3);
		gpo_toggle(LED4);
		delay_ms(500);
	}
	
	while(1)
	{
    gpo_toggle(LED4);
		
		if(!gpi_get_val(KEY1)){
		    gpo_toggle(LED1);
		}
		if(!gpi_get_val(KEY2)){
		    gpo_toggle(LED2);
		}
		delay_ms(100);
	}		
}
