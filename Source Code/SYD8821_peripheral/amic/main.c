/*
UART例程main.c

作者：北京盛源达科技有限公司
日期：2019/11/14
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
#include <string.h>
#include "pdm_module.h"

int main()
{
	static uint8_t record_status = 0;
	
	__disable_irq();
	
	gap_s_ble_init();
	
	sys_mcu_rc_calibration();
	delay_ms(100);
	#ifdef IIC1_MASTER
	sys_mcu_clock_set(MCU_CLOCK_24_MHZ);
	#elif IIC1_SLAVE
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	#endif
	
	// RC bumping
	
	//GPIO 25,23,34,10
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
	gpo_config(LED4,1);
	
	//GPIO 14,15,16,17
	pad_mux_write(KEY1, 0);
	pad_mux_write(KEY2, 0);
	pad_mux_write(KEY3, 0);
	gpi_config(KEY1, PULL_UP);
	gpi_config(KEY2, PULL_UP);
	gpi_config(KEY3, PULL_UP);

	//uart 0 
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();//  ------------------------>>>>> UART_BAUD_921600
	
	dbg_printf("SYD8821 AMIC DEMO\r\n");
	
	pdm_module_init();
			
	__enable_irq();
		
	gpo_clr(LED1);
	
	while(1)
	{
		if(!gpi_get_val(KEY1))
		{
			if(record_status == 0)
			{
				start_record();		// 开始录音
				record_status = 1;
				gpo_clr(LED2);
				while(!gpi_get_val(KEY1));
			}
			else
			{
				stop_record();		// 停止录音
				record_status = 0;
				gpo_set(LED2);
				while(!gpi_get_val(KEY1));
			}
		}
		
		/* 串口打印录音数据 BandRate = 921600 */
		if(pdm_int_flag == 1)
		{
			pdm_int_flag = 0;
			uart_write(0, pdm_enc_buf_ptr, PDM_DMA_16BIT_ENC_BUF_LENGTH);
		}
	}		
}
