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

#define WRITE_READ_LEN 32

int main()
{	
	uint16_t i=0;
	uint16_t addr=0;
	uint8_t tx_buff[32],rx_buff[32],base_data=0;
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
	
	gap_s_ble_init();
	
	__enable_irq();
	
	DBG("SYD8821 flash internal TEST\r\n");
	
	while(1)
	{
		gpo_toggle(LED4);
        for(i=0;i<WRITE_READ_LEN;i++)
		{
			tx_buff[i]=base_data+i;
		}
		gap_s_profile_data_write(addr,WRITE_READ_LEN,tx_buff);
		DBG("write addr:%04x len:%04x\r\n",addr,WRITE_READ_LEN);
		DBGHEXDUMP("",tx_buff,WRITE_READ_LEN);
		
		ble_sched_execute();
        
		gap_s_profile_data_read(addr,WRITE_READ_LEN,rx_buff);
		DBG("read addr:%04x len:%04x\r\n",addr,WRITE_READ_LEN);
		DBGHEXDUMP("",tx_buff,WRITE_READ_LEN);
		
		base_data++;
		addr+=256;
		delay_ms(3000);
	}		
}
