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

#define FLASH_INTERNAL_DATA_ADDR   0x70000

int main()
{	
	uint16_t i=0;
	uint16_t addr=0;
	uint8_t tx_buff[32],rx_buff[32],base_data=0;
	__disable_irq();

	gap_s_ble_init();
	// RC bumping
    sys_mcu_rc_calibration();
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
		
		if(!gpi_get_val(KEY1)){
			for(i=0;i<WRITE_READ_LEN;i++)
			{
				tx_buff[i]=base_data+i;
			}
			//注意ble_flash_erase函数会进行擦除操作，所以必须要保证地址是正确的
			ble_flash_erase(FLASH_INTERNAL_DATA_ADDR+addr,1);   //注意这里是标准的flash操作，在写数据之前必须要擦除，除非要写的地址区域已经被擦除过
			//ble_flash_write(FLASH_INTERNAL_DATA_ADDR+addr,WRITE_READ_LEN, tx_buff);
			ble_flash_write_burst(FLASH_INTERNAL_DATA_ADDR+addr,WRITE_READ_LEN, tx_buff,0);
			DBG("write addr:%04x len:%04x\r\n",FLASH_INTERNAL_DATA_ADDR+addr,WRITE_READ_LEN);
			DBGHEXDUMP("",tx_buff,WRITE_READ_LEN);
			
			base_data++;
		    gpo_toggle(LED1);
			delay_ms(600);
		}
		if(!gpi_get_val(KEY2)){
		    gpo_toggle(LED2);
			ble_flash_read(FLASH_INTERNAL_DATA_ADDR+addr,WRITE_READ_LEN, rx_buff);
			DBG("read addr:%04x len:%04x\r\n",FLASH_INTERNAL_DATA_ADDR+addr,WRITE_READ_LEN);
			
			DBGHEXDUMP("",rx_buff,WRITE_READ_LEN);
			delay_ms(600);
		}
		
		if(!gpi_get_val(KEY3)){
			addr+=256;
			DBG("write read addr:%04x\r\n",FLASH_INTERNAL_DATA_ADDR+addr);
		}
		
		delay_ms(100);
	}		
}
