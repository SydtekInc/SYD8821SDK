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
#include "spi.h"
#include "debug.h"
#include "flash.h"
#include "ble_slave.h"

#define USER_FLASH_BASE_ADDR 0

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
//	//uart 1
//	pad_mux_write(4, 7);
//	pad_mux_write(5, 7);
//	uart_1_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
//	NVIC_EnableIRQ(UART1_IRQn); 
//	uart_write(1,"SYD8821 SPI1 TEST\r\n", 21);

	// uart 0
	 pad_mux_write(20, 7);
	 pad_mux_write(21, 7);
	 uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
}


void SPI_Module_Init(void)
{
	// SPI1
	pad_mux_write(SPI_SCLK_1, 3);
	pad_mux_write(SPI_MISO_1, 3);
	pad_mux_write(SPI_CSN_1,  3);
	pad_mux_write(SPI_MOSI_1, 3);
		
	spi_set_speed(SPIM_SPEED_4M);
	spi_set_mode(SPI_MODE_3_DOUT);
	
	spim_enable_int(0);
	spim_enable(0);
}

int main()
{		
	uint32_t id=0;
	__align(4) uint8_t SPI_RX[1024] = {0}, SPI_TX[1024] = {0};
												
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
	SPI_Module_Init();
	
	dbg_printf("SYD8821 SPI_Master TEST\r\n");
	__enable_irq();
	
	id =SPI_Flash_ReadID();
	if((id != FLASH_ID1) && (id != FLASH_ID2))dbg_printf("flash filed id:%x\r\n",id);
	else dbg_printf("flash succeed id:%x\r\n",id);

	while(1)
	{
		gpo_toggle(LED4);
	  if(!gpi_get_val(KEY1)){
			  SPI_Flash_Erase_Sector(USER_FLASH_BASE_ADDR);
		    dbg_printf("erase addr:%x\r\n",USER_FLASH_BASE_ADDR);
		    gpo_toggle(LED1);
			  delay_ms(2000);
		}
		if(!gpi_get_val(KEY2)){
			  SPI_Flash_Read(SPI_RX,USER_FLASH_BASE_ADDR,233);
			  dbg_hexdump("read addr:0\r\n",SPI_RX, 64);
			  dbg_hexdump("read addr:200\r\n",&SPI_RX[200], 16);
		    gpo_toggle(LED2);
			  delay_ms(2000);
		}
		if(!gpi_get_val(KEY3)){
			  uint32_t i=0;
			  for(i=0;i<sizeof(SPI_TX);i++) SPI_TX[i]=i;
			  SPI_Flash_Erase_Sector(USER_FLASH_BASE_ADDR);
			  SPI_Flash_Write(SPI_TX,USER_FLASH_BASE_ADDR,233);
			  dbg_hexdump("write addr:0\r\n",SPI_TX, 64);
			  dbg_hexdump("write addr:200\r\n",&SPI_TX[200], 16);
		    gpo_toggle(LED3);
		  	delay_ms(2000);
		}
		delay_ms(1000);
	}		
}
