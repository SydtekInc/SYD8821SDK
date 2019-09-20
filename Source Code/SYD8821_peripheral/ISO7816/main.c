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
#include "sc_reader.h"
#include "debug.h"
#include "tpdu.h"



static int count;
static int set_count;
static bool stress_act_trigger;
static bool reseted = false;

extern void sc_reader_warm_reset(void);
extern void sc_reader_config_auto(bool);
extern void sc_reader_deactivate(void);
extern void sc_reader_activate(void);
extern void sc_reader_dump_info(void);
extern void sc_reader_config_clock_stoppable(bool en);
extern void atr_decoder_config_default_FD(uint8_t fd);


void sc_test_init(void)
{
	
//	pad_mux_write(29, 0); //RST
//	gpo_config(GPIO_29,1);//rst
	
    pad_mux_write(25, 11); //VCCCTL
    pad_mux_write(26, 11); //DET
    pad_mux_write(27, 11); //CLK
    pad_mux_write(28, 11); //IO
    pad_mux_write(29, 11); //RST
	
    sc_reader_enable();
}

int main()
{	
	#ifdef IIC1_MASTER
	uint8_t iic_buff[32]={0};
	uint8_t iic_addr, iic_dat[16];
	#endif
	__disable_irq();
	
	gap_s_ble_init();
	
	sys_mcu_clock_set(MCU_CLOCK_96_MHZ);
	// RC bumping
    sys_mcu_rc_calibration();
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	
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
//	pad_mux_write(KEY1, 0);
//	pad_mux_write(KEY2, 0);
//	pad_mux_write(KEY3, 0);
//	gpi_config(KEY1, PULL_UP);
//	gpi_config(KEY2, PULL_UP);
//	gpi_config(KEY3, PULL_UP);

	//uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	
	sc_test_init();
	
	dbg_printf("SYD8821 ISO7816 Demo\r\n");
	
	
	
	__enable_irq();
		

	while(1)
	{
		sc_reader_task();
		gpo_toggle(0x400);
		//delay_ms(100);
	}		
}
