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
#include "stdio.h"
#include "capdet_alg_lib.h"
#include "ble_slave.h"
#include "timer.h"
#include <stdbool.h>
#include <string.h>


#define SYS_CTRL ((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)
#define READ_FRAME_EXAMPLE 1
#define MCU_CLOCK_MHZ 96

CAPDET_ALG_CONTEXT_TYPE g_context;
CAPDET_ALG_CONFIG_TYPE g_reg;

static uint8_t fclk_change_table[12] = {0,1,0,2,0,3,1,2,1,3,2,3};	//fclk_change_table contains every case of fclk switching
static uint8_t tmp = 0;
static uint8_t count;
static volatile uint32_t sys_tick = 0;
bool int_get = 0;


static void fclk_change_callback(void)
{
	if (count == 5) {
        uint8_t fclk_tmp;
		capdet_stop();
        fclk_tmp=SYS_CTRL->FCLK_CONFIG;
		DBG("FCLK_CONFIG = %d\r\n", fclk_change_table[tmp]);
		SYS_CTRL->FCLK_CONFIG = fclk_change_table[tmp];
        DBG("%d->%d\r\n",(MCU_CLOCK_MHZ>>fclk_tmp),(MCU_CLOCK_MHZ>>fclk_change_table[tmp]) );
        capdet_sysclk_change( (MCU_CLOCK_MHZ>>fclk_tmp),(MCU_CLOCK_MHZ>>fclk_change_table[tmp]));
		tmp = (tmp+1)%12;
		count = 0;
		capdet_start();
	}
	else 
	{
		count++;
	}
}


static void timer1_irq_callback(void)
{
	sys_tick++;
}


void mcu_clock_init(void)
{
	gap_s_ble_init();
    // Select External XO
    sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
    // Set MCU Clock 96M
    sys_mcu_clock_set(MCU_CLOCK_96_MHZ);
    // RC bumping
    sys_mcu_rc_calibration();

}

void system_clock_init(void)
{
    // Use timer1 as system tick with interval 1ms
    timer_enable(TIMER_1, timer1_irq_callback, 33, 1);
    NVIC_EnableIRQ(TIMER1_IRQn);
	
	timer_enable(TIMER_2, fclk_change_callback, 32767, 1);
	NVIC_EnableIRQ(TIMER2_IRQn);
}

void fake_frm_init()
{
	//uint8_t i;
	
	g_context.header[0] = HEADER_PATTERN0;
	g_context.header[1] = HEADER_PATTERN1;
	g_context.header[2] = HEADER_PATTERN2;
	g_context.header[3] = HEADER_PATTERN3;
	/*
	for(i=0; i<CELL_MAX_NUM; i++)
	{
		g_context.frm[i] = 0x3031;
		g_context.debug_frm[i] = 0x3233;
	}
	
	g_context.btn = 0x3435;
	//g_context.btn_pkg_idx = 0x36;
	g_context.btn_pkg_idx = 0x00;
	//g_context.frm_pkg_idx = 0x37;
	g_context.frm_pkg_idx = 0x00;
	g_context.op_mode = 0x38;
	*/
}



void capdet_init_config()
{
	
	fake_frm_init();
	capdet_init();	
	capdet_reg_read_all(&g_reg);
#if (READ_FRAME_EXAMPLE)
	//g_reg.debug_frm_show = 3;
	g_reg.frm_show = true;
	capdet_reg_write_all(&g_reg);
#endif	
	
	capdet_reg_write(0x00, 0x03);//总通道数
	capdet_reg_write(0x02, 0x04);//通道0
	capdet_reg_write(0x03, 0x05);//通道1
	capdet_reg_write(0x04, 0x06);//通道1
	//capdet_reg_write(0x09, 0x04);//通道0
	
	capdet_reg_write(0x40, 0x96);//按下阈值0-7
	capdet_reg_write(0x41, 0x00);//按下阈值8-15
	capdet_reg_write(0x42, 0x50);//松开阈值0-7
	capdet_reg_write(0x43, 0x00);//松开阈值8-15
	
	capdet_reg_write(0x44, 0x14);//设定Active mode下Frame Rate
	capdet_reg_write(0x45, 0x14);//设定idle1  mode下Frame Rate
	capdet_reg_write(0x46, 0x10);//设定idle2  mode下Frame Rate
	capdet_reg_write(0x47, 0x10);//设定sleep  mode下Frame Rate
	
	capdet_reg_write(0x48, 0x00);//设定Active mode下，未touch 累積多少frame就進入idle 1 mode，設定0的話，就不進入idle1mode
	//capdet_reg_write(0x09, 0x00);//通道7
	capdet_reg_write(0x4a, 0x7f);//设定idel1 mode下，未touch 累積多少frame就進入idle 2 mode，設定0的話，就不進入idle2 mode
	capdet_reg_write(0x4b, 0x00);//设定idel1 mode下，未touch 累積多少frame就進入idle 2 mode，設定0的話，就不進入idle2 mode
	capdet_reg_write(0x4c, 0x3f);//设定idel2 mode下，未touch 累積多少frame就進入sleep mode，設定0的話， 就不進入sleep mode
	capdet_reg_write(0x4d, 0x00);//设定idel2 mode下，未touch 累積多少frame就進入sleep mode，設定0的話，就不進入sleep mode
	//capdet_reg_write(0x4e, 0xfe);//若要设定Button0可以觸發喚醒，0x4E = 0xFE;若要設定Button7可以觸發喚醒，0x4E = 0x7F
	capdet_reg_write(0x4e, 0x7f);//若要设定Button0可以觸發喚醒，0x4E = 0xFE;若要設定Button7可以觸發喚醒，0x4E = 0x7F
	capdet_reg_write(0x58, 0x01);
	capdet_reg_write(0x59, 0x01);//建議设定为 full cycle，设定值1
	capdet_reg_write(0x5a, 0x01);//設定Touch Force到達的連續Frame Rate次數才report為一個Key，
	capdet_start();
	NVIC_EnableIRQ(CAPDET_IRQn);
}

uint32_t get_tick_count(void)
{
	return sys_tick;
}


int main()
{	
	__disable_irq();
	
	mcu_clock_init();
	system_clock_init();

	
	
	//GPIO 25-23-34-10
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
	gpo_config(LED4,1);
	
	//GPIO 14-15-16-17
//	pad_mux_write(KEY1, 0);
//	pad_mux_write(KEY2, 0);
//	pad_mux_write(KEY3, 0);
//	pad_mux_write(KEY4, 0);
//	gpi_config(KEY1, PULL_UP);
//	gpi_config(KEY2, PULL_UP);
//	gpi_config(KEY3, PULL_UP);
//	gpi_config(KEY4, PULL_UP);
	
	//CAPDET 14-15-16
	pad_mux_write(GPIO_14, 1);
	pad_mux_write(GPIO_15, 1);
	pad_mux_write(GPIO_16, 1);
	
	//uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	dbg_printf("SYD8821 Touch Demo \r\n");
	
	capdet_init_config();

	__enable_irq();
	
	
	while(1)
	{
		gpo_toggle(LED4);
		int_get = capdet_process(&g_context);//pulling rate >= cap detect frame rate
		
		if (int_get) //true if state chage
		{	
			
			int_get = 0;
			dbg_printf("btn = %x\t", g_context.btn);
			dbg_printf("btn = %x\t", g_context.header);
			dbg_printf("btn_pkg = %x\n", g_context.btn_pkg_idx);
			
			if(g_context.btn & 0x01)
				gpo_clr(LED1);
			else
				gpo_set(LED1);
			
			if(g_context.btn & 0x02)
				gpo_clr(LED2);
			else
				gpo_set(LED2);
			
			if(g_context.btn & 0x04)
				gpo_clr(LED3);
			else
				gpo_set(LED3);
			
		}
		//delay_ms(1000);
	}		
}


void CAPDET_IRQHandler(void)
{	
	dbg_printf("CAPDET_IRQHandler\r\n");
	capdet_int_handle(get_tick_count());
	
}

