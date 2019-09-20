/*
UART����main.c

���ߣ�����ʢԴ��Ƽ����޹�˾
���ڣ�2018/3/19
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
	
	capdet_reg_write(0x00, 0x03);//��ͨ����
	capdet_reg_write(0x02, 0x04);//ͨ��0
	capdet_reg_write(0x03, 0x05);//ͨ��1
	capdet_reg_write(0x04, 0x06);//ͨ��1
	//capdet_reg_write(0x09, 0x04);//ͨ��0
	
	capdet_reg_write(0x40, 0x96);//������ֵ0-7
	capdet_reg_write(0x41, 0x00);//������ֵ8-15
	capdet_reg_write(0x42, 0x50);//�ɿ���ֵ0-7
	capdet_reg_write(0x43, 0x00);//�ɿ���ֵ8-15
	
	capdet_reg_write(0x44, 0x14);//�趨Active mode��Frame Rate
	capdet_reg_write(0x45, 0x14);//�趨idle1  mode��Frame Rate
	capdet_reg_write(0x46, 0x10);//�趨idle2  mode��Frame Rate
	capdet_reg_write(0x47, 0x10);//�趨sleep  mode��Frame Rate
	
	capdet_reg_write(0x48, 0x00);//�趨Active mode�£�δtouch �۷e����frame���M��idle 1 mode���O��0��Ԓ���Ͳ��M��idle1mode
	//capdet_reg_write(0x09, 0x00);//ͨ��7
	capdet_reg_write(0x4a, 0x7f);//�趨idel1 mode�£�δtouch �۷e����frame���M��idle 2 mode���O��0��Ԓ���Ͳ��M��idle2 mode
	capdet_reg_write(0x4b, 0x00);//�趨idel1 mode�£�δtouch �۷e����frame���M��idle 2 mode���O��0��Ԓ���Ͳ��M��idle2 mode
	capdet_reg_write(0x4c, 0x3f);//�趨idel2 mode�£�δtouch �۷e����frame���M��sleep mode���O��0��Ԓ�� �Ͳ��M��sleep mode
	capdet_reg_write(0x4d, 0x00);//�趨idel2 mode�£�δtouch �۷e����frame���M��sleep mode���O��0��Ԓ���Ͳ��M��sleep mode
	//capdet_reg_write(0x4e, 0xfe);//��Ҫ�趨Button0�����|�l���ѣ�0x4E = 0xFE;��Ҫ�O��Button7�����|�l���ѣ�0x4E = 0x7F
	capdet_reg_write(0x4e, 0x7f);//��Ҫ�趨Button0�����|�l���ѣ�0x4E = 0xFE;��Ҫ�O��Button7�����|�l���ѣ�0x4E = 0x7F
	capdet_reg_write(0x58, 0x01);
	capdet_reg_write(0x59, 0x01);//���h�趨Ϊ full cycle���趨ֵ1
	capdet_reg_write(0x5a, 0x01);//�O��Touch Force���_���B�mFrame Rate�Δ���report��һ��Key��
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

