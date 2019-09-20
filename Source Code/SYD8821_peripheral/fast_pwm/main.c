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
#include "pwm.h"
#include "ble_slave.h"

int main()
{	
	__align(4) SEQ_COMMON_TYPE pwm_compare[10];  //10组
	__disable_irq();
	gap_s_ble_init();
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// RC bumping
	sys_mcu_rc_calibration();
	// Set MCU Clock 64M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);    //内部晶振64M ROSC作为时钟源
	//sys_mcu_clock_set(MCU_CLOCK_32_MHZ_XOSC);   //外部32MHz晶振作为时钟源
	
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
	
//	//uart 1
//	pad_mux_write(4, 7);
//	pad_mux_write(5, 7);
//	uart_1_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
//	uart_write(1,"SYD8821 UART TEST", 18);

  //uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
  dbg_init();
	dbg_printf("SYD8821 FAST PWM TEST\r\n");
	
	pad_mux_write(26, 8);
	pad_mux_write(27, 8);
	pad_mux_write(28, 8);
	pad_mux_write(29, 8);
	
	pwm_stop();
	pwm_set_prescaler(2);
	pwm_set_decoder(PWM_COMMON);
	pwm_set_seq_mode(SINGLE_SEQUENCE_CONTINUE);
	pwm_compare[0].cmp_apply_all=50 | ACTIVE_LEAVE_POL;   //SEQ[0]
	pwm_compare[1].cmp_apply_all=150 | ACTIVE_LEAVE_POL;   //SEQ[1]
	pwm_compare[2].cmp_apply_all=250 | ACTIVE_LEAVE_POL;   //SEQ[2]
	pwm_compare[3].cmp_apply_all=350 | ACTIVE_LEAVE_POL;   //SEQ[3]
	pwm_compare[4].cmp_apply_all=450 | ACTIVE_LEAVE_POL;   //SEQ[4]
	pwm_compare[5].cmp_apply_all=550 | ACTIVE_LEAVE_POL;   //SEQ[5]
	pwm_compare[6].cmp_apply_all=650 | ACTIVE_LEAVE_POL;   //SEQ[6]
	pwm_compare[7].cmp_apply_all=750 | ACTIVE_LEAVE_POL;   //SEQ[7]
	pwm_compare[8].cmp_apply_all=850 | ACTIVE_LEAVE_POL;   //SEQ[8]
	pwm_compare[9].cmp_apply_all=950 | ACTIVE_LEAVE_POL;  //SEQ[9]
	pwm_set_seq(0,10,&pwm_compare);
	pwm_set_period_length(1000);
	pwm_start(0);

//	pwm_stop();
//	pwm_set_prescaler(2);
//	pwm_set_decoder(PWM_INDIVIDUAL);
//	pwm_set_seq_mode(SINGLE_SEQUENCE_CONTINUE);
//	pwm_compare[0].cmp_apply_all=50 | ACTIVE_LEAVE_POL;   //SEQ[0]
//	pwm_compare[1].cmp_apply_all=150 | ACTIVE_LEAVE_POL;   //SEQ[1]
//	pwm_compare[2].cmp_apply_all=250 | ACTIVE_LEAVE_POL;   //SEQ[2]
//	pwm_compare[3].cmp_apply_all=350 | ACTIVE_LEAVE_POL;   //SEQ[3]
//	pwm_compare[4].cmp_apply_all=450 | ACTIVE_LEAVE_POL;   //SEQ[4]
//	pwm_compare[5].cmp_apply_all=550 | ACTIVE_LEAVE_POL;   //SEQ[5]
//	pwm_compare[6].cmp_apply_all=650 | ACTIVE_LEAVE_POL;   //SEQ[6]
//	pwm_compare[7].cmp_apply_all=750 | ACTIVE_LEAVE_POL;   //SEQ[7]
//	pwm_compare[8].cmp_apply_all=850 | ACTIVE_LEAVE_POL;   //SEQ[8]
//	pwm_compare[9].cmp_apply_all=950 | ACTIVE_LEAVE_POL;  //SEQ[9]
//	pwm_set_seq(0,2,&pwm_compare);
//	pwm_set_period_length(1000);
//	pwm_start(0);
	
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
		if(!gpi_get_val(KEY1)){
		  pwm_stop();
			dbg_printf("pwm_stop\r\n");
			delay_ms(1000);
		}
		if(!gpi_get_val(KEY2)){
			pwm_set_decoder(PWM_COMMON);
			pwm_set_seq_mode(SINGLE_SEQUENCE_CONTINUE);
			pwm_compare[0].cmp_apply_all=50 | ACTIVE_LEAVE_POL;   //SEQ[0]
			pwm_compare[1].cmp_apply_all=150 | ACTIVE_LEAVE_POL;   //SEQ[1]
			pwm_compare[2].cmp_apply_all=250 | ACTIVE_LEAVE_POL;   //SEQ[2]
			pwm_compare[3].cmp_apply_all=350 | ACTIVE_LEAVE_POL;   //SEQ[3]
			pwm_compare[4].cmp_apply_all=450 | ACTIVE_LEAVE_POL;   //SEQ[4]
			pwm_compare[5].cmp_apply_all=550 | ACTIVE_LEAVE_POL;   //SEQ[5]
			pwm_compare[6].cmp_apply_all=650 | ACTIVE_LEAVE_POL;   //SEQ[6]
			pwm_compare[7].cmp_apply_all=750 | ACTIVE_LEAVE_POL;   //SEQ[7]
			pwm_compare[8].cmp_apply_all=850 | ACTIVE_LEAVE_POL;   //SEQ[8]
			pwm_compare[9].cmp_apply_all=950 | ACTIVE_LEAVE_POL;  //SEQ[9]
		  pwm_set_seq(0,10,&pwm_compare);
	    pwm_set_period_length(1000);
//				pwm_set_decoder(PWM_INDIVIDUAL);
//			pwm_set_seq_mode(SINGLE_SEQUENCE_CONTINUE);
//			pwm_compare[0].cmp_apply_all=50 | ACTIVE_LEAVE_POL;   //SEQ[0]
//			pwm_compare[1].cmp_apply_all=150 | ACTIVE_LEAVE_POL;   //SEQ[1]
//			pwm_compare[2].cmp_apply_all=250 | ACTIVE_LEAVE_POL;   //SEQ[2]
//			pwm_compare[3].cmp_apply_all=350 | ACTIVE_LEAVE_POL;   //SEQ[3]
//			pwm_compare[4].cmp_apply_all=450 | ACTIVE_LEAVE_POL;   //SEQ[4]
//			pwm_compare[5].cmp_apply_all=550 | ACTIVE_LEAVE_POL;   //SEQ[5]
//			pwm_compare[6].cmp_apply_all=650 | ACTIVE_LEAVE_POL;   //SEQ[6]
//			pwm_compare[7].cmp_apply_all=750 | ACTIVE_LEAVE_POL;   //SEQ[7]
//			pwm_compare[8].cmp_apply_all=850 | ACTIVE_LEAVE_POL;   //SEQ[8]
//			pwm_compare[9].cmp_apply_all=950 | ACTIVE_LEAVE_POL;  //SEQ[9]
//			pwm_set_seq(0,2,&pwm_compare);
//			pwm_set_period_length(1000);
		  pwm_start(0);
			dbg_printf("pwm_start\r\n");
			delay_ms(1000);
		}
		delay_ms(100);
	}		
}
