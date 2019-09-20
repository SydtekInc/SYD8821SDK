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
#include "rtc.h"
#include "ble_slave.h"

#define RTC_CTRL    ((RTC_CTRL_TYPE2  *) RTC_CTRL_BASE)
uint32_t RTC_second_now = 0;

uint32_t RTC_RUN(void)
{
	RTC_TIME_TYPE time;
	uint32_t second=0,second1=0;
	time=rtc_get_calendar();
	//dbg_printf("t%d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
	second=(time.decimal_format.day-1)*86400+ time.decimal_format.hour*3600+time.decimal_format.minute*60+time.decimal_format.second;
	if(second>=RTC_second_now){
		  second1=second-RTC_second_now;
		  RTC_second_now=second;
	    return second1;
	}else{
	  	second1=second+604800-RTC_second_now;
		  RTC_second_now=second;
	    return second1;
	}
}
void RTC_SET(uint32_t Iterval)
{
	RTC_TIME_TYPE time;
	uint32_t second=Iterval+RTC_second_now;
	//init compare
  time.decimal_format.day    = second/86400+1;
	time.decimal_format.hour   = second%86400/3600;
	time.decimal_format.minute = second%3600/60;
	time.decimal_format.second = second%60;
	dbg_printf("C%d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
	rtc_set_compare(0,&time);
}

void rtc_irq_cb(RTC_INT_TYPE type)
{
    RTC_TIME_TYPE time;
    time=rtc_get_calendar();
	  //dbg_printf("type:%x\r\n",type);
    if(RTC_INT_CMP0&type){
			uint32_t second=0;
			second=RTC_RUN();
			RTC_SET(1);
			gpo_toggle(LED1);
			dbg_printf("c0 %d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
		}
	  if (RTC_INT_CMP1&type){
			dbg_printf("c1 %d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
		}
	  if (RTC_INT_TICK&type){
			//dbg_printf("tk %d %d:%d:%d    %x\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second,RTC_CTRL->COUNTER);
			dbg_printf("tk %d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
		}
		gpo_toggle(LED2);
}

int main()
{	
	RTC_TIME_TYPE time;
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
	
// uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	dbg_printf("SYD8821 RTC TEST\r\n");
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	
	// RTC
	//init calendar
	time.decimal_format.day    = 7;
	time.decimal_format.hour   = 23;
	time.decimal_format.minute = 58;
	time.decimal_format.second = 50;
	rtc_set_calendar(&time);
	
	//init compare0
	time.decimal_format.day    = 7;
	time.decimal_format.hour   = 23;
	time.decimal_format.minute = 58;
	time.decimal_format.second = 55;
	rtc_set_compare(0,&time);
  	
	rtc_stop();
	rtc_clear();
	rtc_int_clear(RTC_INT_ALL);
	rtc_int_disable(RTC_INT_ALL);
	rtc_set_prescaler(32768/1024, 1);  //1HZ
	//rtc_set_prescaler(32768/1000, 0);  //1000HZ,快速查看RTC流程

	rtc_int_enable(RTC_INT_CMP0 | RTC_INT_CMP1);
	//rtc_int_enable(RTC_INT_ALL);
	rtc_set_interrupt_callback(rtc_irq_cb);
	
	if(rtc_status()==false)
	{
		rtc_start();
	}
	NVIC_EnableIRQ(RTC_IRQn);
	
	__enable_irq();
	
//	time=rtc_get_calendar();
//	DBG("rtc_start %d %d:%d:%d\r\n",time.decimal_format.day,time.decimal_format.hour,time.decimal_format.minute,time.decimal_format.second);
	while(1)
	{
		gpo_toggle(LED4);
		if(!gpi_get_val(KEY1)){
		  uint32_t second=0;
			second=RTC_RUN();
			RTC_SET(second+5);
		  gpo_toggle(LED3);
			delay_ms(1000);
		}
	}		
}
