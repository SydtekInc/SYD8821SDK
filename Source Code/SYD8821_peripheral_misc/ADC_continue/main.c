/*
UART例程main.c

作者：北京盛源达科技有限公司
日期：2018年12月4日
ADC模块的DMA功能的使用使用:
ADC的DMA模块比较复杂，简单的由三个寄存器控制：
第一个是gpadc_set_average_sample_number接口设置的GPADC_CTRL->AVERAGE，这里简称为average_sample_number，
第二个是gpadc_set_data_length接口设置的GPADC_CTRL->DATA_LENGTH，这里简称为data_length
第三个是gpadc_start接口的第二个参数设置的GPADC_CTRL->SCAN_COUNT这里简称为scan_cnt
控制逻辑是这样的：
	ADC的DMA最后储存在内存上的数据每个通道占用4个byte,所以这里使用DMA的话会声明足够多的uint32_t数组，比如本工程
中的adc_cintinue_dma该数组中每一个元素代表了相应通道的数据，这个数据是际average_sample_number次在该通道上
采样的和值。
	在本工程中一共使用四个通道VBAT_CHANNEL,GPIO0_CHANNEL,GPIO1_CHANNEL,GPIO2_CHANNEL（在adc_continue_init中
设置），data_length/average_sample_number代表的是某个通道要存储到的个数，比如data_length为8，
average_sample_number为4（这里是指宏的名称，比如GPADC_SUM_FOUR就是这里的4），那么adc_cintinue_dma的第一个
数据是VBAT_CHANNEL上第一次采样4次ADC的和值，adc_cintinue_dma的第二个数据是VBAT_CHANNEL上第二次采样4次ADC的和值，
以此类推，adc_cintinue_dma的第三个数据是GPIO0_CHANNEL上第一次采样4次ADC的和值，adc_cintinue_dma的第二个数据是
GPIO0_CHANNEL上第二次采样4次ADC的和值，。。。。。。。。
	也就是说adc_cintinue_dma中的元素要除以4才是真正ADC的值，并且因为ADC值这里表现为补码的形式，所以ADC转为无符号数
还需要经过如下的转换：adc_value=((adc_cintinue_dma[0]/4 + 0x200)&0x3ff);。
	最后一个参数是scan_cnt，该参数指的是通道转换的次数，比如在该工程中一共有四个通道，所以scan_cnt必须要设置为4以上
所有通道才会转换出正确的数据，如果该值设置为1，那么只有第一个通道的数据被转换出来，后面三个通道都不会转换。
	关于scan_cnt有一点要注意，如果要转换的通道数为4，而该项值为5，那么第五次转换ADC的值将存放在adc_cintinue_dma的
第五个元素中，而不是第一个元素！所以这里要避免内存溢出的情况！
	ADC模块中的中断有三个类型GPADC_INT_TASK、GPADC_INT_HALF以及GPADC_INT_FULL，他们的定义如下：
GPADC_INT_TASK:当scan_cnt转换完成的时候硬件将上报该事件
GPADC_INT_HALF:ADC初始化的时候会调用gpadc_set_DMA接口告诉底层有多少个内存数据供DMA使用，本工程中指的是：adc_cintinue_mem_maxcnt
			   当DMA传输完adc_cintinue_mem_maxcnt/2的数据的时候将上报该事件
GPADC_INT_FULL：当DMA传输完adc_cintinue_mem_maxcnt的数据的时候将上报该事件
	有一点要注意：在本工程中adc_cintinue_mem_maxcnt为64,而本工程的average_sample_number、ata_length以及scan_cnt
都为4，那么本工程DMA最多传输的数据是ata_length/average_sample_number*scan_cnt也就是4个uint32_t，而本工程的adc_cintinue_mem_maxcnt
却是64,那么要达到GPADC_INT_HALF的条件必须要传输32个数据，而本工程永远也不可能达到32个数据的DMA，所以本工程中只能够
看到GPADC_INT_TASK中断！
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
#include "gpadc.h"
__align(4) uint32_t adc_cintinue_dma[64] = {0};
uint32_t * p_dma=adc_cintinue_dma;
uint32_t adc_cintinue_mem_maxcnt = sizeof(adc_cintinue_dma);

void adc_cintinue_irq_callback(uint32_t int_st)
{
	uint16_t adc_value = 0;
	float voltage_value = 0;
	if(GPADC_INT_TASK & int_st)
	{
		dbg_hexdump("INT TASK:\r\n", (void *)adc_cintinue_dma, 32);
		
		adc_value=((adc_cintinue_dma[0]/4 + 0x200)&0x3ff);;
		voltage_value = ((float)adc_value * 3.6) / 1023 ;
		voltage_value = voltage_value * 4.3/1.7;			 //adc_voltage分压倍数*4.3/1.7
		dbg_printf("CHANNEL  VBAT adc:%x voltage:%04f \t",  adc_value, voltage_value);
		
		adc_value=((adc_cintinue_dma[1]/4 + 0x200)&0x3ff);;
		voltage_value = ((float)adc_value * 3.6) / 1023 ;
		dbg_printf("GPIO0 adc:%x voltage:%04f \t",  adc_value, voltage_value);
		
		adc_value=((adc_cintinue_dma[2]/4 + 0x200)&0x3ff);;
		voltage_value = ((float)adc_value * 3.6) / 1023 ;
		dbg_printf("GPIO1 adc:%x voltage:%04f \t",  adc_value, voltage_value);
		
		adc_value=((adc_cintinue_dma[3]/4 + 0x200)&0x3ff);;
		voltage_value = ((float)adc_value * 3.6) / 1023 ;
		dbg_printf("GPIO2 adc:%x voltage:%04f \t",  adc_value, voltage_value);
	} else if(GPADC_INT_HALF & int_st)
	{
		dbg_hexdump("INT HALF:\r\n", (void *)adc_cintinue_dma, 32);
	} else if(GPADC_INT_FULL & int_st)
	{
		dbg_hexdump("INT FULL:\r\n", (void *)adc_cintinue_dma, 32);
	}
}

void adc_continue_init(void)
{
	GPADC_CHANNEL_TYPE ch[4]={VBAT_CHANNEL,GPIO0_CHANNEL,GPIO1_CHANNEL,GPIO2_CHANNEL};
	gpadc_set_DMA(adc_cintinue_dma, adc_cintinue_mem_maxcnt);
	gpadc_set_channel_clock(GPADC_CLK_1M);
//	gpadc_set_average_sample_number(GPADC_DISABLE_AVERAGE);
//	gpadc_set_data_length(1);	
	gpadc_set_average_sample_number(GPADC_SUM_FOUR);
	gpadc_set_data_length(gpadc_get_average_sample_number());	
	
	gpadc_set_channel(4,ch);   //中断模式通道统一设置
	gpadc_set_channel_timing(0x18, 0x04);
	
	gpadc_set_irq_callback(adc_cintinue_irq_callback);
	gpadc_enable_int(1, GPADC_INT_ALL);
	
	((EFUSE_CTRL_TYPE *)EFUSE0_CTRL_BASE)->EFUSE_MAN_EN = 1;
	((EFUSE_CTRL_TYPE *)EFUSE0_CTRL_BASE)->EFUSE_MAN_VAL = 0x88;
	NVIC_EnableIRQ(GPADC_IRQn);
}

int main()
{	
//	uint16_t adc_value = 0;
//	float voltage_value = 0;
	
	__disable_irq();
	
	gap_s_ble_init();
	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// Set MCU Clock 96M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	// RC bumping
	sys_mcu_rc_calibration();	

//uart 0
	pad_mux_write(GPIO_20, 7);
	pad_mux_write(GPIO_21, 7);
	dbg_init();
	dbg_printf("Syd8821 ADC Continue Demo %s:%s\r\n",__DATE__ ,__TIME__);

//ADC GPIO
	pad_mux_write(GPIO_0, 1);
	pad_mux_write(GPIO_1, 1);
	pad_mux_write(GPIO_2, 1);

	//adc_init();
	adc_continue_init();
	__enable_irq();	
	
	//get VBAT 
	
	
	while(1)
	{	
//	 	adc_open(VBAT_CHANNEL); //select VBAT_CHANNEL	
//		adc_value = gpadc_get_value();
//		voltage_value  = (float)adc_value * 3.6 / 1023;//adc采集的电压
//		voltage_value = voltage_value * 4.3/1.7;			 //adc_voltage分压倍数*4.3/1.7
//		dbg_printf("CHANNEL  VBAT adc:%x voltage:%04f \t",  adc_value, voltage_value);

//		//get GPIO4_CHANNEL	
//		adc_open(GPIO4_CHANNEL); //select GPIO4_CHANNEL
//		adc_value = gpadc_get_value();
//		voltage_value = ((float)adc_value * 3.6) / 1023 ;
//		dbg_printf("GPIO4 adc:%x voltage:%04f\r\n", adc_value,voltage_value);
		
		gpadc_start(0, 4);
		delay_ms(1000);
	}		
}
