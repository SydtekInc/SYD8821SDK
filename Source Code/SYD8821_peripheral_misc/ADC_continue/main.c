/*
UART����main.c

���ߣ�����ʢԴ��Ƽ����޹�˾
���ڣ�2018��12��4��
ADCģ���DMA���ܵ�ʹ��ʹ��:
ADC��DMAģ��Ƚϸ��ӣ��򵥵��������Ĵ������ƣ�
��һ����gpadc_set_average_sample_number�ӿ����õ�GPADC_CTRL->AVERAGE��������Ϊaverage_sample_number��
�ڶ�����gpadc_set_data_length�ӿ����õ�GPADC_CTRL->DATA_LENGTH��������Ϊdata_length
��������gpadc_start�ӿڵĵڶ����������õ�GPADC_CTRL->SCAN_COUNT������Ϊscan_cnt
�����߼��������ģ�
	ADC��DMA��󴢴����ڴ��ϵ�����ÿ��ͨ��ռ��4��byte,��������ʹ��DMA�Ļ��������㹻���uint32_t���飬���籾����
�е�adc_cintinue_dma��������ÿһ��Ԫ�ش�������Ӧͨ�������ݣ���������Ǽ�average_sample_number���ڸ�ͨ����
�����ĺ�ֵ��
	�ڱ�������һ��ʹ���ĸ�ͨ��VBAT_CHANNEL,GPIO0_CHANNEL,GPIO1_CHANNEL,GPIO2_CHANNEL����adc_continue_init��
���ã���data_length/average_sample_number�������ĳ��ͨ��Ҫ�洢���ĸ���������data_lengthΪ8��
average_sample_numberΪ4��������ָ������ƣ�����GPADC_SUM_FOUR���������4������ôadc_cintinue_dma�ĵ�һ��
������VBAT_CHANNEL�ϵ�һ�β���4��ADC�ĺ�ֵ��adc_cintinue_dma�ĵڶ���������VBAT_CHANNEL�ϵڶ��β���4��ADC�ĺ�ֵ��
�Դ����ƣ�adc_cintinue_dma�ĵ�����������GPIO0_CHANNEL�ϵ�һ�β���4��ADC�ĺ�ֵ��adc_cintinue_dma�ĵڶ���������
GPIO0_CHANNEL�ϵڶ��β���4��ADC�ĺ�ֵ������������������
	Ҳ����˵adc_cintinue_dma�е�Ԫ��Ҫ����4��������ADC��ֵ��������ΪADCֵ�������Ϊ�������ʽ������ADCתΪ�޷�����
����Ҫ�������µ�ת����adc_value=((adc_cintinue_dma[0]/4 + 0x200)&0x3ff);��
	���һ��������scan_cnt���ò���ָ����ͨ��ת���Ĵ����������ڸù�����һ�����ĸ�ͨ��������scan_cnt����Ҫ����Ϊ4����
����ͨ���Ż�ת������ȷ�����ݣ������ֵ����Ϊ1����ôֻ�е�һ��ͨ�������ݱ�ת����������������ͨ��������ת����
	����scan_cnt��һ��Ҫע�⣬���Ҫת����ͨ����Ϊ4��������ֵΪ5����ô�����ת��ADC��ֵ�������adc_cintinue_dma��
�����Ԫ���У������ǵ�һ��Ԫ�أ���������Ҫ�����ڴ�����������
	ADCģ���е��ж�����������GPADC_INT_TASK��GPADC_INT_HALF�Լ�GPADC_INT_FULL�����ǵĶ������£�
GPADC_INT_TASK:��scan_cntת����ɵ�ʱ��Ӳ�����ϱ����¼�
GPADC_INT_HALF:ADC��ʼ����ʱ������gpadc_set_DMA�ӿڸ��ߵײ��ж��ٸ��ڴ����ݹ�DMAʹ�ã���������ָ���ǣ�adc_cintinue_mem_maxcnt
			   ��DMA������adc_cintinue_mem_maxcnt/2�����ݵ�ʱ���ϱ����¼�
GPADC_INT_FULL����DMA������adc_cintinue_mem_maxcnt�����ݵ�ʱ���ϱ����¼�
	��һ��Ҫע�⣺�ڱ�������adc_cintinue_mem_maxcntΪ64,�������̵�average_sample_number��ata_length�Լ�scan_cnt
��Ϊ4����ô������DMA��ഫ���������ata_length/average_sample_number*scan_cntҲ����4��uint32_t���������̵�adc_cintinue_mem_maxcnt
ȴ��64,��ôҪ�ﵽGPADC_INT_HALF����������Ҫ����32�����ݣ�����������ԶҲ�����ܴﵽ32�����ݵ�DMA�����Ա�������ֻ�ܹ�
����GPADC_INT_TASK�жϣ�
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
		voltage_value = voltage_value * 4.3/1.7;			 //adc_voltage��ѹ����*4.3/1.7
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
	
	gpadc_set_channel(4,ch);   //�ж�ģʽͨ��ͳһ����
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
//		voltage_value  = (float)adc_value * 3.6 / 1023;//adc�ɼ��ĵ�ѹ
//		voltage_value = voltage_value * 4.3/1.7;			 //adc_voltage��ѹ����*4.3/1.7
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
