#include "delay.h"
#include "gpadc.h"
#include "debug.h"
#include "clk.h"
#include <string.h>

#define    GPADC_CTRL    ((GPADC_CTRL_TYPE *)GPADC_CTRL_BASE)

static GPADC_IRQ_CALLBACK_TYPE callback;
static uint32_t average_sample_number = 1;

//uint8_t adc_st = 0;
__align(4) uint8_t adc_dma[4] = {0};
uint32_t adc_mem_maxcnt = sizeof(adc_dma);


void GPADC_IRQHandler(void) {
	
    uint32_t intst = GPADC_CTRL->INT_ST;
    GPADC_CTRL->INT_ST = ~intst;
    if (callback) 
			callback(intst);
    
    if (intst & GPADC_INT_ALL) {
        GPADC_CTRL->DA_ADCGP_EN = 0;
    }
}

void gpadc_set_irq_callback(GPADC_IRQ_CALLBACK_TYPE cb)
{
    callback = cb;
}

void gpadc_enable_int(int en, GPADC_INT_TYPE int_mask)
{
    if (en) {
        GPADC_CTRL->INT_EN_SET = int_mask;
    }
    else {
        GPADC_CTRL->INT_EN_CLR = int_mask;
    }
}

uint32_t gpadc_get_DMA_len(void)
{
    return GPADC_CTRL->GPADC_MEM_MAXCNT * 4;
}

bool gpadc_set_DMA(void *ptr, uint32_t length)
{
    // Must be multiple of 4 bytes
    if (length & 3)
        return false;
    
    GPADC_CTRL->DATA_PTR = (uint32_t)ptr;
    GPADC_CTRL->GPADC_MEM_MAXCNT = (length / 4);
    return true;
}

void gpadc_start(int continuous, uint32_t scan_cnt)
{
    GPADC_CTRL->INT_ST = 0;
    GPADC_CTRL->CONTINUE_SCAN = (continuous != 0);
    GPADC_CTRL->SCAN_COUNT = scan_cnt;
    
    GPADC_CTRL->DA_ADCGP_EN = 1;
    GPADC_CTRL->TASK_START = 1;
    while (GPADC_CTRL->TASK_START);
}

void gpadc_stop(void)
{
    GPADC_CTRL->TASK_STOP = 1;
    while (GPADC_CTRL->TASK_STOP) ;
}

void gpadc_set_channel_timing(uint32_t start, uint32_t channel)
{
    GPADC_CTRL->START_SETTLE = start;
    GPADC_CTRL->CHANNEL_SETTLE = channel;
}

void gpadc_set_data_length(uint32_t data_length)
{
    if (data_length == 0)
        data_length = 1;
		
    GPADC_CTRL->DATA_LENGTH = data_length - 1;
}

uint32_t gpadc_get_data_length(void)
{
    return GPADC_CTRL->DATA_LENGTH + 1;
}

void gpadc_set_channel(int num, GPADC_CHANNEL_TYPE * ch)
{
    uint32_t configuration = 0;
    int i;
    
    GPADC_CTRL->CHANNEL_SET_NUM = num ;
    
    // SET 0 for candidate channel 0~7
    for (i = 0; i < num && i <= 7; i++, ch++) 
	  {
        configuration |= ((*ch & CHANNEL_BIT_MASK) << (i * 4));
    }
    GPADC_CTRL->CHANEL_SET[0] = configuration;
    
    if (num <= 7)  return;
    
    num -= 8;
    configuration = 0;
    // SET 1 configuration
    for (i = 0; i < num && i <= 3; i++, ch++) {
        configuration |= ((*ch & CHANNEL_BIT_MASK) << (i * 4));
    }
    GPADC_CTRL->CHANEL_SET[1] = configuration;
}

uint32_t gpadc_get_average_sample_number(void)
{
    return average_sample_number;
}

void gpadc_set_average_sample_number(GPADC_AVERAGE_TYPE t)
{
    GPADC_CTRL->AVERAGE = t;
    average_sample_number = 1 << ((t - 1) / 2);
}

void gpadc_set_channel_clock(GPADC_CHANNEL_CLK_TYPE c)
{
    uint32_t target_clk = (c != GPADC_CLK_1M) ? 500000 : 1000000;
    
    GPADC_CTRL->CLKRATE = (HCLK / target_clk) / 2 - 1;
}

uint32_t gpadc_get_channel_clock(void)
{
    return HCLK / ((GPADC_CTRL->CLKRATE + 1) * 2);
}



void adc_irq_callback(uint32_t int_st)
{
	if(GPADC_INT_TASK & int_st)
	{
		
	} else if(GPADC_INT_HALF & int_st)
	{
		
	} else if(GPADC_INT_FULL & int_st)
	{
		
	}
}

void adc_init(void)
{
  gpadc_set_DMA(adc_dma, adc_mem_maxcnt);
	gpadc_set_channel_clock(GPADC_CLK_1M);
	gpadc_set_average_sample_number(GPADC_DISABLE_AVERAGE);
	gpadc_set_data_length(1);	
//	gpadc_set_average_sample_number(GPADC_SUM_TWO);
//	gpadc_set_data_length(gpadc_get_average_sample_number());	
	
//	gpadc_set_irq_callback(adc_irq_callback);
//	gpadc_enable_int(1, GPADC_INT_ALL);
	gpadc_enable_int(0, GPADC_INT_ALL);
	
	((EFUSE_CTRL_TYPE *)EFUSE0_CTRL_BASE)->EFUSE_MAN_EN = 1;
	((EFUSE_CTRL_TYPE *)EFUSE0_CTRL_BASE)->EFUSE_MAN_VAL = 0x88;
 // NVIC_EnableIRQ(GPADC_IRQn);
}


void adc_open(GPADC_CHANNEL_TYPE channel)
{
	gpadc_set_channel(1,&channel);
	gpadc_set_channel_timing(0x18, channel);
}

uint16_t gpadc_get_value(void)
{
	gpadc_start(0,1);//¿ªÊ¼×ª»»
	//while(GPADC_CTRL->DA_ADCGP_EN);
	//return  GPADC_CTRL->ADC_DATA_HCLK;
	return((GPADC_CTRL->ADC_DATA_HCLK + 0x200)&0x3ff);
}


