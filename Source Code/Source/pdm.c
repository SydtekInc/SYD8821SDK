

#include "pdm.h"


#define    PDM_CTRL    ((PDM_CTRL_TYPE *) PDM_CTRL_BASE)

static PDM_IRQ_CALLBACK_TYPE irq_cb;
static uint8_t sw_gain[PDM_GAIN_CHANNEL_NUM];


void PDM_IRQHandler(void)
{
    uint8_t int_st = PDM_CTRL->INT_STATUS;
    
    PDM_CTRL->INT_STATUS = int_st;
    if (irq_cb) {
        irq_cb(int_st);
    }
}

void pdm_set_irq_callback(PDM_IRQ_CALLBACK_TYPE cb)
{
    irq_cb = cb;
}

void pdm_enable_int(PDM_INT_STATUS_TYPE int_en)
{
    PDM_CTRL->INT_MASK = int_en ^ PDM_DMA_ALL_INT;
}

void pdm_start(void)
{
	if(PDM_CTRL->R_PDM_EN==0)
    PDM_CTRL->R_PDM_EN = 1;
}

void pdm_stop(void)
{
	if(PDM_CTRL->R_PDM_EN==1)
    PDM_CTRL->R_PDM_EN = 0;
}

bool pdm_set_DMA(void *ptr, int length)
{
    uint32_t address = (uint32_t)ptr;
    if (address & 0x3)
        return false;
    
    if (address) {
        PDM_CTRL->R_PDM_PTR = address;
    }
    PDM_CTRL->R_PDM_MAXCNT = length;
    return true;
}

void pdm_set_channel(PDM_CHANNEL_TYPE c)
{
    PDM_CTRL->R_PDM_MODE_LR = c;
}

void pdm_set_sample_width(PDM_SAMPLE_WIDTH_TYPE w)
{
    PDM_CTRL->R_PDM_SAMPLE_WIDTH = w;
}

void pdm_set_clk(PDM_CLK_CTRL_TYPE c)
{
    PDM_CTRL->R_PDMCLKCTRL = c;
}

void pdm_set_sample_rate(PDM_SAMPLE_RATE_TYPE s)
{
    PDM_CTRL->R_PDM_SAMPLE_RATE = s;
}

void pdm_set_sample_trigger(PDM_SAMPLE_TRIGGER_TYPE t)
{
    PDM_CTRL->R_PDM_MODE_CLK = t;
}

void pdm_set_dc_mode(int en, PDM_DC_MODE_TYPE m, int period)
{
    if (en) {
        PDM_CTRL->R_DC_MODE = m;
        PDM_CTRL->R_DC_PERIOD = period;
    }
    PDM_CTRL->R_DC_ONOFF = (en == 0);
}

void pdm_software_scaling(PDM_GAIN_CHANNEL_TYPE t, int16_t * data)
{
    if (!sw_gain[t])
        return;
    
    *data >>= sw_gain[t];
}

uint8_t pdm_get_sw_gain(PDM_GAIN_CHANNEL_TYPE t)
{
    if (t == PDM_GAINL)
        return sw_gain[PDM_GAINL];
    return sw_gain[PDM_GAINR];
}

static bool pdm_valid_gain(uint8_t gain)
{
    return (gain >= 0x11 && gain <= 0x1B) || gain >= 0x28;
}

void pdm_set_gain(PDM_GAIN_CHANNEL_TYPE t, uint8_t gain)
{
    if (t >= PDM_GAIN_CHANNEL_NUM)
        return;
    
    sw_gain[t] = 0;
    
    while (!pdm_valid_gain(gain)) {
        // 6db unit for shift
        gain += 12;
        sw_gain[t]++;
    }
    
    if (t == PDM_GAINL) {
        PDM_CTRL->R_PDM_GAINL = gain;
    }
    else {
        PDM_CTRL->R_PDM_GAINR = gain;
    }
}

void pdm_enable_amic(int en)
{
    PDM_CTRL->R_AMIC_MODE_EN = (en != 0);
  
}

void pdm_enable_amic_i2s(int en)
{
    PDM_CTRL->R_AMIC48K_EN = (en != 0);
}


void set_amic_gain(uint8_t gain)
{
	uint8_t tmp=0x00;
	PDM_CTRL->R_AMIC_PGA_GAIN=gain;
	BBRFWrite(0x7f, 0x02);
	BBRFRead(0x25, &tmp);
	tmp|=gain;
	BBRFWrite(0x25, tmp);
	BBRFRead(0x25, &tmp);
}


void pdm_set_amic_gain(uint8_t gain)
{
		set_amic_gain(gain);		
}

uint8_t pdm_is_enable(){
	
	return PDM_CTRL->R_PDM_EN;
}

