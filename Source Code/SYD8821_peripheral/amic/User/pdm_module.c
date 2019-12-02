#include "pdm_module.h"
#include "pad_mux_ctrl.h"
#include "gpio.h"
#include "pdm.h"
#include "adpcm.h"

#include <string.h>

static void pdm_callback(uint8_t int_st);

int16_t* pdm_dma_buf_ptr;
uint8_t* pdm_enc_buf_ptr;

uint8_t pdm_int_flag = 0;

__align(4) int16_t pcm_buf[PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH];
__align(4) uint8_t enc_buf[PDM_DMA_16BIT_ENC_BUF_LENGTH];

void pdm_module_init(void)
{
	/*AMIC setiing*/
	pad_mux_write(AMIC_VREF_PIN, 0); 			//GPIO_26
	pad_mux_write(AMIC_BIAS_PIN, 0); 			//GPIO_27
	pad_mux_write(AMIC_INN_PIN, 0); 			//GPIO_28
	pad_mux_write(AMIC_INP_PIN, 0); 			//GPIO_29

	pad_input_configure(AMIC_VREF_PIN, 0);		//GPIO_26
	pad_input_configure(AMIC_BIAS_PIN, 0);		//GPIO_27
	pad_input_configure(AMIC_INN_PIN, 0);		//GPIO_28
	pad_input_configure(AMIC_INP_PIN, 0);		//GPIO_29
	
	pad_mux_write(AMIC_VREF_PIN, 9); 			//GPIO_26
	pad_mux_write(AMIC_BIAS_PIN, 9); 			//GPIO_27
	pad_mux_write(AMIC_INN_PIN, 9); 			//GPIO_28
	pad_mux_write(AMIC_INP_PIN, 9); 			//GPIO_29
	
	pdm_set_clk(PDM_1M);						// Set PDM clock 
	pdm_set_sample_rate(PDM_15625Hz_SAMPLE);	// Set PDM sample rate 16K
	pdm_set_sample_width(PDM_16BIT);			// Set PDM sample width.
	pdm_set_channel(PDM_SAMPLE_MONO);			// Set PDM Stero or Mono
	pdm_set_sample_trigger(PDM_LEFT_MONO_RISING_EDGE);

	pdm_set_amic_gain(RC_AMIC_GAIN);	
	pdm_set_gain(PDM_GAINL, RC_PDM_GAIN);		//Set PDM Gain value. 

	pdm_set_DMA(pcm_buf, sizeof(pcm_buf));
	pdm_enable_int(PDM_DMA_ALL_INT);
	
	pdm_set_irq_callback(pdm_callback);
	
	NVIC_EnableIRQ(PDM_IRQn);
}

static void pdm_callback(uint8_t int_st)
{
	if((int_st & PDM_DMA_HALF_INT)==PDM_DMA_HALF_INT)	//If buffer half full.
	{
		//rc_set_event(RC_EVENT_PDM_HALF);//Set event to notify data is ready.
		pdm_dma_buf_ptr = &pcm_buf[0];		//Set buf pointer to point buf.
		//dbg_printf("PDM_DMA_HALF_INT\r\n");
	}
	
	if((int_st & PDM_DMA_FINISH_INT)==PDM_DMA_FINISH_INT)	//If buffer end full.
	{
		pdm_dma_buf_ptr = &pcm_buf[PDM_DMA_16BIT_SAMPLES_HALF_START_IDX]; //Set buf pointer to point buf.
		//dbg_printf("PDM_DMA_FINISH_INT\r\n");
	}
	
	memset(enc_buf, 0, sizeof(enc_buf));	//Clear the buf.
	Adpcm_encode(pdm_dma_buf_ptr, (char *)enc_buf, PDM_DMA_16BIT_SAMPLES_HALF_START_IDX, &adpcm_state_encdoe);
	
	pdm_enc_buf_ptr = &enc_buf[0];	//Point to the ADPCM compression  data buf.
	pdm_int_flag = 1;	
}

void start_record(void)
{	
	pdm_start();
	pdm_enable_amic(1);
			
	adpcm_state_encdoe.index = 0;
	adpcm_state_encdoe.valprev = 0;
}

void stop_record(void)
{
	pdm_stop();
	pdm_enable_amic(0);
}

