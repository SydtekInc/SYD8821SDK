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
#include "pdm.h"
#include "ble_slave.h"
#include "adpcm.h"
#include <string.h>





#define RC_AMIC_GAIN   		0x0d
#define RC_PDM_GAIN   		0x28

#define  PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH    480  
#define  PDM_DMA_16BIT_ENC_BUF_LENGTH           PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH/4
#define  PDM_DMA_16BIT_SAMPLES_HALF_START_IDX  (PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH/2)  


#define  NUM_OF_VOICE_DATA_FRAME (PDM_DMA_16BIT_ENC_BUF_LENGTH/RC_BLE_PAYLOAD_LEN)


static int16_t* pdm_dma_buf_ptr;
static uint8_t* pdm_enc_buf_ptr;

static __align(4) int16_t buf[PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH];
static __align(4) uint8_t enc_buf[PDM_DMA_16BIT_ENC_BUF_LENGTH];

struct adpcm_state adpcm_state_encdoe;
struct adpcm_state adpcm_state_decdoe;


uint8_t pdm_int_flag = 0;


void init_amic_settings(void)
{
	pdm_set_amic_gain(RC_AMIC_GAIN);		
}

void pdm_init(void)
{

	pdm_set_clk(PDM_1M);// Set PDM clock 
	pdm_set_sample_rate(PDM_15625Hz_SAMPLE); // Set PDM sample rate 16K
	pdm_set_sample_width(PDM_16BIT); // Set PDM sample width.
	pdm_set_channel(PDM_SAMPLE_MONO);// Set PDM Stero or Mono
	pdm_set_sample_trigger(PDM_LEFT_MONO_RISING_EDGE);

   
	pdm_set_gain(PDM_GAINL, 0x28);//Set PDM Gain value. 

	pdm_set_DMA(buf, sizeof(buf));
	pdm_enable_int(PDM_DMA_ALL_INT);
	
	NVIC_EnableIRQ(PDM_IRQn);	
}


static void pdm_callback(uint8_t int_st)
{
	
	uint8_t encode_data=0x00;
	uint8_t nibble_high=0;
	uint8_t nibble_low=0;
	
	
	uint16_t idx=0;
	uint16_t enc_idx=0;
	
	if((int_st & PDM_DMA_HALF_INT)==PDM_DMA_HALF_INT){ //If buffer half full.	
		//rc_set_event(RC_EVENT_PDM_HALF);//Set event to notify data is ready.
		pdm_dma_buf_ptr=&buf[0];//Set buf pointer to point buf.
		//dbg_printf("PDM_DMA_HALF_INT\r\n");
	}
	
	if((int_st & PDM_DMA_FINISH_INT)==PDM_DMA_FINISH_INT){ //If buffer end full.
		
		pdm_dma_buf_ptr=&buf[PDM_DMA_16BIT_SAMPLES_HALF_START_IDX]; //Set buf pointer to point buf.
		//dbg_printf("PDM_DMA_FINISH_INT\r\n");
	}
	

	memset(enc_buf, 0, sizeof(enc_buf));//Clear the buf.
	
	for(idx=0;idx<PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH/2;idx=idx+2)
	{
		nibble_high=Adpcm_encode(*(pdm_dma_buf_ptr+idx),&adpcm_state_encdoe); //Adpmc encoding.
		nibble_high=(nibble_high<<4);
		nibble_low=Adpcm_encode(*(pdm_dma_buf_ptr+idx+1),&adpcm_state_encdoe); //Adpmc encoding.
		encode_data=(nibble_high |nibble_low);
		
		enc_buf[enc_idx]=encode_data;
		enc_idx++;
	}

	
	pdm_enc_buf_ptr=&enc_buf[0];//Point to the ADPCM compression  data buf.
	pdm_int_flag = 1;
}


void start_record(void)
{	
	if(pdm_is_enable()==0)
	{
		pdm_enable_amic(1);

		pdm_start();
		adpcm_state_encdoe.index=0;
		adpcm_state_encdoe.predsample=0;
		//dbg_printf("rc_start_record \r\n");
	}
}

void stop_record(void)
{
	pdm_stop();
	//i2s_stop();
	pdm_enable_amic(0);
	
	//dbg_printf("rc_stop_record \r\n");
}

uint8_t* get_enc_buf_ptr(void) {

	return pdm_enc_buf_ptr;
}

void voice_printf_data(void)
{
	uint8_t *buf_ptr_ptr;
	
	
	buf_ptr_ptr=get_enc_buf_ptr(); //Get the voice data buf pointer.
	uart_write(0, buf_ptr_ptr, PDM_DMA_16BIT_ENC_BUF_LENGTH);
}


int main()
{	
	uint8_t amic_state = 0;
	
	
	__disable_irq();
	
	gap_s_ble_init();
	
	sys_mcu_rc_calibration();
	delay_ms(100);
	#ifdef IIC1_MASTER
	sys_mcu_clock_set(MCU_CLOCK_24_MHZ);
	#elif IIC1_SLAVE
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	#endif
	
	// RC bumping
	
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
	pad_mux_write(KEY1, 0);
	pad_mux_write(KEY2, 0);
	pad_mux_write(KEY3, 0);
	gpi_config(KEY1, PULL_UP);
	gpi_config(KEY2, PULL_UP);
	gpi_config(KEY3, PULL_UP);

	//uart 0 
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();//  ------------------------>>>>> UART_BAUD_460800
	
	dbg_printf("SYD8821 AMIC DEMO\r\n");

	/*AMIC setiing*/
	pad_mux_write(GPIO_26, 1); 				//GPIO_26
	pad_mux_write(GPIO_27, 1); 				//GPIO_27
	pad_mux_write(GPIO_28, 1); 				//GPIO_28
	pad_mux_write(GPIO_29, 1); 				//GPIO_29
		
	pad_input_configure(GPIO_26,0);
	pad_input_configure(GPIO_27,0);
	pad_input_configure(GPIO_28,0);
	pad_input_configure(GPIO_29,0);
	
	
	//mask PDM pin
	pad_mux_write(GPIO_2, 0); 			//GPIO_2
	pad_mux_write(GPIO_3, 0); 			//GPIO_3
	gpi_config(GPIO_2, PULL_UP); 		//GPIO_2
	gpi_config(GPIO_3, PULL_UP); 		//GPIO_3
			
	
	init_amic_settings();
	pdm_init();
	pdm_set_irq_callback(pdm_callback);
	__enable_irq();
	
	
	

	while(1)
	{
		
		if(pdm_int_flag == 1)
		{
			pdm_int_flag = 0;
			voice_printf_data();
		}
		
		
		
		if(!gpi_get_val(KEY1)){
			gpo_toggle(LED1);
			gpo_set(LED2);
			if(amic_state==0)
			{
				
				amic_state=1;
				start_record();
			}
			while(!gpi_get_val(KEY1));
		}
		
		if(!gpi_get_val(KEY2)){
			gpo_toggle(LED2);
			gpo_set(LED1);
			if(amic_state==1)
			{
				
				amic_state=0;
				stop_record();
			}
			while(!gpi_get_val(KEY2));
		}

		gpo_toggle(LED4);
		delay_ms(50);
		
	}		
}
