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
#include "i2s.h"
#include <string.h>
#include "DebugLog.h"
#include "main.h"

#define    I2S_CTRL    ((I2S_CTRL_TYPE *) I2S_CTRL_BASE)
#define    RAM_BUFF_LEN 16*1024

static __align(4) uint8_t buf_Ram[RAM_BUFF_LEN]={0};
uint8_t buf_const[READ_BUFF_LEN]={0};
int DMA_size = 0; 

#define DEBUG_I2S_DMA

#ifdef DEBUG_I2S_DMA
unsigned char DMA_cnt = 0,DMA_cnt_last = 0;
#endif

uint8_t * audio_data_p=buf_const;
uint8_t * audio_rambuff_p=buf_Ram;
/*
0:空闲状态  
1：接收状态
*/
char  audio_state=0;

void i2s_test_init(void)
{
    int i;
    
    for (i = 7; i < 12; i++)
        pad_mux_write(i, 9); // MCK, SCK, LRCK, DO, DI

    // 8K 16-bit configuration
    i2s_set_mclk(I2S_CLK_DIV31);
    i2s_set_bit_clk(I2S_MCLK_DIV4);
    
    i2s_set_channel_clock(I2S_LEFT_LOW);
    i2s_set_channel(I2S_MONO_LEFT);
    i2s_set_bit_order(I2S_RX, I2S_MSB_FIRST);
    i2s_set_transfer_format(I2S_FISRT_BIT_SECOND_CYCLE);
    
	i2s_set_sample_width(I2S_16BIT);
    i2s_enable_int(I2S_DMA_ALL_INT);
    NVIC_EnableIRQ(I2S_IRQn);
}

static void i2s_test_slient_irq_cb(uint32_t int_st)
{
	DMA_cnt++;
	if(audio_state==0)
	{
		//DBG("idle dma_cnt:%x\r\n",DMA_cnt);
	}
	else if(audio_state==1)
	{
		gpo_toggle(LED2);
		//DBG("play dma_cnt:%x ",DMA_cnt);
		if (int_st & I2S_DMA_HALF_DONE_INT_ST) {
			audio_rambuff_p=buf_Ram;
			//memcpy(audio_data_p,audio_rambuff_p,DMA_size/2);
			SEGGER_RTT_Write(0,audio_rambuff_p,DMA_size/2);
			audio_data_p+=DMA_size/2;
			//DBG("buf_Ram_p2:%x ",(buf_Ram+DMA_size/2));
		}
		else if (int_st & I2S_DMA_DONE_INT_ST) {
			audio_rambuff_p=buf_Ram+DMA_size/2;
			//memcpy(audio_data_p,audio_rambuff_p+DMA_size/2,DMA_size/2);
			SEGGER_RTT_Write(0,audio_rambuff_p,DMA_size/2);
			audio_data_p+=DMA_size/2;
			//DBG("buf_Ram_p1:%x ",buf_Ram);
		}
		if(audio_data_p>=(buf_const+READ_BUFF_LEN))
		{
			i2s_stop();
			audio_state=0;
		}
		//DBG("data_p:%x\r\n",audio_data_p);
	}
}

void i2s_test_Silent_RX(void)
{
    DMA_size=RAM_BUFF_LEN;
    
	memset(buf_Ram,0xff,DMA_size);
	
    i2s_set_irq_callback(i2s_test_slient_irq_cb);
    //i2s_trigger_TXDMA(buf_Ram, DMA_size);
    i2s_trigger_RXDMA(buf_Ram,DMA_size);
}

int main()
{	
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

  //uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	dbg_printf("SYD8821 IIS DEMO\r\n");

    i2s_test_init();
	
	DebugLogInit();
	QPRINTF("SYD8821 IIS DEMO\r\n");
	
	__enable_irq();
	
	DMA_cnt=0;
	audio_state=0;
	
	while(1)
	{
		gpo_toggle(LED4);
		
		#ifdef DEBUG_I2S_DMA
		if(DMA_cnt !=DMA_cnt_last)
		{
			DMA_cnt_last=DMA_cnt;
			if(audio_state==0)
			{
				dbg_printf("idle dma_cnt:%x\r\n",DMA_cnt);
			}
			else if(audio_state==1)
			{
				dbg_printf("record dma_cnt:%x buf_Ram_p2:%x data_p:%x",DMA_cnt,audio_rambuff_p,audio_data_p);
				
			}
			else if(audio_state==2)
			{
				dbg_printf("record finish dma_cnt:%x\r\n",DMA_cnt);
			}
		}
		#endif
		
		if(!gpi_get_val(KEY1)){
		    gpo_toggle(LED1);
			if(audio_state==0)
			{
				audio_data_p=buf_const;
				audio_state=1;
				i2s_test_Silent_RX();
			}
		}
	}		
}
