
#include "debug.h"
#include <string.h>
#include "ir.h"


static IR_CTRL_TYPE * IR_CTRL= ((IR_CTRL_TYPE*) IR_CTRL_BASE);

/* Functions */
void ir_nec_init(uint16_t carrier_kHz)
{

	uint32_t carriier_hz=0;

	uint32_t period=0;
	uint32_t invtime=0;
	uint32_t pwmfmt=0 ;
	
	uint32_t one_fh=0 ;
	uint32_t one_lh=0 ;
	uint32_t zero_fh=0 ;
	uint32_t zero_lh =0;

	carriier_hz=carrier_kHz*1000;

	period=(uint32_t)(PCLK/carriier_hz);
	invtime = (uint32_t)(period / IR_NEC_CARRIER_SPACE);
	pwmfmt = 0;
	one_fh = (uint32_t)(IR_NEC_ONE_FH_TIME * carrier_kHz) - 1;
	one_lh = one_fh * IR_NEC_ONE_SPACE;
	zero_fh = one_fh;
	zero_lh = one_fh;


	IR_CTRL->CARRIER_PERIOD=period;
	IR_CTRL->CARRIER_INVTIME=invtime;
	IR_CTRL->CARRIER_PWMFMT=pwmfmt;

	IR_CTRL->LOGICBIT_ONE_FH=one_fh;
	IR_CTRL->LOGICBIT_ONE_LH=one_lh;
	IR_CTRL->LOGICBIT_ZERO_FH=zero_fh;
	IR_CTRL->LOGICBIT_ZERO_LH=zero_lh;

}


void ir_tx(uint32_t dr)
{
	IR_CTRL->DR=dr;
	IR_CTRL->CTRL_IR_EN=1;

	
}
void ir_repeat_tx(uint32_t repcmd)
{
	//IRPORT_REPCMD = repcmd;
	//IR_REP_TX_ENABLE();
	IR_CTRL->REPCMD=repcmd;
	IR_CTRL->CTRL_REPEAT_EN=1;
}
void ir_stop_tx(){
	IR_CTRL->CTRL_REPEAT_EN=0;
	IR_CTRL->CTRL_IR_EN=0;

}
void ir_nec_test_hard()
{

	ir_tx(0x20ab156);	// leader code
	ir_tx(0x00ff9b0);	// custom code
	ir_tx(0x00f1771);	// data code
	ir_tx(0x2744015);	// stop bit & frame space
}

void ir_nec_tx_data(uint16_t custom_code_t, uint16_t data_code_t)
{
	
	uint32_t custom_code=  0x000F0000 ;
	uint32_t data_code  =  0x000F0000 ;

	custom_code|=custom_code_t;
	data_code|=data_code_t;

	
	ir_tx(0x20AB156);		// Leader Code
	ir_tx(custom_code);		// Custom Code
	ir_tx(data_code);		// Data Code
	ir_tx(0x2744015);		// Stop Bit & Frame Space
}
void ir_nec_tx_repeat(uint16_t custom_code_t, uint16_t data_code_t)
{
	ir_nec_tx_data(custom_code_t, data_code_t);

	IR_CTRL->CTRL_REPEAT_TIME=4180;
	ir_repeat_tx(0x2054155);	// 9ms + 2.25ms
	ir_repeat_tx(0x2E90014);	// 560us
	
}


void ir_nec_tx_reverse_data(uint8_t custom_code_t, uint8_t data_code_t)
{
	
	
	uint32_t custom_code=  0x000F0000 ;
	uint32_t data_code  =  0x000F0000 ;

	custom_code|=custom_code_t;
	data_code|=data_code_t;


	custom_code|=((uint16_t)(~custom_code_t) << 8);
	data_code|=  ((uint16_t)(~data_code_t) << 8);


	

	ir_tx(0x20AB156);		// Leader Code
	ir_tx(custom_code);		// Custom Code
	ir_tx(data_code);		// Data Code
	ir_tx(0x2744015);		// Stop Bit & Frame Space
}


void ir_rc5_init(uint16_t carrier_kHz){

	uint32_t carriier_hz=0;

	uint32_t period=0;
	uint32_t invtime=0;
	uint32_t pwmfmt=0 ;

	uint32_t one_fh=0 ;
	uint32_t one_lh=0 ;
	uint32_t zero_fh=0 ;
	uint32_t zero_lh =0;

	carriier_hz=carrier_kHz*1000;

	period=(uint32_t)(PCLK/carriier_hz);
	invtime = (uint32_t)(period / IR_RC5_CARRIER_SPACE);
	pwmfmt = 0;
	one_fh = (uint32_t)(IR_RC5_ONE_FH_TIME * carrier_kHz) - 1;
	one_lh = one_fh * IR_RC5_ONE_SPACE;
	zero_fh = one_fh;
	zero_lh = one_fh;


	IR_CTRL->CARRIER_PERIOD=period;
	IR_CTRL->CARRIER_INVTIME=invtime;
	IR_CTRL->CARRIER_PWMFMT=pwmfmt;

	IR_CTRL->LOGICBIT_ONE_FH=one_fh;
	IR_CTRL->LOGICBIT_ONE_LH=one_lh;
	IR_CTRL->LOGICBIT_ZERO_FH=zero_fh;
	IR_CTRL->LOGICBIT_ZERO_LH=zero_lh;
	IR_CTRL->CTRL_ZERO_FMT=0;
	IR_CTRL->CTRL_ONE_FMT=1;


}
void ir_rc5_test_hard()
{
	ir_tx(0x00D2BA3);
	ir_tx(0x3000C00);	// Stop Space
	ir_tx(0x00D2BA7);
	ir_tx(0x3000C00);	// Stop Space
}
uint8_t bit_reverse(uint8_t b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}


uint32_t ir_rc5_tx_data(uint8_t custom_code, uint8_t data_code,uint8_t toggle)
{
	uint8_t custom = bit_reverse(0x1F & custom_code) >> 3;
	uint8_t  data  = bit_reverse(0x3F & data_code) >> 2;
	uint32_t rc5 = 0;
	
	rc5  |= 0x01<<0; //s1
	rc5  |= 0x01<<1; //s2
	rc5  |= toggle << 2;//toogle
	rc5  |= custom << 3;//custom
	rc5  |= data   << 8;//data
	rc5  |= 0xD    << 16 ;//effect bit
	
    ir_tx(rc5); 		// Custom Code
	ir_tx(0x3000C00);	// Stop Space
	return rc5;

}
void ir_rc5_repeat_test(uint8_t custom_code, uint8_t data_code,uint8_t toggle){

	uint32_t rc5_format = 0;

	rc5_format=ir_rc5_tx_data(custom_code,data_code,toggle);
	
	IR_CTRL->CTRL_REPEAT_TIME=4104; // 114 ms  

	ir_repeat_tx(rc5_format);

}


void IR_IRQHandler()
{
	IR_CTRL->INTSTATE_CMD_DONE=1;
	IR_CTRL->INTSTATE_TRANS_DONE=1;

}


