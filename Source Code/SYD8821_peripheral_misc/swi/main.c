/*
GPIO例程main.c

作者：北京盛源达科技有限公司
日期：2018/3/8
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "uart.h"
#include "queue.h"
#include "debug.h"

#define SDM_SVC_BASE 0x0   

enum SYD_SD_SVCS
{
  SOFTWARE_INTERRUPT0 = SDM_SVC_BASE,  /*software interrupt 0 */
  SOFTWARE_INTERRUPT1,                 /*software interrupt 1 */
  SOFTWARE_INTERRUPT2,                 /*software interrupt 2 */
  SOFTWARE_INTERRUPT3,                 /*software interrupt 3 */
  SOFTWARE_INTERRUPT4,                 /*software interrupt 4 */
	SOFTWARE_INTERRUPTMAX, 
};

uint32_t svc_callback(uint32_t task){
    dbg_printf("svc_callback\r\n");
	  return 1;
}

uint32_t software_interrupt5(){
    dbg_printf("software_interrupt0\r\n");
	  return 1;
}

uint32_t software_interrupt1(){
    dbg_printf("software_interrupt1\r\n");
	  return 1;
}
uint32_t software_interrupt2(){
    dbg_printf("software_interrupt2\r\n");
	  return 1;
}
uint32_t software_interrupt3(){
    dbg_printf("software_interrupt3\r\n");
	  return 1;
}
uint32_t software_interrupt4(){
    dbg_printf("software_interrupt4\r\n");
	  return 1;
}
uint32_t software_interruptmax(){
    dbg_printf("parameter error!\r\n");
	  return 1;
}

//__asm void SVC_call(int task0,int task,const void * p_file_name)
//{
//		swi  1
//}

void SVC_call(int task0,int task,const void * p_file_name)
{
		__asm("swi  1");
}

//__asm void PendSV_call()
//{
//	ldr r0, =0x00006038
//	ldr r0, [r0]
//	bx r0
//	NOP
//}

int main()
{	
	uint32_t err_code;
	uint32_t clock_source=0;
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
	dbg_printf("SYD8821 SWI TEST\r\n");
	__enable_irq();
	while(1)
	{
		dbg_printf("clock_source:%x\r\n",clock_source);
//		err_code = SVC_call(clock_source,svc_callback);
		SVC_call(0,clock_source,svc_callback);
		dbg_printf("err_code:%x\r\n",err_code);
		
		if(clock_source>=SOFTWARE_INTERRUPTMAX) clock_source=SOFTWARE_INTERRUPT0;
		clock_source++;
		
    gpo_toggle(LED4);
		
		delay_ms(3000);
	}		
}


void SVC_Handler(uint32_t handler,uint32_t task,void callback(uint32_t id))
{
	dbg_printf("SVC:%x\r\n",task);
	switch(task){
		case SOFTWARE_INTERRUPT0:
			software_interrupt5();
		  break;
		case SOFTWARE_INTERRUPT1:
			software_interrupt1();
		  break;
		case SOFTWARE_INTERRUPT2:
			software_interrupt2();
		  break;
		case SOFTWARE_INTERRUPT3:
			software_interrupt3();
		  break;
		case SOFTWARE_INTERRUPT4:
			software_interrupt4();
		  break;
		default :
			software_interruptmax();
			break;
	}
	callback(task);
  gpo_toggle(LED1);
}
