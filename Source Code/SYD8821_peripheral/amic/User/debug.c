#define  _DEBUG_C

#include "debug.h"
#include "uart.h"
#include <stdarg.h>
#include <stdio.h>

#ifdef CONFIG_UART_ENABLE

#define MAX_FORMAT_BUFFER_SIZE	(128)
static uint8_t s_formatBuffer[MAX_FORMAT_BUFFER_SIZE];


static  UART_CTRL_TYPE * UART_CTRL[2] =
    { ((UART_CTRL_TYPE *) UART0_CTRL_BASE),
      ((UART_CTRL_TYPE *) UART1_CTRL_BASE) };
	
static void debug_uart_0_init(uint8_t flowctrl, uint8_t baud)
{
    NVIC_DisableIRQ(UART0_IRQn);
	UART_CTRL[0]->BAUD_SEL = baud;
	UART_CTRL[0]->FLOWCTRL_EN = flowctrl;
	UART_CTRL[0]->INT_RX_MASK = 1;
	UART_CTRL[0]->INT_TX_MASK = 1;
    UART_CTRL[0]->UART_ENABLE = 1;
    
	*(uint32_t *)0x20028024 |=U32BIT(UART0_IRQn);
	*(uint32_t *)0x20028020 |=U32BIT(UART0_IRQn);

	NVIC_EnableIRQ(UART0_IRQn);
}

static void debug_uart_write(uint8_t id, uint8_t * buf, uint8_t sz)
{
	int i;
	uint32_t j=0x10000;
    
    for (i = 0; i < sz; i++) {
        while (UART_CTRL[id]->TXFF_FULL);
        UART_CTRL[id]->TX_DATA = buf[i];
    }

    //while (!UART_CTRL[id]->TXFF_EMPTY);
		 while((!UART_CTRL[id]->TXFF_EMPTY) && j){
        j--;
    }
}


void dbg_init(void)
{
	debug_uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_921600);
}

void dbg_printf(char *format,...)
{
	uint8_t iWriteNum = 0;	
	va_list  ap;
	
	if(!format)
		return;
	
	va_start(ap,format);

	iWriteNum = vsprintf((char *)s_formatBuffer,format,ap);

	va_end(ap);
	
	if(iWriteNum > MAX_FORMAT_BUFFER_SIZE)
		iWriteNum = MAX_FORMAT_BUFFER_SIZE;

	//uart_write(1, s_formatBuffer, iWriteNum);
	//uart_write(0, s_formatBuffer, iWriteNum);
	debug_uart_write(0, s_formatBuffer, iWriteNum);
	//uart_0_write(s_formatBuffer, iWriteNum); // test by Chiu
}

void dbg_printf_sel(uint8_t id, char *format, ...)
{
    uint8_t iWriteNum = 0;	
	va_list  ap;
	
	if(!format)
		return;
	
	va_start(ap,format);

	iWriteNum = vsprintf((char *)s_formatBuffer,format,ap);

	va_end(ap);
	
	if(iWriteNum > MAX_FORMAT_BUFFER_SIZE)
        iWriteNum = MAX_FORMAT_BUFFER_SIZE;
    
    //uart_write(id, s_formatBuffer, iWriteNum);
	debug_uart_write(id, s_formatBuffer, iWriteNum);
}

void dbg_hexdump(char *title, uint8_t *buf, uint16_t sz)
{
	int i = 0;
	
	if (title)
		dbg_printf((title));

	for(i = 0; i < sz; i++) 
	{
  		if((i%8) == 0)
			dbg_printf("[%04x] ",(uint16_t)i);

		dbg_printf("%02x ",(uint16_t)buf[i]);

		if(((i+1)%8) == 0)
			dbg_printf("\r\n");

	 } 

	if((i%8) != 0)
		dbg_printf("\r\n");
}
#endif

