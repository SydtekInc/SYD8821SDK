#ifndef _UART_H_
#define _UART_H_

#include "ARMCM0.h"
#include "queue.h"

#define UART_R			        0x10
#define UART_T			        0x20
#define UART_01_FIFO_LENGTH     8

#define QUEUE_SIZE    512

enum UART_BAUD_SEL 
{
	UART_BAUD_1200		= 0x00,
	UART_BAUD_2400		= 0x01,
	UART_BAUD_4800		= 0x02,
	UART_BAUD_9600		= 0x03,
	UART_BAUD_14400		= 0x04,
	UART_BAUD_19200		= 0x05,
	UART_BAUD_38400		= 0x06,
	UART_BAUD_57600		= 0x07,
	UART_BAUD_115200	= 0x08,
	UART_BAUD_230400	= 0x09,
	UART_BAUD_460800	= 0x0A,
	UART_BAUD_921600	= 0x0B,
};

enum UART_FLOWCTRL {
	UART_RTS_CTS_ENABLE		= 0x80,
	UART_RTS_CTS_DISABLE	= 0x00,
};

#ifdef _UART_C
#else
	extern void uart_0_init(uint8_t flowctrl, uint8_t baud);
	extern void uart_1_init(uint8_t flowctrl, uint8_t baud);
    extern UART_CTRL_TYPE * uart_get_ctrl(uint8_t id);
#endif

extern QUEUE rx_queue[2];
extern uint8_t rx_buf[2][QUEUE_SIZE];
extern UART_CTRL_TYPE * UART_CTRL[2];

void uart_write(uint8_t id, uint8_t * buf, uint8_t sz);
int  uart_read(uint8_t id, uint8_t *buf);
uint8_t uart_queue_size(uint8_t id,uint8_t *buf);

#endif
