#define  _UART_C

#include "uart.h"
#include "uart2.h"
#include "string.h"

enum {
    UART_RX_INT  = 0x10,
    UART_TX_INT  = 0x20,
    UART_ALL_INT = 0x30,
};

UART_CTRL_TYPE * UART_CTRL[2] =
    { ((UART_CTRL_TYPE *) UART0_CTRL_BASE),
      ((UART_CTRL_TYPE *) UART1_CTRL_BASE) };

QUEUE rx_queue[2];
uint8_t rx_buf[2][QUEUE_SIZE];
                                        
void uart_0_init(uint8_t flowctrl, uint8_t baud)
{
    NVIC_DisableIRQ(UART0_IRQn);
	UART_CTRL[0]->BAUD_SEL = baud;
	UART_CTRL[0]->FLOWCTRL_EN = flowctrl;
	UART_CTRL[0]->INT_RX_MASK = 0;
    UART_CTRL[0]->UART_ENABLE = 1;
    
    queue_init(&rx_queue[0], rx_buf[0], QUEUE_SIZE);
    
	*(uint32_t *)0x20028024 |=U32BIT(UART0_IRQn);
	*(uint32_t *)0x20028020 |=U32BIT(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
}

void UART0_IRQHandler(void)
{
    uint8_t int_st = UART_CTRL[0]->INT_STATUS;
    uint8_t clear_int = int_st ^ UART_ALL_INT;
    
    // Clear interrrupt status
    UART_CTRL[0]->INT_STATUS = clear_int;
    
    if (int_st & UART_RX_INT) {
        do {
            enqueue(&rx_queue[0], UART_CTRL[0]->RX_DATA);
        } while (!UART_CTRL[0]->RXFF_EMPTY);
    }
}


void uart_1_init(uint8_t flowctrl, uint8_t baud)
{
    NVIC_DisableIRQ(UART1_IRQn);
	UART_CTRL[1]->BAUD_SEL = baud;
	UART_CTRL[1]->FLOWCTRL_EN = flowctrl;
	UART_CTRL[1]->INT_RX_MASK = 0;
    UART_CTRL[1]->UART_ENABLE = 1;
    queue_init(&rx_queue[1], rx_buf[1], QUEUE_SIZE);

	NVIC_EnableIRQ(UART1_IRQn);
}

void UART1_IRQHandler(void)
{
    uint8_t int_st = UART_CTRL[1]->INT_STATUS;
    uint8_t clear_int = int_st ^ UART_ALL_INT;
    
    // Clear interrrupt status
    UART_CTRL[1]->INT_STATUS = clear_int;
	
    if (int_st & UART_RX_INT) {
        do {
            enqueue(&rx_queue[1], UART_CTRL[1]->RX_DATA);
        } while (!UART_CTRL[1]->RXFF_EMPTY);
    }

}

void uart_write(uint8_t id, uint8_t * buf, uint8_t sz)
{
	int i;
	uint32_t j=0x10000;
	
	#ifdef _CONFIG_NOUART2_
    if (id >= 2) {
        uart2_write(buf, sz);
        return;
    }
	#endif
    
    for (i = 0; i < sz; i++) {
        while (UART_CTRL[id]->TXFF_FULL);
        UART_CTRL[id]->TX_DATA = buf[i];
    }

    //while (!UART_CTRL[id]->TXFF_EMPTY);
		 while((!UART_CTRL[id]->TXFF_EMPTY) && j){
        j--;
    }
}

int uart_read(uint8_t id, uint8_t *buf)
{
	#ifdef _CONFIG_NOUART2_
    if (id >= 2)
        return uart2_read(buf);
	#endif
    
    return dequeue(&rx_queue[id], buf);
}

uint8_t uart_queue_size(uint8_t id,uint8_t *buf)
{
	  buf=buf;
    if (id >= 2)
        return 0;
		if (!is_queue_empty(&rx_queue[id])){
			  buf=rx_queue[id].data;
		    return queue_size(&rx_queue[id]);
		}
		return 0;
}


UART_CTRL_TYPE * uart_get_ctrl(uint8_t id)
{
    if (id > 1)
        return NULL;
    return UART_CTRL[id];
}
