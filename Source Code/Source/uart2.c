#include "uart2.h"
#include "debug.h"
#include "queue.h"

#define QUEUE_SIZE    512

static UART2_CTRL_TYPE * UART_CTRL = (UART2_CTRL_TYPE *)UART2_CTRL_BASE;
static QUEUE uart2_rx_queue;
static uint8_t uart2_rx_buf[QUEUE_SIZE];
uint32_t uart2_current_baud;

static void set_baud(uint32_t baud);

UART2_CTRL_TYPE * uart2_get_ctrl(void)
{
    return UART_CTRL;
}

void uart2_init(uint8_t flowctrl, uint32_t baud)
{
    NVIC_DisableIRQ(UART2_IRQn);
    
    set_baud(baud);
    uart2_configure(AUTO_FLOW_CTRL, flowctrl != 0);
    uart2_configure(PARITY_EN, 0);
    uart2_configure(STOP_LENGTH, 1);
    uart2_configure(DATA_LENGTH, 8);
    uart2_configure(ENABLE_FIFO, 1);
    
    queue_init(&uart2_rx_queue, uart2_rx_buf, QUEUE_SIZE);
    
    UART_CTRL->RTS = 1;
    
    // Enable RX DATA Available Interrupt
    UART_CTRL->ERBFI = 1;
    NVIC_EnableIRQ(UART2_IRQn);
}

void UART2_IRQHandler(void)
{
    #define RX_DATA_AVAILABLE        0x04
    #define RX_TIMEOUT_INDICATION    0x0C

    if (UART_CTRL->INTERRUPT_ID == RX_DATA_AVAILABLE ||
        UART_CTRL->INTERRUPT_ID == RX_TIMEOUT_INDICATION) {
        while (UART_CTRL->LSR & DATA_READY) {
            enqueue(&uart2_rx_queue, UART_CTRL->RBR);
        }
    }
}

void uart2_write(uint8_t * buf, uint8_t sz)
{
	uint8_t idx = 0;
    
    for (idx = 0; idx < sz ; idx++) {
        while (!(UART_CTRL->LSR & TX_HOLDING_REG_EMPTY));
        UART_CTRL->THR = buf[idx];
	}
    
    while (!(UART_CTRL->LSR & TX_FIFO_EMPTY));
}

int uart2_read(uint8_t *buf)
{
    return dequeue(&uart2_rx_queue, buf);
}

void uart2_configure(UART2_CONFIG_TYPE type, uint32_t param)
{
    static uint8_t rx_fifo_trigger, tx_fifo_trigger, fifo_enabled;
    int config_fcr = (type < FCR_CONFIG_TYPE);
    uint32_t fifo_reset = 0;
    
    switch (type) {
        case BAUD:
            set_baud((uint32_t)param);
            break;
        case AUTO_FLOW_CTRL:
            UART_CTRL->AFCE = param;
            break;
        case DATA_LENGTH:
            param -= 5;
            UART_CTRL->DLS = (param >= 3) ? 3 : param;
            break;
        case STOP_LENGTH:
            // 0 for 1 stop bit
            UART_CTRL->STOP = (param != 1);
            break;
        case PARITY_EN:
            UART_CTRL->PEN = (param != 0);
            break;
        case PARITY_TYPE:
            UART_CTRL->EPS = (param != 0);
            break;
        case ENABLE_FIFO:
            fifo_enabled = (param != 0);
            break;
        case FIFO_RESET:
            fifo_reset = (param & FIFO_RESET_MASK);
            break;
        case RX_FIFO_TRIGGER:
            rx_fifo_trigger = (param & RX_FIFO_TRIGGER_MASK);
            break;
        case TX_FIFO_TRIGGER:
            tx_fifo_trigger = (param & TX_FIFO_TRIGGER_MASK);
            break;
        case RESET_FCR:
            rx_fifo_trigger = tx_fifo_trigger = fifo_enabled = 0;
            UART_CTRL->FCR = 0;
            break;
        default:
            DBG("UART2 type%d not support\r\n", type);
            break;
    }
    
    if (config_fcr) {
        UART_CTRL->FCR = (fifo_enabled << FIFO_ENABLE_SHIFT)        |
                         (fifo_reset << FIFO_RESET_SHIFT)           |
                         (rx_fifo_trigger << RX_FIFO_TRIGGER_SHIFT) |
                         (tx_fifo_trigger << TX_FIFO_TRIGGER_SHIFT);
    }
}

static void set_baud(uint32_t baud)
{
    uint32_t serial_clk_freq = 32000000;
    uint32_t fix_point_divisor = (serial_clk_freq << 1) / baud;
    uint32_t carry = 0;
    
    if (uart2_current_baud == baud)
        return;
    
    uart2_current_baud = baud;
    
    if (fix_point_divisor & 1) {
        carry = 1;
    }
    fix_point_divisor >>= 1;
    fix_point_divisor += carry;
    
    UART_CTRL->DLAB = 1;
    UART_CTRL->DLF = fix_point_divisor;
    fix_point_divisor >>= 4;
    
    UART_CTRL->DLL = fix_point_divisor;
    fix_point_divisor >>= 8;
    
    UART_CTRL->DLH = fix_point_divisor;
    UART_CTRL->DLAB = 0;
}
