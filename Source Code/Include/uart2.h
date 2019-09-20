#ifndef _UART2_H_
#define _UART2_H_

#include "ARMCM0.h"
#include "ARMCM0.h"

#define UART2_FIFO_LENGTH   16

typedef enum {
    ENABLE_FIFO,
    FIFO_RESET,
    RX_FIFO_TRIGGER,
    TX_FIFO_TRIGGER,
    FCR_CONFIG_TYPE,
    RESET_FCR,
    BAUD,
    AUTO_FLOW_CTRL,
    DATA_LENGTH,
    STOP_LENGTH,
    PARITY_EN,
    PARITY_TYPE,
} UART2_CONFIG_TYPE;

enum {
    UART2_ODD_PARITY,
    UART2_EVEN_PARITY,
};

enum LSR_FIELD_MASK {
    DATA_READY           = 0x0001,
    DATA_OVER_RUN        = 0x0002,
    PARITY_ERROR         = 0x0004,
    FRAME_ERROR          = 0x0008,
    BREAK_INTERRUPT      = 0x0010,
    TX_HOLDING_REG_EMPTY = 0x0020,
    TX_FIFO_EMPTY        = 0x0040,
    RX_FIFO_ERROR        = 0x0080,
    ADDR_REC             = 0x0100,
};

enum FCR_FIELD {
    // FIFO Control bit mask
    FIFO_ENABLE_MASK          = 1UL,
    FIFO_RESET_MASK           = 3UL,
    DMA_MODE_MASK             = 1UL,
    TX_FIFO_TRIGGER_MASK      = 3UL,
    RX_FIFO_TRIGGER_MASK      = 3UL,
    // FIFO RESET
    RX_FIFO_RESET             = 1UL,
    TX_FIFO_RESET             = 2UL,
    // TX FIFO TRIGGER
    TX_FIFO_TRIG_EMPTY        = 0,
    TX_FIFO_TRIG_2BYTES,
    TX_FIFO_TRIG_QUATER,
    TX_FIFO_TRIG_HALF,
    // RX FIFO TRIGGER
    RX_FIFO_TRIG_1BYTE        = 0,
    RX_FIFO_TRIG_QUATER,
    RX_FIFO_TRIG_HALF,
    RX_FIFO_TRIG_FULL_MINUS_2,
    // SHIFT
    FIFO_ENABLE_SHIFT         = 0,
    FIFO_RESET_SHIFT          = 2,
    DMA_MODE_SHIFT            = 3,
    TX_FIFO_TRIGGER_SHIFT     = 4,
    RX_FIFO_TRIGGER_SHIFT     = 6,
};

UART2_CTRL_TYPE * uart2_get_ctrl(void);
void uart2_init(uint8_t flowctrl, uint32_t baud);
void uart2_configure(UART2_CONFIG_TYPE type, uint32_t param);
void uart2_write(uint8_t * buf, uint8_t sz);
int  uart2_read(uint8_t *buf);

#endif
