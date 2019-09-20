#include "i2s.h"
#include "debug.h"
#include <string.h>

#define    I2S_CTRL    ((I2S_CTRL_TYPE *) I2S_CTRL_BASE)

static I2S_IRQ_CALLBACK_TYPE irq_cb;

void I2S_IRQHandler(void)
{
    uint32_t int_st = I2S_CTRL->INT_ST;
    I2S_CTRL->INT_ST = int_st;
    if (irq_cb) {
        irq_cb(int_st);
    }
}

void i2s_set_irq_callback(I2S_IRQ_CALLBACK_TYPE cb)
{
    irq_cb = cb;
}

void i2s_enable_int(I2S_INT_TYPE en)
{
    I2S_CTRL->INT_ST = I2S_DMA_ALL_INT ^ en;
}

bool i2s_set_DMA(I2S_TRANSFER_TYPE type, void *ptr, int length)
{
    uint32_t address = (uint32_t)ptr;
    
    if (address & 0x3)
        return false;
    
    address >>= 2;
    length >>= 2;
    
    if (type == I2S_RX) {
        if (address)
            I2S_CTRL->DMA_RX_PTR = address;
        if (length)
            I2S_CTRL->DMA_RX_MAXLEN = length;
    }
    else {
        if (address)
            I2S_CTRL->DMA_TX_PTR = address;
        if (length)
            I2S_CTRL->DMA_TX_MAXLEN = length;
    }
    return true;
}

bool i2s_trigger_RXDMA(void *ptr, int length)
{
    if (I2S_CTRL->I2S_BUSY)
        return false;
    
    i2s_set_DMA(I2S_RX, ptr, length);
    
    I2S_CTRL->I2S_RXEN = 1;
    return true;
}

bool i2s_trigger_TXDMA(void *ptr, int length)
{
    if (I2S_CTRL->I2S_BUSY)
        return false;
    
    i2s_set_DMA(I2S_TX, ptr, length);
    
    I2S_CTRL->I2S_TXEN = 1;
    return true;
}

void i2s_stop(void)
{
    if (I2S_CTRL->I2S_RXEN)
        I2S_CTRL->I2S_RXEN = 0;
    if (I2S_CTRL->I2S_TXEN)
        I2S_CTRL->I2S_TXEN = 0;
    while (I2S_CTRL->I2S_BUSY);
}

void i2s_set_bit_order(I2S_TRANSFER_TYPE t, I2S_BIT_ORDER_TYPE o)
{
    if (t == I2S_RX)
        I2S_CTRL->LSB_R = o;
    else
        I2S_CTRL->LSB_T = o;
}

void i2s_set_transfer_format(I2S_BIT_FORMAT_TYPE f)
{
    I2S_CTRL->FORMAT = f;
}

void i2s_set_channel(I2S_CHANNEL_TYPE c)
{
    I2S_CTRL->CHANNEL = c;
}

void i2s_set_channel_clock(I2S_CHANNEL_CLK_TYPE c)
{
    I2S_CTRL->LRSWAP = c;
}

void i2s_set_sample_width(I2S_SAMPLE_WIDTH_TYPE w)
{
    I2S_CTRL->SWIDTH = w;
}

void i2s_set_mclk(I2S_MCLK_FREQ_TYPE m)
{
    I2S_CTRL->MFREQ = m;
}

void i2s_set_bit_clk(I2S_BIT_CLK_DIVISOR_TYPE d)
{
    I2S_CTRL->SRATIO = d;
}
