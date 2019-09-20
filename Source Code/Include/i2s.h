#ifndef _I2S_H_
#define _I2S_H_


#include "config.h"
#include "stdbool.h"

typedef void (*I2S_IRQ_CALLBACK_TYPE)(uint32_t int_st);

typedef enum {
    I2S_RX,
    I2S_TX,
} I2S_TRANSFER_TYPE;

typedef enum {
    I2S_MSB_FIRST,
    I2S_LSB_FIRST,
} I2S_BIT_ORDER_TYPE;

typedef enum {
    I2S_FISRT_BIT_SECOND_CYCLE,
    I2S_FISRT_BIT_FIRST_CYCLE,
} I2S_BIT_FORMAT_TYPE;

typedef enum {
    I2S_LEFT_HIGH,
    I2S_LEFT_LOW,
} I2S_CHANNEL_CLK_TYPE;

typedef enum {
    I2S_STEREO,
    I2S_MONO_LEFT,
    I2S_MONO_RIGHT,
} I2S_CHANNEL_TYPE;

typedef enum {
    I2S_16BIT,
    I2S_8BIT,
} I2S_SAMPLE_WIDTH_TYPE;

typedef enum {
    I2S_MCLK_DIV1,
    I2S_MCLK_DIV2,
    I2S_MCLK_DIV4,
    I2S_MCLK_DIV8,
    I2S_MCLK_DIV16,
} I2S_BIT_CLK_DIVISOR_TYPE;

typedef enum {
    I2S_CLK_DIV2,
    I2S_CLK_DIV3,
    I2S_CLK_DIV4,
    I2S_CLK_DIV5,
    I2S_CLK_DIV6,
    I2S_CLK_DIV8,
    I2S_CLK_DIV10,
    I2S_CLK_DIV11,
    I2S_CLK_DIV15,
    I2S_CLK_DIV16,
    I2S_CLK_DIV21,
    I2S_CLK_DIV23,
    I2S_CLK_DIV31,
    I2S_CLK_DIV42,
    I2S_CLK_DIV63,
    I2S_CLK_DIV125,
} I2S_MCLK_FREQ_TYPE;

typedef enum {
    I2S_T_DMA_HALF_DONE_INT_ST = 1UL,
    I2S_R_DMA_HALF_DONE_INT_ST = 1UL << 1,
    I2S_DMA_HALF_DONE_INT_ST = I2S_T_DMA_HALF_DONE_INT_ST | I2S_R_DMA_HALF_DONE_INT_ST,
    I2S_T_DMA_DONE_INT_ST = 1UL << 8,
    I2S_R_DMA_DONE_INT_ST = 1UL << 9,
    I2S_DMA_DONE_INT_ST = I2S_T_DMA_DONE_INT_ST | I2S_R_DMA_DONE_INT_ST,
    I2S_DMA_ALL_INT_ST = I2S_DMA_HALF_DONE_INT_ST | I2S_DMA_DONE_INT_ST,
    I2S_T_DMA_HALF_DONE_INT = 1UL << 4,
    I2S_R_DMA_HALF_DONE_INT = 1UL << 5,
    I2S_DMA_HALF_DONE_INT = I2S_T_DMA_HALF_DONE_INT | I2S_R_DMA_HALF_DONE_INT,
    I2S_T_DMA_DONE_INT = 1UL << 12,
    I2S_R_DMA_DONE_INT = 1UL << 13,
    I2S_DMA_DONE_INT = I2S_T_DMA_DONE_INT | I2S_R_DMA_DONE_INT,
    I2S_DMA_ALL_INT = I2S_DMA_HALF_DONE_INT | I2S_DMA_DONE_INT,
} I2S_INT_TYPE;

void i2s_set_irq_callback(I2S_IRQ_CALLBACK_TYPE cb);
void i2s_enable_int(I2S_INT_TYPE en);
bool i2s_set_DMA(I2S_TRANSFER_TYPE type, void *ptr, int length);
bool i2s_trigger_RXDMA(void *ptr, int length);
bool i2s_trigger_TXDMA(void *ptr, int length);
void i2s_stop(void);
void i2s_set_bit_order(I2S_TRANSFER_TYPE t, I2S_BIT_ORDER_TYPE o);
void i2s_set_transfer_format(I2S_BIT_FORMAT_TYPE f);
void i2s_set_channel(I2S_CHANNEL_TYPE c);
void i2s_set_channel_clock(I2S_CHANNEL_CLK_TYPE c);
void i2s_set_sample_width(I2S_SAMPLE_WIDTH_TYPE w);
void i2s_set_mclk(I2S_MCLK_FREQ_TYPE m);
void i2s_set_bit_clk(I2S_BIT_CLK_DIVISOR_TYPE d);

#endif //_I2S_H_
