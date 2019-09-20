#ifndef _GPADC_H_
#define _GPADC_H_

#include "ARMCM0.h"
#include "stdbool.h"

#define    MAX_CH_COUNT    11

typedef void (*GPADC_IRQ_CALLBACK_TYPE)(uint32_t int_st);

typedef enum {
    GPADC_DISABLE_AVERAGE = 1,
    GPADC_SUM_TWO = 3,
    GPADC_SUM_FOUR = 5,
    GPADC_SUM_EIGHT = 7,
} GPADC_AVERAGE_TYPE;

typedef enum {
    VBAT_CHANNEL  = 0,
    GPIO0_CHANNEL = 1,
    GPIO1_CHANNEL = 2,
    GPIO2_CHANNEL = 3,
    GPIO3_CHANNEL = 4,
    GPIO4_CHANNEL = 5,
    GPIO5_CHANNEL = 6,
    GPIO6_CHANNEL = 7,
    GPIO7_CHANNEL = 8,
    GPIO8_CHANNEL = 9,
    GPIO9_CHANNEL = 10,
    CHANNEL_BIT_MASK = 0xF,
} GPADC_CHANNEL_TYPE;

typedef enum {
    GPADC_CLK_1M,
    GPADC_CLK_500K,
} GPADC_CHANNEL_CLK_TYPE;

typedef enum {
    GPADC_INT_TASK = 1,
    GPADC_INT_HALF = 2,
    GPADC_INT_FULL = 4,
    GPADC_INT_ALL  = (GPADC_INT_TASK | GPADC_INT_HALF | GPADC_INT_FULL),
} GPADC_INT_TYPE;

void     gpadc_set_irq_callback(GPADC_IRQ_CALLBACK_TYPE cb);
void     gpadc_enable_int(int en, GPADC_INT_TYPE int_mask);
uint32_t gpadc_get_DMA_len(void);
bool     gpadc_set_DMA(void *ptr, uint32_t length);
void     gpadc_start(int continuous, uint32_t scan_cnt);
void     gpadc_set_channel_timing(uint32_t start, uint32_t channel);
void     gpadc_set_data_length(uint32_t data_length);
uint32_t gpadc_get_data_length(void);
void     gpadc_set_channel(int num, GPADC_CHANNEL_TYPE * ch);
uint32_t gpadc_get_average_sample_number(void);
void     gpadc_set_average_sample_number(GPADC_AVERAGE_TYPE t);
void     gpadc_set_channel_clock(GPADC_CHANNEL_CLK_TYPE c);
uint32_t gpadc_get_channel_clock(void);

void adc_init(void);
void adc_open(GPADC_CHANNEL_TYPE channel);//切换通道时，只调用一次
uint16_t gpadc_get_value(void);//获取adc_open通道的值
void     gpadc_stop(void);

#endif //_GPADC_H_

