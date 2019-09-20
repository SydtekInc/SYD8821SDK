#ifndef _PDM_H_
#define _PDM_H_



#include <stdint.h>
#include "ARMCM0.h"
#include "ble_slave.h"


typedef void (*PDM_IRQ_CALLBACK_TYPE)(uint8_t int_st);

typedef enum {
    PDM_DMA_FINISH_INT  = 0x80,
    PDM_DMA_HALF_INT    = 0x40,
    PDM_DMA_AMICI2S_INT = 0x20,
    PDM_DMA_ALL_INT     = 0xE0,
} PDM_INT_STATUS_TYPE;

typedef enum {
    PDM_LEFT_MONO_FALLING_EDGE,
    PDM_LEFT_MONO_RISING_EDGE,
} PDM_SAMPLE_TRIGGER_TYPE;

typedef enum {
    PDM_SAMPLE_STEREO,
    PDM_SAMPLE_MONO,
} PDM_CHANNEL_TYPE;

typedef enum {
    PDM_16BIT,
    PDM_8BIT,
} PDM_SAMPLE_WIDTH_TYPE;

typedef enum {
    PDM_15625Hz_SAMPLE,
    PDM_7812_5Hz_SAMPLE,
} PDM_SAMPLE_RATE_TYPE;

typedef enum {
    PDM_2M,
    PDM_4M,
    PDM_1M,
} PDM_CLK_CTRL_TYPE;

typedef enum {
    REMOVE_DC,
    KEEP_DC,
} PDM_DC_MODE_TYPE;

typedef enum {
    PDM_GAINL,
    PDM_GAINR,
    PDM_GAIN_CHANNEL_NUM,
} PDM_GAIN_CHANNEL_TYPE;

void pdm_set_irq_callback(PDM_IRQ_CALLBACK_TYPE cb);
void pdm_enable_int(PDM_INT_STATUS_TYPE int_en);
void pdm_start(void);
void pdm_stop(void);
bool pdm_set_DMA(void *ptr, int length);
void pdm_set_channel(PDM_CHANNEL_TYPE c);
void pdm_set_sample_width(PDM_SAMPLE_WIDTH_TYPE w);
void pdm_set_clk(PDM_CLK_CTRL_TYPE c);
void pdm_set_sample_rate(PDM_SAMPLE_RATE_TYPE s);
void pdm_set_sample_trigger(PDM_SAMPLE_TRIGGER_TYPE t);
void pdm_set_dc_mode(int en, PDM_DC_MODE_TYPE m, int period);
void pdm_software_scaling(PDM_GAIN_CHANNEL_TYPE t, int16_t * data);
uint8_t pdm_get_sw_gain(PDM_GAIN_CHANNEL_TYPE t);
void pdm_set_gain(PDM_GAIN_CHANNEL_TYPE t, uint8_t gain);
void pdm_enable_amic(int en);
void pdm_enable_amic_i2s(int en);
void pdm_set_amic_gain(uint8_t gain);
uint8_t pdm_is_enable(void);

#endif //_PDM_H_
