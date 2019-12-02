#ifndef __PDM_MODULE_H
#define __PDM_MODULE_H

#include "ARMCM0.h"

#define AMIC_VREF_PIN			GPIO_26
#define AMIC_BIAS_PIN			GPIO_27
#define AMIC_INN_PIN			GPIO_28
#define AMIC_INP_PIN			GPIO_29

#define RC_AMIC_GAIN   		0x0D
#define RC_PDM_GAIN   		0x28

#define PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH    480  
#define PDM_DMA_16BIT_ENC_BUF_LENGTH           PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH/4
#define PDM_DMA_16BIT_SAMPLES_HALF_START_IDX   PDM_DMA_16BIT_SAMPLES_TOTOAL_LENGTH/2

#define NUM_OF_VOICE_DATA_FRAME (PDM_DMA_16BIT_ENC_BUF_LENGTH/RC_BLE_PAYLOAD_LEN)

void pdm_module_init(void);
void start_record(void);
void stop_record(void);

extern uint8_t pdm_int_flag;
extern uint8_t* pdm_enc_buf_ptr;

#endif /* __PDM_MODULE_H */
