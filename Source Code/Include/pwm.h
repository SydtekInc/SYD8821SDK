#ifndef _PWM_H_
#define _PWM_H_


#include "config.h"
#include "stdbool.h"

#define ACTIVE_HIGH_LEAVE 0x8000
#define ACTIVE_LOW_LEAVE  0x0000

#define ACTIVE_LEAVE_POL  ACTIVE_HIGH_LEAVE

typedef void (* PWM_IRQ_CALLBACK_TYPE)(uint32_t int_st);

typedef enum {
    PWM_SEQ0_FINISH_INT       = 1,
    PWM_SEQ1_FINISH_INT       = 2,
    PWM_LOOP_COUNT_FINISH_INT = 4,
    PWM_PWM_PERIOD_FINISH_INT = 8,
    PWM_TASK_STOP_INT         = 16,
    PWM_ALL_INT               = 0x1F,
} PWM_INT_TYPE;

typedef enum {
    PWM_COMMON,
    PWM_GROUP,
    PWM_INDIVIDUAL,
    PWM_WAVEFORM,
} PWM_DECODER_TYPE;

typedef enum {
    SINGLE_SEQUENCE_LOOP     = 0,
    DUAL_SEQUENCE_LOOP       = 1,
    SINGLE_SEQUENCE_CONTINUE = 2,
    DUAL_SEQUENCE_CONTINUE   = 3,
    PWM_SEQ_MODE_NUM,
} PWM_SEQ_MODE_TYPE;

typedef enum {
    UP_ONLY,
    UP_AND_DOWN,
} PWM_PERIOD_MODE_TYPE;

#pragma pack(push, 2)
typedef struct {
    uint16_t cmp_apply_all;
} SEQ_COMMON_TYPE;

typedef struct {
    uint16_t cmp_apply_2[2];
} SEQ_GROUP_TYPE;

typedef struct {
    uint16_t cmp_apply_1[4];
} SEQ_INDIVIDUAL_TYPE;

typedef struct {
    uint16_t cmp_apply_1[3];
    uint16_t counterop;
} SEQ_WAVEFORM_TYPE;
#pragma pack(pop)

void pwm_set_irq_callback(PWM_IRQ_CALLBACK_TYPE cb);
void pwm_set_prescaler(int p);
void pwm_set_decoder(PWM_DECODER_TYPE decoder);
void pwm_set_seq_mode(PWM_SEQ_MODE_TYPE mode);
void pwm_set_seq(int id, int count, void *addr);
void pwm_set_period_mode(PWM_PERIOD_MODE_TYPE mode);
void pwm_set_period_length(uint32_t tick);
void pwm_set_loop_number(uint32_t loop);
void pwm_start(int seq_id);
void pwm_stop(void);
void pwm_enable_int(uint32_t int_mask, bool en);

#endif //_PWM_H_
