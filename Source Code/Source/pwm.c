#include "pwm.h"
#include "debug.h"
#include <string.h>

#define    PWM_CTRL    ((PWM1_CTRL_TYPE *) PWM1_CTRL_BASE)
static PWM_IRQ_CALLBACK_TYPE irq_cb;

void pwm_set_irq_callback(PWM_IRQ_CALLBACK_TYPE cb)
{
    if (irq_cb && irq_cb != cb) {
        DBG("PWM1_IRQ callback is overwritten\r\n");
    }
    irq_cb = cb;
}

void PWM1_IRQHandler(void)
{
    uint32_t int_st = PWM_CTRL->INT_STATUS;
    if (irq_cb)
        irq_cb(int_st);
    PWM_CTRL->INT_STATUS = ~int_st;
}

void pwm_set_prescaler(int p)
{
    PWM_CTRL->PRESCALER = p;
}

void pwm_set_decoder(PWM_DECODER_TYPE decoder)
{
    PWM_CTRL->DECODER = decoder;
}

void pwm_set_seq_mode(PWM_SEQ_MODE_TYPE mode)
{
    PWM_CTRL->SEQ_MODE = mode;
}

void pwm_set_seq(int id, int count, void *addr)
{
    uint32_t ptr = (uint32_t)addr;
    if (ptr & 0x03) {
        DBG("PWM DMA memory not aligned\r\n");
        return;
    }
    if (id != 0)
        id = 1;
    
    PWM_CTRL->PWM_SEQ[id].SEQ_CNT = count;
    PWM_CTRL->PWM_SEQ[id].SEQ_PTR = ptr;
}

void pwm_set_period_mode(PWM_PERIOD_MODE_TYPE mode)
{
    PWM_CTRL->MODE = mode;
}

void pwm_set_period_length(uint32_t tick)
{
    PWM_CTRL->COUNTERTOP = tick;
}

void pwm_set_loop_number(uint32_t loop)
{
    PWM_CTRL->LOOP_CNT = loop;
}

void pwm_start(int id)
{
    if (id != 0)
        id = 1;
    PWM_CTRL->TASKS_SEQSTART[id] = 1;
    while (PWM_CTRL->TASKS_SEQSTART[id]);
}

void pwm_stop(void)
{
    PWM_CTRL->TASK_STOP = 1;
    while (PWM_CTRL->TASK_STOP) ;
}

void pwm_enable_int(uint32_t int_mask, bool en)
{
    if (en) {
        PWM_CTRL->INTENSET = int_mask;
        return;
    }
    
    PWM_CTRL->INTENCLR = int_mask;
}
