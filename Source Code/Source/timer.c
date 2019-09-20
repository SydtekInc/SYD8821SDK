#include "timer.h"



static TIMER_CTRL_TYPE *ctrl[TIMER_NUM] = {
    (TIMER_CTRL_TYPE *)TIMER0_CTRL_BASE,
    (TIMER_CTRL_TYPE *)TIMER1_CTRL_BASE,
    (TIMER_CTRL_TYPE *)TIMER2_CTRL_BASE,
    (TIMER_CTRL_TYPE *)TIMER3_CTRL_BASE,
    (TIMER_CTRL_TYPE *)TIMER4_CTRL_BASE,
};

static TIMER_IRQ_CALLBACK_TYPE cb[TIMER_NUM];

void TIMER0_IRQHandler(void)
{
    ctrl[0]->INTERRUPT_OUT = 1;
    if (cb[0]) cb[0]();
}

void TIMER1_IRQHandler(void)
{
    ctrl[1]->INTERRUPT_OUT = 1;
    if (cb[1]) cb[1]();
}

void TIMER2_IRQHandler(void)
{
    ctrl[2]->INTERRUPT_OUT = 1;
    if (cb[2]) cb[2]();
}

void TIMER3_IRQHandler(void)
{
    ctrl[3]->INTERRUPT_OUT = 1;
    if (cb[3]) cb[3]();
}

void TIMER4_IRQHandler(void)
{
    ctrl[4]->INTERRUPT_OUT = 1;
    if (cb[4]) cb[4]();
}


uint8_t is_timer_enabled(int id)
{
    return ctrl[id]->TIMER_ENABLE;
}

uint8_t timer_enable(TIMER_ID id, TIMER_IRQ_CALLBACK_TYPE callback, int interval, int int_enable)
{
    if (is_timer_enabled(id) || id >= TIMER_NUM)
        return 0;
    
    cb[id] = callback;
		if(id==TIMER_4)ctrl[id]->TIMER4_RELOAD_VAL = interval;
		else ctrl[id]->TIMER_RELOAD_VAL = interval;
    
    if (int_enable)
        ctrl[id]->TIMER_INTERRUPT_ENABLE = 1;
    else
        ctrl[id]->TIMER_INTERRUPT_DISABLE = 1;
    
    ctrl[id]->TIMER_ENABLE = 1;
    
    return 1;
}

void timer_disable(int id)
{
    if (!is_timer_enabled(id) || id >= TIMER_NUM)
        return;
    ctrl[id]->TIMER_ENABLE = 0;
}


uint8_t timer_state_get(int id)
{
    return ctrl[id]->TIMER_ENABLE;
}


