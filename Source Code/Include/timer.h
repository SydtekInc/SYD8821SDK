#ifndef _TIMER_H_
#define _TIMER_H_

#include "ARMCM0.h"

#define TIMER_NUM 5

typedef void (* TIMER_IRQ_CALLBACK_TYPE)(void);

typedef enum {
    TIMER_0 = 0,
    TIMER_1 ,
    TIMER_2 ,
		TIMER_3 ,
		TIMER_4 ,
} TIMER_ID;



uint32_t	get_tick_count(void);
uint8_t is_timer_enabled(int id);
uint8_t timer_enable(TIMER_ID id, TIMER_IRQ_CALLBACK_TYPE callback, int interval, int int_enable);
void timer_disable(int id);
uint8_t timer_state_get(int id);

#endif //_TIMER_H_
