#include "rtc.h"
#include "debug.h"
#include <string.h>

#define RTC_CTRL    ((RTC_CTRL_TYPE2  *) RTC_CTRL_BASE)
#define SYS_CONFIG  ((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)
#define PMU_CTRL    ((PMU_CTRL_TYPE   *) PMU_CTRL_BASE)

static RTC_IRQ_CALLBACK irq_cb;
static bool enabled = false;
static uint32_t int_status = 0;

void RTC_IRQHandler(void)
{
    RTC_INT_TYPE status = (RTC_INT_TYPE)RTC_CTRL->EVENTS;
    
    if (irq_cb)
        irq_cb(status);
    
    rtc_int_clear(status);
}

void rtc_set_interrupt_callback(RTC_IRQ_CALLBACK cb)
{
    irq_cb = cb;
}

void rtc_int_clear(RTC_INT_TYPE type)
{
    RTC_CTRL->EVENTS = type;
}

void rtc_int_enable(RTC_INT_TYPE type)
{
    RTC_CTRL->INTENSET = type;
    int_status |= type;
}

void rtc_int_disable(RTC_INT_TYPE type)
{
    RTC_CTRL->INTENCLR = type;
    int_status &= ~type;
}

void rtc_start(void)
{
    RTC_CTRL->TASK_START = 1;
    while (RTC_CTRL->TASK_START);
    enabled = true;
}

void rtc_stop(void)
{
    /* This reset RTC counter for prescaler*/
    RTC_CTRL->TASK_STOP = 1;
    while (RTC_CTRL->TASK_STOP);
    enabled = false;
}

void rtc_clear(void)
{
    /* This clear COUNTER & CALENDAR */
    RTC_CTRL->TASK_CLEAR = 1;
    while (RTC_CTRL->TASK_CLEAR);
}

void rtc_set_prescaler(uint32_t tick, bool adjust_seconds_bit)
{
    uint32_t prescaler;
    RTC_CTRL->PRESCALER = tick - 1;
    prescaler = (tick - 1) & 0x7FFF;
    
    if (adjust_seconds_bit) {
        uint32_t i;
        
        for (i = 0; i < 32; i++) {
            if (prescaler == 0)
                break;
            prescaler >>= 1;
        }
        RTC_CTRL->SECONDS_BIT = 15 - i;
    }   
}

void rtc_set_seconds_bit(uint32_t order)
{
    RTC_CTRL->SECONDS_BIT = order;
}

RTC_TIME_TYPE rtc_get_compare(int id)
{
    __IO uint32_t * compare = id ? &RTC_CTRL->COMPARE1 : &RTC_CTRL->COMPARE0;
    int i;
    RTC_TIME_TYPE time;
    
    time.u32 = *compare;
    
    // time format to decimal
    for (i = 0 ; i < 3; i++) {
        uint8_t tmp = time.u8[i];
        time.u8[i] = (tmp >> 4) * 10;
        time.u8[i] += (tmp & 0xF);
    }
    
    return time;
}

void rtc_set_compare(int id, RTC_TIME_TYPE *time)
{
    __IO uint32_t * compare = id ? &RTC_CTRL->COMPARE1 : &RTC_CTRL->COMPARE0;
    int i;
    // Decimal to time format
    for (i = 0; i < 3; i++) {
        uint8_t tmp = time->u8[i];
        time->u8[i] = (tmp / 10) << 4;
        time->u8[i] |= tmp % 10;
    }
    *compare = time->u32;
		
		//Workaround set compare on the fly
    if (enabled) {
        RTC_CTRL->CALENDAR = RTC_CTRL->CALENDAR;
        RTC_CTRL->COUNTER = RTC_CTRL->COUNTER;
        RTC_CTRL->TASK_START = 1;
		//while (RTC_CTRL->TASK_START);
    }
}

RTC_TIME_TYPE rtc_get_calendar(void)
{
    int i;
    RTC_TIME_TYPE time;
    
    time.u32 = RTC_CTRL->CALENDAR;
    
    // time format to decimal
    for (i = 0 ; i < 3; i++) {
        uint8_t tmp = time.u8[i];
        time.u8[i] = (tmp >> 4) * 10;
        time.u8[i] += (tmp & 0xF);
    }
    
    return time;
}    

void rtc_set_calendar(RTC_TIME_TYPE *time)
{
    int i;
    // Decimal to time format
    for (i = 0; i < 3; i++) {
        uint8_t tmp = time->u8[i];
        time->u8[i] = (tmp / 10) << 4;
        time->u8[i] |= tmp % 10;
    }
    
    RTC_CTRL->CALENDAR = time->u32;
		
		//Workaround set calendar on the fly
    if (enabled) {
        RTC_CTRL->COUNTER = RTC_CTRL->COUNTER;
        RTC_CTRL->TASK_START = 1;
    }
}

bool rtc_status(void)
{
    return enabled;
}

uint32_t rtc_interrupt_status(void)
{
    return int_status;
}
