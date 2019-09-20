#include "wdt.h"
#include "debug.h"
#include <string.h>

#define WDT_MAGIC   0x5F763214

#define WDT_CTRL    ((WDT_CTRL_TYPE2 *)  WDT_CTRL_BASE)
#define SYS_CONFIG  ((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)

static WDT_IRQ_CALLBACK irq_cb;

void WDT_IRQHandler(void)
{
    if (irq_cb)
        irq_cb();
    wdt_int_clear();
}

void wdt_set_interrupt_callback(WDT_IRQ_CALLBACK cb)
{
    irq_cb = cb;
}

void wdt_reset_counter(void)
{
    wdt_reload_RR(WDT_RR_ALL);
    while (!WDT_CTRL->RR[7]);
}

void wdt_int_clear(void)
{
    WDT_CTRL->EVENTS_TIME_OUT = 1;
}

void wdt_int_enable(void)
{
    WDT_CTRL->INTENSET = 1;
	  NVIC_EnableIRQ(WDT_IRQn);
}

void wdt_int_disable(void)
{
    WDT_CTRL->INTENCLR = 1;
}

void wdt_start(bool reset_counter)
{
    if (reset_counter)
        wdt_reset_counter();
    WDT_CTRL->TASK_START = 1;
    while (WDT_CTRL->TASK_START);
}

void wdt_stop(void)
{
    WDT_CTRL->CONFIG = 1;
    while (WDT_CTRL->CONFIG);
}

void wdt_set_crv(uint32_t crv)
{
    WDT_CTRL->CRV = crv;
}

void wdt_set_RR_enable(uint32_t bitmask)
{
    WDT_CTRL->RR_EN = bitmask;
}

void wdt_reload_RR(uint32_t bitmask)
{
    int i;
    
    for (i = 0; i < WDT_RR_NUM; i++) {
        if (!bitmask)
            break;
        if (bitmask & 1) {
            WDT_CTRL->RR[i] = WDT_MAGIC;
        }
        bitmask >>= 1;
    }
}

void wdt_set_reset_type(uint32_t type)
{
    SYS_CONFIG->WDT_RST_EN     = (type & WDT_RESET_MCU) != 0;
    SYS_CONFIG->WDT_RST_ALL_EN = (type & WDT_RESET_SYSTEM) != 0;
}
