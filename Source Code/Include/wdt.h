#ifndef _WDT_H_
#define _WDT_H_

#include "config.h"
#include "stdbool.h"

typedef void (* WDT_IRQ_CALLBACK) (void);

#define WDT_RR_ALL  0xFF

enum {
    WDT_RESET_MCU = 1,
    WDT_RESET_SYSTEM = 2,
    WDT_RESET_ALL = 3,
};

void wdt_set_interrupt_callback(WDT_IRQ_CALLBACK cb);
void wdt_int_clear(void);
void wdt_int_enable(void);
void wdt_int_disable(void);
void wdt_start(bool reset_counter);
void wdt_stop(void);
void wdt_set_crv(uint32_t crv);
void wdt_set_RR_enable(uint32_t bitmask);
void wdt_reload_RR(uint32_t bitmask);
void wdt_reset_counter(void);
void wdt_set_reset_type(uint32_t type);

#endif //_WDT_H_
