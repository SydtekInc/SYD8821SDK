#ifndef _RTC_H_
#define _RTC_H_

#include "config.h"
#include "stdbool.h"

typedef enum {
    RTC_INT_CMP0 = 1,
    RTC_INT_CMP1 = 2,
    RTC_INT_TICK = 4,
    RTC_INT_ALL  = 7,
    RTC_INT_NUM  = 3,
} RTC_INT_TYPE;

typedef void (* RTC_IRQ_CALLBACK) (RTC_INT_TYPE type);

#pragma pack(push, 1)
typedef union {
    struct {
        uint8_t  second;
        uint8_t  minute;
        uint8_t  hour;
        uint8_t  day;
    } decimal_format;
    uint8_t  u8[4];
    uint32_t u32;
} RTC_TIME_TYPE;
#pragma pack(pop)

void rtc_set_interrupt_callback(RTC_IRQ_CALLBACK cb);
void rtc_int_clear(RTC_INT_TYPE type);
void rtc_int_enable(RTC_INT_TYPE type);
void rtc_int_disable(RTC_INT_TYPE type);
void rtc_start(void);
void rtc_stop(void);
void rtc_clear(void);
void rtc_set_prescaler(uint32_t tick, bool adjust_seconds_bit);
void rtc_set_seconds_bit(uint32_t order);
RTC_TIME_TYPE rtc_get_compare(int id);
void rtc_set_compare(int id, RTC_TIME_TYPE *time);
RTC_TIME_TYPE rtc_get_calendar(void);
void rtc_set_calendar(RTC_TIME_TYPE *time);
bool rtc_status(void);
uint32_t rtc_interrupt_status(void);

#endif //_RTC_H_
