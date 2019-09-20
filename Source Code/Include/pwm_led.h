#ifndef _PWM_LED_H_
#define _PWM_LED_H_

#include "armcm0.h"
#include "config.h"

#define    PWM_LED_ID_NUM    6

typedef enum {
    PWM_LED_ALWAYS_ON_MODE,
    PWM_LED_PWM_MODE,
    PWM_LED_FLASH_MODE,
    PWM_LED_BREATH_MODE,
} PWM_LED_MODE_TYPE;

PWM_LED_CTRL_TYPE * pwm_led_get_ctrl(uint32_t id);
PWM_LED_MODE_TYPE pwm_led_get_mode(uint32_t id);
void pwm_led_set_polarity(uint32_t id, int high);
void pwm_led_flash_repeat(uint32_t id, uint32_t repeat);
void pwm_led_set_flash(uint32_t id, uint32_t duty_kept_length, uint32_t duty_cycle_length,
    uint32_t duty_cycle_count, uint32_t duty_cycle_end_delay, uint32_t duty_cycles_count);
void pwm_led_set_breath(uint32_t id, uint32_t min, uint32_t max, uint32_t period, uint32_t step);
void pwm_led_set_always_on(uint32_t id);
void pwm_led_set_pwm(uint32_t id, uint32_t start_high_period, uint32_t total_period);
void pwm_led_start(uint32_t id);
void pwm_led_stop(uint32_t id);
void pwm_led_pause(uint32_t id);
void pwm_led_resume(uint32_t id);

#endif //_PWM_LED_H_
