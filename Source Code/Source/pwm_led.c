#include "pwm_led.h"
#include "debug.h"
#include <string.h>

static PWM_LED_CTRL_TYPE * PWM_LED_CTRL[PWM_LED_ID_NUM] = 
{
    (PWM_LED_CTRL_TYPE *) PWM_LED0_CTRL_BASE,
    (PWM_LED_CTRL_TYPE *) PWM_LED1_CTRL_BASE,
    (PWM_LED_CTRL_TYPE *) PWM_LED2_CTRL_BASE,
    (PWM_LED_CTRL_TYPE *) PWM_LED3_CTRL_BASE,
    (PWM_LED_CTRL_TYPE *) PWM_LED4_CTRL_BASE,
    (PWM_LED_CTRL_TYPE *) PWM_LED5_CTRL_BASE,
};
static PWM_LED_MODE_TYPE pwm_led_mode[PWM_LED_ID_NUM];

PWM_LED_CTRL_TYPE * pwm_led_get_ctrl(uint32_t id)
{
    return PWM_LED_CTRL[id];
}

PWM_LED_MODE_TYPE pwm_led_get_mode(uint32_t id)
{
    return pwm_led_mode[id];
}

void pwm_led_set_polarity(uint32_t id, int high)
{
    PWM_LED_CTRL[id]->POL = (high != 0);
}

void pwm_led_flash_repeat(uint32_t id, uint32_t repeat)
{
    int rp_n[2] = {0};
    
    repeat &= 0x3;
    
    if (repeat & 1) { // RP_N1
        rp_n[0] = 1;
    }
    
    if (repeat & 2) { // RP_N2
        rp_n[1] = 1;
    }

    PWM_LED_CTRL[id]->RP_N1 = rp_n[0];
    PWM_LED_CTRL[id]->RP_N2 = rp_n[1];
}

void pwm_led_set_flash(uint32_t id, uint32_t duty_kept_length, uint32_t duty_cycle_length,
    uint32_t duty_cycle_count, uint32_t duty_cycle_end_delay, uint32_t duty_cycles_count)
{
    PWM_LED_CTRL[id]->T1 = duty_kept_length;
    PWM_LED_CTRL[id]->T2 = duty_cycle_length;
    PWM_LED_CTRL[id]->N1 = duty_cycle_count;
    PWM_LED_CTRL[id]->T3 = duty_cycle_end_delay;
    PWM_LED_CTRL[id]->N2 = duty_cycles_count;
    pwm_led_mode[id] = PWM_LED_FLASH_MODE;
}

void pwm_led_set_breath(uint32_t id, uint32_t min, uint32_t max, uint32_t period, uint32_t step)
{
    PWM_LED_CTRL[id]->BR_TH_MAX = max;
    PWM_LED_CTRL[id]->BR_TH_MIN = min;
    PWM_LED_CTRL[id]->T4 = period;
    PWM_LED_CTRL[id]->BR_SP = step;
    pwm_led_mode[id] = PWM_LED_BREATH_MODE;
}

void pwm_led_set_always_on(uint32_t id)
{
    pwm_led_mode[id] = PWM_LED_ALWAYS_ON_MODE;
}

void pwm_led_set_pwm(uint32_t id, uint32_t start_high_period, uint32_t total_period)
{
    pwm_led_mode[id] = PWM_LED_PWM_MODE;
    PWM_LED_CTRL[id]->PWM_N = start_high_period;
    PWM_LED_CTRL[id]->PWM_M = total_period;
}

void pwm_led_start(uint32_t id)
{
    pwm_led_stop(id);
    PWM_LED_CTRL[id]->MODE = pwm_led_mode[id];
    pwm_led_resume(id);
}

void pwm_led_stop(uint32_t id)
{
    pwm_led_pause(id);
    while (!PWM_LED_CTRL[id]->LED_RESETB);
    PWM_LED_CTRL[id]->LED_RESETB = 0;
    while (!PWM_LED_CTRL[id]->LED_RESETB);
}

void pwm_led_pause(uint32_t id)
{
    PWM_LED_CTRL[id]->LED_EN = 0;
    while (PWM_LED_CTRL[id]->LED_EN);
}

void pwm_led_resume(uint32_t id)
{
    PWM_LED_CTRL[id]->LED_EN = 1;
    while (!PWM_LED_CTRL[id]->LED_EN);
}
