#ifndef _GPIO_H_
#define _GPIO_H_

#include "armcm0.h"
#include "gpio.h"


//GPIO_0 <--> GPIO_31
#define GPIO_CTRL_0 ((GPIO_CTRL_TYPE *) GPIO_CTRL_BASE)
//GPIO_32 <--> GPIO_39
#define GPIO_CTRL_1 ((GPIO_CTRL_TYPE *) GPIO32_CTRL_BASE)

typedef enum {
    GPIO_BANK_0,  // GPIO  0 ~ 31
    GPIO_BANK_1,  // GPIO 32 ~ 38
    GPIO_BANK_NUM,
    GPIO_BANK_SIZE = 32,
} GPIO_BANK_TYPE;

typedef enum {
    PULL_UP,
    PULL_DOWN,
} GPIO_INPUT_PULL_TYPE;

typedef enum {
    LEVEL_TRIGGER,
    EDGE_TRIGGER,
} GPI_INT_TRIGGER_TYPE;

typedef enum {
    POL_RISING_HIGH,
    POL_FALLING_LOW,
} GPI_INT_POLARITY_TYPE;

typedef void (*GPI_IRQ_CB_TYPE)(GPIO_BANK_TYPE type, uint32_t bitmask);
//typedef void (*GPI_IRQ_CB_TYPE)(void);


void gpi_irq_set_cb(GPI_IRQ_CB_TYPE cb);
void gpi_config(uint32_t io, GPIO_INPUT_PULL_TYPE pull);
void pad_input_configure(uint32_t io, int en);
void gpo_config(uint32_t io, int val);
void gpo_toggle(uint32_t io);
void gpo_set(uint32_t io);
void gpo_clr(uint32_t io);
int  gpi_get_val(uint32_t io);
void gpi_enable_int(uint32_t io, GPI_INT_TRIGGER_TYPE trigger, GPI_INT_POLARITY_TYPE pol);
void gpi_disable_int(uint32_t io);

#endif //_GPIO_H_
