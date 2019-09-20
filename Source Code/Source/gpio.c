#include "gpio.h"
#include "armcm0.h"

#define    PAD_CTRL    ((PAD_CTRL_TYPE *) PAD_CTRL_BASE)

static GPIO_CTRL_TYPE * GPIO_CTRL[GPIO_BANK_NUM] =  \
{(GPIO_CTRL_TYPE *) GPIO_CTRL_BASE, (GPIO_CTRL_TYPE *) GPIO32_CTRL_BASE};

//static GPI_IRQ_CB_TYPE gpio_callback;
static GPI_IRQ_CB_TYPE gpio_callback;

void gpi_irq_set_cb(GPI_IRQ_CB_TYPE cb)
{
    gpio_callback = cb;
}

void gpi_config(uint32_t io, GPIO_INPUT_PULL_TYPE pull)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIO_DIR_SET = config;
    if (index) {
        if (pull == PULL_UP) {
            PAD_CTRL->PAD3_PUP |= config;
        }
        else {
            PAD_CTRL->PAD3_PUP &= ~config;
        }
    }
    else {
        if (pull == PULL_UP) {
            PAD_CTRL->PAD_PUP |= config;
        }
        else {
            PAD_CTRL->PAD_PUP &= ~config;
        }
    }
}

void pad_input_configure(uint32_t io, int en)
{
	int index = ((io & GPIO_BANK_SIZE) != 0);
	int config = 1 << (io & ~GPIO_BANK_SIZE);
	
	if (index) {
		if (en) {
			PAD_CTRL->PAD3_IE |= config;
		}
		else {
			PAD_CTRL->PAD3_IE &= ~config;
		}
	}
	else {
		if (en) {
			PAD_CTRL->PAD_IE |= config;
		}
		else {
			PAD_CTRL->PAD_IE &= ~config;
		}
	}
}


void gpo_config(uint32_t io, int val)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIO_DIR_CLR = config;
    if (val) {
        GPIO_CTRL[index]->GPIO_OUT_SET = config;
    }
    else {
        GPIO_CTRL[index]->GPIO_OUT_CLR = config;
    }
}

void gpo_toggle(uint32_t io)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIO_OUT_TOG = config;
}

void gpo_set(uint32_t io)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIO_OUT_SET = config;
}

void gpo_clr(uint32_t io)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIO_OUT_CLR = config;
}

int gpi_get_val(uint32_t io)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    return ((GPIO_CTRL[index]->GPIO_IN & config) != 0);
}

void gpi_enable_int(uint32_t io, GPI_INT_TRIGGER_TYPE trigger, GPI_INT_POLARITY_TYPE pol)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    
    if (trigger == LEVEL_TRIGGER) {
        GPIO_CTRL[index]->GPIOINT_TYPE_CLR = config;
    }
    else { // EDGE TRIGGER
        GPIO_CTRL[index]->GPIOINT_TYPE_SET = config;
    }
    
    if (pol == POL_RISING_HIGH) {
        GPIO_CTRL[index]->GPIOINT_POL_CLR = config;
    }
    else { // FALLING_LOW
        GPIO_CTRL[index]->GPIOINT_POL_SET = config;
    }
    
    GPIO_CTRL[index]->GPIOINT_ENABLE_SET = config;
	NVIC_EnableIRQ(GPIO_IRQn);
}

void gpi_disable_int(uint32_t io)
{
    int index = ((io & GPIO_BANK_SIZE) != 0);
    int config = 1 << (io & ~GPIO_BANK_SIZE);
    
    GPIO_CTRL[index]->GPIOINT_ENABLE_CLR = config;
}


//void GPIO_IRQHandler(void)
//{		
//	// check interrupt
//	if(GPIO_CTRL[0]->GPIOINT_STATUS & GPIO_CTRL[0]->GPIOINT_ENABLE)
//	{
//		if(gpio_callback != 0)(*gpio_callback)();
//		// clear interrupt
//		GPIO_CTRL[0]->GPIOINT_STATUS = GPIO_CTRL[0]->GPIOINT_STATUS;
//	} 
//}



void GPIO_IRQHandler(void)
{
    GPIO_BANK_TYPE i;
    
    for (i = GPIO_BANK_0; i < GPIO_BANK_NUM; i++) {
        uint32_t gpio_int_status = GPIO_CTRL[i]->GPIOINT_STATUS & GPIO_CTRL[i]->GPIOINT_ENABLE;
        if (!gpio_int_status)
            continue;
        if (gpio_callback)
            gpio_callback(i, gpio_int_status);
        
        GPIO_CTRL[i]->GPIOINT_STATUS = gpio_int_status;
    }
}


