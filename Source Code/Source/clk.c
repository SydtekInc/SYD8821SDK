#include "clk.h"
#include "ble_slave.h"

#define  SYS_CTRL          ((SYS_CONFIG_TYPE *) SYS_CTRL_BASE)

uint32_t RC_get(void)
{
    uint8_t config;
    uint32_t clock = 64000000;
    
    BBRFWrite(0x7F,0x00);//bank A
    BBRFRead(0x3F, &config);
    config &= 0xC0;
    
    switch (config) {
        case 0x80:
            clock = 96000000;
            break;
        case 0x40:
            clock = 80000000;
            break;
    }
    
    if (SYS_CTRL->CLK_SPEED_SEL)
        clock >>= 2;
    
    return clock;
}

uint32_t FCLK_get(void)
{
    return (RC_get() / (1 << SYS_CTRL->FCLK_CONFIG));
}

uint32_t PCLK_get(void)
{
    return (FCLK_get() / (1 << (SYS_CTRL->PCLK_CONFIG + 1)));
}

uint32_t SCLK_get(void)
{
    return 32000000;
}



