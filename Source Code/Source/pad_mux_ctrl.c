#include "pad_mux_ctrl.h"

#define   MAX_PAD_MUX_NUM     39
#define   SEPERATE_NUM        32

static __IO uint8_t *pad[2]  = {(uint8_t *)PAD_SEL_BASE, (uint8_t *)PAD2_SEL_BASE};

int pad_mux_read(int pad_num)
{
    int high_4bits, index;
    uint8_t data;
    
    if (pad_num >= MAX_PAD_MUX_NUM || pad_num < 0)
        return -1;
    
    high_4bits = (pad_num & 1);
    if (pad_num < SEPERATE_NUM) { // PAD_SEL_BASE
        index = pad_num >> 1;
        data = pad[0][index];
    }
    else { // PAD2_SEL_BASE
        index = (pad_num - SEPERATE_NUM) >> 1;
        data = pad[1][index];
    }
    
    return high_4bits ? (data >> 4) : (data & 0xF);
}

void pad_mux_write(int pad_num, uint8_t select)
{
    int high_4bits, index;
    uint8_t data;
    __IO uint8_t *target;
    
    if (pad_num >= MAX_PAD_MUX_NUM || pad_num < 0)
        return;
    
    select &= 0x0F;
    high_4bits = (pad_num & 1);
    if (pad_num < SEPERATE_NUM) { // PAD_SEL_BASE
        index = pad_num >> 1;
        target = &pad[0][index];
    }
    else { // PAD2_SEL_BASE
        index = (pad_num - SEPERATE_NUM) >> 1;
        target = &pad[1][index];
    }
    data = *target;
    data &= (high_4bits) ? 0x0F : 0xF0;
    if (high_4bits)
        select <<= 4;
    
    *target = data | select;
}
