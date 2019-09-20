#ifndef _PAD_MUX_CTRL_H_
#define _PAD_MUX_CTRL_H_


#include "ARMCM0.h"
#include "stdbool.h"

int pad_mux_read(int pad_num);
void pad_mux_write(int pad_num, uint8_t select);

#endif //_PAD_MUX_CTRL_H_
