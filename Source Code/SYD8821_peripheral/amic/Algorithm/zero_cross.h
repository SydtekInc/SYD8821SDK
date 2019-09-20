#ifndef _ZERO_CROSSING_H_
#define _ZERO_CROSSING_H_

#include <stdint.h>
#include <math.h>


#define NO_CROSS_ZERO 0
#define CROSS_ZERO 1




extern int32_t is_zero_crossing(uint32_t len,int16_t * buf);

#endif //_ZERO_CROSSING_H_



