#ifndef _ADPCM_H_
#define _ADPCM_H_



#include <stdint.h>
#include "debug.h"


struct adpcm_state {
	int16_t 	 predsample;		/* Previous output value */
	int16_t 	index;			/* Index into stepsize table */
};
uint8_t Adpcm_encode(int16_t  sample, struct adpcm_state *adpcm_state_t);
int16_t Adpcm_decode(uint8_t code,	 struct adpcm_state *adpcm_state_t);


#endif
