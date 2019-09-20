





#include "adpcm.h"




static const uint16_t StepSizeTable[89] = { 7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767 };
/* Table of index changes */

static const int16_t IndexTable[16] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };



uint8_t Adpcm_encode(int16_t  sample,struct adpcm_state *adpcm_state_t)
{
	uint8_t code = 0;
	uint16_t tmpstep = 0;
	int32_t diff = 0;
	int32_t diffq = 0;
	uint16_t step = 0;

	step = StepSizeTable[adpcm_state_t->index];

	/* 2. compute diff and record sign and absolut value */
	diff = sample - adpcm_state_t->predsample;
	//DBGPRINTF(("diff:%d sample:%d predsample:%d \r\n ",diff,sample,adpcm_state_t->predsample));
   

	if (diff < 0)
	{
		code = 8;
		diff = -diff;
	}

	/* 3. quantize the diff into ADPCM code */
	/* 4. inverse quantize the code into a predicted diff */
	tmpstep = step;
	diffq = (step >> 3);
   
	if (diff >= tmpstep)
	{
		code |= 0x04;
		diff -= tmpstep;
		diffq += step;
	}

	tmpstep = (uint16_t)(tmpstep >> 1);

	if (diff >= tmpstep)
	{
		code |= 0x02;
		diff -= tmpstep;
		diffq += (step >> 1);
	}

	tmpstep =  (uint16_t)(tmpstep >> 1);

	if (diff >= tmpstep)
	{
		code |= 0x01;
		diffq += (step >> 2);
	}

	
	/* 5. fixed predictor to get new predicted sample*/
	if ((code & 8)>0)
	{
		adpcm_state_t->predsample -=(int16_t)diffq;
	}
	else
	{
		adpcm_state_t->predsample += (int16_t)diffq;
	}

	/* check for overflow*/
	if (adpcm_state_t->predsample > 32767)
	{
		adpcm_state_t->predsample = 32767;
	}
	else if (adpcm_state_t->predsample < -32768)
	{
		adpcm_state_t->predsample = -32768;
	}

	/* 6. find new stepsize index */
	adpcm_state_t->index += IndexTable[code];
	/* check for overflow*/
	if (adpcm_state_t->index <0)
	{
		adpcm_state_t->index = 0;
	}
	else if (adpcm_state_t->index > 88)
	{
		adpcm_state_t->index = 88;
	}

	/* 8. return new ADPCM code*/
	return (uint8_t)(code & 0x0f);
}

int16_t Adpcm_decode(uint8_t code, struct adpcm_state *adpcm_state_t)
{
	
		uint16_t step = 0;
		int32_t diffq = 0;

		step = StepSizeTable[adpcm_state_t->index];

		/* 2. inverse code into diff */
		diffq = step >> 3;
		if (code & 4)
		{
			diffq += step;
		}

		if (code & 2)
		{
			diffq += step >> 1;
		}

		if (code & 1)
		{
			diffq += step >> 2;
		}

		/* 3. add diff to predicted sample*/
		if (code & 8)
		{
			adpcm_state_t->predsample -= diffq;
		}
		else
		{
			adpcm_state_t->predsample += diffq;
		}

		/* check for overflow*/
		if (adpcm_state_t->predsample > 32767)
		{
			adpcm_state_t->predsample = 32767;
		}
		else if (adpcm_state_t->predsample < -32768)
		{
			adpcm_state_t->predsample = -32768;
		}

		/* 4. find new quantizer step size */
		adpcm_state_t->index += IndexTable[code];
		/* check for overflow*/
		if (adpcm_state_t->index < 0)
		{
			adpcm_state_t->index = 0;
		}
		if (adpcm_state_t->index > 88)
		{
			adpcm_state_t->index = 88;
		}

		/* 5. save predict sample and index for next iteration */
		/* done! static variables */

		/* 6. return new speech sample*/
		return ((int16_t)adpcm_state_t->predsample);
	}

	
	
	
	
void Adpcm_encode_block_test(int16_t sample_src_buf[],uint16_t sample_src_buf_len,uint8_t final_data_buf[], struct adpcm_state *adpcm_state_t){
	
		uint16_t idx=0;
		uint16_t final_data_idx=0;
		uint16_t final_data=0x00;
	
		uint8_t nibble_high=0x00;
		uint8_t nibble_low=0x00;
		//uint8_t code=0x00;
	
		
		for(idx=0;idx<sample_src_buf_len;idx=idx+2){

			dbg_printf("sample_src_buf[] %d \r\n",sample_src_buf[idx]);
			dbg_printf("sample_src_buf[] %d\r\n",sample_src_buf[idx+1]);
			
			nibble_high=Adpcm_encode(sample_src_buf[idx],adpcm_state_t);
			nibble_low=Adpcm_encode(sample_src_buf[idx+1],adpcm_state_t);

			nibble_high=(nibble_high<<4);
			final_data=(nibble_high |nibble_low);
			
			final_data_buf[final_data_idx]=final_data;
			final_data_idx++;
		 

		}
		dbg_printf("final_data_idx%d\r\n",final_data_idx);

}





