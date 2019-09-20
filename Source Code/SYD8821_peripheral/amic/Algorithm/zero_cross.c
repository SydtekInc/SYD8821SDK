#include "zero_cross.h"




int32_t is_zero_crossing(uint32_t len,int16_t * buf){
	
	uint32_t idx=0;
	int32_t is_crossing=0;
	
	for(idx=0;idx<len-1;idx++){
		//if((buf[idx] > 0 && buf[idx + 1] <= 0) || (buf[idx] < 0 && buf[idx + 1] >= 0)){
		if((buf[idx] > 0 && buf[idx + 1] <= 0)){
				is_crossing=1;
				break;
		}
	}
	return is_crossing;
	
	 
}
int32_t rc_find_buf_max_peak(uint32_t len,int16_t * buf){
	
	uint32_t idx=0;
	int32_t peak=0;
	
	for(idx=0;idx<len-1;idx++){
		
		if(buf[idx] >= peak){
			peak=buf[idx];
		}
	}
//	DBGPRINTF(("max peak %d\r\n",peak));
	return peak;
	
}
int32_t rc_find_buf_min_peak(uint32_t len,int16_t * buf){
	
	uint32_t idx=0;
	int32_t peak=0;
	peak=buf[0];
	for(idx=0;idx<len-1;idx++){
		
		if(buf[idx] <= peak){
			peak=buf[idx];
		}
	}
//	DBGPRINTF(("min peak %d\r\n",peak));
	return peak;
	
}

int32_t rc_find_buf_abs_peak(uint32_t len,int16_t * buf){
	
	uint32_t idx=0;
	int32_t peak=0;

	for(idx=0;idx<len-1;idx++){
		
		if(abs(buf[idx]) >= peak){
			peak=buf[idx];
		}
	}
//	DBGPRINTF(("abs peak %d\r\n",peak));
	return peak;
	
}

int32_t rc_find_buf_average(uint32_t len,int16_t * buf){
	
	uint32_t 	idx=0;
	int32_t 	sum=0;
	int32_t  	average=0;
	for(idx=0;idx<len-1;idx++){
		sum=sum+buf[idx];
	}
	average=(sum/len);
//	DBGPRINTF(("average peak %d\r\n",average));
	return average;
	
}


