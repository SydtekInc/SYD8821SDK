/*
��ʱ�����ļ�

���ߣ�����ʢԴ��Ƽ����޹�˾
���ڣ�2018/3/8
*/
#include "delay.h"
/*
n������ʱ����
����: uint16_t n Ҫ��ʱ�ĺ����� ��λ��1ms
*/
//8821 8000 1ms MCU64Mhz
void delay_ms(uint16_t n){
	uint16_t i,j;
	for(i=0;i<n;i++)
			for(j=0;j<8000;j++);//1ms
}
/*
n΢����ʱ����
����: uint16_t n Ҫ��ʱ��΢���� ��λ��1us
*/
//MCU64Mhz
void delay_us(uint16_t n){
	uint16_t i,j;
	for(i=0;i<n;i++)
			for(j=0;j<9;j++);
}
