/*
延时函数文件

作者：北京盛源达科技有限公司
日期：2018/3/8
*/
#include "delay.h"
/*
n毫秒延时函数
参数: uint16_t n 要延时的毫秒数 单位：1ms
*/
//8821 8000 1ms MCU64Mhz
void delay_ms(uint16_t n){
	uint16_t i,j;
	for(i=0;i<n;i++)
			for(j=0;j<8000;j++);//1ms
}
/*
n微秒延时函数
参数: uint16_t n 要延时的微秒数 单位：1us
*/
//MCU64Mhz
void delay_us(uint16_t n){
	uint16_t i,j;
	for(i=0;i<n;i++)
			for(j=0;j<9;j++);
}
