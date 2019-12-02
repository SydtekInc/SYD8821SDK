/*
算法来源:https://github.com/ctuning
*/

#ifndef _ADPCM_H_
#define _ADPCM_H_

#include <stdint.h>

//ADPCM压缩参考点
struct adpcm_state
{
	short valprev; /* Previous output value */
	short index;  /* Index into stepsize table */
};
extern struct adpcm_state adpcm_state_encdoe;

/*
**函数名称：Adpcm_encode
**函数功能：PCM音频压缩函数
**输入参数：
********** indata：原始音频数据
********** outdata：输出压缩后的音频数据
********** len：原始音频数据的字节数/2
********** state：压缩中间变量，记录上一次原始值，初始化为0
**返回参数：无
*/
extern void Adpcm_encode(short indata[], char outdata[], int len, struct adpcm_state *state);

/*
**函数名称：Adpcm_decode
**函数功能：PCM音频解压函数
**输入参数：
********** indata：压缩后的音频数据
********** outdata：输出解压后的音频数据
********** len：解压后音频数据的字节数/2
********** state：解压中间变量，记录上一次压缩值，初始化为0
**返回参数：无
*/
extern void Adpcm_decode(char indata[], short outdata[], int len, struct adpcm_state *state);

#endif
