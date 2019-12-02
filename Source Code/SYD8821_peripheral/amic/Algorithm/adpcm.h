/*
�㷨��Դ:https://github.com/ctuning
*/

#ifndef _ADPCM_H_
#define _ADPCM_H_

#include <stdint.h>

//ADPCMѹ���ο���
struct adpcm_state
{
	short valprev; /* Previous output value */
	short index;  /* Index into stepsize table */
};
extern struct adpcm_state adpcm_state_encdoe;

/*
**�������ƣ�Adpcm_encode
**�������ܣ�PCM��Ƶѹ������
**���������
********** indata��ԭʼ��Ƶ����
********** outdata�����ѹ�������Ƶ����
********** len��ԭʼ��Ƶ���ݵ��ֽ���/2
********** state��ѹ���м��������¼��һ��ԭʼֵ����ʼ��Ϊ0
**���ز�������
*/
extern void Adpcm_encode(short indata[], char outdata[], int len, struct adpcm_state *state);

/*
**�������ƣ�Adpcm_decode
**�������ܣ�PCM��Ƶ��ѹ����
**���������
********** indata��ѹ�������Ƶ����
********** outdata�������ѹ�����Ƶ����
********** len����ѹ����Ƶ���ݵ��ֽ���/2
********** state����ѹ�м��������¼��һ��ѹ��ֵ����ʼ��Ϊ0
**���ز�������
*/
extern void Adpcm_decode(char indata[], short outdata[], int len, struct adpcm_state *state);

#endif
