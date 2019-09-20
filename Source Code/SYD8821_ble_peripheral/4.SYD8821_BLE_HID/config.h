#ifndef _CONFIG_H_
#define _CONFIG_H_ 

#include "ARMCM0.h"






//#define USER_32K_CLOCK_RCOSC

//RTT debug和串口debug在没有特殊处理的情况下只能够使用一个，也就是说下面的宏只能够开一个
#define CONFIG_UART_ENABLE

//#define _SYD_RTT_DEBUG_

//#define _WECHAT_
#define _HID_

#define KEY1_Pin GPIO_14
#define KEY2_Pin GPIO_15
#define KEY3_Pin GPIO_16
#define KEY4_Pin GPIO_17

#define LED1_Pin GPIO_25
#define LED2_Pin GPIO_23
#define LED3_Pin GPIO_34
#define LED4_Pin GPIO_10

#define UART_RXD_0 GPIO_20
#define UART_TXD_0 GPIO_21


#define _CONFIG_NOUART2_

//开启这个宏，SWD就不能连续下载，会进入睡眠模式
//#define _SWDDEBUG_DISENABLE_

/*
bit0:整10s事件(采集电池电量)
bit1:整60s事件(闹钟等)
bit2:校准时刻事件(整185s)
*/

enum SYD_1S_EVENT{
    SYD_1S_10S          =   0x00000001,
    SYD_1S_60S          =   0x00000002,	
    SYD_1S_185S         =   0x00000004,		
    SYD_1S_600S         =   0x00000008,		
};

enum CCCD_STATUS
{
	DISABLE = 0x00,
	ENABLE = 0x01
};

struct HID_KeyBoard_Report
{
	uint8_t modifier_key;
	uint8_t reserved;
	uint8_t keycode_1; 
	uint8_t keycode_2; 
	uint8_t keycode_3; 
	uint8_t keycode_4; 
	uint8_t keycode_5; 
	uint8_t keycode_6; 
};

struct HID_KeyBoard_Media_Report
{
	uint16_t data;
};

struct HID_KeyBoard_Power_Report
{
	uint8_t data;
};

#endif
