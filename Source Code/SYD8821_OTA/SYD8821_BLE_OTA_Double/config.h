#ifndef _CONFIG_H_
#define _CONFIG_H_ 

#include "ARMCM0.h"
#include "ble_slave.h"

//#define USER_32K_CLOCK_RCOSC

//RTT debug�ʹ���debug��û�����⴦��������ֻ�ܹ�ʹ��һ����Ҳ����˵����ĺ�ֻ�ܹ���һ��
//#define _SYD_RTT_DEBUG_
#define CONFIG_UART_ENABLE

#define KEY_INT_PIN GPIO_14
#define LED1_Pin GPIO_34
#define LED2_Pin GPIO_10


#define MAX_UART_WAIT	2
#define UART_TOBLE_THU	64
#define UART_WAIT_TIMER_ID	TIMER_1
#define UART_TOBLE_QUEUE_ID	0
#define BLE_TOUART_QUEUE_ID	1


#define _CONFIG_NOUART2_

#define _SWDDEBUG_DISENABLE_

/*
bit0:��10s�¼�(�ɼ���ص���)
bit1:��60s�¼�(���ӵ�)
bit2:У׼ʱ���¼�(��185s)
*/
enum SYD_1S_EVENT{
    SYD_1S_10S          =   0x00000001,
    SYD_1S_60S          =   0x00000002,	
    SYD_1S_185S         =   0x00000004,		
    SYD_1S_600S         =   0x00000008,		
};

#endif
