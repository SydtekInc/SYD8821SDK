/*
UART����main.c

���ߣ�����ʢԴ��Ƽ����޹�˾
���ڣ�2018/3/19
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "uart.h"
#include "queue.h"
#include "spi0.h"
#include "gsensor.h"
#include "debug.h"

int main()
{	
	int16_t x,y,z;
	uint16_t i=0;
	float vat=0.0;
	__disable_irq();
	//GPO
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
  gpo_config(LED4,1);
	
	//GPI
	pad_mux_write(KEY1, 0);
	pad_mux_write(KEY2, 0);
	pad_mux_write(KEY3, 0);
	pad_mux_write(KEY4, 0);
	gpi_config(KEY1, PULL_UP);
	gpi_config(KEY2, PULL_UP);
	gpi_config(KEY3, PULL_UP);
	gpi_config(KEY4, PULL_UP);
	//uart 1
//	pad_mux_write(4, 7);
//	pad_mux_write(5, 7);
//	uart_1_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
//	uart_write(1,"SYD8821 UART TEST", 18);

// uart 0
	pad_mux_write(2, 7);
	pad_mux_write(3, 7);
	uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
	dbg_printf(0,"SYD8821 spio there line TEST", 18);
	// spi0
	pad_mux_write(26, 3);
	pad_mux_write(27, 3);
	pad_mux_write(28, 3);
	pad_mux_write(29, 3);
	
  if(kx022_init()) dbg_printf("kx022 init succeed\r\n");
	else dbg_printf("kx022 init faild\r\n");
	
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
		
				/*
		���㹫ʽ�����ٶ� = ������ԭʼֵ * 8000mG / 65536 / 1000  
		���е�8000mG�������ģ������̵Ĳ��������� +-4G  Ҳ��������4G�ķ�Χ����������Resolution/Range�Ĵ��������õ�
		��ô���ܶ��������������65536��Ҳ����˵ÿ�����ݵĿ̶��� 8000/65536 mg
		*/
		gensor_read_xyz(&x, &y, &z);
		vat=(float)x*8000/65536/1000;
		dbg_printf("x : %02x vat: %4.3f i:%d\r\n",x,vat,i);
		vat=(float)y*8000/65536/1000;
		dbg_printf("y : %02x vat: %4.3f i:%d\r\n",y,vat,i);
		vat=(float)z*8000/65536/1000;
		dbg_printf("z : %02x vat: %4.3f i:%d\r\n",z,vat,i);
		delay_ms(500);
		i++;
		
    delay_ms(100);
	}		
}
