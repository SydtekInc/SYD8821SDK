/*
UART例程main.c

作者：北京盛源达科技有限公司
日期：2018/3/19
*/
#include "ARMCM0.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "delay.h"
#include "led_key.h"
#include "uart.h"
#include "queue.h"
#include "debug.h"
#include "i2c_master.h"
#include "gsensor.h"

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

  //uart 0
	pad_mux_write(12, 7);
	pad_mux_write(13, 7);
	dbg_init();
	dbg_printf("SYD8821 IIC DEMO\r\n");

  //kx022 gsensor SDA AND CS LOW
	pad_mux_write(15, 0);
	gpo_config(15,0);
	pad_mux_write(17, 0);
	gpo_config(17,1);
	
	// IIC
	pad_mux_write(20, 6); //i2c 1 scl
	pad_mux_write(21, 6); //i2c 1 sda

	/* Masters Init M0*/
	//set i2c clk rate param 0, (48/8)*50K=300k 
	i2c_master_set_speed(1, 0);
	i2c_master_enable(1, true);
	i2c_master_enable_int(1, true);
	i2c_master_set_address_mode(1, I2C_MASTER_1BYTE_ADDRESS);
	NVIC_EnableIRQ(I2CM0_IRQn);
	
	if(kx022_init()) dbg_printf("kx022 init succeed\r\n");
	else dbg_printf("kx022 init faild\r\n");

	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);
		
				/*
		计算公式：加速度 = 读到的原始值 * 8000mG / 65536 / 1000  
		其中的8000mG是这样的：本例程的测量量程是 +-4G  也就是正负4G的范围，这里是在Resolution/Range寄存器里设置的
		那么所能读到的最大数据是65536，也就是说每个数据的刻度是 8000/65536 mg
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

		delay_ms(1000);
	}		
}
