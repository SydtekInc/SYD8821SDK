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
#include "i2c_slave.h"
#include "ble_slave.h"

//#define IIC1_MASTER  0x01
#define IIC1_SLAVE 0x01
#define    I2C_CTRL    ((I2C_SLAVE_CTRL_TYPE *) I2C_SLAVE_CTRL_BASE)

int iic_s_int_flag = 0;
uint32_t iic_s_rx_buf[16] = {0x00};
uint32_t iic_s_tx_buf[16] = {0x01,0x02,0x03,0x04,0x05,0x06};

static void i2c_slave_irq_handle(int int_st)
{
	iic_s_int_flag = int_st;
	
	//主机写完数据,从机接收完数据响应INT_I2CS_WRITE中断
	if(int_st&I2C_CTRL->INT_I2CS_WRITE)
	{
		dbg_printf("INT_I2CS_WRITE:%d\r\n",iic_s_int_flag);
		dbg_hexdump("RX:\r\n",(uint8_t*)iic_s_rx_buf, 16);
	}
	//结束IIC操作，响应INT_I2CS_STOPPED中断
	if(int_st&I2C_CTRL->INT_I2CS_STOPPED)
	{
		dbg_printf("INT_I2CS_STOPPED:%d\r\n",iic_s_int_flag);
	}
	//主机读数据，从机收到寄存器地址，响应INT_I2CS_READ断
	if(int_st&I2C_CTRL->INT_I2CS_READ)//主机读数据,从机发送数据完成
	{
		dbg_printf("INT_I2CS_READ:%d\r\n",iic_s_int_flag);
		dbg_hexdump("RX:\r\n",(uint8_t*)iic_s_rx_buf, 1);//收到地址
		iic_s_tx_buf[0] += 1;
		i2c_slave_set_dma(I2C_SLAVE_TX, iic_s_tx_buf, 4);//4是一个buf的大小。一个buf=4byte
	}
}
void i2c_slave_init(void)
{
	i2c_slave_set_irq_callback(i2c_slave_irq_handle);
	i2c_slave_set_address_mode(I2C_SLAVE_0BYTE_ADDRESS);
	
	i2c_slave_set_slave_id(0x3C,0x3C);
	i2c_slave_enable(1);
	i2c_slave_set_dma(I2C_SLAVE_RX, iic_s_rx_buf, 16);
	i2c_slave_set_dma(I2C_SLAVE_TX, iic_s_tx_buf, 4);
	i2c_slave_enable_int(1);
	NVIC_EnableIRQ(I2CS_IRQn);
}


int main()
{	
	#ifdef IIC1_MASTER
	uint8_t iic_buff[32]={0};
	uint8_t iic_addr, iic_dat[16];
	#endif
	__disable_irq();
	
	gap_s_ble_init();
	
	sys_mcu_rc_calibration();
	delay_ms(100);
	#ifdef IIC1_MASTER
	sys_mcu_clock_set(MCU_CLOCK_24_MHZ);
	#elif IIC1_SLAVE
	sys_mcu_clock_set(MCU_CLOCK_96_MHZ);
	#endif
	// RC bumping
	
	//GPIO 25,23,34,10
	pad_mux_write(LED1, 0);
	pad_mux_write(LED2, 0);
	pad_mux_write(LED3, 0);
	pad_mux_write(LED4, 0);
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
	gpo_config(LED4,1);
	
	//GPIO 14,15,16,17
//	pad_mux_write(KEY1, 0);
//	pad_mux_write(KEY2, 0);
//	pad_mux_write(KEY3, 0);
//	gpi_config(KEY1, PULL_UP);
//	gpi_config(KEY2, PULL_UP);
//	gpi_config(KEY3, PULL_UP);

	//uart 0
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	
	#ifdef IIC1_MASTER
	dbg_printf("SYD8821 IIC Masters DEMO\r\n");

	//IIC Masters
	pad_mux_write(8, 6); //i2c 0 scl
	pad_mux_write(9, 6); //i2c 0 sda
	//set i2c clk rate param 0, (48/8)*50K=300k 
	i2c_master_set_speed(1, 0);
	i2c_master_enable(1, true);
	i2c_master_enable_int(1, false);
	i2c_master_set_address_mode(1, I2C_MASTER_1BYTE_ADDRESS);
	#elif IIC1_SLAVE
	dbg_printf("SYD8821 IIC Slave DEMO\r\n");
	//IIC Slave
	pad_mux_write(16, 6); //i2c 0 scl
	pad_mux_write(17, 6); //i2c 0 sda
	i2c_slave_init();
	#endif
	__enable_irq();
		

	while(1)
	{
		gpo_toggle(0x400);
//		iic_addr = 0x02;
//		iic_dat[0] = 0x00;
//		//i2c_write(1,0x3c,iic_addr, iic_dat,0x01);
//		i2c_read(1,0x3c,iic_addr,iic_dat,0x01);
//		delay_ms(100);
//		
//		dbg_printf("iic_dat:%x\r\n",iic_dat[0]);
		#ifdef IIC1_MASTER
			iic_addr = 0x12;
			iic_dat[0] = 0xaa;
			iic_dat[1] = 0x55;
			iic_dat[2] = 0x11;
			i2c_write(1,0x3c,iic_addr, iic_dat,0x03);
		#endif
		
		delay_ms(100);
	}		
}
