#include "ARMCM0.h"
#include "gpio.h"
#include "debug.h"
#include "delay.h"
#include "ota.h"
#include "config.h"
#include "ble_master.h"
#include "SYD_ble_service_Profile.h"
#include "queue.h"
#include "uart.h"
#include <string.h>
#include "timer.h"
#include "pad_mux_ctrl.h"
#include "Debuglog.h"
#include "led_key.h"
#include "def.h"

#define CS_NUM	3

struct ble_scan_device_info_t ble_scan_device_info[CS_NUM];
struct ble_scan_cs_num_t ble_scan_cs_num = {0x00, false};

uint8_t bd0_addr[BD_ADDR_SZ] = {0x01, 0x00, 0x00, 0x00, 0x00, 0xa0};
uint8_t bd1_addr[BD_ADDR_SZ] = {0x02, 0x00, 0x00, 0x00, 0x00, 0xa0};
uint8_t bd2_addr[BD_ADDR_SZ] = {0x03, 0x00, 0x00, 0x00, 0x00, 0xa0};

static void setup_scan_data(uint8_t cs_num)
{
	struct gap_scan_params_central scan_params;
	
	scan_params.type = ACTIVE_SCAN;  //主动扫描
	scan_params.interval= 64;  // 扫描间隔
	scan_params.window = 150;    // 窗口
	scan_params.white_list_en = 0x00;  	//不使能百名单 
	scan_params.cs_num = cs_num;    //在0位置扫描
	scan_params.address_type=PUBLIC_ADDRESS_TYPE;

	gap_c_scan_parameters_set(&scan_params);
}


static void setup_connect_data(uint8_t cs_num, uint8_t *addr)
{
	struct INITIATOR_CONFIG_TYPE connect_config= {
		32, //.scn_itv
		32, //.scn_window
		0, //.filter_policy
		PUBLIC_ADDRESS_TYPE, //.address_type
		NULL, //.addr
		PUBLIC_ADDRESS_TYPE, //.own_addr_type
		12, //.conn_itv
		400, //.conn_svto   200->200ms
		0, //.conn_latency
		24, //.conn_evt_length
	};
	connect_config.addr = addr;
	gap_c_add_white_list(addr, WL_PUBLIC);
    gap_c_connect_connect_config_set(cs_num, &connect_config);
}


void ble_evt_callback_csnum0(struct gap_ble_evt *p_evt)
{
	static uint8_t current_device_num = 0;
	//DBGPRINTF(("evt_callback0:%x\r\n",p_evt->evt_code));
	if(p_evt->evt_code == GAP_EVT_ADV_REPORT)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		//DBGPRINTF(("\r\nCS_NUM = 1\r\n"));
		//DBGPRINTF(("ScanRx type:%x rssi:%x len:%x\r\n",p_evt->evt.advertising_report_evt.type, p_evt->evt.advertising_report_evt.rssi,p_evt->evt.advertising_report_evt.len));
		//DBGHEXDUMP("addr:",(void *)&p_evt->evt.advertising_report_evt.peer_dev_addr,sizeof(struct gap_ble_addr));
		//DBGHEXDUMP("buf:",(void *)p_evt->evt.advertising_report_evt.buf,p_evt->evt.advertising_report_evt.len);
		#endif
		p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len]='\0';
		
		/* 查询列表中的设备 */
		for(uint8_t i = 0; i < CS_NUM; i++)
		{
			if (strstr((const char *)&p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len-8],(const char *)ble_scan_device_info[i].scan_name) != NULL)
			{
				if(memcmp(p_evt->evt.advertising_report_evt.peer_dev_addr.addr,ble_scan_device_info[i].scan_addr,BD_ADDR_SZ)==0)
				{
					DBGPRINTF(("Find device, current_device_num = %d, cs_num = 0\r\n", i));
					DBGHEXDUMP("MAC:", ble_scan_device_info[i].scan_addr, BD_ADDR_SZ);
					ble_scan_device_info[i].device_cs_num = CS_NUM_0;
					ble_scan_device_info[i].device_status = 1;
					current_device_num = i;
					
					gap_c_scan_stop(CS_NUM_0);					
					DBGPRINTF(("Stop cs_num = %d\r\n", CS_NUM_0));
					
					break;
				}
			}
		}
	}
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("have Connect to slave then discover sevice\r\n"));
		#endif
		gap_c_discover_sevice_start(CS_NUM_0);
		ble_scan_device_info[current_device_num].device_status = 3;
	}
	else if(p_evt->evt_code == GAP_EVT_DISCOVER_LOOP)
	{		
		if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_LOOP)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service loop:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif
		}
		else if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_FINISH)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service finish:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif

			ble_scan_device_info[current_device_num].device_status = 4;
			ble_scan_cs_num.wait_for_connected = false;
		}
	}
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("slave disconnect %x\r\n",p_evt->evt.disconn_evt.reason));
		#endif
		
		ble_scan_device_info[current_device_num].device_status = 5;
		current_device_num = 0;
	}
}

void ble_attc_evt_callback_csnum0(struct attc_ble_evt *p_evt)
{
	if(p_evt->attc_code == ATT_READ_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("CS_NUM0, ATT_READ_RSP:\r\n",(void *)&p_evt->attc.AttReadRsp,p_evt->attc_sz);
		#endif
	}
	else if(p_evt->attc_code == ATT_HANDLE_VAL_NOTIFICATION)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("Slave notify:\r\n",p_evt->attc.AttHdlValNotification.buf,p_evt->attc_sz-2);
		#endif
	}
}


void ble_evt_callback_csnum1(struct gap_ble_evt *p_evt)
{
	static uint8_t current_device_num = 0;
	
	//DBGPRINTF(("evt_callback0:%x\r\n",p_evt->evt_code));
	if(p_evt->evt_code == GAP_EVT_ADV_REPORT)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		//DBGPRINTF(("\r\nCS_NUM = 2\r\n"));
		//DBGPRINTF(("ScanRx type:%x rssi:%x len:%x\r\n",p_evt->evt.advertising_report_evt.type, p_evt->evt.advertising_report_evt.rssi,p_evt->evt.advertising_report_evt.len));
		//DBGHEXDUMP("addr:",(void *)&p_evt->evt.advertising_report_evt.peer_dev_addr,sizeof(struct gap_ble_addr));
		//DBGHEXDUMP("buf:",(void *)p_evt->evt.advertising_report_evt.buf,p_evt->evt.advertising_report_evt.len);
		#endif
		p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len]='\0';
		
		/* 查询列表中的设备 */
		for(uint8_t i = 0; i < CS_NUM; i++)
		{
			if (strstr((const char *)&p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len-8],(const char *)ble_scan_device_info[i].scan_name) != NULL)
			{
				if(memcmp(p_evt->evt.advertising_report_evt.peer_dev_addr.addr,ble_scan_device_info[i].scan_addr,BD_ADDR_SZ)==0)
				{
					DBGPRINTF(("Find device, current_device_num = %d, cs_num = 1\r\n", i));
					DBGHEXDUMP("MAC:", ble_scan_device_info[i].scan_addr, BD_ADDR_SZ);

					ble_scan_device_info[i].device_cs_num = CS_NUM_1;
					ble_scan_device_info[i].device_status = 1;
					current_device_num = i;
					
					gap_c_scan_stop(CS_NUM_1);
					DBGPRINTF(("Stop cs_num = %d\r\n", CS_NUM_1));
					
					break;
				}
			}
		}
	}
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("have Connect to slave then discover sevice\r\n"));
		#endif
		gap_c_discover_sevice_start(CS_NUM_1);
//		device_state=3;
		ble_scan_device_info[current_device_num].device_status = 3;
	}
	else if(p_evt->evt_code == GAP_EVT_DISCOVER_LOOP)
	{		
		if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_LOOP)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service loop:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif
		}
		else if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_FINISH)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service finish:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif
			ble_scan_device_info[current_device_num].device_status = 4;
			ble_scan_cs_num.wait_for_connected = false;
		}
	}
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("slave disconnect %x\r\n",p_evt->evt.disconn_evt.reason));
		#endif
		ble_scan_device_info[current_device_num].device_status = 5;
		current_device_num = 0;
	}

}

void ble_attc_evt_callback_csnum1(struct attc_ble_evt *p_evt)
{
	if(p_evt->attc_code == ATT_READ_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("CS_NUM1, ATT_READ_RSP:\r\n",(void *)&p_evt->attc.AttReadRsp,p_evt->attc_sz);
		#endif
	}
	else if(p_evt->attc_code == ATT_HANDLE_VAL_NOTIFICATION)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("Slave notify:\r\n",p_evt->attc.AttHdlValNotification.buf,p_evt->attc_sz-2);
		#endif
	}
}


void ble_evt_callback_csnum2(struct gap_ble_evt *p_evt)
{
	static uint8_t current_device_num = 0;
	
	//DBGPRINTF(("evt_callback0:%x\r\n",p_evt->evt_code));
	if(p_evt->evt_code == GAP_EVT_ADV_REPORT)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		//DBGPRINTF(("\r\nCS_NUM = 2\r\n"));
		//DBGPRINTF(("ScanRx type:%x rssi:%x len:%x\r\n",p_evt->evt.advertising_report_evt.type, p_evt->evt.advertising_report_evt.rssi,p_evt->evt.advertising_report_evt.len));
		//DBGHEXDUMP("addr:",(void *)&p_evt->evt.advertising_report_evt.peer_dev_addr,sizeof(struct gap_ble_addr));
		//DBGHEXDUMP("buf:",(void *)p_evt->evt.advertising_report_evt.buf,p_evt->evt.advertising_report_evt.len);
		#endif
		p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len]='\0';
		
		/* 查询列表中的设备 */
		for(uint8_t i = 0; i < CS_NUM; i++)
		{
			if (strstr((const char *)&p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len-8],(const char *)ble_scan_device_info[i].scan_name) != NULL)
			{
				if(memcmp(p_evt->evt.advertising_report_evt.peer_dev_addr.addr,ble_scan_device_info[i].scan_addr,BD_ADDR_SZ)==0)
				{
					DBGPRINTF(("Find device, current_device_num = %d, cs_num = 2\r\n", i));
					DBGHEXDUMP("MAC:", ble_scan_device_info[i].scan_addr, BD_ADDR_SZ);

					ble_scan_device_info[i].device_cs_num = CS_NUM_2;
					ble_scan_device_info[i].device_status = 1;
					current_device_num = i;
					
					gap_c_scan_stop(CS_NUM_2);
					DBGPRINTF(("Stop cs_num = %d\r\n", CS_NUM_2));
					
					break;
				}
			}
		}
	}
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("have Connect to slave then discover sevice\r\n"));
		#endif
		gap_c_discover_sevice_start(CS_NUM_2);
		ble_scan_device_info[current_device_num].device_status = 3;
	}
	else if(p_evt->evt_code == GAP_EVT_DISCOVER_LOOP)
	{		
		if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_LOOP)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service loop:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif
		}
		else if(p_evt->evt.discover_loop_evt.evt_code==DISCOVER_LOOP_SERVICE_FINISH)
		{
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
			DBGPRINTF(("discover service finish:%x\r\n",p_evt->evt.discover_loop_evt.uuid));
			#endif
			ble_scan_device_info[current_device_num].device_status = 4;
			ble_scan_cs_num.wait_for_connected = false;
		}
	}
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("slave disconnect %x\r\n",p_evt->evt.disconn_evt.reason));
		#endif
		ble_scan_device_info[current_device_num].device_status = 5;
		current_device_num = 0;
	}

}


void ble_attc_evt_callback_csnum2(struct attc_ble_evt *p_evt)
{
	if(p_evt->attc_code == ATT_READ_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("CS_NUM2, ATT_READ_RSP:\r\n",(void *)&p_evt->attc.AttReadRsp,p_evt->attc_sz);
		#endif
	}
	else if(p_evt->attc_code == ATT_HANDLE_VAL_NOTIFICATION)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("Slave notify:\r\n",p_evt->attc.AttHdlValNotification.buf,p_evt->attc_sz-2);
		#endif
	}
}


static void ble_init()
{
	struct gap_evt_callback gap_event;
	
	gap_c_ble_init();
	
	ble_scan_device_info[0].scan_name = "SYD8811";
	ble_scan_device_info[0].scan_addr = bd0_addr;
	ble_scan_device_info[0].device_status = 0;

	ble_scan_device_info[1].scan_name = "SYD8811";
	ble_scan_device_info[1].scan_addr = bd1_addr;
	ble_scan_device_info[1].device_status = 0;

	ble_scan_device_info[2].scan_name = "SYD8811";
	ble_scan_device_info[2].scan_addr = bd2_addr;
	ble_scan_device_info[2].device_status = 0;

	gap_event.evt_mask = 0;
	
	gap_event.p_callback = ble_evt_callback_csnum0;
	gap_c_evt_handler_set(CS_NUM_0, &gap_event);
	gap_s_att_mc_evt_handler_set(CS_NUM_0, &ble_attc_evt_callback_csnum0);
	
	gap_event.p_callback = ble_evt_callback_csnum1;
	gap_c_evt_handler_set(CS_NUM_1, &gap_event);
	gap_s_att_mc_evt_handler_set(CS_NUM_1, &ble_attc_evt_callback_csnum1);
	
	gap_event.p_callback = ble_evt_callback_csnum2;
	gap_c_evt_handler_set(CS_NUM_2, &gap_event);
	gap_s_att_mc_evt_handler_set(CS_NUM_2, &ble_attc_evt_callback_csnum2);
}

void gpio_init(void)
{
	uint8_t i;
   for(i=0;i<39;i++)
    {
        switch(i)
        {
            case GPIO_0:
                pad_mux_write(i, 0);
                gpi_config(i,PULL_DOWN);
                gpo_config(i,0);
            break;
			
			case LED2_Pin:
			case LED1_Pin:
                pad_mux_write(i, 0);
                gpo_config(i,1);
            break;
						
			case GPIO_2:
			case GPIO_4:
			#ifdef _SWDDEBUG_DISENABLE_
			case GPIO_31:
			#endif
                pad_mux_write(i, 0);
                gpo_config(i,0);
            break;
						
			case GPIO_24:
			case GPIO_30:
			#ifndef _SWDDEBUG_DISENABLE_
			case GPIO_31:
			#endif
			#ifdef CONFIG_UART_ENABLE
			case GPIO_20:
			case GPIO_21:
			#endif
            break;
						
            default:
                pad_mux_write(i, 0);
                gpi_config(i,PULL_UP);
        }
    }
}

int main()
{	
	__disable_irq();
	
	// Adjust IRQ priority  
    {
        int i;
        for (i = 0; i < 32; i++) {
            NVIC_SetPriority((IRQn_Type)i, 3);
        }
        NVIC_SetPriority(LLC2_IRQn, 0);  //主机的优先级最大
    }
	
	#if defined(CONFIG_UART_ENABLE)
	uart_0_init(UART_RTS_CTS_DISABLE, UART_BAUD_115200);
	#endif
	
	#ifdef _SYD_RTT_DEBUG_
	DebugLogInit();
	#endif
	
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
	dbg_printf("SYD8821 BLEuart %s:%s\r\n",__DATE__ ,__TIME__);
	dbg_printf("SYD-TEK.Inc\r\n");
	#endif
	
	ble_init();
	
	sys_mcu_clock_set(MCU_CLOCK_96_MHZ);
	// RC bumping
    sys_mcu_rc_calibration();
	
	#ifdef USER_32K_CLOCK_RCOSC
	sys_32k_clock_set(SYSTEM_32K_CLOCK_LPO);
	sys_32k_lpo_calibration();						//这是内部RC32k晶振的校准函数	经过该函数后定时器能够得到一个比较准确的值
	#else
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	#endif
	
	gpio_init();								 //gpio初始化
	
	__enable_irq();	
	
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
	DBGPRINTF(("gap_s_scan_start\r\n"));
	#endif
	
	while(1)
	{				
		ble_sched_execute();
		
		/* 查询空闲的通道，一次只打开一个通道 */
		for(uint8_t i = 0; i < CS_NUM; i++)
		{
			if( (ble_scan_cs_num.idle_num & (0x01 << i)) == 0)
			{
				if(ble_scan_cs_num.wait_for_connected == false)
				{
					/* 打开通道 */
					setup_scan_data(i);
					gap_c_scan_start(i);
					
					ble_scan_cs_num.idle_num |= (0x01 << i);
					ble_scan_cs_num.wait_for_connected = true;
					DBGPRINTF(("\r\nOpen cs_num %d, idle_num = 0x%02x\r\n", i, ble_scan_cs_num.idle_num ));
					break;
				}
			}	
		}
			
		for(uint8_t i = 0; i < CS_NUM; i++)
		{
			if(ble_scan_device_info[i].device_status == 1)
			{
				setup_connect_data(ble_scan_device_info[i].device_cs_num, ble_scan_device_info[i].scan_addr);
				gap_c_connect_start(ble_scan_device_info[i].device_cs_num);
				ble_scan_device_info[i].device_status = 2;
				DBGPRINTF(("Connect to slave, cs_num = %d.\r\n", ble_scan_device_info[i].device_cs_num));
			}
			
			/* 标志断开连接的通道等待下次开启扫描 */
			if(ble_scan_device_info[i].device_status == 5)
			{
				gap_c_scan_stop(ble_scan_device_info[i].device_cs_num);
				ble_scan_cs_num.idle_num &= ~(0x01 << ble_scan_device_info[i].device_cs_num);
				ble_scan_device_info[i].device_status = 0;
				ble_scan_cs_num.wait_for_connected = 0;
				DBGPRINTF(("cs_num = %d disconnect, idle_num = 0x%02x.\r\n", ble_scan_device_info[i].device_cs_num,
																			 ble_scan_cs_num.idle_num));
			}
		}
		
		if(!gpi_get_val(KEY1)){   //读蓝牙名称
			delay_ms(30);
			if(!gpi_get_val(KEY1))
			{	
				struct att_read_req cmd;
				
				DBGPRINTF(("Key1\r\n"));
				
				cmd.hdl = 0x03;
				att_mc_readreq(CS_NUM_0, &cmd);
				att_mc_readreq(CS_NUM_1, &cmd);
				att_mc_readreq(CS_NUM_2, &cmd);
				DBGPRINTF(("att_mc_readreq:%x\r\n",cmd.hdl));
				delay_ms(1000);
			}
			while(!gpi_get_val(KEY1));
		}
		
		if(!gpi_get_val(KEY2)){   //使能notified
			delay_ms(30);
			if(!gpi_get_val(KEY2))
			{
				struct att_write_req cmd;
				uint8_t idx;
				
				DBGPRINTF(("Key2\r\n"));
				
				cmd.hdl=0x1f;
				idx=0;
				cmd.buf[idx++]=0X01;
				cmd.buf[idx++]=0X00;
				att_mc_writereq(CS_NUM_0,&cmd,idx);
				att_mc_writereq(CS_NUM_1,&cmd,idx);
				att_mc_writereq(CS_NUM_2,&cmd,idx);
				DBGPRINTF(("att_mc_writereq to enable notify:%x\r\n",cmd.hdl));
				delay_ms(1000);
			}
			while(!gpi_get_val(KEY2));		
		}
		if(!gpi_get_val(KEY3)){   //写数据
			delay_ms(30);
			if(!gpi_get_val(KEY3))
			{
				struct att_write_req cmd;
				uint8_t idx;
				
				DBGPRINTF(("Key3\r\n"));
				
				cmd.hdl=0x1c;
				idx=0;
				cmd.buf[idx++]=0Xa1;
				cmd.buf[idx++]=0X01;
				cmd.buf[idx++]=0X01;
				cmd.buf[idx++]=0X01;
				att_mc_writereq(CS_NUM_0,&cmd,idx);
				att_mc_writereq(CS_NUM_1,&cmd,idx);
				att_mc_writereq(CS_NUM_2,&cmd,idx);
				DBGPRINTF(("att_mc_writereq:%x\r\n",cmd.hdl));
				delay_ms(1000);
			}
			while(!gpi_get_val(KEY3));	
		}
	}	
}

