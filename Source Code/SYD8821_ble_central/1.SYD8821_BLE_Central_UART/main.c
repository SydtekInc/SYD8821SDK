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

#define CS_NUM_INDEX 0
uint8_t bd_addr[BD_ADDR_SZ] = {0x00};
struct gap_evt_callback GAP_Event_CsNum0;
/*
0:正在扫描 
1：扫描到设备，准备连接
2：正在连接
3:连接成功  将要发现服务
4：发现服务成功
5：蓝牙从机断线
*/
uint8_t  device_state=0;

static void setup_scan_data()
{
	struct gap_scan_params_central scan_params;	
	scan_params.type = ACTIVE_SCAN;  //主动扫描
	scan_params.interval= 64;  // 扫描间隔
	scan_params.window = 128;    // 窗口
	scan_params.white_list_en = 0x00;  //不使能百名单 
	scan_params.cs_num = CS_NUM_INDEX;    //在0位置扫描
	scan_params.address_type=PUBLIC_ADDRESS_TYPE;

	gap_c_scan_parameters_set(&scan_params);
}

static void setup_connect_data(uint8_t *addr)
{
	struct INITIATOR_CONFIG_TYPE connect_config= {
		32, //.scn_itv
		32, //.scn_window
		0, //.filter_policy
		PUBLIC_ADDRESS_TYPE, //.address_type
		NULL, //.addr
		PUBLIC_ADDRESS_TYPE, //.own_addr_type
		12, //.conn_itv
		600, //.conn_svto   200->200ms
		0, //.conn_latency
		24, //.conn_evt_length
	};
	connect_config.addr = bd_addr;
	gap_c_add_white_list(addr, WL_PUBLIC);
    gap_c_connect_connect_config_set(CS_NUM_INDEX,&connect_config);
}

void ble_evt_callback_csnum0(struct gap_ble_evt *p_evt)
{
	//DBGPRINTF(("evt_callback0:%x\r\n",p_evt->evt_code));
	if(p_evt->evt_code == GAP_EVT_ADV_REPORT)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ScanRx type:%x rssi:%x len:%x\r\n",p_evt->evt.advertising_report_evt.type, p_evt->evt.advertising_report_evt.rssi,p_evt->evt.advertising_report_evt.len));
		DBGHEXDUMP("addr:",(void *)&p_evt->evt.advertising_report_evt.peer_dev_addr,sizeof(struct gap_ble_addr));
		//DBGHEXDUMP("buf:",(void *)p_evt->evt.advertising_report_evt.buf,p_evt->evt.advertising_report_evt.len);
		#endif
		p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len]='\0';
		if (strstr((const char *)&p_evt->evt.advertising_report_evt.buf[p_evt->evt.advertising_report_evt.len-8],"SYD_UART") != NULL)
		{
			if(memcmp(p_evt->evt.advertising_report_evt.peer_dev_addr.addr,bd_addr,BD_ADDR_SZ)==0)  //地址全为0
			{
				DBGPRINTF(("find device then stop scan\r\n"));
				gap_c_scan_stop(CS_NUM_INDEX);
				device_state=1;
			}
		}
	}
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("have Connect to slave then discover sevice\r\n"));
		#endif
		gap_c_discover_sevice_start(CS_NUM_INDEX);
		device_state=3;
	}
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED)
	{		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("slave disconnect %x\r\n",p_evt->evt.disconn_evt.reason));
		#endif
		device_state=5;
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
			device_state=4;
		}
	}
}
void ble_attc_evt_callback_csnum0(struct attc_ble_evt *p_evt)
{
	//DBGPRINTF(("att_response:%x\r\n",p_evt->attc_code));
	if(p_evt->attc_code == ATT_ERR_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_ERR_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_MTU_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_MTU_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_FIND_INFO_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_FIND_INFO_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_FIND_BY_TYPE_VALUE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_FIND_BY_TYPE_VALUE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_READ_BY_TYPE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_READ_BY_TYPE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_READ_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("ATT_READ_RSP:\r\n",(void *)&p_evt->attc.AttReadRsp,p_evt->attc_sz);
		#endif
	}
	else if(p_evt->attc_code == ATT_READ_BLOB_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_READ_BLOB_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_READ_MULTIPLE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_READ_MULTIPLE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_READ_BY_GROUP_TYPE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_READ_BY_GROUP_TYPE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_WRITE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_WRITE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_PREPARE_WRITE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_PREPARE_WRITE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_EXECUTE_WRITE_RSP)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_EXECUTE_WRITE_RSP\r\n"));
		#endif
	}
	else if(p_evt->attc_code == ATT_HANDLE_VAL_NOTIFICATION)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGHEXDUMP("Slave notify:\r\n",p_evt->attc.AttHdlValNotification.buf,p_evt->attc_sz-2);
		#endif
	}
	else if(p_evt->attc_code == ATT_HANDLE_VAL_INDICATION)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("ATT_HANDLE_VAL_INDICATION\r\n"));
		#endif
	}
}


static void ble_init()
{	
	gap_c_ble_init();

	//set connect event callback 
	GAP_Event_CsNum0.evt_mask=0;
	GAP_Event_CsNum0.p_callback=&ble_evt_callback_csnum0;
    gap_c_evt_handler_set(CS_NUM_INDEX,&GAP_Event_CsNum0);
	
	 gap_s_att_mc_evt_handler_set(CS_NUM_INDEX,&ble_attc_evt_callback_csnum0);
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
	uint8_t idx=0;
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
	
	device_state=0;
	setup_scan_data();
	gap_c_scan_start(CS_NUM_INDEX);
	__enable_irq();	
	setup_connect_data(bd_addr);
	gap_c_connect_start(CS_NUM_INDEX);
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
	DBGPRINTF(("gap_s_scan_start\r\n"));
	#endif
	
	while(1)
	{				
		ble_sched_execute(); 
		if(device_state==1)
		{
			DBGPRINTF(("connect to slave\r\n"));
			setup_connect_data(bd_addr);
			gap_c_connect_start(CS_NUM_INDEX);
			device_state=2;
		}
		else if(device_state==5)
		{
			setup_scan_data();
			gap_c_scan_start(CS_NUM_INDEX);
			DBGPRINTF(("scan restart\r\n"));
			device_state=0;
		}
		
		if(!gpi_get_val(KEY1)){   //读蓝牙名称
			struct att_read_req cmd;
			cmd.hdl=0x03;
			att_mc_readreq(CS_NUM_INDEX,&cmd);
			DBGPRINTF(("att_mc_readreq:%x\r\n",cmd.hdl));
		   delay_ms(1000);
		}
		if(!gpi_get_val(KEY2)){   //使能notified
			struct att_write_req cmd;
			cmd.hdl=0x20;
			idx=0;
			cmd.buf[idx++]=0X01;
			cmd.buf[idx++]=0X00;
			att_mc_writereq(CS_NUM_INDEX,&cmd,idx);
			DBGPRINTF(("att_mc_writereq to enable notify:%x\r\n",cmd.hdl));
		    delay_ms(1000);
		}
		if(!gpi_get_val(KEY3)){   //写数据
			struct att_write_req cmd;
			cmd.hdl=0x1d;
			idx=0;
			cmd.buf[idx++]=0Xa1;
			cmd.buf[idx++]=0X01;
			cmd.buf[idx++]=0X01;
			cmd.buf[idx++]=0X01;
			att_mc_writereq(CS_NUM_INDEX,&cmd,idx);
			DBGPRINTF(("att_mc_writereq:%x\r\n",cmd.hdl));
		    delay_ms(1000);
		}
		
		//PMU_CTRL->UART_EN = 0;
		//SystemSleep(POWER_SAVING_RC_OFF, FLASH_LDO_MODULE, 11000 , (PMU_WAKEUP_CONFIG_TYPE)(FSM_SLEEP_EN|TIMER_WAKE_EN|RTC_WAKE_EN));
	}	
}

