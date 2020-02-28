#include "ARMCM0.h"
#include "gpio.h"
#include "debug.h"
#include "delay.h"
#include "ota.h"
#include "config.h"
#include "ble_slave.h"
#include "SYD_ble_service_Profile.h"
#include "queue.h"
#include "uart.h"
#include <string.h>
#include "timer.h"
#include "pad_mux_ctrl.h"
#include "Debuglog.h"
#include "led_key.h"
#include "debug.h"
#include "i2c_master.h"
#include "oled_9632.h"

struct gap_evt_callback GAP_Event;
static struct gap_att_report_handle *g_report;

//notify 标志 1代表已经使能 0代表未使能
static uint8_t start_tx=0;
static uint8_t wechat_tx=0;

//connection
uint8_t  connect_flag=0;
uint8_t update_latency_mode=0;
uint8_t latency_state=0;

uint8_t BLE_SendData(uint8_t *buf, uint8_t len);

uint8_t timer_wait_cnt=0;

uint8_t uart_data_temp[20];
/*
BIT7:1:被占用 0：空闲
*/
uint8_t uart_data_temp_state=0;

uint8_t ADV_DATA[] = {
						0x03,// length
						0x19,
						0x00,
						0x00,
						0x02,// length
						0x01,// AD Type: Flags
						0x05,// LE Limited Discoverable Mode & BR/EDR Not Supported
						0x03,// length
						0x03,// AD Type: Complete list of 16-bit UUIDs 
						0x01,// UUID: Human Interface Device (0x0001)//12
						0x00,// UUID: Human Interface Device (0x0001)//18
						0X09,// length
						0XFF,// AD Type: MANUFACTURER SPECIFIC DATA
						0X00,// Company Identifier (0x00)
						0X00,// Company Identifier (0x00)
						0X00,
						0X00,
						0X00,
						0X00,
						0X00,
						0X00,
						0x09,// length
						0x09,// AD Type: Complete local name
						'S',
						'Y',
						'D',
						'_',
						'U',
						'A',
						'R',
						'T',
					  };

uint16_t ADV_DATA_SZ = sizeof(ADV_DATA); 
uint8_t SCAN_DATA[]={0x00};
uint16_t SCAN_DATA_SZ = 0; 

static void ble_init(void);

static void setup_adv_data()
{
	struct gap_adv_params adv_params;	
	static struct gap_ble_addr dev_addr;
	
	adv_params.type = ADV_IND;
	adv_params.channel = 0x07;    // advertising channel : 37 & 38 & 39
	adv_params.interval = 0x640;  // advertising interval : unit 0.625ms)
	adv_params.timeout = 0x3FFF;    // timeout : uint seconds
	adv_params.hop_interval = 0x03;  //0x1c 
	adv_params.policy = 0x00;   

	gap_s_adv_parameters_set(&adv_params);

	/*get bluetooth address */
	gap_s_ble_address_get(&dev_addr);
	ADV_DATA[15] = dev_addr.addr[0];
	ADV_DATA[16] = dev_addr.addr[1];
	ADV_DATA[17] = dev_addr.addr[2];
	ADV_DATA[18] = dev_addr.addr[3];
	ADV_DATA[19] = dev_addr.addr[4];
	ADV_DATA[20] = dev_addr.addr[5];
	
//	DBGPRINTF(("addr type %02x \r\n",dev_addr.type));
//	DBGHEXDUMP("addr", dev_addr.addr, 6);

	gap_s_adv_data_set(ADV_DATA, ADV_DATA_SZ, SCAN_DATA, SCAN_DATA_SZ); 
}

/*
uint8_t target
0:fast
1:slow
*/
void BLSetConnectionUpdate(uint8_t target){
	struct gap_link_params  link_app;
	struct gap_smart_update_params smart_params;
	uint8_t buffer_cha1[5]={0XFC,0X01,0X00,0X00,0X00};
	gap_s_link_parameters_get(&link_app);
	#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
	dbg_printf("interval:%x latency:%x\r\n",link_app.interval,link_app.latency);
	#endif
	switch(target){
		case 0: 
				if((link_app.latency !=0) && (link_app.interval >0x10)){
					latency_state=0;
					/* connection parameters */
						smart_params.updateitv_target=0x0010;  //target connection interval (60 * 1.25ms = 75 ms)
						smart_params.updatesvto=0x00c8;  //supervisory timeout (400 * 10 ms = 4s)
					smart_params.updatelatency=0x0000;
					smart_params.updatectrl=SMART_CONTROL_LATENCY | SMART_CONTROL_UPDATE;
					smart_params.updateadj_num=MAX_UPDATE_ADJ_NUM;
					gap_s_smart_update_latency(&smart_params);
				}
			  #if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("SetUpdate ota link\r\n"));
			  #endif
				BLE_SendData(buffer_cha1,5);
		break;
		case 1:
				if((link_app.latency <0x000A) && (link_app.interval <0x0050)){
					/* connection parameters */
					smart_params.updateitv_target=0x0050;
					smart_params.updatelatency=0x000A;
					smart_params.updatesvto=0x0258;
					smart_params.updatectrl=SMART_CONTROL_LATENCY | SMART_CONTROL_UPDATE;
					smart_params.updateadj_num=MAX_UPDATE_ADJ_NUM;
					gap_s_smart_update_latency(&smart_params);	   
			#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
					DBGPRINTF(("SetUpdate ios link\r\n"));
			#endif
				}
		break;
	}
}
static void ble_gatt_read(struct gap_att_read_evt evt)
{
	if(evt.uuid == BLE_DEVICE_NAME_UUID)
	{
		uint8_t gatt_buf[]={'U', 'A', 'R', 'T','F'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_APPEARANCE_UUID)
	{
		uint8_t gatt_buf[]={0xff, 0xff};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_MANUFACTURER_NAME_STRING_UUID)
	{
		uint8_t gatt_buf[]={'S','Y','D',' ','I', 'n', 'c', '.'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_MODEL_NUMBER_STRING_UUID)
	{
		uint8_t gatt_buf[]={'B', 'L', 'E', ' ', '1', '.', '0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_SERIAL_NUMBER_STRING_UUID)
	{
		uint8_t gatt_buf[]={'1','.','0','.','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_HARDWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'2','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_FIRMWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'3','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_SOFTWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'4','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_PNP_ID_UUID)
	{
		uint8_t gatt_buf[]={ 0x02, 						//		Vendor ID Source
						    0x3a,0x09,					//		USB Vendor ID
						    0x05,0x0a,					//		Product ID
						    0x02,0x00					//		Product Version
												 };
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 

		gap_s_gatt_read_rsp_set(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_WECHAT_Read_UUID)
	{
		uint8_t buf[]={0x00,0x01,0x03,0x04};
		
		BLE_SendData(buf, 4);
	}
	#ifdef _OTA_
	else if(evt.uuid == BLE_OTA_Read_Write_UUID)
	{
		uint8_t sz=0;

		uint8_t rsp[sizeof(struct hci_evt)]={0};

		ota_rsp(rsp, &sz);
		
		gap_s_gatt_read_rsp_set(sz, rsp);
	}
	#endif
	else if(evt.uuid == BLE_Battry_Level_UUID)
	{
		//gap_s_gatt_read_rsp_set(0x01, 0X64);
	}
}

//接收函数
static void ble_gatt_write(struct gap_att_write_evt evt)
{
	// rx data
	//evt.data是收取到的数据buf
	if(evt.uuid== BLE_UART_Write_UUID)
	{
		enqueue_all(&rx_queue[BLE_TOUART_QUEUE_ID], evt.data,evt.sz);
	}
	#ifdef _OTA_
	else if(evt.uuid== BLE_OTA_Read_Write_UUID)
	{
		update_latency_mode=0;
		ota_cmd(evt.data, evt.sz);
	}
	#endif
}
//发送函数
//发送函数
uint8_t BLE_SendData(uint8_t *buf, uint8_t len)
{
	struct gap_att_report report;
	
	if(start_tx == 1)
	{
		report.primary = BLE_UART;
		report.uuid = BLE_UART_NOTIFY_UUID;
		report.hdl = BLE_UART_NOTIFY_VALUE_HANDLE;					
		report.value = BLE_GATT_NOTIFICATION;
		return gap_s_gatt_data_send(BLE_GATT_NOTIFICATION, &report, len, buf);
	}
	return 0;
}

void ble_evt_callback(struct gap_ble_evt *p_evt)
{
	if(p_evt->evt_code == GAP_EVT_ADV_END)
	{		
		gap_s_adv_start_powersaving();
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
		DBGPRINTF(("GAP_EVT_ADV_END\r\n"));
		#endif
	}
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)	 //连接事件
	{
		start_tx = 0;
		connect_flag=1;								 //连接状态
		latency_state=0;
		update_latency_mode=0;
		ota_state =0;
		
		update_latency_mode=0;
		
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_) 
		DBGHEXDUMP("GAP_EVT_CONNECTED addr:",p_evt->evt.bond_dev_evt.addr,sizeof(p_evt->evt.bond_dev_evt.addr));
		#endif
		
		//BLSetConnectionUpdate(1);
	}
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED) //断连事件
	{	
		connect_flag=0;								 //连接状态
		ota_state =0;
		start_tx = 0;
		#if defined(CONFIG_UART_ENABLE)
			DBGPRINTF(("GAP_EVT_DISCONNECTED(%02x)\r\n",p_evt->evt.disconn_evt.reason));
		#elif defined(_SYD_RTT_DEBUG_)
			DBGPRINTF("GAP_EVT_DISCONNECTED(%02x)\r\n",p_evt->evt.disconn_evt.reason);
		#endif   
		gap_s_adv_start_powersaving();
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_HANDLE_CONFIGURE)
	{					
		if(p_evt->evt.att_handle_config_evt.uuid == BLE_UART)
		{
			if(p_evt->evt.att_handle_config_evt.hdl == (BLE_UART_NOTIFY_VALUE_HANDLE + 1))
			{			
				if(p_evt->evt.att_handle_config_evt.value == BLE_GATT_NOTIFICATION)
				{
					DBGPRINTF(("start_tx enable\r\n"));
					start_tx = 1;
					PMU_CTRL->UART_EN = 1;    //不允许RF sleep时关闭XO，休眠的时候因为32Mhz晶振还在，所以功耗很高
				}
				else
				{		
					DBGPRINTF(("start_tx disable\r\n"));
					start_tx = 0;
					PMU_CTRL->UART_EN = 0;   //允许硬件自由控制32Mhz晶振，休眠的时候功耗很低
				}
			}
		}	
		else if(p_evt->evt.att_handle_config_evt.uuid == BLE_WECHAT)
		{
			if(p_evt->evt.att_handle_config_evt.hdl == (BLE_WECHAT_Indication_VALUE_HANDLE + 1))
			{
				if(p_evt->evt.att_handle_config_evt.value == BLE_GATT_NOTIFICATION)
					wechat_tx = 1;
				else
					wechat_tx = 0;
			}
		}		

		//DBGPRINTF(("GAP_EVT_ATT_HANDLE_CONFIGURE uuid:(%02x)\r\n",p_evt->evt.att_handle_config_evt.uuid));
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_WRITE)
	{
		ble_gatt_write(p_evt->evt.att_write_evt);
		//DBGPRINTF(("GAP_EVT_ATT_WRITE uuid:(%02x)\r\n",p_evt->evt.att_write_evt.uuid));
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_READ)
	{
		ble_gatt_read(p_evt->evt.att_read_evt);
    //DBGPRINTF(("GAP_EVT_ATT_READ uuid:(%02x)\r\n",p_evt->evt.att_write_evt.uuid));
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_HANDLE_CONFIRMATION)
	{
		//DBGPRINTF(("GAP_EVT_ATT_HANDLE_CONFIRMATION uuid:(%02x)\r\n",p_evt->evt.att_handle_config_evt.uuid));
	}
	else if(p_evt->evt_code == GAP_EVT_ENC_KEY)
	{
		update_latency_mode=1;  
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_) 
			DBGPRINTF(("GAP_EVT_ENC_KEY\r\n"));
		#endif
	}
	else if(p_evt->evt_code == GAP_EVT_ENC_START)
	{
		#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_) 
			DBGPRINTF(("GAP_EVT_ENC_START\r\n"));
		#endif
	}
  else if(p_evt->evt_code == GAP_EVT_CONNECTION_UPDATE_RSP)
	{
		switch(p_evt->evt.connection_update_rsp_evt.result)
		{
			case CONN_PARAM_ACCEPTED:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_) 
				DBGPRINTF(("update rsp ACCEPTED\r\n"));
				#endif
				break;
			case CONN_PARAM_REJECTED:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("update rsp REJECTED\r\n"));
				#endif
				break;
			case CONN_PARAM_SMART_TIMEROUT:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("update rsp TIMEROUT\r\n"));
				#endif
				break;
			case CONN_PARAM_SMART_SUCCEED:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("update rsp SUCCEED\r\n"));
				#endif
				break;
			case CONN_PARAM_LATENCY_ENABLE:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("Enable latency\r\n"));
				#endif
				break;
			case CONN_PARAM_LATENCY_DISABLE:
				#if defined(CONFIG_UART_ENABLE) || defined(_SYD_RTT_DEBUG_)
				DBGPRINTF(("Disable latency\r\n"));
				#endif
				break;
		}
	}
}

static void ble_init()
{	
	struct gap_wakeup_config pw_cfg;
	struct gap_profile_struct gatt;
	struct gap_pairing_req sec_params;
	struct gap_connection_param_rsp_pdu connection_params;

	gap_s_ble_init();
	
	//set profile
	gatt.report_handle_address = (uint32_t)_gatt_database_report_handle;
	gatt.primary_address	= (uint32_t)_gatt_database_primary;
	gatt.include_address	= (uint32_t)_gatt_database_include;
	gatt.characteristic_address	= (uint32_t)_gatt_database_characteristic;
	gatt.value_address = (uint32_t)_gatt_database_value;
	gap_s_gatt_profiles_set(&gatt);

	//set device bond configure
	sec_params.io = IO_NO_INPUT_OUTPUT;
	sec_params.oob = OOB_AUTH_NOT_PRESENT;
	sec_params.flags = AUTHREQ_BONDING;
	sec_params.mitm = 0;
	sec_params.max_enc_sz = 16;
	sec_params.init_key = 0;
	sec_params.rsp_key = (GAP_KEY_MASTER_IDEN |GAP_KEY_ADDR_INFO);
	gap_s_security_parameters_set(&sec_params);
 
	//set ble connect params
	connection_params.Interval_Min = 6;
	connection_params.Interval_Max = 9;
	connection_params.Latency = 100;
	connection_params.Timeout = 100;
	connection_params.PeferredPeriodicity = 6;
	connection_params.ReferenceConnEventCount = 50;
	connection_params.Offset[0] = 0;
	connection_params.Offset[1] = 1;
	connection_params.Offset[2] = 2;
	connection_params.Offset[3] = 3;
	connection_params.Offset[4] = 4;
	connection_params.Offset[5] = 5;
	gap_s_connection_param_set(&connection_params);

	//set connect event callback 
	GAP_Event.evt_mask=(GAP_EVT_CONNECTION_EVENT);
	GAP_Event.p_callback=&ble_evt_callback;
  gap_s_evt_handler_set(&GAP_Event);

	gap_s_gatt_report_handle_get(&g_report);

	bm_s_bond_manager_idx_set(0);
	
	setup_adv_data();

	//set MCU wakup source
	pw_cfg.timer_wakeup_en = 1;
	pw_cfg.gpi_wakeup_en = 0;
	pw_cfg.gpi_wakeup_cfg = 0 ; 
	pw_cfg.gpi_wakeup_pol = 0; 
	pmu_wakeup_config(&pw_cfg);
}

void uart_to_ble_transfer(void){
	if(start_tx==1){
		if(!(is_queue_empty(&rx_queue[UART_TOBLE_QUEUE_ID])))
		{
			uint8_t i=0,j=0,num=queue_size(&rx_queue[UART_TOBLE_QUEUE_ID])/20+1,num_residue=queue_size(&rx_queue[UART_TOBLE_QUEUE_ID])%20;
			for(i=0;i<num;i++){
				
				if(!(uart_data_temp_state & BIT7))  //空闲
				{
					for(j=0;j<20;j++) dequeue(&rx_queue[UART_TOBLE_QUEUE_ID], &uart_data_temp[j]);
				}
				if(i>=(num-1))
				{ 
					uart_data_temp_state |=BIT7;
					if(BLE_SendData(uart_data_temp,num_residue)){
						num=0;
						uart_data_temp_state &=~BIT7;
					}
					else break;
				}
				else
				{
					uart_data_temp_state |=BIT7;
					if(BLE_SendData(uart_data_temp,20)) 
					{
						uart_data_temp_state &=~BIT7;
					}
					else break;
				}
			}
		}
	}
}

void ble_to_uart_transfer(void){
	uint8_t buf=0;
	while(dequeue(&rx_queue[BLE_TOUART_QUEUE_ID], &buf))
	{
		uart_write(0, &buf,1);
	}
}

void timer_uart_wait(void)
{
	timer_wait_cnt++;
	if(timer_wait_cnt>=MAX_UART_WAIT)
	{
		timer_wait_cnt=0;
		timer_disable(UART_WAIT_TIMER_ID);
		
		if (!is_queue_empty(&rx_queue[UART_TOBLE_QUEUE_ID]))
		{
			uart_to_ble_transfer();
		}
	}
}

void Timer_Module_Init(void)
{
	timer_disable(TIMER_1); 
	timer_enable(TIMER_1, timer_uart_wait, 32768/100, 1);//32768 = 1S  16384 = 500ms
    NVIC_EnableIRQ(TIMER1_IRQn);
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
            break;
						
            default:
                pad_mux_write(i, 0);
                gpi_config(i,PULL_UP);
        }
    }
}

void ble_uart_init(uint8_t flowctrl, uint8_t baud)
{
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	if(flowctrl)
	{
		pad_mux_write(18, 7);
		pad_mux_write(19, 7);
	}
	
	NVIC_DisableIRQ(UART0_IRQn);
	UART_CTRL[0]->BAUD_SEL = baud;
	UART_CTRL[0]->FLOWCTRL_EN = flowctrl;
	UART_CTRL[0]->INT_RX_MASK = 0;
	UART_CTRL[0]->UART_ENABLE = 1;
    
	queue_init(&rx_queue[0], rx_buf[0], QUEUE_SIZE);
    
	*(uint32_t *)0x20028024 |=U32BIT(UART0_IRQn);
	*(uint32_t *)0x20028020 |=U32BIT(UART0_IRQn);
	NVIC_EnableIRQ(UART0_IRQn);
}

int main()
{	
	uint8_t verdor_datas[16]={0};
	__disable_irq();
	gap_s_ble_init();

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
	pad_mux_write(20, 7);
	pad_mux_write(21, 7);
	dbg_init();
	dbg_printf("SYD8821 IIC DEMO\r\n");

	// IIC
	pad_mux_write(8, 6); //i2c 0 scl
	pad_mux_write(9, 6); //i2c 0 sda

	/* Masters Init M0*/
	//set i2c clk rate param 0, (48/8)*50K=300k 
	i2c_master_set_speed(1, 0);
	i2c_master_enable(1, true);
	i2c_master_enable_int(1, true);
	i2c_master_set_address_mode(1, I2C_MASTER_1BYTE_ADDRESS);
	NVIC_EnableIRQ(I2CM0_IRQn);
	
	oled_init();
	
	oled_printf(0,0,"SYD Inc."); 
	oled_printf(0,2,"SYD8821 EVB"); 
	#ifdef VERDOR_DATAS
	gap_s_verdor_datas_get(verdor_datas);
	oled_printf(0,4,"%02x%02x%02x%02x%02x%02x%02x%02x",verdor_datas[15],verdor_datas[14],verdor_datas[13],verdor_datas[12],verdor_datas[11],verdor_datas[10],verdor_datas[9],verdor_datas[8]); 
	oled_printf(0,6,"%02x%02x%02x%02x%02x%02x%02x%02x",verdor_datas[7],verdor_datas[6],verdor_datas[5],verdor_datas[4],verdor_datas[3],verdor_datas[2],verdor_datas[1],verdor_datas[0]); 
	verdor_datas[15]++;
	gap_s_verdor_datas_set(verdor_datas);   //每次复位加1
	#else
	oled_printf(0,4,"20200228"); 
	#endif
	__enable_irq();
	while(1)
	{
		gpo_toggle(LED4);

		delay_ms(100);
	}		
}

