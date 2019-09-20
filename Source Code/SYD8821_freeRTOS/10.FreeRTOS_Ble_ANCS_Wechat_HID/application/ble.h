#ifndef _BLE_H_
#define _BLE_H_

#include "ARMCM0.h"



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

struct queue_att_write_data {
	uint16_t uuid;
	uint8_t	sz;
	uint8_t	data[20];
};

extern uint32_t current_step;	
extern uint32_t current_distance;   
extern uint32_t current_calorie;
extern uint32_t target_step;


extern uint8_t per_device_system;
extern uint8_t  connect_flag;



extern void ble_init(void);
extern uint8_t BLE_SendData(uint8_t *buf, uint8_t len);
extern void BLSetConnectionUpdate(uint8_t target);
extern uint8_t Wechat_SendData(uint16_t uuid, uint8_t *buf, uint8_t len);
extern uint8_t HID_Key_Vlaue_Send(uint8_t *buf, uint8_t len);
#endif

