#ifndef __DEF_H
#define __DEF_H

/*
0:正在扫描 
1：扫描到设备，准备连接
2：正在连接
3: 连接成功  将要发现服务
4：发现服务成功
5：蓝牙从机断线
*/
struct ble_scan_device_info_t
{
	uint8_t *scan_name;			// 扫描的设备名
	uint8_t *scan_addr;			// 扫描的设备地址
	uint8_t device_cs_num;		// 连接设备使用的通道
	uint8_t device_status;		// 设备状态
};


struct ble_scan_cs_num_t
{
	uint8_t idle_num;			// 一位代表一个通道状态，0:空闲 1:扫描
	bool wait_for_connected;	// 等待连接
};

typedef enum
{
	CS_NUM_0 = 0,
	CS_NUM_1 = 1,
	CS_NUM_2 = 2,
	CS_NUM_3 = 3,
	CS_NUM_4 = 4,
	CS_NUM_5 = 5,
	CS_NUM_6 = 6,
	CS_NUM_7 = 7,
}cs_num_t;


#endif /* __DEF_H */
