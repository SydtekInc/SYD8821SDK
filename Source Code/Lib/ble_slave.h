#ifndef _BLE_LIB_H_
#define _BLE_LIB_H_

#include "ARMCM0.h"

#pragma pack(1)

/*************************************************************************************************
*�궨�壺BD_ADDR_SZ
*˵���� ������ַ��С�����ݹ淶���ú�Ϊ6��byte
**************************************************************************************************/
#define BD_ADDR_SZ 			6



/*************************************************************************************************
*�궨�壺MAX_EDIV_SZ
*˵���� ����EDIV������С�����ݹ淶���ú�Ϊ2��byte
**************************************************************************************************/
#define MAX_EDIV_SZ			2



/*************************************************************************************************
*�궨�壺MAX_RAND_SZ
*˵���� ����RAND������С�����ݹ淶���ú�Ϊ8��byte
**************************************************************************************************/
#define MAX_RAND_SZ		8



/*************************************************************************************************
*�궨�壺MAX_KEY_SZ
*˵���� ����LTK�ܳ׵Ĵ�С�����ݹ淶���ú�Ϊ16��byte��128bit
**************************************************************************************************/
#define MAX_KEY_SZ			16



/*************************************************************************************************
*�궨�壺MAX_IRK_SZ
*˵���� ����IRK�����Ĵ�С�����ݹ淶���ú�Ϊ16��byte��128bit
**************************************************************************************************/
#define MAX_IRK_SZ			16



/*************************************************************************************************
*�궨�壺LL_WIN_OFFSET_SZ
*˵���� ����BLE��LL_CONNECTION_PARAM_RSP��Ӧ��OFFSET�Ĵ�С�����ݹ淶��һ����6��OFFSET,��Ϊ�ú�����
LL_CONNECTION_PARAM_RSP�ṹ���е�Offset�����С���������ﶨ��Ϊ����6
**************************************************************************************************/
#define LL_WIN_OFFSET_SZ	6




/*************************************************************************************************
*�궨�壺MAX_ATT_DATA_SZ
*˵���� ����BLE��ATT���MTUֵ��Ҳ��������ATT���ݴ�С������4.2�淶���壬��ֵΪ512
**************************************************************************************************/
#define MAX_ATT_DATA_SZ	512



/*************************************************************************************************
*�궨�壺MAX_ADV_DATA_SZ
*˵���� ����BLE�й㲥ͨ���������������ֵ��Ҳ��������ATT���ݴ�С������4�淶���壬��ֵΪ31
**************************************************************************************************/
#define MAX_ADV_DATA_SZ	31



/*************************************************************************************************
*�궨�壺MAX_ATT_REPORT_HDL
*˵���� SYD8821����notify����indicateͨ����Ŀ��Ҳ��gap_att_report_handle�ṹ���е�gap_att_report
��Ա��������
**************************************************************************************************/
#define MAX_ATT_REPORT_HDL 20



/*************************************************************************************************
*�궨�壺MAX_UPDATE_ADJ_NUM
*˵���� SYD8821�������Ӳ����ĵ����������ú�����smart params���������Ӳ���������smart params����
������Է��ܾ���SYD8821��������Ӳ�������ôSYD8821���ᷢ�͸����öԷ����׽��ܵ����Ӳ����������������
����MAX_UPDATE_ADJ_NUM��ô���������Ӳ����ĵ�������
**************************************************************************************************/
#define MAX_UPDATE_ADJ_NUM		4




/*************************************************************************************************
*ö������_BLE_ADDRESS_TYPE_
*��Ա�� PUBLIC_ADDRESS_TYPE	������ַ����
				RANDOM_ADDRESS_TYPE	�����ַ����
*˵���� BLE��MAC��ַ�����й�����ַ�������ַ���֣�ǰ���������̺�SIG�������Ψһ�Եĵ�ַ�����������
SIG���룬RANDOM_ADDRESS_TYPE�ַ�ΪStatic Device Address��Private Device Address
**************************************************************************************************/
enum _BLE_ADDRESS_TYPE_{
	PUBLIC_ADDRESS_TYPE	= 0x00,
	RANDOM_ADDRESS_TYPE 	= 0x01,
};




/*************************************************************************************************
*ö������_ADV_CH_PKT_TYPE_
*��Ա�� ADV_IND	connectable undirected advertising event����������ӹ㲥�¼�
				ADV_DIRECT_IND	connectable directed advertising event��������ӹ㲥�¼�
				ADV_NOCONN_IND	non-connectable undirected advertising event�������ӹ㲥�¼�
				SCAN_REQ	ɨ�������¼�
				SCAN_RSP	ɨ����Ӧ�¼�
				CONNECT_REQ	�㲥ͨ�������¼�
				ADV_SCAN_IND	scannable undirected advertising event��ɨ��δ����㲥�¼�������BLE4.2���ӵ�
*˵���� ��ö�ٶ����˿��ܷ�����BLE�Ĺ㲥ͨ���ϵ��¼���������¼���ȷ�е�˵��Ӧ���ǿ��ܷ�������Ϊ��
���ò�ͬ�����ͣ��Ͳ�ͻᷢ����Ӧ����Ϊ������״̬֮ǰ����Ϊ���Ƿ����ڹ㲥ͨ���ϵģ�ADV_IND�
ADV_DIRECT_IND��ADV_NOCONN_IND��ADV_SCAN_IND����BLE�Ĺ㲥�¼�����Ӧ����Ӧ�Ĺ㲥���ͣ�SCAN_REQ��
SCAN_RSP����BLE��ɨ����ص��¼���ǰ������ɨ���������Ϊɨ����Ӧ��CONNECT_REQΪ�����¼�������SCAN_REQ
��CONNECT_REQ�Ƕ���SYD8821��master���������˲��ܹ��е���Ϊ��
**************************************************************************************************/
enum _ADV_CH_PKT_TYPE_{
	ADV_IND 			= 0x00,
	ADV_DIRECT_IND 	= 0x01,
	ADV_NOCONN_IND	= 0x02,
	SCAN_REQ			= 0x03,
	SCAN_RSP			= 0x04,
	CONNECT_REQ		= 0x05,
	ADV_SCAN_IND		= 0x06,
};




/*************************************************************************************************
*ö������BLE_SEND_TYPE
*��Ա�� BLE_GATT_NOTIFICATION 	Characteristic Value Notification.BLE��Ϣ����Ϊnotify
				BLE_GATT_INDICATION		Characteristic Value Indications BLE��Ϣ����ΪIndications
*˵���� ����SYD8821�ӻ���Characteristic Value������Ϊ�����֣���Notification��Indications��ǰ����
SYD8821����������Ϣ��������Ҫ�ȴ�master�����κ���Ӧ�������߻���Ҫ�ȴ�master������ȷ����Ӧ�Ż����
��һ������
**************************************************************************************************/
enum BLE_SEND_TYPE {
	BLE_GATT_NOTIFICATION	= 0x0001,
	BLE_GATT_INDICATION		= 0x0002,
};



/*************************************************************************************************
*ö������_MCU_CLOCK_SEL_
*��Ա�� MCU_CLOCK_16_MHZ 	MCUʱ��Ϊ16Mhz
				MCU_CLOCK_20_MHZ	MCUʱ��Ϊ20Mhz
				MCU_CLOCK_24_MHZ	MCUʱ��Ϊ24Mhz
				MCU_CLOCK_64_MHZ	MCUʱ��Ϊ64Mhz
				MCU_CLOCK_80_MHZ	MCUʱ��Ϊ80Mhz
				MCU_CLOCK_96_MHZ	MCUʱ��Ϊ96Mhz
*˵���� SYD8821��MCU��ʱ�ӿ��ԴӸ�ö����ѡȡ�������ϵ��ʱ�����ѡ���ʵ���MCUʱ�ӣ����ҵ���MCU�ĸ�Ƶ
RC����У׼������sys_mcu_rc_calibration������MCU��ʱ�ӣ�������ԭ���ǣ�ʱ��Ƶ��Խ�ߣ���Ӧ�Ĺ���Ҳ��
��Ӧ�����󣨹��Ĳ������гɱ���������ֻ����΢��Щ���죩��һ���������ʱ��֮�������Ž���RC��У׼��
				SYD8821��ʱ����ƴ�Ƶ���������е�Ƶ��32.768KHz�ĵ�Ƶʱ�ӣ����ǹ̶���Ƶ�ʣ���ʱ�ӵ�ʱ��Դ
���ڲ�RX�������ⲿ���������Լ��ڲ�32MHz�ķ�Ƶ���֣���Ƶ��ʱ����Ҫ������Ƶ�Ķ�ʱ����PWMʹ�ã����˵�
Ƶʱ���⣬SYD8821оƬ�ڲ�����һ����MCU�͸��ٵ�ͨѶ�ӿڣ�����SPI��I2C�ȣ�ʹ�õĸ���ʱ��������������
�ԣ�����ʱ�ӵ�ʱ��Դһ������оƬ�ڲ��ĸ�Ƶ������
				SYD8821�ĸ���ʱ�ӵ�ʱ��Դֻ���ڲ��ĸ���RC���������������úú����Ҫ����У׼�����̣����Ƕ���
����ʱ�Ӷ��ԣ�ֻ��Ҫ�ڳ���������ʱ�����һ��У׼���ɣ�SYD8821�ĵ�Ƶʱ��Դ�����ڲ��ģ����LPO��Ҳ���ⲿ
�ģ����XO)�����ʹ���ڲ���ʱ��Դ����ô�����ÿ��һ��ʱ�䣨����3���ӣ�����һ���ڲ��ĵ�Ƶʱ�ӵ�У׼��
				���и�ƵRCʱ��У׼��ʱ������ȵ���gap_s_ble_init������ʼ��BLEЭ��ջ��
**************************************************************************************************/
enum _MCU_CLOCK_SEL_{
	MCU_CLOCK_16_MHZ	= 0x00,
	MCU_CLOCK_20_MHZ	= 0x01,
	MCU_CLOCK_24_MHZ	= 0x02,
	MCU_CLOCK_64_MHZ	= 0x03,
	MCU_CLOCK_80_MHZ	= 0x04,
	MCU_CLOCK_96_MHZ	= 0x05,
};




/*************************************************************************************************
*ö������_32K_CLOCK_SEL_
*��Ա�� SYSTEM_32K_CLOCK_LPO 	��Ƶ��32Kʱ��ԴΪ�ڲ�RC����
				SYSTEM_32K_CLOCK_LPO 	��Ƶ��32Kʱ��ԴΪ�ⲿ����
				SYSTEM_32K_CLOCK_32M_DIV	��Ƶ��32Kʱ��ԴΪ�ⲿ����ķ�Ƶ����Ƶָ��Ϊ977��Ƶ�����յ�Ƶ�ʽӽ�
																	32.768KHz��32000/977=32.7533Khz
*˵���� SYD8821�ĵ�Ƶʱ�ӵ�Ƶ�ʹ̶�Ϊ32.768KHz,��Ҫ�Ǹ���Ƶ�Ķ�ʱ����PWM�Ѿ�BLEЭ��ջʹ�ã���Ƶʱ��
Դ��׼ȷ�Ļ�����Ը�ʱ�ӵ������߶���Ӱ�죬����BLE���Ӳ��ϣ���ʱ����PWM��ʱ��׼ȷ�ȡ�
				���ѡ��SYSTEM_32K_CLOCK_LPOΪʱ��Դ�Ļ�������Ҫÿ��һ���̶���ʱ�䣨����3���ӣ�ҪУ׼һ�Σ�
��Ϊ���RC���������¶�Ư�Ƶ����ԣ�ʱ�䳤��ͻ���ƫ��
**************************************************************************************************/
enum _32K_CLOCK_SEL_{
	SYSTEM_32K_CLOCK_LPO	= 0x00,
	SYSTEM_32K_CLOCK_XO	= 0x01,
	SYSTEM_32K_CLOCK_32M_DIV	= 0x02,
};




/*************************************************************************************************
*�ṹ������gap_ble_addr
*��Ա�� type 	BLE mac��ַ���ͣ�Ϊö��_BLE_ADDRESS_TYPE_�ĳ�Ա
				addr 	BLE mac��ַ
*˵���� SYD8821���豸��ַ��ͨ��gap_s_ble_address_get��������ȡ��ͨ��gap_s_ble_address_set������
**************************************************************************************************/
struct gap_ble_addr {
	uint8_t	type;
	uint8_t	addr[BD_ADDR_SZ];
};




/*************************************************************************************************
*�ṹ������gap_key_params
*��Ա�� ediv 	BLE��ȫ�����е�Encrypted Diversifier (EDIV)����
				rand 	BLE��ȫ�����е�Random Number (Rand)����
				ltk 	BLE��ȫ�����е�Long Term Key (LTK) ����
				local_irk 	BLE��ȫ�����еı���Identity Resolving Key (IRK)����
				peer_irk 	BLE��ȫ�����еĶԵ��豸Identity Resolving Key (IRK)����
*˵���� �ڼ��ܽ����󽻻��ܳ׵�ʱ��SYD8821���ϱ�enc_key_evt�¼������¼��а����иýṹ�壬�����˰�ȫ
�������õ��ĸ�������
**************************************************************************************************/
struct gap_key_params{
	uint8_t	ediv[MAX_EDIV_SZ];
	uint8_t	rand[MAX_RAND_SZ];	
	uint8_t	ltk[MAX_KEY_SZ];
	uint8_t	local_irk[MAX_IRK_SZ];
	uint8_t	peer_irk[MAX_IRK_SZ];	
};




/*************************************************************************************************
*�ṹ������gap_adv_params
*��Ա�� type 	�㲥���ͣ�Ϊö��_ADV_CH_PKT_TYPE_�еĳ�Ա
				peer_addr	�Ե��豸��ַ���ó�Ա����Ϊ����㲥�е�InitA��ֻ�ڶ���㲥����
				policy	���˲��ԣ����Ϲ淶���ἰ��LINK LAYER DEVICE FILTERING  Policy
				channel	�㲥ͨ�����ò���ʹ��bitmask��ʽ��37~39ͨ����Ӧbit0~bit2,����0x03������37��38ͨ���㲥
				interval	�㲥�������λ��0.625ms,����0x640Ϊ1S�Ĺ㲥���
				timeout	�㲥��ʱʱ�䣬��λ��1S������0x64Ϊ100S�����ֵΪ0x3FFF�����ܹ�����0
				hop_interval	�㲥�¼���ͨ����ͨ��֮��ļ��������㲥ͨ��Ϊ37,38��39����ôhop_interval��Ϊ
											һ���㲥�¼��е�37��38����38��39��ʱ����
*˵���� SYD8821�Ĺ㲥�ɹ㲥�����͹㲥������ɣ��㲥�������ƾ������Ϊ���㲥����Ϊ�㲥�о�������ݡ�
				�㲥����ǻ��ڹ��ĺ��������������������ȶ��Ե�ƽ�⣬���ҲС����Խ�죬����ҲԽ�ȶ�������ҲԽ��
�㲥���Խ��������Խ�ͣ���Ȼ���ľ�Խ��
				����SYD8821��Ҫ��interval����2ms,hop_interval<interval
**************************************************************************************************/
struct gap_adv_params {
	uint8_t				type;
	struct gap_ble_addr	peer_addr;
	uint8_t       policy;
	uint8_t				channel;
	uint16_t      interval; 		 /**< Between 0x0020 and 0x4000 in 0.625 ms units (20ms to 10.24s) */
	uint16_t      timeout;	 		 /**< Between 0x0001 and 0x3FFF in seconds, 0x0000 is disable */
	uint8_t       hop_interval;  /** 30.5us ~ 85583us in 335.5us units **/
};




/*************************************************************************************************
*�ṹ������gap_scan_params
*��Ա�� type 	ɨ�����ͣ��б���ɨ�������ɨ��
				interval	ɨ��������λ��0.625ms,Time Range: 2.5 msec to 10.24 seconds
				window	ɨ�贰�ڣ���λ��0.625ms��Time Range: 2.5 msec to 10.24 seconds
*˵���� ɨ����Ҫ����ɨ�贰�ڣ���Ϊ����ɨ�跢����ɨ�贰���ڼ䣬�����п���һ�������ɨ�費���㲥��
�������SYD8821ɨ�赽����Ҫ��Ĺ㲥��ʱ��Э��ջ���ϱ�ɨ����Ӧ�¼���
**************************************************************************************************/
struct gap_scan_params {
	uint8_t	type;
	uint16_t	interval;
	uint16_t	window;
};




/*************************************************************************************************
*ö������_ADV_SCAN_MODE_
*��Ա�� IDLE_MODE 	��ǰ״̬Ϊ����״̬
		ADV_MODE 	��ǰ״̬Ϊ�㲥״̬
		SCAN_MODE 	��ǰ״̬Ϊɨ��״̬
		COEX_ADV_MODE 	��ǰ״̬Ϊ����״̬�µĹ㲥״̬
		COEX_SCAN_MODE 	��ǰ״̬Ϊ����״̬�µ�ɨ��״̬
*˵���� �ڹ㲥ͨ���ϵ���Ϊ���п����ǹ㲥Ҳ�п�����ɨ�裬�����������������״̬�£��㲥��ɨ���ֲ�һ����
**************************************************************************************************/
enum _ADV_SCAN_MODE_{
	IDLE_MODE			= 0x00,
	ADV_MODE			= 0x01,
	SCAN_MODE			= 0x02,
	COEX_ADV_MODE		= 0x03,
	COEX_SCAN_MODE	= 0x04,
};





/*************************************************************************************************
*�ṹ������gap_profile_struct
*��Ա�� report_handle_address 	report_handle��ָ�룬ָ����������е�_gatt_database_report_handle
				primary_address	��Ҫ����ָ�룬ָ������е�_gatt_database_primary
				include_address	��Ҫ����ָ�룬ָ������е�_gatt_database_include
				characteristic_address	����ָ�룬ָ������е�_gatt_database_characteristic
				value_address	����ֵָ�룬ָ������е�_gatt_database_value
*˵���� ����ڴ�����Դ�е���ŵ�SYD8801��SYD8821�Ĵ�����Դ�㹻���㣬��������ֱ�Ӱ�BLE��profileֱ��
�ŵ����������У�����ԭ����ȡprofile��ͨ����ȡflash�ķ�ʽ��������ͨ��ֱ�Ӵ���ָ��ķ�ʽȥ��ȡ���ٶ�
�Ϳ��޸��Զ��кô�����Ҫ�������ԡ�����ֵ��ͨ�����ߡ�BtGatt.exe����ȡ
				gap_profile_struct����������profile�ĸ���Ԫ��
**************************************************************************************************/
struct gap_profile_struct {
	uint32_t report_handle_address;
	uint32_t primary_address;
	uint32_t include_address;
	uint32_t characteristic_address;
	uint32_t value_address;
};





/*************************************************************************************************
*ö������GAP_IO_CAPABILITY
*��Ա�� 
																		 \Local output |										|
													Local input \capacity    |   No output        |  Numeric output
													capacity		 \           |                    |
													-------------------------|--------------------|-------------------
													No input                 | NoInputNoOutput    |   DisplayOnly
													-------------------------|--------------------|-------------------
													Yes/No                   |  NoInputNoOutput		|		DisplayYesNo 
													-------------------------|--------------------|-------------------
													Keyboard 								 |	 KeyboardOnly 		|			KeyboardDisplay
*˵���� ��ö�ٴ�����֧�ֵļ��ܵ�IO�������ܹ��Ժ��ַ�ʽ�������룩	
**************************************************************************************************/
enum GAP_IO_CAPABILITY {
	IO_DISPLAY_ONLY		  = 0x00,
	IO_DISPLAY_YESNO		= 0x01,
	IO_KEYBOARD_ONLY		= 0x02,
	IO_NO_INPUT_OUTPUT	= 0x03,	
	IO_KEYBOARD_DISPLAY	= 0x04,	
};



/*************************************************************************************************
*ö������GAP_OOB_FLAG
*��Ա�� OOB_AUTH_NOT_PRESENT	û��ODB����
				OOB_AUTH_PRESENT	��ҪODB����
*˵���� ��ν��ODB���ܾ����Դ����ͨѶ��ʽ�����ܳף�����BLE���ܳ�ͨ�����ڷ��ͣ�����������BLE��ͨѶ
��BLEץ�����ߣ����ƽ�BLE�Ͳ����ܳɹ���
**************************************************************************************************/
enum GAP_OOB_FLAG {
	OOB_AUTH_NOT_PRESENT= 0x00,
	OOB_AUTH_PRESENT		= 0x01,
};



/*************************************************************************************************
*ö������GAP_BONDING_FLAGS
*��Ա�� AUTHREQ_NO_BONDING	������ɺ���Ҫ��
				AUTHREQ_BONDING	������ɺ���Ҫ��
*˵���� ��������˰󶨹��ܣ��ٴ����ӵ�ʱ��Ͳ���Ҫ�ٴν�����ԵĹ����ˣ������Ϣ������оƬ�ڲ���flash
**************************************************************************************************/
enum GAP_BONDING_FLAGS {
	AUTHREQ_NO_BONDING	= 0x00,
	AUTHREQ_BONDING		  = 0x01,
};




/*************************************************************************************************
*ö������GAP_KEY_DISTRIBUTION
*��Ա��  GAP_KEY_MASTER_IDEN	�ܳ׷����ʱ���Ƿ񽻻�EDIV
	     GAP_KEY_ADDR_INFO	�ܳ׷����ʱ���Ƿ񽻻�ADDR
	     GAP_KEY_SIGNIN_INFO	�ܳ׷����ʱ���Ƿ񽻻�SIGNIN��Ϣ
*˵���� �����ɺ�����˫������Ҫ��ʱ��ύ��һЩ��Ϣ������IDEN��ADDR�Լ�SIGNIN����Щ��Ϣ�ǿ�ѡ�ģ�
�����LTK���Ǳ���Ҫ�ġ�
**************************************************************************************************/
enum GAP_KEY_DISTRIBUTION {
	GAP_KEY_MASTER_IDEN	= 0x01,
	GAP_KEY_ADDR_INFO	  = 0x02,
	GAP_KEY_SIGNIN_INFO	= 0x04,
};




/*************************************************************************************************
*�ṹ������gap_pairing_req
*��Ա�� io	��Թ����е�IO������Ϊö��GAP_IO_CAPABILITY��Ա
				oob	����������Ƿ���ҪODB��Ϊö��GAP_OOB_FLAG��Ա
				flags ��Թ������Ƿ���Ҫ��
							Bonding_Flags |
							b1b0          |   Bonding Type
							--------------|------------------
							00            |   No Bonding
							01 						|	 Bonding
							10 						|   Reserved
							11            |   Reserved
				mitm	��Թ������Ƿ���Ҫ�м��˱���set to one if the device is requesting	MITM protection, 
							otherwise it shall be set to 0
				sc	��Թ������Ƿ�ʹ�ð�ȫ������ԣ�����BLE4.2���ӵģ� set to one to request LE Secure 
Connection pairing, othe rwise it shall be set to 0 based on the supported features of the 
initiator and responder
				keypress	 ��Թ������Ƿ�ʹ��Passkey Entry 
				rsvd	����λ
				max_enc_sz	��Թ����������ܳ׳��ȣ�The maximum key size shall be in the range 7 to 16 octets.
				init_key	��Թ����г�ʼ�ߣ�������������ܳ�
				rsp_key	��Թ����дӻ�������ܳ�
*˵���� SYD8821�İ�ȫ������������ʱ�����api:gap_s_security_parameters_set�������ã�����Կ�ʼ��ʱ��
����api:gap_s_security_req��ʼ������̣�����gap_s_security_req����Ҳ��flags��mitm��������������������
�Ͱ�ȫ������Ĳ����г�ͻ����������ʹ�õ���gap_s_security_req����ĺ���
**************************************************************************************************/
struct gap_pairing_req{
	uint8_t io;
	uint8_t oob;
	uint8_t flags:2;
	uint8_t mitm:1;
	uint8_t sc:1;
	uint8_t keypress:1;
	uint8_t rsvd:3;
	uint8_t max_enc_sz;
	uint8_t init_key;
	uint8_t rsp_key;
};




/*************************************************************************************************
*�ṹ������gap_connection_param_rsp_pdu
*��Ա�� Interval_Min	���Ӳ����е����Ӽ����Сֵ
				Interval_Max	���Ӳ����е����Ӽ�����ֵ
				Latency	���Ӳ����е�latency
				Timeout	���Ӳ����е�Timeout
				PeferredPeriodicity	���Ӳ����е�PreferredPeriodicity
				ReferenceConnEventCount ���Ӳ����е�ReferenceConnEventCount
				Offset ���Ӳ����е�Offset����
*˵���� BLE4.2������LL_CONNECTION_PARAM_REQ��LL_CONNECTION_PARAM_RSP����LL������ݰ��������Ӳ�����
�˸�������ƣ����ߵĲ�����һ���ġ�
				gap_connection_param_rsp_pdu�ṹ��ľ���������������LL_CONNECTION_PARAM_RSP�����ϣ������
��ʽBLE4.0���߲�֧��LL_CONNECTION_PARAM������ýṹ�������õ�
				������Э��ջ��ʼ�������е���api:gap_s_connection_param_set��������
**************************************************************************************************/
struct gap_connection_param_rsp_pdu {
	uint16_t Interval_Min;
	uint16_t Interval_Max;
	uint16_t Latency;
	uint16_t Timeout;
	uint8_t PeferredPeriodicity;
	uint16_t ReferenceConnEventCount;
	uint16_t Offset[LL_WIN_OFFSET_SZ];	
};




/*************************************************************************************************
*�ṹ������gap_update_params
*��Ա�� updateitv_min	���Ӳ��������е����Ӽ����Сֵ
				updateitv_max	���Ӳ��������е����Ӽ�����ֵ
				updatelatency	���Ӳ��������е�latency
				updatesvto	���Ӳ��������еĳ�ʱʱ��
*˵���� SYD8821�����Ӳ���ʹ��api:gap_s_connection_update�������ã������ȷ�е������Ӳ������£�BLE��
��˵��BLE�Ĵӻ�û��ֱ�ӷ���LL���LL_CONNECTION_UPDATE_REQ���ֻ�ܹ�ͨ��L2CAP������ͨ������
CONNECTION PARAMETER UPDATE REQUEST����������Ӳ������ýṹ��ΪCONNECTION PARAMETER UPDATE REQUEST
�����еĸ�������
**************************************************************************************************/
struct gap_update_params {
	uint16_t  updateitv_min;
	uint16_t  updateitv_max;
	uint16_t  updatelatency;
	uint16_t  updatesvto;
};




/*************************************************************************************************
*ö������GAP_SMART_CONTROL_SET
*��Ա�� SMART_CONTROL_LATENCY	ʹ��latency�Զ����ƻ���
				SMART_CONTROL_UPDATE	ʹ�����Ӳ����Զ�����
*˵���� ��ö��ʹ��bitmask���Σ���ö��ֻ����smart_update_latency������ʹ�û����ʽ���룬����ɴ��룺
				SMART_CONTROL_LATENCY|SMART_CONTROL_UPDATE
**************************************************************************************************/
enum GAP_SMART_CONTROL_SET {
	SMART_CONTROL_LATENCY	= 0x80,
	SMART_CONTROL_UPDATE	= 0x40,
};




/*************************************************************************************************
*�ṹ������gap_smart_update_params
*��Ա�� updatectrl	����smart_update_latency�Ĺ���Χ��Ϊö��GAP_SMART_CONTROL_SET�ĳ�Ա��ֵ
				updateadj_num	����smart_update_latency�ĵ���������һ�㴫�뱾�ļ��ĺ꣺MAX_UPDATE_ADJ_NUM
				updateitv_target	����Ŀ�����Ӳ��������smart_update_latency����BLE�����Ӳ������Ľӽ�
													�ò���
				updatelatency	�������Ӳ�����latency��smart_update_latency��ֱ�ӰѸò����������Ӳ���������
				updatesvto �������Ӳ����ĳ�ʱʱ�䣬smart_update_latency��ֱ�ӰѸò����������Ӳ���������
*˵���� �ýṹ��ֻ����smart_update_latency��Э��ջ����ݸýṹ���Զ��������Ӳ�����������������������
�������ܲ����ܹ���ȫ�ýṹ���Ҫ�󣬵��ǻ����̶ȵĽӽ��ýṹ��Ĳ���
**************************************************************************************************/
struct gap_smart_update_params {
	uint8_t 	updatectrl;
	uint8_t 	updateadj_num;
	uint16_t  updateitv_target;
	uint16_t  updatelatency;
	uint16_t  updatesvto;
};




/*************************************************************************************************
*�ṹ������gap_link_params
*��Ա�� interval	Ŀǰ��·������Ӳ������
				latency		Ŀǰ��·������Ӳ�����latency
				svto	Ŀǰ��·������Ӳ����ĳ�ʱʱ��
*˵���� SYD8821��Э��ջ�����е�ǰ�����Ӳ�������ͨ��gap_s_link_parameters_get������ȡ���ò��������յ�
LL���LL_CONNECTION_UPDATE_REQ�����ʱ�򽫻ᱻ����
**************************************************************************************************/
struct gap_link_params {	
	uint16_t	interval;
	uint16_t	latency;
	uint16_t	svto;
};




/*************************************************************************************************
*ö������_GAP_EVT_
*��Ա�� GAP_EVT_ADV_END	�����㲥����¼������㲥�����еĳ�ʱʱ�䵽����Э��ջ�ϱ����¼�
				GAP_EVT_ADV_REPORT		����ɨ���¼���
				GAP_EVT_CONNECTED	���������¼��������ӳɹ���ʱ��Э��ջ�ϱ����¼������ݹ淶��˵������������
													�ɹ��ı�־�������յ���������CONNECT_REQ��ʱ�̣������ڽ��յ�CONNECT_REQ��
													���Ҫ����һ�����ݰ����������������ӳɹ�
				GAP_EVT_DISCONNECTED	���������¼���Э��ջ��ͬʱ�ϱ����ߵ�ԭ�򣬷���HCI�Ķ���ԭ��
				GAP_EVT_ENC_KEY	�����������¼���BLE��Ե�������������ܳ׵Ľ�����������ɺ��ϱ����¼�
				GAP_EVT_PASSKEY_REQ	�������������¼������BLEʹ��������ܣ�����Ҫ���������ʱ���ϱ����¼�
														������������
				GAP_EVT_ENC_START	������ʼ�����¼��������ɻ��߰�������������Ҫ��ʼ����
				GAP_EVT_CONNECTION_EVENT	���Ӽ��ʱ���¼��������������¼�������ʱ���ϱ�
				GAP_EVT_CONNECTION_UPDATE_RSP	���Ӳ���������Ӧ�¼���SYD8821�ӻ�����CONNECTION PARAMETER 
																		  UPDATE REQUEST��������������Ӧ���յ���Ӧ��Э��ջ�ϱ����¼�
				GAP_EVT_ATT_READ	������SYD8821�����˶�������Э��ջ���ϱ����¼�
				GAP_EVT_ATT_WRITE	������SYD8821������д������Э��ջ���ϱ����¼�
				GAP_EVT_ATT_PREPARE_WRITE	������SYD8821������Ԥ��д������Э��ջ���ϱ����¼�
				GAP_EVT_ATT_EXECUTE_WRITE	������SYD8821������ִ��д������Э��ջ���ϱ����¼�
				GAP_EVT_ATT_HANDLE_CONFIRMATION	�������յ�ָʾȷ��ʱ�ϱ����¼�����־��ָʾ�������
				GAP_EVT_ATT_HANDLE_CONFIGURE	�������յ�����CCCDҲ��������ʹ��notify����ָʾ���ܵ�ʱ���ϱ�
																			���¼�����������Ҫ������Ӧ����
*˵���� SYD8821Э��ջ��ʼ����ʱ��Ҫ����api:gap_s_evt_handler_set����gap_evt_callback�ṹ�壬�ýṹ��
������Э��ջ�ϱ�BLE�¼��Ľӿڣ�SYD8821��Э��ջ�ڷ���BLE��״̬�����仯�����緢�������ӻ��߶��ߣ���ʱ
��Э��ջ������APP�㣨�û����룩�ϱ�BLE�¼���ͬʱ�����ϱ�gap_ble_evt�ṹ�壬�ýṹ���е�evt_code��
������_GAP_EVT_ö�ٶ���ĳ�Ա
**************************************************************************************************/
enum _GAP_EVT_{
	GAP_EVT_ADV_END					= 0x0001,
	GAP_EVT_ADV_REPORT				= 0x0002,
	GAP_EVT_CONNECTED				= 0x0004,
	GAP_EVT_DISCONNECTED			= 0x0008,
	GAP_EVT_ENC_KEY					= 0x0010,
	GAP_EVT_PASSKEY_REQ				= 0x0020,
	GAP_EVT_SHOW_PASSKEY_REQ		= 0x0040,
	GAP_EVT_ENC_START				= 0x0080,
	GAP_EVT_CONNECTION_EVENT		= 0x0100,
	GAP_EVT_CONNECTION_UPDATE_RSP	= 0x0200,
	GAP_EVT_ATT_READ				= 0x0400,
	GAP_EVT_ATT_WRITE				= 0x0800,
	GAP_EVT_ATT_PREPARE_WRITE	    = 0x1000,
	GAP_EVT_ATT_EXECUTE_WRITE	    = 0x2000,
	GAP_EVT_ATT_HANDLE_CONFIRMATION	= 0x4000,
	GAP_EVT_ATT_HANDLE_CONFIGURE	= 0x8000,
};




/*************************************************************************************************
*ö������CONNECTION_UPDATE_RSP_
*��Ա�� CONN_PARAM_ACCEPTED	���Ӳ������±���������
				CONN_PARAM_REJECTED	���Ӳ������±������ܾ�
				CONN_PARAM_SMART_TIMEROUT	�������Ӳ������������ʱ�¼��������͵��¼������Ǳ�׼��BLE�¼�������
																	����smart_update���ܶ���ӵģ���ʹ��smart_update���ܲ����ϱ����¼�
				CONN_PARAM_SMART_SUCCEED	�������Ӳ�����������ɹ��¼��������͵��¼������Ǳ�׼��BLE�¼�������
																	����smart_update���ܶ���ӵģ���ʹ��smart_update���ܲ����ϱ����¼�
				CONN_PARAM_LATENCY_ENABLE	�������Ӳ�������ʹ��latency��Э��ջʹ��latency��ʱ���ϱ����¼���
																	�����͵��¼������Ǳ�׼��BLE�¼�����������smart_update���ܶ���ӵģ�
																	��ʹ��smart_update���ܲ����ϱ����¼�
				CONN_PARAM_LATENCY_DISABLE	�������Ӳ�������ʧ��latency��Э��ջʹ��latency��ʱ���ϱ����¼���
																	�����͵��¼������Ǳ�׼��BLE�¼�����������smart_update���ܶ���ӵģ�
																	��ʹ��smart_update���ܲ����ϱ����¼�
*˵���� BLE�淶��ֻ�Ƕ�����CONN_PARAM_ACCEPTED��CONN_PARAM_REJECTED�����¼�����Ϊsmart_update_latency
����Ҳ��Ҫ�ϱ�һЩ�¼���Ӧ�ò㣬����������CONNECTION_UPDATE_RSP�����Ӽ����ϱ��¼������ӵ���Щ�¼����
��ʹ��smart_update���ܲ����ϱ����¼�
**************************************************************************************************/
enum CONNECTION_UPDATE_RSP_ {
	CONN_PARAM_ACCEPTED	= 0x0000,
	CONN_PARAM_REJECTED	= 0x0001,
	CONN_PARAM_SMART_TIMEROUT	= 0x0002,	
	CONN_PARAM_SMART_SUCCEED	= 0x0003,	
	CONN_PARAM_LATENCY_ENABLE	= 0x0004, 
	CONN_PARAM_LATENCY_DISABLE	= 0x0005, 
};




/*************************************************************************************************
*�ṹ������gap_disconnected_evt
*��Ա�� reason	����ԭ�򣬸�ԭ�����HCI��ERROR CODE
*˵���� BLE���ߵ�ԭ���кܶ࣬SYD8821�ڷ������ߵ�ʱ���Ѷ��ߵ�ԭ���ϱ���������ԭ�����Ϊ�ο�������
��Ե�ɿɿ��淶�ĵڶ��µ�Part D ERROR CODES�ĵڶ���ERROR CODE DESCRIPTIONS
**************************************************************************************************/
struct gap_disconnected_evt {
	uint8_t	reason;
};




/*************************************************************************************************
*�ṹ������gap_att_read_evt
*��Ա�� primary	BLE���¼����������UUID
				uuid	BLE���¼�������UUID
				hdl	BLE���¼������Ե�hdl(���)
				offset	BLE���¼���ƫ��ֵ��������ĳЩ����������Ч��
*˵���� BLE�������Ķ�����BLE������ֵ��Ӧ�ò�ȿ���ͨ��UUID���ж϶Է����ȡ�ľ�������ֵ��Ҳ����ͨ��
���Եľ�����жϾ�������ԣ���ͬһ��profile�о����Ψһ�ģ����ǿ��ܲ�ͬ��profile��ʹuuid��ͬ������
���ǵľ��Ҳ�����в��졣���κ�һ��profile�����Ե�UUID��һ�µģ�������ͬһ��profile�п��ܻ��м�����
ͬUUID�����ԣ���ʱ��͵�ͨ��hdl���ж�hdl,���ڸ��ӵ�profile�������ʹ��uuid��hdl
**************************************************************************************************/
struct gap_att_read_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	offset;
};



/*************************************************************************************************
*�ṹ������gap_att_write_evt
*��Ա�� primary	BLEд�¼����������UUID
				uuid	BLEд�¼�������UUID
				hdl	BLEд�¼������Ե�hdl(���)
				sz	BLEд���ݵĴ�С
				data	BLEд�ľ�������
*˵����SYD8821��MAX_ATT_DATA_SZ��MTU)��512��byte����Щ�����Ƿŵ���ջ�еģ�������Ӷ�ջ����Ŀ����ԣ�
���Բ�ͬ�Ĺ��̣���ͬ�����󣬿��ʵ�����SYD8821�Ķ�ջ
		BLEд�����Ķ�����BLE������ֵ��Ӧ�ò�ȿ���ͨ��UUID���ж϶Է����ȡ�ľ�������ֵ��Ҳ����ͨ��
���Եľ�����жϾ�������ԣ���ͬһ��profile�о����Ψһ�ģ����ǿ��ܲ�ͬ��profile��ʹuuid��ͬ������
���ǵľ��Ҳ�����в��졣���κ�һ��profile�����Ե�UUID��һ�µģ�������ͬһ��profile�п��ܻ��м�����
ͬUUID�����ԣ���ʱ��͵�ͨ��hdl���ж�hdl,���ڸ��ӵ�profile�������ʹ��uuid��hdl
**************************************************************************************************/
struct gap_att_write_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint8_t	sz;
	uint8_t	data[MAX_ATT_DATA_SZ];
};




/*************************************************************************************************
*�ṹ������gap_att_pre_write_evt
*��Ա�� primary	BLEԤ��д�¼����������UUID
				uuid	BLEԤ��д�¼�������UUID
				hdl	BLEԤ��д�¼������Ե�hdl(���)
				sz	BLEԤ��д���ݵĴ�С
				data	BLEԤ��д�ľ�������
*˵����BLE��Ԥ��д��ִ��д������ʹ�õģ�Ҫд������������ݵ�ʱ����ܶԷ�����ͨ��Ԥ��д��������ݷֶ�
���͸�SYD8821��Ȼ��ͨ��ִ��д��������д������ֵ��������֤д�����ݵ�ԭ����
		SYD8821��MAX_ATT_DATA_SZ��MTU)��512��byte����Щ�����Ƿŵ���ջ�еģ�������Ӷ�ջ����Ŀ����ԣ�
���Բ�ͬ�Ĺ��̣���ͬ�����󣬿��ʵ�����SYD8821�Ķ�ջ
		BLEԤ��д�����Ķ�����BLE������ֵ��Ӧ�ò�ȿ���ͨ��UUID���ж϶Է����ȡ�ľ�������ֵ��Ҳ����ͨ��
���Եľ�����жϾ�������ԣ���ͬһ��profile�о����Ψһ�ģ����ǿ��ܲ�ͬ��profile��ʹuuid��ͬ������
���ǵľ��Ҳ�����в��졣���κ�һ��profile�����Ե�UUID��һ�µģ�������ͬһ��profile�п��ܻ��м�����
ͬUUID�����ԣ���ʱ��͵�ͨ��hdl���ж�hdl,���ڸ��ӵ�profile�������ʹ��uuid��hdl
**************************************************************************************************/
struct gap_att_pre_write_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	offset;
	uint8_t	sz;
	uint8_t	data[MAX_ATT_DATA_SZ];
};



/*************************************************************************************************
*�ṹ������gap_att_exec_write_evt
*��Ա�� flags	0x00:Cancel all prepared writes 0x01:Immediately write all pending prepared values
*˵���� ��Ԥ��д������ᷢ��ִ��д��ִ������ֵд��Ĳ��������������һ����������ʾ�Ƿ�д��֮ǰ��
����
**************************************************************************************************/
struct gap_att_exec_write_evt {
	uint8_t	flags;
};




/*************************************************************************************************
*�ṹ������gap_att_handle_configure_evt
*��Ա�� uuid	BLE����CCCD�¼��ķ����UUID
				hdl	BLE����CCCD�¼�����������hdl(���)
				value	��CCCD����Ϊ�����������bitmask����ʽ��Ϊö��BLE_SEND_TYPE�ĳ�Ա�������Ա�Ļ�ֵ
*˵���� ���ݹ淶��BLE��notify(֪ͨ������indicate��ָʾ����ʹ�ܻ���ʧ�ܶ���������Ҳ����master���Ƶģ�
��������SYD8821��notify����indicate�Ŀ��ؽ��в�����ʱ�򣬵ײ�Э��ջ���ϱ����¼�
				handle_configure�����Ķ�����BLE����������Ӧ�ò�ȿ���ͨ��UUID���ж϶Է����ȡ�ľ���ķ���
����������ʱ���ַ�ʽ����һ�����������ж������CCCD�������������ַ�ʽ�Ͳ������ˣ����������ֻ�ܹ�ͨ
���������ľ�����жϾ����������,���ڸ��ӵ�profile�������ʹ��uuid��hdl
				һ�����Կ��ܴ���notify��indicate���֣����������ͨ���жϾ����λ���ж��Ƿ�ʹ�ܻ���ʧ�ܡ����磺
���value=0x03���Ǿ���ͬʱʹ��notify��indicate��value=0x01�򵥵�ʹ��notify
**************************************************************************************************/
struct gap_att_handle_configure_evt {
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	value;
};




/*************************************************************************************************
*�ṹ������gap_connection_update_rsp_evt
*��Ա�� result	BLE���Ӳ������½�� ��Ϊö��CONNECTION_UPDATE_RSP_�ĳ�Ա
*˵���� SYD8821�յ�CONNECTION PARAMETER UPDATE RESPONSE��Ӧ���ϱ�GAP_EVT_CONNECTION_UPDATE_RSP�¼���
������¼�����Ÿýṹ��Ĳ�������ͨ���ò����ж������Ƿ�ͬ�����Ӳ�������
				BLE�淶��ֻ�Ƕ�����CONN_PARAM_ACCEPTED��CONN_PARAM_REJECTED�����¼�����Ϊsmart_update_latency
����Ҳ��Ҫ�ϱ�һЩ�¼���Ӧ�ò㣬����������CONNECTION_UPDATE_RSP�����Ӽ����ϱ��¼������ӵ���Щ�¼����
��ʹ��smart_update���ܲ����ϱ����¼�
**************************************************************************************************/
struct gap_connection_update_rsp_evt {
	uint16_t result;
};




/*************************************************************************************************
*�ṹ������gap_advertising_report_evt
*��Ա�� type	ɨ���ϱ��¼�������
				peer_dev_addr	�㲥�ߵ�ַ
				len	ɨ�赽�Ĺ㲥���ݵĳ���
				buf	ɨ�赽�Ĺ㲥������
				rssi	ɨ�赽�Ĺ㲥���ź�ǿ��
*˵���� SYD8821��ɨ���ʱ������յ���ȷ�Ĺ㲥�źŻ��ϱ���Ӧ���¼������¼������иýṹ�壬������Щ
��Ա�ڲ�ͬ�Ĺ㲥��������û�е�
**************************************************************************************************/
struct gap_advertising_report_evt {
	uint8_t type;
	struct gap_ble_addr	peer_dev_addr;
	int8_t	len;
	uint8_t	buf[MAX_ADV_DATA_SZ];
	uint8_t	rssi;
};




/*************************************************************************************************
*�ṹ������gap_att_handle_confirmation_evt
*��Ա�� primary	BLEָʾȷ���¼����������UUID
				uuid	BLEָʾȷ���¼�������UUID
				hdl	BLEָʾȷ���¼������Ե�����ֵ��hdl(���)
*˵���� BLE��ָʾ�Ĳ����֪ͨ���ܶ���һ�����صĹ��̣���SYD8801�յ���������������Ӧʱ�ϱ�
GAP_EVT_ATT_HANDLE_CONFIRMATION�¼������¼������ýṹ�塣
				������չ淶��GAP_EVT_ATT_HANDLE_CONFIRMATION��Ӧ�������κε����ݣ����Ըýṹ��������Ǳ���
����·��Ļ������У���APP�����ڷ���ָʾ��ʱ�򴫸�Э��ջ��				
**************************************************************************************************/
struct gap_att_handle_confirmation_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
};



/*************************************************************************************************
*�ṹ������gap_ble_evt
*��Ա�� evt_type	Э��ջ�ϱ��¼������ͣ�����APPֻ���ϱ�GAP�¼�������ֻ��GAP�¼�
				evt_code	Э��ջ�ϱ��¼��ı�ʶ�루code)
				evt	Э��ջ�ϱ��¼��ľ������ݣ���һ�������壬�����и����¼�������
					disconn_evt	�����¼�����
					bond_dev_evt	���豸�ĵ�ַ�������������ߵ�ʱ����ϱ��ýṹ�壬ָʾ�Է����豸��ַ
					enc_key_evt	�ܳ׷����¼����ݣ���������Թ��̵ĸ����ܳ�
					att_read_evt	BLE�������¼���ָʾ�����Ҫ��ȡ�Ķ���UUID,hdl��
					att_write_evt	BLEд�����¼���ָʾ�����Ҫ�����Ķ������Ӧ�����ݣ�UUID,hdl�Լ����ݵ�
					att_pre_write_evt	BLEԤ��д�����¼���ָʾ�����Ҫ�����Ķ������Ӧ�����ݣ�UUID,hdl�Լ����ݵ�
					att_exec_write_evt	BLEִ��д�����¼���ָʾ�����Ҫ�����Ķ������Ӧ�����ݣ�UUID,hdl�Լ����ݵ�
					att_handle_config_evt	BLE����notify(֪ͨ������indicate��ָʾ���¼���
					att_handle_confirmation_evt	BLEָʾ����ȷ����Ӧ�¼���������·�������
					connection_update_rsp_evt	BLE���Ӳ���������Ӧ�¼���������
					advertising_report_evt	BLEɨ���ϱ��¼�
*˵���� SYD8821���յ������¼���ʱ��ͨ��p_callback�ϱ������¼���gap_ble_evt�ṹ��Ϊ�����¼��ľ���
����
**************************************************************************************************/
struct gap_ble_evt {
	uint8_t	evt_type;
	uint32_t	evt_code;
	union
	{
		struct gap_disconnected_evt 		disconn_evt;
		struct gap_ble_addr				bond_dev_evt;
		struct gap_key_params			enc_key_evt;
		struct gap_att_read_evt			att_read_evt;
		struct gap_att_write_evt			att_write_evt;
		struct gap_att_write_evt			att_pre_write_evt;
		struct gap_att_write_evt			att_exec_write_evt;
		struct gap_att_handle_configure_evt	att_handle_config_evt;
		struct gap_att_handle_confirmation_evt	att_handle_confirmation_evt;
		struct gap_connection_update_rsp_evt connection_update_rsp_evt;
		struct gap_advertising_report_evt 	advertising_report_evt;
	} evt; 
};





/*************************************************************************************************
*�ṹ������gap_evt_callback
*��Ա�� evt_mask	�¼���������λ���ñ���ʹ��bitmask��ʽ��Ϊö��_GAP_EVT_��ֵ������ֵ�Ļ�ֵ
				p_callback	BLE�¼��ϱ��Ĺ��캯����Ϊһ������ָ��
*˵���� SYD8821���յ������¼���ʱ��ͨ��p_callback�ϱ������¼���gap_ble_evt�ṹ��Ϊ�����¼��ľ���
����
**************************************************************************************************/
struct gap_evt_callback {
	uint32_t	evt_mask;
	void 	(*p_callback)(struct gap_ble_evt *p_evt);
};




/*************************************************************************************************
*�ṹ������gap_att_report
*��Ա�� primary	BLE����ṹ����������UUID
				uuid	BLE����ṹ�������UUID
				hdl	BLE����ṹ������Ե�����ֵhdl(��� val_hdl)
				config	BLE����ṹ������Ե�CCCD��������handle(���)
				value �������Ϊ����notify��indicate����
*˵���� SYD8821��profile��notify��indicate����GATT service��Ϊ���ϳ�Ϊreport��ר�������ṹ��
gap_att_report_handle������report����Ӧ������Ϣ��ÿ��CCCD��Ӧ�Žṹ��gap_att_report_handle�е�һ��
�����Ա��������CCCD���ڵ�λ�ã�����Ҫ�������ԣ�val_hdl����
**************************************************************************************************/
struct gap_att_report {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	config;
	uint16_t	value;
};




/*************************************************************************************************
*�ṹ������att_err_rsp
*��Ա�� opcode	ATT������Ӧ�е�Request Opcode In Error����
				hdl	ATT������Ӧ�е�Attribute Handle In Error����
				err	ATT������Ӧ�е�Error Code
*˵����	BLE�ڷ����������������ʱ���ӦError Response�¼�����ATT�����а����д����ԭ�����hdl��
**************************************************************************************************/
struct att_err_rsp{
	uint8_t opcode;
	uint16_t hdl;
	uint8_t err;
};



/*************************************************************************************************
*�ṹ������att_find_by_type_val_req
*��Ա�� start_hdl		ATT�ͻ���Find By Type Value Request�����Starting Handle����
				end_hdl		ATT�ͻ���Find By Type Value Request�����Ending Handle����
				att_type	 ATT�ͻ���Find By Type Value Request�����Attribute Type����
				att_val		ATT�ͻ���Find By Type Value Request�����Attribute Value����
*˵����	���ṹ���Ӧ��Find By Type Value Request�ĸ�������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_find_by_type_val_req{
	uint16_t start_hdl;
	uint16_t end_hdl;
	uint16_t att_type;
	uint8_t att_val[MAX_ATT_DATA_SZ-7];
};




/*************************************************************************************************
*�ṹ������att_mtu_rsp
*��Ա�� mtu		ATT���Exchange MTU Response�����Server Rx MTU����
*˵����	���ṹ���Ӧ��Exchange MTU Response�ĸ�������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�Ľṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_mtu_rsp{
	uint16_t mtu;
};




/*************************************************************************************************
*�ṹ������att_find_info_128
*��Ա�� hdl		Handleֵ
				uuid		16bit��UUIDֵ
*˵����	Find Information Response�����formatΪ0x01ʱ����Information DataΪ���ṹ��
**************************************************************************************************/
struct att_find_info_16{
	uint16_t hdl;
	uint8_t uuid[2];
};



/*************************************************************************************************
*�ṹ������att_find_info_128
*��Ա�� hdl		Handleֵ
				uuid		128bit��UUIDֵ
*˵����	Find Information Response�����formatΪ0x02ʱ����Information DataΪ���ṹ��
**************************************************************************************************/
struct att_find_info_128{
	uint16_t hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*����������att_find_info_payload
*��Ա�� uuid16		16bituuid�µ�Information Data����
				uuid128		128bituuid�µ�Information Data����
*˵����	���������Ӧ��Find Information Response��Information Data���������ݹ淶��Ϊformat��һ��
Information Data�п�����handle��16bit uuid����ϣ�Ҳ�п�����handle��128bit uuid�����
**************************************************************************************************/
union  att_find_info_payload {
	struct att_find_info_16   uuid16[5];
	struct att_find_info_128 uuid128;
};



/*************************************************************************************************
*�ṹ������att_find_info_rsp
*��Ա�� format		�ò���������Information Data�ĳ���
				pair		Information Data����
*˵����	�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_find_info_rsp{
	uint8_t format;
	union att_find_info_payload pair;
};



/*************************************************************************************************
*�ṹ������att_find_by_type_val_rsp
*��Ա�� list		Handles Information List����
*˵����	���ṹ���Ӧ��Find By Type Value Response�Ĳ��������ݹ淶list��һ��Found Attribute Handle��
Group End Handle�����
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_find_by_type_val_rsp{
	uint8_t list[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*�ṹ������att_read_by_type_16
*��Ա�� hdl		Attribute Data List�����е�Attribute Handle����
				property		Attribute Data List�����е�Attribute Value�������ڷ���characteristicʱ
										Attribute Value�е�Characteristic Properties
				val_hdl		Attribute Data List�����е�Attribute Value�����ڷ���characteristicʱAttribute Value
									�е�Characteristic Value Attribute Handle
				char_uuid		Attribute Data List�����е�Attribute Value�����ڷ���characteristicʱAttribute Value
										�е�Characteristic UUID
*˵����	���ṹ���Ӧ��Read By Type Response������Attribute Data List�Ĳ���
**************************************************************************************************/
struct att_read_by_type_16{
	uint16_t hdl;
	uint8_t property;
	uint16_t val_hdl;
	uint8_t char_uuid[2];
};




/*************************************************************************************************
*�ṹ������att_read_by_type_128
*��Ա�� hdl		Attribute Data List�����е�Attribute Handle����
				property		Attribute Data List�����е�Attribute Value�������ڷ���characteristicʱ
										Attribute Value�е�Characteristic Properties
				val_hdl		Attribute Data List�����е�Attribute Value�����ڷ���characteristicʱAttribute Value
									�е�Characteristic Value Attribute Handle
				char_uuid		Attribute Data List�����е�Attribute Value�����ڷ���characteristicʱAttribute Value
										�е�Characteristic UUID
*˵����	���ṹ���Ӧ��Read By Type Response������Attribute Data List�Ĳ���
**************************************************************************************************/
struct att_read_by_type_128{
	uint16_t hdl;
	uint8_t property;
	uint16_t val_hdl;
	uint8_t char_uuid[16];
};




/*************************************************************************************************
*����������att_read_by_type_payload
*��Ա�� uuid16		16bituuid�µ�Attribute Data List����
				uuid128		128bituuid�µ�Attribute Data List����
*˵����	Read By Type Response�е�Length������Attribute Data List�ĳ��ȣ�����16bit uuid��1258bit uuid
				���ṹֻ�����ڲ���������characteristic�����
**************************************************************************************************/
union  att_read_by_type_payload {
	struct att_read_by_type_16   uuid16[3];
	struct att_read_by_type_128 uuid128;
};




/*************************************************************************************************
*����������att_read_by_type_rsp
*��Ա�� length		ATT���Read By Type Response�����µ�Length����
				pair		ATT���Read By Type Response�����µ�Attribute Data List����
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength��һ��������length������
Attribute Data List�ĳ���
				Attribute Data List�п�����handle��16bit uuid��characteristic��ϣ�Ҳ�п�����handle��128bit uuid��
characteristic���
				���ṹֻ�����ڲ���������characteristic�����
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_type_rsp{
	uint8_t length;
	union att_read_by_type_payload pair;
};




/*************************************************************************************************
*����������att_read_by_type_include_rsp
*��Ա�� length		ATT���Read By Type Response�����µ�Length����
				hdl		ATT���Read By Type Response�����µ�Attribute Data List�����е�Attribute Handle
				buf		ATT���Read By Type Response�����µ�Attribute Data List�����е�Attribute Value
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength��һ��������length������
Attribute Data List�ĳ���
				���ṹֻ�����ڲ��������Ǵ�Ҫ����include�������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_type_include_rsp{
	uint8_t length;
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-2];
};




/*************************************************************************************************
*����������att_read_by_type_pair_val
*��Ա�� hdl		Attribute Data List�����е�Attribute Handle
				val		Attribute Data List�����е�Attribute Value
*˵����	���ṹֻ�����ڲ��������ǵ���pairֵ�����
**************************************************************************************************/
struct att_read_by_type_pair_val{
	uint16_t hdl;
	uint8_t val[10];
};




/*************************************************************************************************
*����������att_read_by_type_val_rsp
*��Ա�� length		Length����
				pair		Attribute Data List����
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength��һ��������length������
Attribute Data List�ĳ���
				���ṹֻ�����ڲ��������ǵ���pairֵ�����
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_type_val_rsp{
	uint8_t length;
	struct att_read_by_type_pair_val pair[1];
};




/*************************************************************************************************
*����������att_read_by_type_service_16
*��Ա:hdl		Attribute Data List�����е�Attribute Handle
			uuid	Attribute Data List�����е�Attribute Value
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ���
				���ṹֻ�����ڲ�����������Ҫ��������
**************************************************************************************************/
struct att_read_by_type_service_16{
	uint16_t hdl;
	uint8_t uuid[2];
};



/*************************************************************************************************
*����������att_read_by_type_service_128
*��Ա:hdl		Attribute Data List�����е�Attribute Handle
			uuid	Attribute Data List�����е�Attribute Value
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ���
				���ṹֻ�����ڲ�����������Ҫ��������
**************************************************************************************************/
struct att_read_by_type_service_128{
	uint16_t hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*����������att_read_by_type_service_payload
*��Ա:uuid16	16bituuid�µ�Attribute Data List����
			uuid128	128bituuid�µ�Attribute Data List����
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength��һ��Attribute Data List�п�
����handle��16bit uuid����Ҫ������ϣ�Ҳ�п�����handle��128bit uuid����Ҫ�������
				���ṹֻ�����ڲ�����������Ҫ��������
**************************************************************************************************/
union  att_read_by_type_service_payload {
	struct att_read_by_type_service_16   uuid16[3];
	struct att_read_by_type_service_128 uuid128;
};




/*************************************************************************************************
*����������att_read_by_type_service_rsp
*��Ա:length	Length����
			pair	attribute Data List����
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength��һ��Attribute Data List�п�
����handle��16bit uuid����Ҫ������ϣ�Ҳ�п�����handle��128bit uuid����Ҫ�������
				���ṹֻ�����ڲ�����������Ҫ��������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_type_service_rsp{
	uint8_t length;
	union att_read_by_type_service_payload pair;
};





/*************************************************************************************************
*����������att_read_by_type_chartextend_rsp
*��Ա:length	Length����
			hdl		Attribute Data List�����е�Attribute Handle
			val 	128bituuid�µ�Attribute Data List����
*˵����	���ṹ���Ӧ��Read By Type Response�Ĳ��������ݹ淶��Ϊlength����Attribute Data List�ĳ���
				���ṹֻ�����ڲ������������ԣ���att_read_by_type_rsp��ͬ������һ����չ�Ľṹ��
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_type_chartextend_rsp{
	uint8_t length;
	uint16_t hdl;
	uint8_t val[MAX_ATT_DATA_SZ-4];
};




/*************************************************************************************************
*����������att_read_rsp
*��Ա:buf	Attribute Value����
*˵����	�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_rsp{
	uint8_t buf[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*����������att_read_blob_rsp
*��Ա:buf	Part Attribute Value����
*˵����	�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_blob_rsp{
	uint8_t buf[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*����������att_read_multiple_rsp
*��Ա:val	Set Of Values����
*˵����	�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_multiple_rsp{
	uint8_t val[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*����������att_read_by_group_type_16
*��Ա:hdl	16bituuid�µ�Attribute Data List��Attribute Handle����
			end_hdl	16bituuid�µ�Attribute Data List��End Group Handle����
			uuid 16bituuid�µ�Attribute Data List��Attribute Value����
*˵����	���ṹ���Ӧ��Read by Group Type Response�Ĳ���
**************************************************************************************************/
struct att_read_by_group_type_16{
	uint16_t hdl;
	uint16_t end_hdl;
	uint8_t uuid[2];
};




/*************************************************************************************************
*����������att_read_by_group_type_128
*��Ա:hdl	128bituuid�µ�Attribute Data List��Attribute Handle����
			end_hdl	128bituuid�µ�Attribute Data List��End Group Handle����
			uuid 128bituuid�µ�Attribute Data List��Attribute Value����
*˵����	���ṹ���Ӧ��Read by Group Type Response�Ĳ���
**************************************************************************************************/
struct att_read_by_group_type_128{
	uint16_t hdl;
	uint16_t end_hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*����������att_read_by_group_type_payload
*��Ա:uuid16	16bituuid�µ�Attribute Data List����
			uuid128	128bituuid�µ�Attribute Data List����
*˵����	���ṹ���Ӧ��Read by Group Type Response�Ĳ��������ݹ淶��Ϊlength��һ��Attribute Data List
�п�����handle��16bit uuid����Ҫ������ϣ�Ҳ�п�����handle��128bit uuid����Ҫ�������
**************************************************************************************************/
union  att_read_by_group_type_payload {
	struct att_read_by_group_type_16   uuid16[3];
	struct att_read_by_group_type_128 uuid128;
};




/*************************************************************************************************
*����������att_read_by_group_type_rsp
*��Ա:length	Length����
			pair	attribute Data List����
*˵����	���ṹ���Ӧ��Read by Group Type Response�Ĳ��������ݹ淶��Ϊlength��һ��Attribute Data List
�п�����handle��16bit uuid����Ҫ������ϣ�Ҳ�п�����handle��128bit uuid����Ҫ�������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_read_by_group_type_rsp{
	uint8_t length;
	union  att_read_by_group_type_payload pair;
};




/*************************************************************************************************
*����������att_hdl_val_notifivation
*��Ա:hdl	Attribute Handle����
			buf	Attribute Value����
*˵����	���ṹ���Ӧ��Handle Value Notification�Ĳ��������ݹ淶buf���ֵΪATT_MTU�
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_hdl_val_notifivation{
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-3];
};



/*************************************************************************************************
*����������att_hdl_val_indication
*��Ա:hdl	Attribute Handle����
			buf	Attribute Value����
*˵����	���ṹ���Ӧ�Handle Value Indication�Ĳ��������ݹ淶buf���ֵΪATT_MTU�
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct att_hdl_val_indication{
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-3];
};




/*************************************************************************************************
*�ṹ������attc_ble_evt
*��Ա�� attc_code	Э��ջ�ϱ�att�ͻ����¼��ı�ʶ�루code)
				attc_sz Э��ջ�ϱ�att�ͻ����¼��ĳ���
				evt	Э��ջ�ϱ�att�¼��ľ������ݣ���һ�������壬�����и����¼�������
					AttErrRsp	ATT�ͻ��˴����¼�������
					AttMtuRsp	ATT�ͻ��˽���MTU�¼�������
					AttFindInfoRsp	ATT�ͻ����ϱ�Find Information Response�¼�������
					AttFindByTypeValRsp	ATT�ͻ����ϱ�Read By Type Response�¼�������
					AttReadByTypeRsp	ATT�ͻ����ϱ�Read By Type Response�¼������ݣ�������ֻ������characteristic
					AttReadByTypeIncludeRsp	ATT�ͻ����ϱ�Read By Type Response�¼������ݣ�������ֻ�����ڰ�������
					AttReadByTypeValRsp	ATT�ͻ����ϱ�Read By Type Response�¼������ݣ�������ֻ�����ڵ���pairֵ
					AttReadByTypeServiceRsp	ATT�ͻ����ϱ�Read By Type Response�¼������ݣ�������ֻ�����ڷ���
					AttReadByTypeChartExtendRsp	ATT�ͻ����ϱ�Read By Type Response�¼������ݣ�������ֻ��������չ
																			characteristic
					AttReadRsp	ATT�ͻ����ϱ�Read Response�¼�������
					AttReadBlobRsp	ATT�ͻ����ϱ�Read Blob Response�¼�������
					AttReadMultipleRsp	ATT�ͻ����ϱ�Read Multiple Response�¼�������
					AttReadByGroupTypeRsp	ATT�ͻ����ϱ�Read by Group Type Response�¼�������
					AttHdlValNotification	ATT�ͻ����ϱ�Handle Value Notification�¼�������
					AttHdlValIndication	ATT�ͻ����ϱ�Handle Value Indication�¼�������
*˵���� SYD8821���յ�����ATT�ͻ��˵��¼���ʱ��ͨ��gap_s_att_c_evt_handler_set���õĽӿ��ϱ������¼���
attc_ble_evt�ṹ��Ϊ��ATT�ͻ������¼��ľ�������
				�ýṹ������GATT�ͻ��ˣ�����GATT�Ľ�ɫΪ����������ô�ýṹ�������õġ�SYD8821ֻ���ṩATT�ͻ�
����Ӧ��API����û���ṩGAP�Ĵ������̣�������APP���������
**************************************************************************************************/
struct attc_ble_evt {
	uint8_t	attc_code;
	uint8_t	attc_sz;
	union
	{
		struct att_err_rsp AttErrRsp;
		struct att_mtu_rsp AttMtuRsp;
		struct att_find_info_rsp AttFindInfoRsp;
		struct att_find_by_type_val_rsp AttFindByTypeValRsp;
		struct att_read_by_type_rsp AttReadByTypeRsp;
		struct att_read_by_type_include_rsp AttReadByTypeIncludeRsp;
		struct att_read_by_type_val_rsp AttReadByTypeValRsp;
		struct att_read_by_type_service_rsp AttReadByTypeServiceRsp;
		struct att_read_by_type_chartextend_rsp AttReadByTypeChartExtendRsp;
		struct att_read_rsp AttReadRsp;
		struct att_read_blob_rsp AttReadBlobRsp;
		struct att_read_multiple_rsp AttReadMultipleRsp;
		struct att_read_by_group_type_rsp AttReadByGroupTypeRsp;
		struct att_hdl_val_notifivation AttHdlValNotification;
		struct att_hdl_val_indication AttHdlValIndication;
	} attc; 
};




/*************************************************************************************************
*�ṹ������gap_att_report_handle
*��Ա�� cnt	BLE��report(notify����indicate)�ĸ�����Ҳ����CCCD�������ĸ���
				report	BLE��report(notify����indicate)�ľ������ݣ����������Ϊ20
*˵���� SYD8821��profile��notify��indicate����GATT service��Ϊ���ϳ�Ϊreport��ר�������ṹ��
gap_att_report_handle������report����Ӧ������Ϣ��ÿ��CCCD��Ӧ�Žṹ��gap_att_report_handle�е�һ��
�����Ա��������CCCD���ڵ�λ�ã�����Ҫ�������ԣ�val_hdl����
**************************************************************************************************/
struct gap_att_report_handle {
	uint8_t	cnt;
	struct	gap_att_report report[MAX_ATT_REPORT_HDL];
};




/*************************************************************************************************
*�ṹ������gap_bond_dev
*��Ա�� addr	BLE���豸�ĵ�ַ
				key	BLE����Ϣ�ľ����������Ժ��������ܳף�
*˵���� BLE�����֮�������Ҫ���ܻ���а󶨵Ĳ����������ѱ��ṹ����оƬ�ڲ���flash��
����ͨ��api:bm_s_bond_info_get��ȡ����Ϣ
**************************************************************************************************/
struct gap_bond_dev {			
	struct gap_ble_addr 		addr;
	struct gap_key_params	key;	
};




/*************************************************************************************************
*�ṹ������gap_wakeup_config
*��Ա�� timer_wakeup_en	��ʱ���Ƿ���Ϊ˯�ߺ�Ļ���Դ
				gpi_wakeup_en	GPIO�Ƿ���Ϊ˯�ߺ�Ļ���Դ
				gpi_wakeup_cfg	������Ϊ˯�ߺ�Ļ���Դ��GPIO
				gpi_wakeup_pol	������Ϊ˯�ߺ�Ļ���Դ��GPIO�ļ���
*˵���� ���ṹ����������SYD8821˯�ߵĻ���Դ������ֻ�ǶԶ�ʱ����gpio�Ļ���Դ�������ã�����˯�ߵĻ���
Դ������Ҫ���SystemPowerDown�Լ�SystemSleep�����ĵ��ò���
				gpi_wakeup_pol�������û���Դ��GPIO�ļ��ԣ�ͬʱGPIOģ��Ҳ��GPIOINT_POL_SET�ļ������ã���������
�����Ƕ����ģ�����Դ������ֻ�Ǻ�gpi_wakeup_pol�йأ���GPIOģ��������޹�
**************************************************************************************************/
struct gap_wakeup_config {
	uint8_t timer_wakeup_en;
	uint8_t gpi_wakeup_en;
	uint32_t gpi_wakeup_cfg;
	uint32_t gpi_wakeup_pol;
};




/*************************************************************************************************
*ö������POWER_SAVING_TYPE
*��Ա�� POWER_SAVING_RC_ON	˯�ߺ��ƵRCʱ�����ɴ�
				POWER_SAVING_RC_OFF	˯�ߺ��ƵRCʱ�ӽ����ر�
				POWER_SAVING_DSLEEP_LPO_ON_RETAIN	˯�ߺ��ƵRC(LPO)���ɴ򿪣����һ��Ѻ����˯��ǰ��λ��ִ��
				POWER_SAVING_DSLEEP_LPO_OFF_RETAIN	˯�ߺ��ƵRC(LPO)���رգ����һ��Ѻ����˯��ǰ��λ��ִ��
				POWER_SAVING_DSLEEP_LPO_ON_RESET	˯�ߺ��ƵRC(LPO)���ɴ򿪣����һ��Ѻ󽫻Ḵλ
				POWER_SAVING_DSLEEP_LPO_OFF_RESET	˯�ߺ��ƵRC(LPO)���رգ����һ��Ѻ󽫻Ḵλ
				POWER_SAVING_TYPE_NUM	ʡ��ģʽ������
*˵���� ʡ��ģʽ���;�����ƵRC�����͵�ƵRC������˯�ߺ��Ƿ�򿪣�����Խ��Խʡ�磬��������LPO��
ģʽ��RC�����Ǳ��رյģ���������POWER_SAVING_DSLEEP_LPO_ON_RETAIN��ô�����Ҳ������
POWER_SAVING_RC_OFF
				���С�DSLEEP����������ζ�Ž���������˯��ģʽ
				���С�RESET����������ζ�Ż��Ѻ󽫻Ḵλ���ڴ�Ƚ���������
**************************************************************************************************/
typedef enum {
	POWER_SAVING_RC_ON 	               = 0,
	POWER_SAVING_RC_OFF	               = 1,
	POWER_SAVING_DSLEEP_LPO_ON_RETAIN  = 2,
	POWER_SAVING_DSLEEP_LPO_OFF_RETAIN = 3,
  POWER_SAVING_DSLEEP_LPO_ON_RESET   = 4,
	POWER_SAVING_DSLEEP_LPO_OFF_RESET  = 5,
  POWER_SAVING_TYPE_NUM,    
} POWER_SAVING_TYPE;




/*************************************************************************************************
*ö������MODULE_CONTROL_TYPE
*��Ա�� NO_MODULE	˯�ߺ󲻹ر��κ�ģ��
				PER_MODULE	˯�ߺ�ر�����ģ�飬����GPIO SPI iic�ȣ����ø�ģ�黽�Ѻ����Ҫ������Щģ��
				BLE_MODULE	˯�ߺ�ر�BLEģ�飬�����Ϻ��ܹ����ã���ʱ�ڲ��㲥��ʱ���������
				FLASH_LDO_MODULE	˯�ߺ�ر�FLASH_LDO,Ҳ����FLASH�ĵ�Դ
				PB_MODULE	ΪPER_MODULE��BLE_MODULE�ļ���
				PF_MODULE	ΪPER_MODULE��FLASH_LDO_MODULE�ļ���
				PBF_MODULE	ΪPER_MODULE��FLASH_LDO_MODULE�Լ�FLASH_LDO_MODULE�ļ���
*˵���� ģ������������ڿ�����˯�ߺ�ĳЩģ��Ŀ���
				FLASH_LDO_MODULEģʽ�»��ѵ�ʱ�����Ҫ��flash�㹻���¼���Ҳ����SystemSleep����ĵ���������
**************************************************************************************************/
typedef enum {
    NO_MODULE            = 0,
    PER_MODULE           = BIT0,
    BLE_MODULE           = BIT1,
    FLASH_LDO_MODULE     = BIT2,
    PB_MODULE            = PER_MODULE | BLE_MODULE,
    PF_MODULE            = PER_MODULE | FLASH_LDO_MODULE,
    BF_MODULE            = BLE_MODULE | FLASH_LDO_MODULE,
    PBF_MODULE           = PER_MODULE | BLE_MODULE | FLASH_LDO_MODULE,
} MODULE_CONTROL_TYPE;




/*************************************************************************************************
*ö������PMU_WAKEUP_CONFIG_TYPE
*��Ա�� PIN_WAKE_EN	GPIO�Ƿ���Ϊ˯�ߺ�Ļ���Դ
				TIMER_WAKE_EN	��ʱ���Ƿ���Ϊ˯�ߺ�Ļ���Դ
				FSM_SLEEP_EN	�����¼��Ƿ���Ϊ˯�ߺ�Ļ���Դ��ֻҪʹ��BLE���뿪���ù���
				ANA_WAKE_EN	ADC�Ƚ����Ƿ���Ϊ˯�ߺ�Ļ���Դ
				RTC_WAKE_EN	RTC�Ƿ���Ϊ˯�ߺ�Ļ���Դ,rtc�����ֻ���Դ
				WDT_WAKE_EN	���Ź��Ƿ���Ϊ˯�ߺ�Ļ���Դ,SYD8821�Ŀ��Ź����жϹ��ܣ�ʹ���жϱ��뿪���ù���
				CAPDET_WAKE_EN	SYD8821����ģ���Ƿ���Ϊ˯�ߺ�Ļ���Դ,
				KEEP_ORIGINAL_WAKEUP	
*˵���� ���ѿ������������ĸ�ģ���ܹ���Ϊ����Դ��gap_wakeup_configҲ��GPIO��TIMER��ʹ�ܣ��������õ�
������Ч����SystemSleep����Ļ���Դ
**************************************************************************************************/
typedef enum {
    PIN_WAKE_EN          = BIT0,
    TIMER_WAKE_EN        = BIT1,
    FSM_SLEEP_EN         = BIT2,
    ANA_WAKE_EN          = BIT3,
    RTC_WAKE_EN          = BIT4,
    WDT_WAKE_EN          = BIT5,
    CAPDET_WAKE_EN       = BIT6,
    KEEP_ORIGINAL_WAKEUP = BIT7,
} PMU_WAKEUP_CONFIG_TYPE;


typedef enum {
	BLE_TX_POWER_MINUS_31_DBM	= 0,
	BLE_TX_POWER_MINUS_25_DBM	= 1,
	BLE_TX_POWER_MINUS_19_DBM	= 2,
	BLE_TX_POWER_MINUS_13_DBM	= 3,
	BLE_TX_POWER_MINUS_8_DBM	= 4,
	BLE_TX_POWER_MINUS_3_DBM	= 5,
	BLE_TX_POWER_0_DBM    		= 6,
	BLE_TX_POWER_2_DBM    		= 7,
	BLE_TX_POWER_4_DBM    		= 8,
} BLE_TX_POWER;



/*************************************************************************************************
*ö������AMIC_BIAS
*��Ա��  AMIC_BIAS_2_94	amic�� adc bias Ϊ2.94V
		AMIC_BIAS_2_5	amic�� adc bias Ϊ2.5V
		AMIC_BIAS_1_4	amic�� adc bias Ϊ1.4V
		AMIC_BIAS_1_25	amic�� adc bias Ϊ1.25V
*˵���� ����AMIC��bias������amic_set_bias����Ĳ���
**************************************************************************************************/
typedef enum {
	AMIC_BIAS_2_94	= 0,
	AMIC_BIAS_2_5	= 1,
	AMIC_BIAS_1_4	= 2,
	AMIC_BIAS_1_25	= 3,
} AMIC_BIAS;


#pragma pack()

/*************************************************************************************************
*��������gap_s_ble_init
*�����������
*���������uint8_t -->��ʼ��Э��ջ�Ľ��
						0	-->Э��ջ��ʼ��ʧ��
						1	-->Э��ջ��ʼ���ɹ�
*˵���� 1.Э��ջ��ʼ��������ѵײ��RF��صļĴ������г�ʼ��
				2.�ú�����Ѻ�BLEЭ����ص����ݽ��г�ʼ��������profile����ȫ���������Ӳ�����
**************************************************************************************************/
extern uint8_t gap_s_ble_init(void);




/*************************************************************************************************
*��������gap_s_disconnect
*�����������
*�����������
*˵���� �ú����ᷢ��LL_TERMINATE_IND,
				����Э�������յ���������ظ�һ���հ���������߽��������߽�����Э��ջ���ϱ�һ�������¼�
**************************************************************************************************/
extern void gap_s_disconnect(void);




/*************************************************************************************************
*��������gap_s_validate_irk
*�����������
*�����������
*˵���� �������˽�е�ַ��irk
**************************************************************************************************/
extern uint8_t gap_s_validate_irk(uint8_t *irk);




/*************************************************************************************************
*��������gap_s_ble_gen_random_private_address
*�������:struct gap_ble_addr * rpa	���ص�ַ��ָ��
*�������:��
*˵��:��ȡ˽�������ַ
**************************************************************************************************/
extern void gap_s_ble_gen_random_private_address(struct gap_ble_addr * rpa);




/*************************************************************************************************
*��������gap_s_ble_address_set
*�������:struct gap_ble_addr* p_dev	���õ�ַ��ָ��
*�������:��
*˵��:�����豸��ַ
			�������������ӿڣ�BLE�����õ�ַ���Ըõ�ַΪ׼���������������ӿڣ�BLE���豸��ַ����������
			����Ϊ׼
			ע�⣺���������ĳ���������øýӿڣ���ô������Ʒ�ĵ�ַ��һ��
**************************************************************************************************/
extern void gap_s_ble_address_set(struct gap_ble_addr* p_dev);




/*************************************************************************************************
*��������gap_s_ble_address_get
*�������:struct gap_ble_addr* p_dev	��ȡ��ַ��ָ��
*�������:��
*˵��:��ȡ�豸�ĵ�ַ
**************************************************************************************************/
extern void gap_s_ble_address_get(struct gap_ble_addr* p_dev);




/*************************************************************************************************
*��������gap_s_ble_feature_set
*�������:uint8_t *p_feature	����feature��ָ�� ���ݹ淶Ҫ������Ĵ�������ݳ���Ӧ��Ϊ8
*�������:��
*˵��:�ú������������˵�feature������ڸ��BLE5.0��BLE4.0ֻ��һ��bit�Ļ��������ӵ���16��bit
			SYD8821�����Ĭ��ֵ��0x0000000000000133
			�ú���ֻ����SYD8821��������ģʽ������
**************************************************************************************************/
extern void gap_s_ble_feature_set(uint8_t *p_feature);




/*************************************************************************************************
*��������gap_s_ble_feature_get
*�������:uint8_t *p_feature	���ر�����feature��ָ�� ���ݹ淶Ҫ������᷵��8��byte������
*�������:��
*˵��:�ú�����ȡ�����˵�feature������ڸ��BLE5.0��BLE4.0ֻ��һ��bit�Ļ��������ӵ���16��bit
			SYD8821�����Ĭ��ֵ��0x0000000000000133
			�ú���ֻ����SYD8821��������ģʽ������
**************************************************************************************************/
extern void gap_s_ble_feature_get(uint8_t *p_feature);




/*************************************************************************************************
*��������gap_s_ble_feature_set
*�������:uint8_t *p_feature	����feature��ָ�� ���ݹ淶Ҫ������Ĵ�������ݳ���Ӧ��Ϊ8
*�������:��
*˵��:�ú������ôӻ��˵�feature������ڸ��BLE5.0��BLE4.0ֻ��һ��bit�Ļ��������ӵ���16��bit
			SYD8821�����Ĭ��ֵ��0x0000000000000133
			�ú���ֻ����SYD8821���ڴӻ�ģʽ������
**************************************************************************************************/
extern void gap_s_ble_slave_feature_set(uint8_t *p_feature);




/*************************************************************************************************
*��������gap_s_ble_feature_get
*�������:uint8_t *p_feature	���ر�����feature��ָ�� ���ݹ淶Ҫ������᷵��8��byte������
*�������:��
*˵��:�ú�����ȡ�ӻ��˵�feature������ڸ��BLE5.0��BLE4.0ֻ��һ��bit�Ļ��������ӵ���16��bit
			SYD8821�����Ĭ��ֵ��0x0000000000000133
			�ú���ֻ����SYD8821���ڴӻ�ģʽ������
**************************************************************************************************/
extern void gap_s_ble_slave_feature_get(uint8_t *p_feature);




/*************************************************************************************************
*��������gap_s_adv_access_code_set
*�������:uint8_t *p_acc	���ù㲥ͨ�����ʵ�ַָ�루4byte��
*�������:��
*˵��:���ù㲥ͨ���ķ��ʵ�ַ�����������������ʹ��
**************************************************************************************************/
extern void gap_s_adv_access_code_set(uint8_t *p_acc);





/*************************************************************************************************
*��������gap_s_adv_access_code_get
*�������:uint8_t *p_acc	���ع㲥ͨ�����ʵ�ַָ�루4byte��
*�������:��
*˵��:��ȡ�㲥���ʵ�ַ���ڲ�����gap_s_adv_access_code_set������·���0x8E89BED6,����������²�����
			���øýӿ�
**************************************************************************************************/
extern void gap_s_adv_access_code_get(uint8_t *p_acc);





/*************************************************************************************************
*��������gap_s_adv_parameters_set
*�������:struct gap_adv_params * p_adv	�㲥����ָ��
*�������:��
*˵��:���ù㲥��������������뿴gap_adv_params�ṹ��
**************************************************************************************************/
extern void gap_s_adv_parameters_set(struct gap_adv_params * p_adv);





/*************************************************************************************************
*��������gap_s_adv_data_set
*�������:uint8_t *p_adv	�㲥����ָ��
					uint8_t adv_sz	�㲥���ݴ�С
					uint8_t *p_scan	ɨ����Ӧ����ָ��
					uint8_t sacn_sz	ɨ����Ӧ���ݴ�С
*�������:��
*˵��:�ú���ֻ�ǰѹ㲥���ݵ����ñ��浽Э��ջ�ײ㻺��������û��������Ч��������Ч���ڵ�gap_s_adv_start
			������ʱ��
			��Ϊ��������ɨ���ʱ�������ӻ���ʱ��̣ܶ�����������ǰ������׼���ã�������ɨ�������������������
**************************************************************************************************/
extern void gap_s_adv_data_set(uint8_t *p_adv, uint8_t adv_sz,uint8_t *p_scan, uint8_t sacn_sz);





/*************************************************************************************************
*��������gap_s_adv_start
*�������:��
*�������:��
*˵��:���øú�����ʼ�㲥����gap_s_adv_data_set�������������ݷ��͵��㲥ͨ��
**************************************************************************************************/
extern void gap_s_adv_start(void);





/*************************************************************************************************
*��������gap_s_adv_stop
*�������:��
*�������:��
*˵��:���øú���ֹͣ�㲥
**************************************************************************************************/
extern void gap_s_adv_stop(void);





/*************************************************************************************************
*��������gap_s_scan_parameters_set
*�������:struct gap_scan_params*p_scan	ɨ���������
*�������:��
*˵��:���������������������뿴gap_scan_params�ṹ�� ע�⣺����ɨ��Ķ����͹㲥���ܹ�ͬʱ����
**************************************************************************************************/
extern void gap_s_scan_parameters_set(struct gap_scan_params*p_scan);





/*************************************************************************************************
*��������gap_s_scan_start
*�������:��
*�������:��
*˵��:���øú�����ʼɨ��
**************************************************************************************************/
extern void gap_s_scan_start(void);





/*************************************************************************************************
*��������gap_s_adv_state_get
*�������:��
*�������:uint8_t	�����㲥״̬	�÷���ֵΪö��_ADV_SCAN_MODE_�ĳ�Ա
*˵��:���øú����ɻ�ȡ�ײ������㲥��״̬���㲥״̬��Ϊ����״̬���㲥״̬��ɨ��״̬�ȣ�����ɿ���ö�٣�
_ADV_SCAN_MODE_
**************************************************************************************************/
extern uint8_t gap_s_adv_state_get(void);





/*************************************************************************************************
*��������gap_s_scan_stop
*�������:��
*�������:��
*˵��:���øú���ֹͣ�㲥
**************************************************************************************************/
extern void gap_s_scan_stop(void);





/*************************************************************************************************
*��������gap_s_coex_adv_data_set
*�������:uint8_t advtype	����״̬�¹㲥������
					uint8_t *buf		����״̬�¹㲥������ָ��
					uint8_t sz			����״̬�¹㲥�����ݵĴ�С
					uint8_t *p_addr	����״̬�¹㲥�ĵ�ַָ��
					uint8_t addr_type	����״̬�¹㲥�ĵ�ַ����
*�������:��
*˵��:�ú��������gap_s_adv_data_set�еĲ�֮ͬ�����ڸú�����������״̬�е��ã�����״̬�з���Ĺ㲥
			�������ӣ���ΪSYD8821�Ĵӻ�Ŀǰ����֧�ֻ��״̬���Ļ���
**************************************************************************************************/
extern void gap_s_coex_adv_data_set(uint8_t advtype, uint8_t *buf, uint8_t sz, uint8_t *p_addr, uint8_t addr_type);





/*************************************************************************************************
*��������gap_s_coex_scan_rsp_data_set
*�������:uint8_t *buf		����״̬��ɨ����Ӧ������ָ��
					uint8_t sz			����״̬��ɨ����Ӧ�����ݵĴ�С
					uint8_t *p_addr	����״̬��ɨ����Ӧ�ĵ�ַָ��
					uint8_t addr_type	����״̬��ɨ����Ӧ�ĵ�ַ����
*�������:��
*˵��:�ú��������gap_s_adv_data_set�е�ɨ����Ӧ���ݲ�֮ͬ�����ڸú�����������״̬�е���
**************************************************************************************************/
extern void gap_s_coex_scan_rsp_data_set(uint8_t *buf, uint8_t sz, uint8_t *p_addr, uint8_t addr_type);





/*************************************************************************************************
*��������gap_s_coex_adv_start
*�������:��
*�������:��
*˵��:���øú�����ʼ�㲥����gap_s_coex_adv_data_set�������������ݷ��͵��㲥ͨ��
			�ú��������gap_s_adv_start�Ĳ�֮ͬ�����ڸú�����������״̬�е���
**************************************************************************************************/
extern void gap_s_coex_adv_start(void);





/*************************************************************************************************
*��������gap_s_coex_adv_stop
*�������:��
*�������:��
*˵��:���øú���ֹͣ�㲥
			�ú��������gap_s_adv_start�Ĳ�֮ͬ�����ڸú�����������״̬�е���
**************************************************************************************************/
extern void gap_s_coex_adv_stop(void);





/*************************************************************************************************
*��������gap_s_coex_scan_req_data_set
*�������:uint8_t *buf		����״̬��ɨ�����������ָ��
*�������:��
*˵��:�ú�����������״̬�µ�ɨ���������ݣ����õ������������еĹ㲥��ַ
**************************************************************************************************/
extern void gap_s_coex_scan_req_data_set(uint8_t *buf);





/*************************************************************************************************
*��������gap_s_coex_scan_start
*�������:��
*�������:��
*˵��:���øú�����ʼɨ��
			�ú��������gap_s_scan_start�Ĳ�֮ͬ�����ڸú�����������״̬�е���
**************************************************************************************************/
extern void gap_s_coex_scan_start(void);





/*************************************************************************************************
*��������gap_s_coex_scan_stop
*�������:��
*�������:��
*˵��:���øú���ֹͣɨ��
			�ú��������gap_s_scan_stop�Ĳ�֮ͬ�����ڸú�����������״̬�е���
**************************************************************************************************/
extern void gap_s_coex_scan_stop(void);





/*************************************************************************************************
*��������gap_s_gatt_profiles_set
*�������:struct gap_profile_struct *p_gatt_profile profileԪ��(��:�������ԡ�ֵ�Լ�report_handle)
*�������:��
*˵��:ָ��profile����Ԫ�ص�λ�ã��ײ�Э��ջ��ȥp_gatt_profile�ṹ��ָ����λ��ȥ��ȡprofile
**************************************************************************************************/
extern void gap_s_gatt_profiles_set(struct gap_profile_struct *p_gatt_profile);





/*************************************************************************************************
*��������gap_s_security_parameters_set
*�������:struct gap_pairing_req *p_sec_params ������ȫ����
*�������:��
*˵��:����������ȫ��������Щ����Ӱ�쵽����ԵĹ���
**************************************************************************************************/
extern void gap_s_security_parameters_set(struct gap_pairing_req *p_sec_params);





/*************************************************************************************************
*��������gap_s_security_parameters_get
*�������:struct gap_pairing_req *p_sec_params ������ȫ����
*�������:��
*˵��:��ȡ������ȫ����
**************************************************************************************************/
extern void gap_s_security_parameters_get(struct gap_pairing_req *p_sec_params);





/*************************************************************************************************
*��������gap_s_security_req
*�������:uint8_t flag -->�Ƿ��
						0	-->	��Ժ󲻰�
						1	-->	��Ժ��
					uint8_t mitm	-->�м��˱���
						0	-->	û���м��˱���
						1	-->	��Ҫ�м��˱�������������ȣ�
*�������:��
*˵��:�����м��˱��������������뻹��ODB���ַ�ʽ
**************************************************************************************************/
extern void gap_s_security_req(uint8_t flag, uint8_t mitm);





/*************************************************************************************************
*��������gap_s_connection_param_set
*�������:struct gap_connection_param_rsp_pdu *p_connection_params	��������������Ӧ�����ṹ��ָ��
*�������:��
*˵��:��������������Ӧ�����������BLE4.1���ϵİ汾֧�ֵĽӿ�
			�ú���ֻ�ǰ����Ӳ��������ñ��浽Э��ջ�ײ㻺��������û��������Ч��������Ч������������
			LL_CONNECTION_PARAM_REQ��ʱ��
**************************************************************************************************/
extern void gap_s_connection_param_set(struct gap_connection_param_rsp_pdu *p_connection_params);





/*************************************************************************************************
*��������gap_s_connection_param_get
*�������:struct gap_connection_param_rsp_pdu *p_connection_params	�������Ӳ����ṹ��ָ��
*�������:��
*˵��:��ȡ����������Ӧ�����������BLE4.1���ϵİ汾֧�ֵĽӿ�
**************************************************************************************************/
extern void gap_s_connection_param_get(struct gap_connection_param_rsp_pdu *p_connection_params);





/*************************************************************************************************
*��������gap_s_connection_update
*�������:struct gap_update_params *p_update_params	�������Ӳ�������ṹ��ָ��
*�������:��
*˵��:�����������Ӳ����������뿴gap_update_params�ṹ��
			ע�⣺ֻ���ڴӻ�����״̬�²��ܹ��������Ӳ���
			�ú���ֻ�Ƿ������Ӳ����������󣬲�û��������Ӧ�Ļ������ж���Щ���Ӳ��������Ƿ�����ܹ���Ч��
��Ȼ�ú�����
			��Ȼ���Ӳ���������latency�������������SYD8821��Э��ջ��û�ж�latency���й�����ʱʹ�ܺ�ʱ
ʧ�ܲ�û����gap_s_connection_update�����Ĺ���Χ֮��
			SYD8821�������Ӳ����Ĺ���ר�ų�����ר�ŵ����Ӳ���������ƣ�smart_update_latency
**************************************************************************************************/
extern void gap_s_connection_update(struct gap_update_params *p_update_params);





/*************************************************************************************************
*��������gap_s_link_parameters_get
*�������:struct gap_link_params* p_link	������·�����ṹ��ָ��
*�������:��
*˵��:��ȡ����������·�����������뿴gap_link_params�ṹ��
			��·������˫�������Ӳ��������õ��Ľ����������������������õ����Ӳ���
**************************************************************************************************/
extern void gap_s_link_parameters_get(struct gap_link_params* p_link);





/*************************************************************************************************
*��������gap_s_passkey_set
*�������:uint32_t passkey	��Թ����е����룬������Ϊʮ�������ݣ��������ﴫ��passkey=123456��
														��ô��������Ӧ�����ַ���123456��
*�������:��
*˵��:�����м��˱�������Թ����ڿ�ʼ��Ե�ʱ����ϱ�GAP_EVT_SHOW_PASSKEY_REQ�¼���app����Ҫ�ڸ���
			���е���gap_s_passkey_set�ӿ���������
**************************************************************************************************/
extern void gap_s_passkey_set(uint32_t passkey);





/*************************************************************************************************
*��������gap_s_evt_handler_set
*�������:struct gap_evt_callback* p_callback	����Э��ջ�ϱ��ṹ��
*�������:��
*˵��:�ú���Ϊapp��������Э��ջ��GAP���ϱ������¼��Ľӿڣ�Ҳ��GAP���Ϻ�APP�����Ľӿ�
			gap_evt_callback����������Ա������evt_mask��Ա�ܹ����ε�����Ҫ���¼���������Ӧ��bitmask�ܹ�
			�赲GAP���ϱ���
			gap_evt_callback��p_callback��Ա������GAP�ϱ��¼���APP�Ľӿ�ָ�룬�ýӿڵĲ�����һ��
			gap_ble_evt�ṹ�壬��Э��ջ��������״̬�б仯ʱ��
			GAP�����øú���ͬʱ����һ��gap_ble_evt�ṹ�壬APP��p_callback�����и���gap_ble_evt�ṹ�崦
			����������¼���
**************************************************************************************************/
extern void gap_s_evt_handler_set(struct gap_evt_callback* p_callback);





/*************************************************************************************************
*��������gap_s_att_c_evt_handler_set
*�������:void* p_callback	����Gatt��Client�¼��ϱ��ĺ���ָ��
*�������:��
*˵��:�ú���Ϊapp��������Gatt��Client�ϱ�attc�¼��Ľӿڣ���Ϊ�ӻ�һ���������server�ˣ�����GATT���¼���
�����ൽGAP���¼��У����ǵ�������һ���ӿ�
			p_callbackָ�������һ��attc_ble_evt�ṹ�壬��Э��ջ����GATT��Client�б仯ʱ�����������˶���ĳ��
ATTָ����������Ӧ������øú�����ͬʱ����һ��attc_ble_evt�ṹ�壬APP��p_callback�����и��ݸ�attc_ble_evt
�ṹ�崦����������¼���
**************************************************************************************************/
extern void gap_s_att_c_evt_handler_set(void* p_callback);





/*************************************************************************************************
*��������gap_s_gatt_report_handle_get
*�������:struct gap_att_report_handle** p_hdl	����profile��report_handle�Ķ���ָ��
*�������:��
*˵��:notify����indicate����Ϊ����Ҫ���´�gap_att_report�ṹ�壬�ýṹ��ָ���˷��͵�uuid��hdl�ȣ�����Щ
��Ϣ���洢��_gatt_database_report_handle�ṹ���У�
			��Ȼ���øú�����ֱ�Ӷ�ȡ_gatt_database_report_handleЧ����һ����
**************************************************************************************************/
extern void gap_s_gatt_report_handle_get(struct gap_att_report_handle** p_hdl);





/*************************************************************************************************
*��������gap_s_gatt_read_rsp_set
*�������:uint8_t len	ָ������Ӧ�ĳ���
					uint8_t *p_data ����Ӧ�ľ�������
*�������:��
*˵��:�ú��������˶���Ӧ�ķ������ݣ�Ҳ���������˶�ȡ��Ӧ���Եõ�������
			���øú�����ʱ��Э��ջ���ܲ����ܹ����������ݷ��ͳ�ȥ��ֻ�ǰ����ݷŵ��ײ㷢�Ͷ����У�����ײ㻺����
�������ݴ����ͣ��������ΪҪ�ȴ�֮ǰ�����ݷ������
**************************************************************************************************/
extern void gap_s_gatt_read_rsp_set(uint8_t len,uint8_t *p_data);





/*************************************************************************************************
*��������gap_s_gatt_write_err_rsp_set
*�������:uint16_t hdl	 att��Error Response��Ӧ��Attribute Handle In Error����
					uint8_t err att��Error Response��Ӧ��Error Code����
*�������:��
*˵��:�ú�����Ҫ��Ŀ������APPӵ����Э��ջ�ײ�ظ��������������app�����д�ʱ���Ե��øýӿ�
**************************************************************************************************/
extern void gap_s_gatt_write_err_rsp_set(uint16_t hdl, uint8_t err);





/*************************************************************************************************
*��������gap_s_check_fifo_sz
*�������:��
*�������:uint16_t	ָʾ�ײ��fifo��ʣ���С
*˵��:����SYD8821���������ݶ�������ڵײ��fifo�У��ú����ܹ���ȡ�ײ�fifo��ʣ��Ĵ�С
**************************************************************************************************/
extern uint16_t gap_s_check_fifo_sz(void);





/*************************************************************************************************
*��������gap_s_gatt_data_send
*�������:uint8_t type ���ݷ��͵����ͣ�ΪBLE_SEND_TYPEö�ٳ�Ա��������notify��ʽ��BLE_GATT_NOTIFICATION��
											��Ҳ������ָʾ��ʽ��BLE_GATT_INDICATION��
					struct gap_att_report* p_report	ָ��Ҫ���͵�ͨ���������ĳ�����ԣ��ķ��ͽṹ�壬�˽ṹ��Ϊ
																					_gatt_database_report_handle��ĳ����Ա
					uint8_t len	Ҫ���͵����ݵĳ���
					uint8_t *p_data	Ҫ���͵ľ�������
*�������:uint8_t	���صķ��ͽ�������ײ��fifo�Ѿ����������߹����indicate��û�еȵ����ص�ʱ�򣬸�ֵΪ0��
����Ϊ1�����ͳɹ�
*˵��:notify�ǲ���Ҫ���صķ���ģʽ����ģʽ���ܻ�����������ϲ㴦����ʱ���������ݵ��������indicate��
��Ҫ����ȷ�ϵķ�����ʽ��
			����������ɺ���Ҫ�ȴ�Э��ջ�ϱ�GAP_EVT_ATT_HANDLE_CONFIRMATION�¼�
**************************************************************************************************/
extern uint8_t gap_s_gatt_data_send(uint8_t type, struct gap_att_report* p_report, uint8_t len, uint8_t *p_data);







/*************************************************************************************************
*��������gap_s_gatt_data_send_report_confirmation_handle
*�������:uint8_t type ���ݷ��͵����ͣ�ΪBLE_SEND_TYPEö�ٳ�Ա��������notify��ʽ��BLE_GATT_NOTIFICATION��
											��Ҳ������ָʾ��ʽ��BLE_GATT_INDICATION��
					struct gap_att_report* p_report	ָ��Ҫ���͵�ͨ���������ĳ�����ԣ��ķ��ͽṹ�壬�˽ṹ��Ϊ
																					_gatt_database_report_handle��ĳ����Ա
					uint8_t len	Ҫ���͵����ݵĳ���
					uint8_t *p_data	Ҫ���͵ľ�������
*�������:gap_s_gatt_data_send_report_confirmation_handle��gap_s_gatt_data_send�÷�һ�£�Ψһ��������
����indicate����gap_s_gatt_data_send_report_confirmation_handle��Э��ջ�ϱ�GAP_EVT_ATT_HANDLE_CONFIRMATION�¼�
��ʱ���ѷ������ݵ����Ե�handle��ͬʱ�ϱ�������
uint8_t	���صķ��ͽ�������ײ��fifo�Ѿ����������߹����indicate��û�еȵ����ص�ʱ�򣬸�ֵΪ0��
����Ϊ1�����ͳɹ�
*˵��:notify�ǲ���Ҫ���صķ���ģʽ����ģʽ���ܻ�����������ϲ㴦����ʱ���������ݵ��������indicate��
��Ҫ����ȷ�ϵķ�����ʽ��
			����������ɺ���Ҫ�ȴ�Э��ջ�ϱ�GAP_EVT_ATT_HANDLE_CONFIRMATION�¼�
**************************************************************************************************/
extern uint8_t gap_s_gatt_data_send_report_confirmation_handle(uint8_t type, struct gap_att_report* p_report, uint8_t len, uint8_t *p_data);





/*************************************************************************************************
*��������gap_s_connection_latency_mode
*�������:uint8_t en	-->�Ƿ�ʹ��latency
										0-->�ر�latency
										1-->��latency
*�������:��
*˵��:�����������Ӻ������������Ӳ����е�interval��Ϊ�����ӻ�����һ�ν��������淶�й涨�ӻ��ܹ����Ե�һ��
������interval���ﵽ��ʡ���ĵ�Ŀ�ģ�
			�ܹ�����interval�ĸ�������latency�����ú���ָʾЭ��ջ�ײ�ʱ��ʹ��latency
**************************************************************************************************/
extern void gap_s_connection_latency_mode(uint8_t en);





/*************************************************************************************************
*��������gap_s_profile_data_read
*�������:uint16_t addr	ƫ�Ƶ�ַ����Ч��Χ��0x0-0xfff
					uint16_t len	��ȡ���ݵĳ��ȣ���Ч��Χ��0x0-0xfff
					uint8_t *p_buf	�������ݵ�ָ�룬SYD8821��flashҪ���ָ��ָ����ڴ������4�ֽڶ����
*�������:uint8_t	ָʾ��ȡ���ݵĽ��
									0	-->ʧ��
									1	-->�ɹ�
*˵��:SYD8821�ڲ���flash���˴洢����������ļ��⻹��������4K�������app�洢��Ҫ���ݣ���Ϊ��Щ�����Ǵ洢��
flash�еģ����Ե����Ҳ���ᶪʧ
			������Ĳ����Ƿ�ʱ���ú������ش��������ص�ע������ָ����������ֽڶ��䣬��������
**************************************************************************************************/
extern uint8_t gap_s_profile_data_read(uint16_t addr , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*��������gap_s_profile_data_write
*�������:uint16_t addr	ƫ�Ƶ�ַ����Ч��Χ��0x0-0xfff
					uint16_t len	д���ݵĳ��ȣ���Ч��Χ��0x0-0xfff
					uint8_t *p_buf	д�����ݵ�ָ�룬SYD8821��flashҪ���ָ��ָ����ڴ������4�ֽڶ����
*�������:uint8_t	ָʾд�����ݵĽ��
									0	-->ʧ��
									1	-->�ɹ�
*˵��:SYD8821�ڲ���flash���˴洢����������ļ��⻹��������4K�������app�洢��Ҫ���ݣ���Ϊ��Щ�����Ǵ洢��
flash�еģ����Ե����Ҳ���ᶪʧ
			������Ĳ����Ƿ�ʱ���ú������ش���,�����ص�ע������ָ����������ֽڶ��䣬��������
			��Ϊ�ú�����дflash֮ǰ���в���flash���������Խ��鲻ҪƵ�����øú������������flash����
			�ú���ֻ�ǰ����ݷŵ��ײ㻺�����У�����ִ��flash�Ĳ���������ble_sched_execute������
**************************************************************************************************/
extern uint8_t gap_s_profile_data_write(uint16_t addr , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*��������att_c_mtureq
*�������:uuint16_t mtu	�������õ�mtu��С
*�������:uint8_t	ָʾ Exchange MTU Requestָ���Ƿ��ͳɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã�SYD8821��ȫ֧�ֹ淶��l2cap��Ĺ��ܣ�����֧��app
����ı�mtu������Ҫ����������ݵ�ʱ����԰�MTU����
			���Է��յ�Exchange MTU Request�����Ӧ����Exchange MTU Response���ݰ�����SYD8821�յ������ݰ���
ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_mtureq(uint16_t mtu);





/*************************************************************************************************
*��������att_c_findinforeq
*�������:uint16_t start_hdl ����ATT��Find Information Requestָ���Starting Handle����
					uint16_t end_hdl	����ATT��Find Information Requestָ���Ending Handle����
*�������:uint8_t	ָʾ Find Information Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� 
Find Information Request������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ�att_c_findinforeq�����Ӧ����Find Information Response���ݰ�����SYD8821�յ������ݰ���
ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_findinforeq(uint16_t start_hdl, uint16_t end_hdl);





/*************************************************************************************************
*��������att_c_findbytypevaluereq
*�������:uint16_t start_hdl ����ATT��Find By Type Value Requestָ���Starting Handle����
					uint16_t end_hdl	����ATT��Find By Type Value Requestָ���Ending Handle����
					uint16_t type	����ATT��Find By Type Value Requestָ���Attribute Type����
					uint8_t val_sz	����ATT��Find By Type Value Requestָ���Attribute Value�����ĳ���
					uint8_t *p_val	����ATT��Find By Type Value Requestָ���Attribute Value����
*�������:uint8_t	ָʾ Find By Type Value Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� 
Find By Type Value Request������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ�Find By Type Value Request�����Ӧ����Find By Type Value Response���ݰ�����SYD8821�յ�
�����ݰ���ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_findbytypevaluereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type, uint8_t val_sz, uint8_t *p_val);





/*************************************************************************************************
*��������att_c_readbytypereq
*�������:uint16_t start_hdl ����ATT��Read By Type Requestָ���Starting Handle����
					uint16_t end_hdl	����ATT��Read By Type Requestָ���Ending Handle����
					uint16_t type_sz	����ATT��Read By Type Requestָ���Attribute Type�����ĳ��ȣ����ݹ淶����
														ֻ�ܹ����0����16
					uint8_t *p_type	����ATT��Read By Type Requestָ���Attribute Type����
*�������:uint8_t	ָʾ Read By Type Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Read By Type Request
������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ�Read By Type Request�����Ӧ����Read By Type Response���ݰ�����SYD8821�յ������ݰ���
ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_readbytypereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type_sz, uint8_t *p_type);





/*************************************************************************************************
*��������att_c_readreq
*�������:uint16_t hdl ����ATT��Read Requestָ���Attribute Handle����
*�������:uint8_t	ָʾ Read Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ��Read Request�������
������ľ������ݿɿ��淶��ATT������½�
			���Է��յ�Read Request�����Ӧ����Read Response���ݰ�����SYD8821�յ������ݰ���ʱ��Э��ջ��GATT��
����gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_readreq(uint16_t hdl);





/*************************************************************************************************
*��������att_c_readblobreq
*�������:uint16_t hdl ����ATT��Read Blob Requestָ���Attribute Handle����
					uint16_t offset ����ATT��Read Blob Requestָ���Value Offset����
*�������:uint8_t	ָʾ  Read Blob Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Read Blob Request���
���ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� Read Blob Request�����Ӧ����Read Blob Response���ݰ�����SYD8821�յ������ݰ���ʱ��Э��
ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_readblobreq(uint16_t hdl, uint16_t offset);





/*************************************************************************************************
*��������att_c_readmultiplereq
*�������:uint8_t hdl_sz ����ATT��Read Multiple Requestָ���Set Of Handles�����ĳ��ȣ�
												���ݹ淶�ò�������С��4
					uint8_t *p_hdl ����ATT��Read Multiple Requestָ���Set Of Handles����
*�������:uint8_t	ָʾ  Read Multiple Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Read Multiple Request
������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� �Read Multiple Request�����Ӧ����Read Multiple Response���ݰ�����SYD8821�յ������ݰ�
��ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_readmultiplereq(uint8_t hdl_sz, uint8_t *p_hdl);





/*************************************************************************************************
*��������att_c_readbygrouptypereq
*�������:uint16_t start_hdl ����ATT��Read by Group Type Requestָ���Starting Handle����
					uint16_t end_hdl ����ATT��Read by Group Type Requestָ���Ending Handle����
					uint16_t type_sz ����ATT��Read by Group Type Requestָ���Attribute Group Type�����Ĵ�С��
													���ݹ淶�ô�С����Ϊ2����16
					uint8_t *p_type	����ATT��Read by Group Type Requestָ���Attribute Group Type����
*�������:uint8_t	ָʾ  Read by Group Type Requesָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� 
Read by Group Type Request������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� Read by Group Type Request�����Ӧ����Read by Group Type Response���ݰ�����SYD8821
�յ������ݰ���ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_readbygrouptypereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type_sz, uint8_t *p_type);





/*************************************************************************************************
*��������att_c_writereq
*�������:uint16_t hdl ����ATT��Write Requestָ���Attribute Handle����
					uint16_t sz ����ATT��Write Requestָ���Attribute Value�����Ĵ�С
					uint8_t *p_buf	����ATT��Write Requestָ���Attribute Value����
*�������:uint8_t	ָʾ  Read Blob Requestָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Write Request�����
�ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� Write Request�����Ӧ����Write Response���ݰ�����SYD8821�յ������ݰ���ʱ��Э��ջ��
GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_writereq(uint16_t hdl, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*��������att_c_writecmdreq
*�������:uint16_t hdl ����ATT��Write Commandָ���Attribute Handle����
					uint16_t sz ����ATT��Write Commandָ���Attribute Value�����Ĵ�С
					uint8_t *p_buf	����ATT��Write Commandָ���Attribute Value����
*�������:uint8_t	ָʾ  Write Commandָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Write Command�������
������ľ������ݿɿ��淶��ATT������½�
			��ΪWrite Command���û����Ӧ���ݰ����������ﲻ���ϱ��κ��¼�
**************************************************************************************************/
extern uint8_t att_c_writecmdreq(uint16_t hdl, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*��������att_c_preparewritereq
*�������:uint16_t hdl ����ATT��Prepare Write Requestָ���Attribute Handle����
					uint16_t offset ����ATT��Prepare Write Requestָ���Value Offset����
					uint16_t sz ����ATT��Prepare Write Requestָ���Part Attribute Value�����Ĵ�С
					uint8_t *p_buf	����ATT��Write Commandָ���Part Attribute Value����
*�������:uint8_t	ָʾ  att_c_preparewritereqָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� Prepare Write Request
������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� Prepare Write Request�����Ӧ����WPrepare Write Response���ݰ�����SYD8821�յ������ݰ�
��ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_preparewritereq(uint16_t hdl, uint16_t offset, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*��������att_c_executewritereq
*�������:uint8_t flags ����ATT��Execute Write Requestָ���Flags���� 
										0x00 -->Cancel all prepared writes
										0x01 --> Immediately write all pending prepared values
*�������:uint8_t	ָʾ  att_c_preparewritereqָ���Ƿ��ͳɹ�
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� 
Execute Write Request������ڸ�����ľ������ݿɿ��淶��ATT������½�
			���Է��յ� Execute Write Request�����Ӧ����Execute Write Response���ݰ�����SYD8821�յ������ݰ�
��ʱ��Э��ջ��GATT�����
			gap_s_att_c_evt_handler_set����ָ���Ľӿ��ϱ����¼�
**************************************************************************************************/
extern uint8_t att_c_executewritereq(uint8_t flags);





/*************************************************************************************************
*��������att_c_confirmation
*�������:��
*�������:uint8_t	ָʾ  att_c_preparewritereqָ���Ƿ��ͳɹ� 
									0	-->ʧ��
									1	-->�ɹ�
*˵��:��ָ��ֻ���ڵ�ǰ��GATT�Ǵ���clientģʽ�²ſ�ʹ�ã����øýӿ�SYD8821���ᷢ�� 
			Handle Value Confirmation������ڸ�����ľ������ݿɿ��淶��ATT������½�
			��ΪWrite Command���û����Ӧ���ݰ����������ﲻ���ϱ��κ��¼�
**************************************************************************************************/
extern uint8_t att_c_confirmation(void);





/*************************************************************************************************
*��������bm_s_bond_manager_idx_set
*�������:uint8_t idx	���ð���Ϣ�洢λ������
*�������:��
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�������ʹ���ĸ�λ�õİ���Ϣ��ͨ���ú�������ʵ�ְ��豸
���л�
			��ΪĿǰSYD8821��Ϊ�ӻ�ֻ�ܹ�����һ������������ͬһʱ��ֻ�ܹ���һ������Ϣ
**************************************************************************************************/
extern void bm_s_bond_manager_idx_set(uint8_t idx);





/*************************************************************************************************
*��������bm_s_bond_manager_idx_get
*�������:uint8_t idx	��ȡ����Ϣ�洢λ������
*�������:��
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�����ȡ��ǰ����ʹ���ĸ�λ�õİ���Ϣ��
			��ΪĿǰSYD8821��Ϊ�ӻ�ֻ�ܹ�����һ������������ͬһʱ��ֻ�ܹ���һ������Ϣ
**************************************************************************************************/
extern void bm_s_bond_manager_idx_get(uint8_t *p_idx);





/*************************************************************************************************
*��������bm_s_bond_info_get
*�������:struct gap_bond_dev *p_device	����Ϣ�ṹ��
*�������:uint8_t ָʾ����Ϣ�Ƿ��ȡ�ɹ� 
										0	-->ʧ��
										1	-->�ɹ�
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�����ȡbm_s_bond_manager_idx_setָ��������λ�õľ����
��Ϣ����
			����Ϣ��������ο�gap_bond_dev�ṹ��
**************************************************************************************************/
extern uint8_t bm_s_bond_info_get(struct gap_bond_dev *p_device);





/*************************************************************************************************
*��������bm_s_bond_info_add
*�������:struct gap_bond_dev *p_device	����Ϣ�ṹ��
*�������:uint8_t ָʾ��Ӱ���Ϣ�Ƿ�ɹ�
					0	-->ʧ��
					1	-->�ɹ�
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�������һ�����豸����Ϊ���Ӱ󶨵Ĳ��������������ɺ���
Э��ջ���ӣ����Ըú������������������appʹ��
			����Ϣ��������ο�gap_bond_dev�ṹ��
**************************************************************************************************/
extern uint8_t bm_s_bond_info_add(struct gap_bond_dev *p_device);





/*************************************************************************************************
*��������bm_s_bond_info_delete_all
*�������:��
*�������:uint8_t ָʾɾ������Ϣ�Ƿ�ɹ�
					0	-->ʧ��
					1	-->�ɹ�
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�����ɾ�����еİ���Ϣ��֮����豸�������°�
**************************************************************************************************/
extern uint8_t bm_s_bond_info_delete_all(void);





/*************************************************************************************************
*��������bm_s_bond_info_delete
*�������:��
*�������:uint8_t ָʾɾ������Ϣ�Ƿ�ɹ�
					0	-->ʧ��
					1	-->�ɹ�
*˵��:SYD8821��������洢����Ϣ��λ�ã��ú�����ɾ��bm_s_bond_manager_idx_setָ��������λ�õİ���Ϣ
**************************************************************************************************/
extern uint8_t bm_s_bond_info_delete(void);





/*************************************************************************************************
*��������ecdh_public_key_get
*�������:��
*�������:
*˵��:
**************************************************************************************************/
extern void ecdh_public_key_get(uint8_t *p_x, uint8_t *p_y);





/*************************************************************************************************
*��������sys_mcu_clock_get
*�������:uint8_t *p_sel  ���ص�ǰʱ��Դ��ָ�룬������ö��_MCU_CLOCK_SEL_�ĳ�Ա
*�������:��
*˵��:�ú�����ȡ��ǰmcuʱ��Դ��
**************************************************************************************************/
extern void sys_mcu_clock_get(uint8_t *p_sel);





/*************************************************************************************************
*��������sys_mcu_clock_set
*�������:uint8_t  sel  ����MCUʱ��Դ��������ö��_MCU_CLOCK_SEL_�ĳ�Ա
*�������:��
*˵��:�ú�������mcuʱ��Դ����ΪMCU��ʱ�ӱ���Ҫ����У׼���ܹ�ʹ�ã����Ե����˸ú������������ŵ���
sys_mcu_rc_calibration����
**************************************************************************************************/
extern void sys_mcu_clock_set(uint8_t  sel);





/*************************************************************************************************
*��������sys_mcu_clock_div_set
*�������:
*�������:��
*˵��:
**************************************************************************************************/
extern void sys_mcu_clock_div_set(uint8_t  div);
extern void sys_mcu_clock_div_get(uint8_t *p_div);





/*************************************************************************************************
*��������sys_32k_clock_get
*�������:uint8_t *p_sel  ���ص�ǰʱ��Դ��ָ�룬������ö32K_CLOCK_SEL_��Ա
*�������:��
*˵��:�ú�����ȡ��ǰ32.768KHz��Ƶʱ��Դ����Ϊ��Ƶʱ�����л�����ܸߵĹ��ģ�����SYD8821��ϵͳ���и�Ƶ��
ʱ��Դ��MCUʹ�ã�Ҳ�е�Ƶ��ʱ��Դ��timerʹ��
**************************************************************************************************/
extern void sys_32k_clock_get(uint8_t *p_sel);





/*************************************************************************************************
*��������sys_32k_clock_set
*�������:uint8_t sel  ���ص�ǰʱ��Դ��ָ�룬������ö32K_CLOCK_SEL_��Ա
*�������:��
*˵��:�ú�������32.768KHz��Ƶʱ��Դ����Ϊ��Ƶʱ�����л�����ܸߵĹ��ģ�����SYD8821��ϵͳ���и�Ƶ��ʱ��Դ
��MCUʹ�ã�Ҳ�е�Ƶ��ʱ��Դ��timerʹ��
**************************************************************************************************/
extern void sys_32k_clock_set(uint8_t sel);





/*************************************************************************************************
*��������sys_32k_lpo_calibration
*�������:��
*�������:uint8_t
					0	-->ʧ��
					1	-->�ɹ�
*˵��:�ú�������У׼оƬ�ڲ���Ƶʱ��Դ(Ҳ��ΪLPO��Ϊ�ڲ�32.768KHz RC����)����ΪRC���������¶�Ӱ���
���ԣ�
			����APP���ʹ���ڲ�RC�����Ļ�Ҫ������һ���ļ��ȥУ׼�ڲ�RC���������Ƽ��ļ����3-10���ӣ�����
ʹ��3����
			��У׼���ڲ�RC��������ںܴ��ƫ�BLE���ӻ���ֶ����Լ����Ӳ��ϵ�����
**************************************************************************************************/
extern uint8_t sys_32k_lpo_calibration(void);





/*************************************************************************************************
*��������sys_mcu_rc_calibration
*�������:��
*�������:��
*˵��:�ú�������У׼оƬ�ڸ�Ƶʱ��Դ����Ϊ����RC�������ϵ��ʱ�����ƫ��Ƚϴ�������
			����APP���ʹ���ڲ�RC�����Ļ�Ҫ�����ڿ�����ѡ����MCUʱ��֮��У׼һ�θ�Ƶʱ��Դ
**************************************************************************************************/
extern void sys_mcu_rc_calibration(void);





/*************************************************************************************************
*��������pmu_wakeup_config
*�������:struct gap_wakeup_config *p_cfg	����Դ���ýṹ��
*�������:��
*˵��:Ϊ�˽�ʡ���ģ�BLE����������ʱʱ�̹̿����ģ�Ӧ��˵�ܳ���һ��ʱ�䶼��˯��״̬������Ȼ�ͻ��������˯��
�����ã�ĿǰSYD8821�ܹ�����MCU������������ʱ����GPIO�������ж�
			����GPIO���������Ҫ��������������ѡ��gpi_wakeup_en��gpi_wakeup_cfg�ܹؼ���ǰ�ߴ����Ƿ�ʹ��GPIO��
��MCU�����ߴ�����Щ�ܽŻ���MCU��gpi_wakeup_cfg��bit_mask
			��ʾ�ģ�����gpi_wakeup_cfg=0x00000900����GPIO11��GPIO8�ܹ�����
**************************************************************************************************/
extern void pmu_wakeup_config(struct gap_wakeup_config *p_cfg);





/*************************************************************************************************
*��������pmu_mcu_off
*�������:��
*�������:��
*˵��:�ú������ر�MCU���ڴ�������״̬����
**************************************************************************************************/
extern void pmu_mcu_off(void);





/*************************************************************************************************
*��������pmu_system_off
*�������:��
*�������:��
*˵��:�ú������ر�оƬ����ģ�飬�ڴ������ȶ��ᱻ�ر�
**************************************************************************************************/
extern void pmu_system_off(void);





/*************************************************************************************************
*��������pmu_reset
*�������:uint32_t reset_type	��λ����,������ö��PMU_RESET_FLAG_TYPE�ĳ�Ա
*�������:��
*˵��:������MCU_RESETģʽ��ֻ�Ǹ�λMCU��һЩ���ֵ�·����GPIO,TIMER,RTC��pem_led��watchdog��Щ������û��
����λ����,SYSTEM_RESET�������ģ�鶼��λ��
**************************************************************************************************/
extern void pmu_reset(uint32_t reset_type);





/*************************************************************************************************
*��������pmu_system_reset
*�������:��
*�������:��
*˵��:ϵͳ��λ���൱�ڣ�pmu_reset(SYSTEM_RESET);
**************************************************************************************************/
extern void pmu_system_reset(void);





/*************************************************************************************************
*��������pmu_mcu_reset
*�������:��
*�������:��
*˵��:MCU��λ���൱�ڣ�pmu_reset(MCU_RESET);
**************************************************************************************************/
extern void pmu_mcu_reset(void);





/*************************************************************************************************
*��������ble_flash_erase
*�������:uint32_t address  �����ĵ�ַ�����������ַ
					uint8_t num	Ҫ������������
*�������:uint8_t	ָʾ�Ƿ�����ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:ble_flash_erase��ble_flash_read�Լ�ble_flash_write��������������оƬ�ڲ�flash����������������
app����ʹ��������������оƬ�ڲ�flash���в���
			�ڵ���ble_flash_write֮ǰ�������ble_flash_erase���в������������ñ��ݣ���Ϊble_flash_erase����
�ĵ�λ��������Ҳ����4096��byte��
			���Ե�ble_flash_write�ĵ�ַ��Χ��û��д�����ǾͲ���Ҫ�ٴε���ble_flash_erase��������������˵���
��������0x4000)����һ��д0x4000-0x4100,
			�ڶ���д0x4100-0x4200�Ͳ���Ҫ�ٴβ����ˣ���Ϊ���ǵĵ�ַ��û���ص����һ���ͬһ��������
			SYD8821���ڲ���flash�ռ�Ĵ����������Ƶģ�һ��Ϊ10��Σ�,���Բ�����Ƶ���Ĳ���
**************************************************************************************************/
extern uint8_t ble_flash_erase(uint32_t address,uint8_t num);





/*************************************************************************************************
*��������ble_flash_read
*�������:uint32_t address  Ҫ��ȡ�ĵ�ַ�����������ַ
					uint16_t len	Ҫ��ȡ�ĳ���
					uint8_t *pbuf	�������ݵ�ָ��
*�������:uint8_t	ָʾ�Ƿ�����ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:�ú���������ڲ�flash�ж�ȡ���ݣ�
			Ϊ�˷�ֹ��ȡ�������������������Ҫ���������ǰ��48Kbyte�ռ䲻�ɶ�
**************************************************************************************************/
extern uint8_t ble_flash_read(uint32_t address,uint16_t len, uint8_t *pbuf);





/*************************************************************************************************
*��������ble_flash_write
*�������:uint32_t address  Ҫд��ĵ�ַ�����������ַ
					uint16_t len	Ҫд��ĳ���
					uint8_t *pbuf	д�����ݵ�ָ��
*�������:uint8_t	ָʾ�Ƿ�����ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:�ú�������д�����ݵ��ڲ�falsh�ռ��У�SYD8821�ڲ���flash�ռ�Ҳ��ͨ�õ�flash��������ѭflash������
�淶���ڵ���д����֮ǰ������ò�������
			��Ϊ�����˲�������֮��flash�ڲ��ռ�����ݾͻظ���Ĭ��ֵ�ˣ������ڲ���֮ǰһ��Ҫ���ö�ȡ�����Ȱ�
����������ȡ���ڴ�ռ�������У�
			Ȼ���޸��ڴ����������ݣ���֮���ò����������ٰ��ڴ��е�����д��flash��
**************************************************************************************************/
extern uint8_t ble_flash_write(uint32_t address,uint16_t len, uint8_t *pbuf);





/*************************************************************************************************
*��������ble_flash_write_burst
*�������:uint32_t address  Ҫд��ĵ�ַ�����������ַ
					uint16_t len	Ҫд��ĳ���
					uint8_t *pbuf	д�����ݵ�ָ��
					uint8_t flush	����0
*�������:uint8_t	ָʾ�Ƿ�����ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:�ú�������д�����ݵ��ڲ�falsh�ռ��У�SYD8821�ڲ���flash�ռ�Ҳ��ͨ�õ�flash��������ѭflash������
�淶���ڵ���д����֮ǰ������ò�������
			��Ϊ�����˲�������֮��flash�ڲ��ռ�����ݾͻظ���Ĭ��ֵ�ˣ������ڲ���֮ǰһ��Ҫ���ö�ȡ�����Ȱ�
����������ȡ���ڴ�ռ�������У�
			Ȼ���޸��ڴ����������ݣ���֮���ò����������ٰ��ڴ��е�����д��flash��
**************************************************************************************************/
extern uint8_t ble_flash_write_burst(uint32_t address,uint16_t len, uint8_t *pbuf,uint8_t flush);





/*************************************************************************************************
*��������ota_code_erase
*�������:��
*�������:uint8_t	ָʾ�Ƿ�����ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:SYD8821��flash�л�����������������һ����Ϊ��ǰ�������������һ����Ϊota�����д洢����ı���ȥ������
��ʹ��OTA�����г��������ܹ��������д������Ĵ��룬
			���������������ota����������ȷ�ģ��Ͱѱ�������Ϊ����������ԭ��������������Ȼ����˱�������OTA��Ϊ��
���׶Σ�������д�Լ����飬
			����ota_code_erase�����������flash�ڲ��ı��ݴ�����������
			�ú���ֻ���������ڲ�����������������ʽ�����������flash�ռ䶼��Ϊһ�����������Ǹú�������������
**************************************************************************************************/
extern uint8_t ota_code_erase(void);





/*************************************************************************************************
*��������ota_code_write
*�������:uint32_t offset	��ǰ���ݰ��е�flash���������ota��ʼ���ݵ�ƫ��
					uint16_t len	�����ݰ��е���Чflash���ݵĳ���
					uint8_t *p_buf	�˴����ݰ���flash���ݵ�ָ��
*�������:uint8_t	ָʾ�Ƿ�д���ݳɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:SYD8821��flash�л�����������������һ����Ϊ��ǰ�������������һ����Ϊota�����д洢����ı���ȥ������
��ʹ��OTA�����г��������ܹ��������д������Ĵ��룬���������������ota����������ȷ�ģ��Ͱѱ�������Ϊ��������
��ԭ��������������Ȼ����˱�������OTA��Ϊ�����׶Σ�������д�Լ����飬
			����ota_code_erase�����������flash�ڲ��ı��ݴ�����������
			�ú���ֻ���������ڲ�����������������ʽ�����������flash�ռ䶼��Ϊһ�����������Ǹú�������������
			SYD8801��ota�����ǰ�ota�ļ��ֳɺܶ�����ݰ���Ȼ�������·�����ʽ
**************************************************************************************************/
extern uint8_t ota_code_write(uint32_t offset , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*��������ota_code_update
*�������:uint8_t *p_desc	ota���������
					uint8_t *p_ver	ota����İ汾
					uint32_t sz	ota����Ĵ�С
					uint16_t checksum	ota����ļ����
*�������:uint8_t	ָʾ�Ƿ���£�У�飩�ɹ�
									0 --> ʧ��
									1	-->	�ɹ�
*˵��:SYD8821��flash�л�����������������һ����Ϊ��ǰ�������������һ����Ϊota�����д洢����ı���ȥ��
������ʹ��OTA�����г��������ܹ��������д������Ĵ��룬���������������ota����������ȷ�ģ��Ͱѱ�������Ϊ��������
��ԭ��������������Ȼ����˱�������OTA��Ϊ�����׶Σ�������д�Լ����飬
			�ú����������flash�е����ݵļ���ֵ�Ƿ���ڲ���checksum������������򷵻�ʧ�ܣ���ʱ��λ���еĳ�����
ԭ��δOTA�ĳ���p_desc��p_ver���������ã�
			�������ֵ��ȣ��ú��������������ļ��е�p_desc��p_ver�����󷵻سɹ���
			ע�⣺ota_code_update�����в����ᷢ��λ������app�ڵ���ota_code_update����Է���λ�Ĳ��������
����ɹ�����OTA�ɹ�������OTAʧ��
**************************************************************************************************/
extern uint8_t ota_code_update(uint8_t *p_desc, uint8_t *p_ver, uint32_t sz, uint16_t checksum);





/*************************************************************************************************
*��������ble_sched_execute
*�������:��
*�������:��
*˵��:�ú���Ϊble���Ⱥ�������ν�ĵ��Ⱥ�������˵�ú�����ȥ�ж�������״̬��ִ��һЩ��Ϊ����Ϊflash�Ĳ�����
�ϳ�ʱ�����������Э��ջ��Ҫ���д���������Ҫ�Ĺ�����ʱ��
		 ��������Գɹ���󱣴����Ϣ�Ĳ���������flash�Ʊػ���������Э��ջ������������һЩflash�Ĳ�����
���ú��������У�ͬʱgap_s_profile_data_writeҲ���ڸú����������ã�����ble_flash_erase��ble_flash_read��
ble_flash_write��ota_code_erase��ota_code_write��ota_code_update��������������Ч��
			����Ҫ��app��while��1����ѭ���е��øú�����
**************************************************************************************************/
extern void ble_sched_execute(void);





/*************************************************************************************************
*��������BBRFWrite
*�������:uint8_t addr	BBR�Ĵ����ĵ�ַƫ��
					uint8_t data	Ҫд�������
*�������:��
*˵��:BBR�Ĵ�����SYD8821����Ƚ������һЩ�Ĵ������ú�������дBBR�Ĵ���
			BBR�Ĵ����漰������оƬ����Ϊ������ȷ��д���п������оƬ���ң�������Щ�Ĵ����Ĳ�������Ҫ�ڹٷ�����
ȷ˵��ָ���½���
**************************************************************************************************/
extern void BBRFWrite(uint8_t addr, uint8_t data);





/*************************************************************************************************
*��������BBRFRead
*�������:uint8_t addr	BBR�Ĵ����ĵ�ַƫ��
					uint8_t data	Ҫ����������ָ��
*�������:��
*˵��:BBR�Ĵ�����SYD8821����Ƚ������һЩ�Ĵ������ú����������BBR�Ĵ���������
			BBR�Ĵ����漰������оƬ����Ϊ������ȷ��д���п������оƬ���ң�������Щ�Ĵ����Ĳ�������Ҫ�ڹٷ���
��ȷ˵��ָ���½���
**************************************************************************************************/
extern void BBRFRead(uint8_t addr, uint8_t* data);





/*************************************************************************************************
*��������ota_code_update_496kb
*�������:uint8_t *p_desc	ota���������
					uint8_t *p_ver	ota����İ汾
					uint16_t sz	ota����Ĵ�С
					uint16_t checksum	ota����ļ����
*�������:��
*˵��:SYD8821��flash��������������248KB��������ģʽ�⣬����֧��һ�����496�Ĵ���������ʽ��������496kB
��ģʽ��OTA����Ҫ��оƬ�ⲿ����һ��flashоƬ��ota�����аѴ���ŵ���оƬ�У������ش�����ⲿflash��ɺ�
������BootLoader���루����BootLoader����SYD8821�Ļ������ٽ���һ��������ROM����Ĵ��룬�ô�����Է���
�����У�����Ҫ��ʱ�򿽱����ڴ����У�Ҳ����ֱ�ӷ���оƬ�ڲ�falsh�У�����Ҫ���ֺ����򣩸�����ⲿflash��
���뿽�����ڲ�flash��
			�ú�������Ѹ�������ָ��������д�뵽оƬ�ڲ�����������֮��λ���в��ܹ����е���ȷ��496KB����
			ע�⣺ota_code_update�����в����ᷢ��λ������app�ڵ���ota_code_update����Է���λ�Ĳ��������
����ɹ�����OTA�ɹ�������OTAʧ��
**************************************************************************************************/
extern uint8_t ota_code_update_496kb(uint8_t *p_desc, uint8_t *p_ver, uint32_t sz, uint16_t checksum);




/*************************************************************************************************
*��������_checksum_cache_496kb
*�������:uint32_t adr 496kb��������ڲ�flash��λ�ã���Ϊ����λ��
					uint16_t sz	ota����Ĵ�С
					uint16_t checksum	ota����ļ����
*�������:��
*˵��:SYD8821��flash��������������248KB��������ģʽ�⣬����֧��һ�����496�Ĵ���������ʽ��������496kB
��ģʽ��OTA����Ҫ��оƬ�ⲿ����һ��flashоƬ��ota�����аѴ���ŵ���оƬ�У������ش�����ⲿflash��ɺ�
������BootLoader���루����BootLoader����SYD8821�Ļ������ٽ���һ��������ROM����Ĵ��룬�ô�����Է���
�����У�����Ҫ��ʱ�򿽱����ڴ����У�Ҳ����ֱ�ӷ���оƬ�ڲ�falsh�У�����Ҫ���ֺ����򣩸�����ⲿflash��
���뿽�����ڲ�flash��
			�ú����������flash�е����ݵļ���ֵ�Ƿ���ڲ���checksum������������򷵻�ʧ��
			�����ota_code_update�������ú���ӵ�м�������496kb���������
**************************************************************************************************/
extern uint8_t _checksum_cache_496kb(uint32_t adr,uint32_t sz, uint16_t checksum);



/*************************************************************************************************
*��������gap_s_smart_update_latency
*�������:struct gap_smart_update_params *p_smart_params  �������Ӳ����ṹ��
*�������:��
*˵��:��Ϊ�淶�в�û�й���latency�Ĺ�������˵������������SYD8821����һ�����ܹ������Ӳ�����latency
�Ļ��ƣ�����smart_update_latency���ú����������latency�Ŀ�ʼ�������Ӳ����ĸ��²����������Ӳ�������
�������ܾ���ʱ�򣬸ú�������̬�ĵ������Ӳ����ٴ�������£��������ݽ��н�����ʱ��ú�����̬�Ŀ���
���߹ر�latency��ʵ�ֹ��ĵ�ƥ�䡣
			��Ȼ���ú����Ƕ�����ԭ����Э��ջ���Ƶģ�Ҳ����˵�����øú�������ԭ����Э��ջ����Ϊ��û���κ�
Ӱ���
			��Ϊlatency�����Ӳ����ȸ����漰���������ȶ��Ժͼ����Եȣ��������ｨ��ʹ�����ܹ������
			smart_update_latency��Э��ջ�����Ƕ����ģ�������������øú�������Э��ջ�Ĺ�����û���κε�Ӱ��
**************************************************************************************************/
extern uint8_t gap_s_smart_update_latency(struct gap_smart_update_params *p_smart_params);




/*************************************************************************************************
*��������SystemSleep
*�������:POWER_SAVING_TYPE mode	��Դʡ�����ͣ�ΪPOWER_SAVING_TYPEö�ٳ�Ա
					MODULE_CONTROL_TYPE c ģ��������ͣ�ΪMODULE_CONTROL_TYPEö������
					uint32_t ldo_delay	MCU���Ѻ�nop"ִ��ִ�еĴ���
					PMU_WAKEUP_CONFIG_TYPE	����Դʹ�ܿ���
*�������:��
*˵��:�ú���ʹSYD8821����ǳ��˯��ģʽ���͹��ĵĳ̶���mode��c�����������ƣ����ڸú������ɴ������RESET��
�ı�����
			������÷�ʽ��SystemSleep(POWER_SAVING_RC_OFF, FLASH_LDO_MODULE, 11000 , 
												(PMU_WAKEUP_CONFIG_TYPE)(FSM_SLEEP_EN|PIN_WAKE_EN|TIMER_WAKE_EN|RTC_WAKE_EN));
**************************************************************************************************/
extern uint8_t SystemSleep(POWER_SAVING_TYPE mode, MODULE_CONTROL_TYPE c,uint32_t ldo_delay,PMU_WAKEUP_CONFIG_TYPE w);







/*************************************************************************************************
*gap_s_att_mtu_get
*�������:��
*�������:uint16_t ��ǰatt���MTU
*˵��:�ú�����ȡ����ǰatt���mtu����mtuΪ�����ʹӻ������õ��Ľ����Ӧ�ó����͵��������ݰ����ܹ�
���ڸú������ص�mtu��С
			������÷�ʽ��mtu_now=gap_s_att_mtu_get;
**************************************************************************************************/
extern uint16_t gap_s_att_mtu_get(void);



/*************************************************************************************************
*ble_SetTxPower
*�������:BLE_TX_POWER value RF���书��ֵ��ΪBLE_TX_POWER�ٳ�Ա
*�������:��
*˵��:�ú�������RF���书�ʣ��ϵ��ble_init֮��������Ч��RF���书��Ĭ��Ϊ0dbm
**************************************************************************************************/
extern void ble_SetTxPower(BLE_TX_POWER value);

/*************************************************************************************************
*gap_s_att_mtu_get
*�������:��
*�������:uint8_t ��ǰӲ��TX buffer״̬
*˵��:�ú�����ȡ����ǰTXӲ��buffer��״̬�Ƿ�Ϊ��
			����1�������ݣ�����0��������
**************************************************************************************************/			
extern uint8_t BBCheckTXFIFOEmpty(void);

/*************************************************************************************************
*ble_SetTxPower
*�������:uint8_t timer �����¼������һ�����ݰ�����һ�������¼�������ʱ�䣬��λ��30.25us
*�������:��
*˵��:�ú����Ĳ��������TH_LAST��������һ�������¼������һ��moredata���ݰ�����һ�������¼���ʱ�䣬
��λΪ30.5us��Ҳ����˵�ú����Ĳ���Ҳ������ôһ�������¼��ڵ����ݰ���ĿԽС��
**************************************************************************************************/
extern void ll_set_replying_packet_timer(uint8_t timer);

/*************************************************************************************************
*amic_set_bias
*�������:AMIC_BIAS value amicģ���biasֵ��ΪAMIC_BIAS�ٳ�Ա
*�������:��
*˵��:�ú�������amicģ���biasֵ���ϵ��ble_init֮��������Ч��amicģ���biasֵĬ��Ϊ1.4V
**************************************************************************************************/
extern void amic_set_bias(AMIC_BIAS value);

#endif
