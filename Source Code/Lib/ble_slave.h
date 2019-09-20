#ifndef _BLE_LIB_H_
#define _BLE_LIB_H_

#include "ARMCM0.h"

#pragma pack(1)

/*************************************************************************************************
*ºê¶¨Òå£ºBD_ADDR_SZ
*ËµÃ÷£º À¶ÑÀµØÖ·´óÐ¡£¬¸ù¾Ý¹æ·¶£¬¸ÃºêÎª6¸öbyte
**************************************************************************************************/
#define BD_ADDR_SZ 			6



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_EDIV_SZ
*ËµÃ÷£º À¶ÑÀEDIV²ÎÊý´óÐ¡£¬¸ù¾Ý¹æ·¶£¬¸ÃºêÎª2¸öbyte
**************************************************************************************************/
#define MAX_EDIV_SZ			2



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_RAND_SZ
*ËµÃ÷£º À¶ÑÀRAND²ÎÊý´óÐ¡£¬¸ù¾Ý¹æ·¶£¬¸ÃºêÎª8¸öbyte
**************************************************************************************************/
#define MAX_RAND_SZ		8



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_KEY_SZ
*ËµÃ÷£º À¶ÑÀLTKÃÜ³×µÄ´óÐ¡£¬¸ù¾Ý¹æ·¶£¬¸ÃºêÎª16¸öbyte£¬128bit
**************************************************************************************************/
#define MAX_KEY_SZ			16



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_IRK_SZ
*ËµÃ÷£º À¶ÑÀIRK²ÎÊýµÄ´óÐ¡£¬¸ù¾Ý¹æ·¶£¬¸ÃºêÎª16¸öbyte£¬128bit
**************************************************************************************************/
#define MAX_IRK_SZ			16



/*************************************************************************************************
*ºê¶¨Òå£ºLL_WIN_OFFSET_SZ
*ËµÃ÷£º À¶ÑÀBLEÖÐLL_CONNECTION_PARAM_RSPÏìÓ¦ÖÐOFFSETµÄ´óÐ¡£¬¸ù¾Ý¹æ·¶£¬Ò»¹²ÓÐ6¸öOFFSET,ÒòÎª¸ÃºêÓÃÓÚ
LL_CONNECTION_PARAM_RSP½á¹¹ÌåÖÐµÄOffsetÊý×é´óÐ¡£¬ËùÒÔÕâÀï¶¨ÒåÎª×î´óµÄ6
**************************************************************************************************/
#define LL_WIN_OFFSET_SZ	6




/*************************************************************************************************
*ºê¶¨Òå£ºMAX_ATT_DATA_SZ
*ËµÃ÷£º À¶ÑÀBLEÖÐATT²ãµÄMTUÖµ£¬Ò²¾ÍÊÇ×î´óµÄATTÊý¾Ý´óÐ¡£¬¸ù¾Ý4.2¹æ·¶¶¨Òå£¬¸ÃÖµÎª512
**************************************************************************************************/
#define MAX_ATT_DATA_SZ	512



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_ADV_DATA_SZ
*ËµÃ÷£º À¶ÑÀBLEÖÐ¹ã²¥Í¨µÀµÄÊý¾ÝÁ¿µÄ×î´óÖµ£¬Ò²¾ÍÊÇ×î´óµÄATTÊý¾Ý´óÐ¡£¬¸ù¾Ý4¹æ·¶¶¨Òå£¬¸ÃÖµÎª31
**************************************************************************************************/
#define MAX_ADV_DATA_SZ	31



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_ATT_REPORT_HDL
*ËµÃ÷£º SYD8821×î´óµÄnotify»òÕßindicateÍ¨µÀÊýÄ¿£¬Ò²ÊÇgap_att_report_handle½á¹¹ÌåÖÐµÄgap_att_report
³ÉÔ±µÄ×î´ó¸öÊý
**************************************************************************************************/
#define MAX_ATT_REPORT_HDL 20



/*************************************************************************************************
*ºê¶¨Òå£ºMAX_UPDATE_ADJ_NUM
*ËµÃ÷£º SYD8821×î´óµÄÁ¬½Ó²ÎÊýµÄµ÷Õû´ÎÊý£¬¸ÃºêÓÃÓÚsmart params£¨ÖÇÄÜÁ¬½Ó²ÎÊý¹ÜÀí£©£¬smart params¹ý³Ì
ÖÐÈç¹û¶Ô·½¾Ü¾øÁËSYD8821ÇëÇóµÄÁ¬½Ó²ÎÊý£¬ÄÇÃ´SYD8821½«»á·¢ËÍ¸ü¼ÓÈÃ¶Ô·½ÈÝÒ×½ÓÊÜµÄÁ¬½Ó²ÎÊý£¬Èç¹ûµ÷Õû´ÎÊý
´óÓÚMAX_UPDATE_ADJ_NUMÄÇÃ´½«½áÊøÁ¬½Ó²ÎÊýµÄµ÷Õû¹ý³Ì
**************************************************************************************************/
#define MAX_UPDATE_ADJ_NUM		4




/*************************************************************************************************
*Ã¶¾ÙÃû£º_BLE_ADDRESS_TYPE_
*³ÉÔ±£º PUBLIC_ADDRESS_TYPE	¹«¹²µØÖ·ÀàÐÍ
				RANDOM_ADDRESS_TYPE	Ëæ»úµØÖ·ÀàÐÍ
*ËµÃ÷£º BLEµÄMACµØÖ·ÀàÐÍÓÐ¹«¹²µØÖ·ºÍËæ»úµØÖ·Á½ÖÖ£¬Ç°ÕßÊÇÖÆÔìÉÌºÍSIGÉêÇë¾ßÓÐÎ¨Ò»ÐÔµÄµØÖ·£¬ºóÕßÎÞÐèºÍ
SIGÉêÇë£¬RANDOM_ADDRESS_TYPEÓÖ·ÖÎªStatic Device AddressºÍPrivate Device Address
**************************************************************************************************/
enum _BLE_ADDRESS_TYPE_{
	PUBLIC_ADDRESS_TYPE	= 0x00,
	RANDOM_ADDRESS_TYPE 	= 0x01,
};




/*************************************************************************************************
*Ã¶¾ÙÃû£º_ADV_CH_PKT_TYPE_
*³ÉÔ±£º ADV_IND	connectable undirected advertising event²»¶¨Ïò¿ÉÁ¬½Ó¹ã²¥ÊÂ¼þ
				ADV_DIRECT_IND	connectable directed advertising event¶¨Ïò¿ÉÁ¬½Ó¹ã²¥ÊÂ¼þ
				ADV_NOCONN_IND	non-connectable undirected advertising event²»¿ÉÁ¬½Ó¹ã²¥ÊÂ¼þ
				SCAN_REQ	É¨ÃèÇëÇóÊÂ¼þ
				SCAN_RSP	É¨ÃèÏìÓ¦ÊÂ¼þ
				CONNECT_REQ	¹ã²¥Í¨µÀÁ¬½ÓÊÂ¼þ
				ADV_SCAN_IND	scannable undirected advertising event¿ÉÉ¨ÃèÎ´¶¨Ïò¹ã²¥ÊÂ¼þ£¬ÕâÊÇBLE4.2Ôö¼ÓµÄ
*ËµÃ÷£º ¸ÃÃ¶¾Ù¶¨ÒåÁË¿ÉÄÜ·¢ÉúÔÚBLEµÄ¹ã²¥Í¨µÀÉÏµÄÊÂ¼þ£¬ÕâÀïµÄÊÂ¼þ¸üÈ·ÇÐµÄËµ·¨Ó¦¸ÃÊÇ¿ÉÄÜ·¢ÉúµÄÐÐÎª£¬
ÒýÓÃ²»Í¬µÄÀàÐÍ£¬µÍ²ã¾Í»á·¢ÉúÏàÓ¦µÄÐÐÎª£¬Á¬½Ó×´Ì¬Ö®Ç°µÄÐÐÎª¶¼ÊÇ·¢ÉúÔÚ¹ã²¥Í¨µÀÉÏµÄ£¬ADV_IND¡
ADV_DIRECT_IND¡¢ADV_NOCONN_IND¡¢ADV_SCAN_INDÊôÓÚBLEµÄ¹ã²¥ÊÂ¼þ£¬¶ÔÓ¦×ÅÏàÓ¦µÄ¹ã²¥ÀàÐÍ£»SCAN_REQºÍ
SCAN_RSPÊôÓÚBLEµÄÉ¨ÃèÏà¹ØµÄÊÂ¼þ£¬Ç°ÕßÊôÓÚÉ¨ÃèÇëÇóºóÕßÎªÉ¨ÃèÏìÓ¦£»CONNECT_REQÎªÁ¬½ÓÊÂ¼þ¡£ÆäÖÐSCAN_REQ
ºÍCONNECT_REQÊÇ¶ÔÓÚSYD8821µÄmaster£¨Ö÷»ú£©¶Ë²ÅÄÜ¹»ÓÐµÄÐÐÎª¡£
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
*Ã¶¾ÙÃû£ºBLE_SEND_TYPE
*³ÉÔ±£º BLE_GATT_NOTIFICATION 	Characteristic Value Notification.BLEÏûÏ¢ÀàÐÍÎªnotify
				BLE_GATT_INDICATION		Characteristic Value Indications BLEÏûÏ¢ÀàÐÍÎªIndications
*ËµÃ÷£º ¶ÔÓÚSYD8821´Ó»ú£¬Characteristic Value·¢ËÍÐÐÎªÓÐÁ½ÖÖ£¬¼´NotificationºÍIndications£¬Ç°ÕßÊÇ
SYD8821µ¥µ¥·¢ËÍÏûÏ¢£¬²¢²»ÐèÒªµÈ´ýmaster×ö³öÈÎºÎÏìÓ¦£¬¶øºóÕß»¹ÐèÒªµÈ´ýmaster×ö³öÕýÈ·µÄÏìÓ¦²Å»á½øÐÐ
ÏÂÒ»²½·¢ËÍ
**************************************************************************************************/
enum BLE_SEND_TYPE {
	BLE_GATT_NOTIFICATION	= 0x0001,
	BLE_GATT_INDICATION		= 0x0002,
};



/*************************************************************************************************
*Ã¶¾ÙÃû£º_MCU_CLOCK_SEL_
*³ÉÔ±£º MCU_CLOCK_16_MHZ 	MCUÊ±ÖÓÎª16Mhz
				MCU_CLOCK_20_MHZ	MCUÊ±ÖÓÎª20Mhz
				MCU_CLOCK_24_MHZ	MCUÊ±ÖÓÎª24Mhz
				MCU_CLOCK_64_MHZ	MCUÊ±ÖÓÎª64Mhz
				MCU_CLOCK_80_MHZ	MCUÊ±ÖÓÎª80Mhz
				MCU_CLOCK_96_MHZ	MCUÊ±ÖÓÎª96Mhz
*ËµÃ÷£º SYD8821µÄMCUµÄÊ±ÖÓ¿ÉÒÔ´Ó¸ÃÃ¶¾ÙÖÐÑ¡È¡£¬²¢ÇÒÉÏµçµÄÊ±ºò±ØÐëÑ¡ÔñÊÊµ±µÄMCUÊ±ÖÓ£¬²¢ÇÒµ÷ÓÃMCUµÄ¸ßÆµ
RCÕñµ´Æ÷Ð£×¼º¯Êý£ºsys_mcu_rc_calibration£¬¶ÔÓÚMCUµÄÊ±ÖÓ£¬»ù±¾µÄÔ­ÔòÊÇ£ºÊ±ÖÓÆµÂÊÔ½¸ß£¬ÏàÓ¦µÄ¹¦ºÄÒ²»á
ÏàÓ¦µÄÔö´ó£¨¹¦ºÄ²¢²»»áÓÐ³É±¶µÄÔö³¤£¬Ö»ÊÇÂÔÎ¢ÓÐÐ©²îÒì£©¡£Ò»°ã¶øÑÔÅäÖÃÊ±ÖÓÖ®ºó»á½ô½Ó×Å½øÐÐRCµÄÐ£×¼¡£
				SYD8821µÄÊ±ÖÓÉè¼Æ´ÓÆµÂÊÀ´·ÖÀàÓÐµÍÆµµÄ32.768KHzµÄµÍÆµÊ±ÖÓ£¬ÕâÊÇ¹Ì¶¨µÄÆµÂÊ£¬¸ÃÊ±ÖÓµÄÊ±ÖÓÔ´
ÓÐÄÚ²¿RXÕñµ´Æ÷ºÍÍâ²¿¾§ÕñÁ½ÖÖÒÔ¼°ÄÚ²¿32MHzµÄ·ÖÆµÈýÖÖ£¬µÍÆµµÄÊ±ÖÓÖ÷Òª¹©¸øµÍÆµµÄ¶¨Ê±Æ÷ºÍPWMÊ¹ÓÃ£»³ýÁËµÍ
ÆµÊ±ÖÓÍâ£¬SYD8821Ð¾Æ¬ÄÚ²¿»¹ÓÐÒ»¸ö¹©MCUºÍ¸ßËÙµÄÍ¨Ñ¶½Ó¿Ú£¨±ÈÈçSPI¡¢I2CµÈ£©Ê¹ÓÃµÄ¸ßËÙÊ±ÖÓÕñµ´Æ÷£¬Õý³£¶ø
ÑÔ£¬¸ßËÙÊ±ÖÓµÄÊ±ÖÓÔ´Ò»°ãÀ´×ÔÐ¾Æ¬ÄÚ²¿µÄ¸ßÆµÕñµ´Æ÷¡£
				SYD8821µÄ¸ßËÙÊ±ÖÓµÄÊ±ÖÓÔ´Ö»ÓÐÄÚ²¿µÄ¸ßËÙRCÕñµ´Æ÷£¬ËùÒÔÔÚÅäÖÃºÃºó±ØÐëÒª¾­¹ýÐ£×¼µÄÁ÷³Ì£¬µ«ÊÇ¶ÔÓÚ
¸ßËÙÊ±ÖÓ¶øÑÔ£¬Ö»ÐèÒªÔÚ³ÌÐòÆô¶¯µÄÊ±ºò½øÐÐÒ»´ÎÐ£×¼¼´¿É£¡SYD8821µÄµÍÆµÊ±ÖÓÔ´¼ÈÓÐÄÚ²¿µÄ£¨¼ò³ÆLPO£©Ò²ÓÐÍâ²¿
µÄ£¨¼ò³ÆXO)£¬Èç¹ûÊ¹ÓÃÄÚ²¿µÄÊ±ÖÓÔ´£¬ÄÇÃ´±ØÐëµÃÃ¿¸ôÒ»¶ÎÊ±¼ä£¨½¨Òé3·ÖÖÓ£©½øÐÐÒ»´ÎÄÚ²¿µÄµÍÆµÊ±ÖÓµÄÐ£×¼¡£
				½øÐÐ¸ßÆµRCÊ±ÖÓÐ£×¼µÄÊ±ºò±ØÐëÏÈµ÷ÓÃgap_s_ble_initº¯Êý³õÊ¼»¯BLEÐ­ÒéÕ»£¡
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
*Ã¶¾ÙÃû£º_32K_CLOCK_SEL_
*³ÉÔ±£º SYSTEM_32K_CLOCK_LPO 	µÍÆµµÄ32KÊ±ÖÓÔ´ÎªÄÚ²¿RCÕñµ´Æ÷
				SYSTEM_32K_CLOCK_LPO 	µÍÆµµÄ32KÊ±ÖÓÔ´ÎªÍâ²¿¾§Õñ
				SYSTEM_32K_CLOCK_32M_DIV	µÍÆµµÄ32KÊ±ÖÓÔ´ÎªÍâ²¿¾§ÕñµÄ·ÖÆµ£¬·ÖÆµÖ¸ÊýÎª977·ÖÆµ£¬×îÖÕµÄÆµÂÊ½Ó½ü
																	32.768KHz£¬32000/977=32.7533Khz
*ËµÃ÷£º SYD8821µÄµÍÆµÊ±ÖÓµÄÆµÂÊ¹Ì¶¨Îª32.768KHz,Ö÷ÒªÊÇ¸øµÍÆµµÄ¶¨Ê±Æ÷ºÍPWMÒÑ¾­BLEÐ­ÒéÕ»Ê¹ÓÃ£¬µÍÆµÊ±ÖÓ
Ô´²»×¼È·µÄ»°½«»á¶Ô¸ÃÊ±ÖÓµÄÏû·ÑÕß¶¼ÓÐÓ°Ïì£¬±ÈÈçBLEÁ¬½Ó²»ÉÏ£¬¶¨Ê±Æ÷ºÍPWMµÄÊ±Ðò²»×¼È·µÈ¡£
				Èç¹ûÑ¡ÔñSYSTEM_32K_CLOCK_LPOÎªÊ±ÖÓÔ´µÄ»°£¬±ØÐëÒªÃ¿¸ôÒ»¸ö¹Ì¶¨µÄÊ±¼ä£¨½¨Òé3·ÖÖÓ£©ÒªÐ£×¼Ò»´Î£¬
ÒòÎªÈç¹ûRCÕñµ´Æ÷»áÓÐÎÂ¶ÈÆ¯ÒÆµÈÌØÐÔ£¬Ê±¼ä³¤ºó¾Í»áÓÐÆ«²î
**************************************************************************************************/
enum _32K_CLOCK_SEL_{
	SYSTEM_32K_CLOCK_LPO	= 0x00,
	SYSTEM_32K_CLOCK_XO	= 0x01,
	SYSTEM_32K_CLOCK_32M_DIV	= 0x02,
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_ble_addr
*³ÉÔ±£º type 	BLE macµØÖ·ÀàÐÍ£¬ÎªÃ¶¾Ù_BLE_ADDRESS_TYPE_µÄ³ÉÔ±
				addr 	BLE macµØÖ·
*ËµÃ÷£º SYD8821µÄÉè±¸µØÖ·¿ÉÍ¨¹ýgap_s_ble_address_getº¯ÊýÀ´»ñÈ¡£¬Í¨¹ýgap_s_ble_address_setÀ´ÉèÖÃ
**************************************************************************************************/
struct gap_ble_addr {
	uint8_t	type;
	uint8_t	addr[BD_ADDR_SZ];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_key_params
*³ÉÔ±£º ediv 	BLE°²È«¹ÜÀíÖÐµÄEncrypted Diversifier (EDIV)²ÎÊý
				rand 	BLE°²È«¹ÜÀíÖÐµÄRandom Number (Rand)²ÎÊý
				ltk 	BLE°²È«¹ÜÀíÖÐµÄLong Term Key (LTK) ²ÎÊý
				local_irk 	BLE°²È«¹ÜÀíÖÐµÄ±¾µØIdentity Resolving Key (IRK)²ÎÊý
				peer_irk 	BLE°²È«¹ÜÀíÖÐµÄ¶ÔµÈÉè±¸Identity Resolving Key (IRK)²ÎÊý
*ËµÃ÷£º ÔÚ¼ÓÃÜ½áÊøºó½»»»ÃÜ³×µÄÊ±ºòSYD8821»áÉÏ±¨enc_key_evtÊÂ¼þ£¬¸ÃÊÂ¼þÖÐ°üº¬ÓÐ¸Ã½á¹¹Ìå£¬°üº¬ÁË°²È«
¹ÜÀíÖÐÓÃµ½µÄ¸÷¸ö²ÎÊý
**************************************************************************************************/
struct gap_key_params{
	uint8_t	ediv[MAX_EDIV_SZ];
	uint8_t	rand[MAX_RAND_SZ];	
	uint8_t	ltk[MAX_KEY_SZ];
	uint8_t	local_irk[MAX_IRK_SZ];
	uint8_t	peer_irk[MAX_IRK_SZ];	
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_adv_params
*³ÉÔ±£º type 	¹ã²¥ÀàÐÍ£¬ÎªÃ¶¾Ù_ADV_CH_PKT_TYPE_ÖÐµÄ³ÉÔ±
				peer_addr	¶ÔµÈÉè±¸µØÖ·£¬¸Ã³ÉÔ±ÌåÏÖÎª¶¨Ïò¹ã²¥ÖÐµÄInitA£¬Ö»ÔÚ¶¨Ïò¹ã²¥ÓÐÓÃ
				policy	¹ýÂË²ßÂÔ£¬·ûºÏ¹æ·¶ÖÐÌá¼°µÄLINK LAYER DEVICE FILTERING  Policy
				channel	¹ã²¥Í¨µÀ£¬¸Ã²ÎÊýÊ¹ÓÃbitmaskÐÎÊ½£¬37~39Í¨µÀ¶ÔÓ¦bit0~bit2,±ÈÈç0x03´ú±íÔÚ37¡¢38Í¨µÀ¹ã²¥
				interval	¹ã²¥¼ä¸ô£¬µ¥Î»ÊÇ0.625ms,±ÈÈç0x640Îª1SµÄ¹ã²¥¼ä¸ô
				timeout	¹ã²¥³¬Ê±Ê±¼ä£¬µ¥Î»ÊÇ1S£¬±ÈÈç0x64Îª100S£¬×î´óÖµÎª0x3FFF£¬²»ÄÜ¹»´«Èë0
				hop_interval	¹ã²¥ÊÂ¼þÖÐÍ¨µÀºÍÍ¨µÀÖ®¼äµÄ¼ä¸ô£¬±ÈÈç¹ã²¥Í¨µÀÎª37,38ºÍ39£¬ÄÇÃ´hop_interval¼´Îª
											Ò»¸ö¹ã²¥ÊÂ¼þÖÐµÄ37µ½38»òÕß38µ½39µÄÊ±¼ä¼ä¸ô
*ËµÃ÷£º SYD8821µÄ¹ã²¥ÓÉ¹ã²¥²ÎÊýºÍ¹ã²¥Êý¾Ý×é³É£¬¹ã²¥²ÎÊý¿ØÖÆ¾ßÌåµÄÐÐÎª£¬¹ã²¥Êý¾ÝÎª¹ã²¥ÖÐ¾ßÌåµÄÊý¾Ý¡£
				¹ã²¥¼ä¸ôÊÇ»ùÓÚ¹¦ºÄºÍÁ¬½ÓÁéÃô¶ÈÉõÖÁÁ¬½ÓÎÈ¶¨ÐÔµÄÆ½ºâ£¬¼ä¸ôÒ²Ð¡Á¬½ÓÔ½¿ì£¬¿ÉÄÜÒ²Ô½ÎÈ¶¨£¬¹¦ºÄÒ²Ô½¸ß
¹ã²¥¼ä¸ôÔ½´óÁéÃô¶ÈÔ½µÍ£¬µ±È»¹¦ºÄ¾ÍÔ½µÍ
				¶ÔÓÚSYD8821£¬ÒªÇóinterval´óÓÚ2ms,hop_interval<interval
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
*½á¹¹ÌåÃû£ºgap_scan_params
*³ÉÔ±£º type 	É¨ÃèÀàÐÍ£¬ÓÐ±»¶¯É¨ÃèºÍÖ÷¶¯É¨Ãè
				interval	É¨Ãè¼ä¸ô£¬µ¥Î»ÊÇ0.625ms,Time Range: 2.5 msec to 10.24 seconds
				window	É¨Ãè´°¿Ú£¬µ¥Î»ÊÇ0.625ms£¬Time Range: 2.5 msec to 10.24 seconds
*ËµÃ÷£º É¨Ãè¼ä¸ôÒª´óÓÚÉ¨Ãè´°¿Ú£¬ÒòÎªÕæÕýÉ¨Ãè·¢ÉúÔÚÉ¨Ãè´°¿ÚÆÚ¼ä£¬ËùÒÔÓÐ¿ÉÄÜÒ»¸ö¼ä¸ôÄÚÉ¨Ãè²»µ½¹ã²¥µÄ
Çé¿ö£¬µ±SYD8821É¨Ãèµ½·ûºÏÒªÇóµÄ¹ã²¥µÄÊ±ºò£¬Ð­ÒéÕ»½«ÉÏ±¨É¨ÃèÏìÓ¦ÊÂ¼þ£¡
**************************************************************************************************/
struct gap_scan_params {
	uint8_t	type;
	uint16_t	interval;
	uint16_t	window;
};




/*************************************************************************************************
*Ã¶¾ÙÃû£º_ADV_SCAN_MODE_
*³ÉÔ±£º IDLE_MODE 	µ±Ç°×´Ì¬Îª¿ÕÏÐ×´Ì¬
		ADV_MODE 	µ±Ç°×´Ì¬Îª¹ã²¥×´Ì¬
		SCAN_MODE 	µ±Ç°×´Ì¬ÎªÉ¨Ãè×´Ì¬
		COEX_ADV_MODE 	µ±Ç°×´Ì¬ÎªÁ¬½Ó×´Ì¬ÏÂµÄ¹ã²¥×´Ì¬
		COEX_SCAN_MODE 	µ±Ç°×´Ì¬ÎªÁ¬½Ó×´Ì¬ÏÂµÄÉ¨Ãè×´Ì¬
*ËµÃ÷£º ÔÚ¹ã²¥Í¨µÀÉÏµÄÐÐÎª¼ÈÓÐ¿ÉÄÜÊÇ¹ã²¥Ò²ÓÐ¿ÉÄÜÊÇÉ¨Ãè£¬¶øÈç¹ûÀ¶ÑÀ´¦ÓÚÁ¬½Ó×´Ì¬ÏÂ£¬¹ã²¥ºÍÉ¨ÃèÓÖ²»Ò»Ñù£¡
**************************************************************************************************/
enum _ADV_SCAN_MODE_{
	IDLE_MODE			= 0x00,
	ADV_MODE			= 0x01,
	SCAN_MODE			= 0x02,
	COEX_ADV_MODE		= 0x03,
	COEX_SCAN_MODE	= 0x04,
};





/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_profile_struct
*³ÉÔ±£º report_handle_address 	report_handleµÄÖ¸Õë£¬Ö¸Ïò´úÂëÇøÓòÖÐµÄ_gatt_database_report_handle
				primary_address	Ö÷Òª·þÎñÖ¸Õë£¬Ö¸Ïò´úÂëÖÐµÄ_gatt_database_primary
				include_address	´ÎÒª·þÎñÖ¸Õë£¬Ö¸Ïò´úÂëÖÐµÄ_gatt_database_include
				characteristic_address	ÌØÐÔÖ¸Õë£¬Ö¸Ïò´úÂëÖÐµÄ_gatt_database_characteristic
				value_address	ÌØÐÔÖµÖ¸Õë£¬Ö¸Ïò´úÂëÖÐµÄ_gatt_database_value
*ËµÃ÷£º Ïà¶ÔÓÚ´úÂë×ÊÔ´ÓÐµã½ôÕÅµÄSYD8801£¬SYD8821µÄ´úÂë×ÊÔ´×ã¹»³ä×ã£¬ËùÒÔÕâÀïÖ±½Ó°ÑBLEµÄprofileÖ±½Ó
·Åµ½´úÂëÇøÓòÖÐ£¬ÕâÑùÔ­À´»ñÈ¡profileÊÇÍ¨¹ý¶ÁÈ¡flashµÄ·½Ê½£¬ÏÖÔÚÊÇÍ¨¹ýÖ±½Ó´úÂëÖ¸ÕëµÄ·½Ê½È¥»ñÈ¡£¬ËÙ¶È
ºÍ¿ÉÐÞ¸ÄÐÔ¶¼ÓÐºÃ´¦¡£Ö÷Òª·þÎñ¡¢ÌØÐÔ¡¢ÌØÐÔÖµµÈÍ¨¹ý¹¤¾ß¡¶BtGatt.exe¡·»ñÈ¡
				gap_profile_structÄÒÀ¨ÁËÕû¸öprofileµÄ¸÷¸öÔªËØ
**************************************************************************************************/
struct gap_profile_struct {
	uint32_t report_handle_address;
	uint32_t primary_address;
	uint32_t include_address;
	uint32_t characteristic_address;
	uint32_t value_address;
};





/*************************************************************************************************
*Ã¶¾ÙÃû£ºGAP_IO_CAPABILITY
*³ÉÔ±£º 
																		 \Local output |										|
													Local input \capacity    |   No output        |  Numeric output
													capacity		 \           |                    |
													-------------------------|--------------------|-------------------
													No input                 | NoInputNoOutput    |   DisplayOnly
													-------------------------|--------------------|-------------------
													Yes/No                   |  NoInputNoOutput		|		DisplayYesNo 
													-------------------------|--------------------|-------------------
													Keyboard 								 |	 KeyboardOnly 		|			KeyboardDisplay
*ËµÃ÷£º ¸ÃÃ¶¾Ù´ú±í±¾»úÖ§³ÖµÄ¼ÓÃÜµÄIOÄÜÁ¦£¨ÄÜ¹»ÒÔºÎÖÖ·½Ê½ÊäÈëÃÜÂë£©	
**************************************************************************************************/
enum GAP_IO_CAPABILITY {
	IO_DISPLAY_ONLY		  = 0x00,
	IO_DISPLAY_YESNO		= 0x01,
	IO_KEYBOARD_ONLY		= 0x02,
	IO_NO_INPUT_OUTPUT	= 0x03,	
	IO_KEYBOARD_DISPLAY	= 0x04,	
};



/*************************************************************************************************
*Ã¶¾ÙÃû£ºGAP_OOB_FLAG
*³ÉÔ±£º OOB_AUTH_NOT_PRESENT	Ã»ÓÐODB¹¦ÄÜ
				OOB_AUTH_PRESENT	ÐèÒªODB¹¦ÄÜ
*ËµÃ÷£º ËùÎ½µÄODB¹¦ÄÜ¾ÍÊÇÒÔ´øÍâµÄÍ¨Ñ¶·½Ê½´«ÊäÃÜ³×£¬±ÈÈçBLEµÄÃÜ³×Í¨¹ý´®¿Ú·¢ËÍ£¬ÕâÑùµ¥µ¥ÒÔBLEµÄÍ¨Ñ¶
£¨BLE×¥°ü¹¤¾ß£©À´ÆÆ½âBLE¾Í²»¿ÉÄÜ³É¹¦ÁË
**************************************************************************************************/
enum GAP_OOB_FLAG {
	OOB_AUTH_NOT_PRESENT= 0x00,
	OOB_AUTH_PRESENT		= 0x01,
};



/*************************************************************************************************
*Ã¶¾ÙÃû£ºGAP_BONDING_FLAGS
*³ÉÔ±£º AUTHREQ_NO_BONDING	¼ÓÃÜÍê³Éºó²»ÐèÒª°ó¶¨
				AUTHREQ_BONDING	¼ÓÃÜÍê³ÉºóÐèÒª°ó¶¨
*ËµÃ÷£º Èç¹ûÉèÖÃÁË°ó¶¨¹¦ÄÜ£¬ÔÙ´ÎÁ¬½ÓµÄÊ±ºò¾Í²»ÐèÒªÔÙ´Î½øÐÐÅä¶ÔµÄ¹ý³ÌÁË£¬Åä¶ÔÐÅÏ¢±£´æÔÚÐ¾Æ¬ÄÚ²¿µÄflash
**************************************************************************************************/
enum GAP_BONDING_FLAGS {
	AUTHREQ_NO_BONDING	= 0x00,
	AUTHREQ_BONDING		  = 0x01,
};




/*************************************************************************************************
*Ã¶¾ÙÃû£ºGAP_KEY_DISTRIBUTION
*³ÉÔ±£º  GAP_KEY_MASTER_IDEN	ÃÜ³×·ÖÅäµÄÊ±ºòÊÇ·ñ½»»»EDIV
	     GAP_KEY_ADDR_INFO	ÃÜ³×·ÖÅäµÄÊ±ºòÊÇ·ñ½»»»ADDR
	     GAP_KEY_SIGNIN_INFO	ÃÜ³×·ÖÅäµÄÊ±ºòÊÇ·ñ½»»»SIGNINÐÅÏ¢
*ËµÃ÷£º Åä¶ÔÍê³ÉºóÖ÷´ÓË«·½ÔÚÐèÒªµÄÊ±ºò»á½»»»Ò»Ð©ÐÅÏ¢£¬°üÀ¨IDENºÍADDRÒÔ¼°SIGNIN£¬ÕâÐ©ÐÅÏ¢ÊÇ¿ÉÑ¡µÄ£¬
ÁíÍâµÄLTKµÈÊÇ±ØÐëÒªµÄ¡£
**************************************************************************************************/
enum GAP_KEY_DISTRIBUTION {
	GAP_KEY_MASTER_IDEN	= 0x01,
	GAP_KEY_ADDR_INFO	  = 0x02,
	GAP_KEY_SIGNIN_INFO	= 0x04,
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_pairing_req
*³ÉÔ±£º io	Åä¶Ô¹ý³ÌÖÐµÄIOÄÜÁ¦£¬ÎªÃ¶¾ÙGAP_IO_CAPABILITY³ÉÔ±
				oob	Åä¹ý¹ý³ÌÖÐÊÇ·ñÐèÒªODB£¬ÎªÃ¶¾ÙGAP_OOB_FLAG³ÉÔ±
				flags Åä¶Ô¹ý³ÌÖÐÊÇ·ñÐèÒª°ó¶¨
							Bonding_Flags |
							b1b0          |   Bonding Type
							--------------|------------------
							00            |   No Bonding
							01 						|	 Bonding
							10 						|   Reserved
							11            |   Reserved
				mitm	Åä¶Ô¹ý³ÌÖÐÊÇ·ñÐèÒªÖÐ¼äÈË±£»¤set to one if the device is requesting	MITM protection, 
							otherwise it shall be set to 0
				sc	Åä¶Ô¹ý³ÌÖÐÊÇ·ñÊ¹ÓÃ°²È«Á¬½ÓÅä¶Ô£¬ÕâÊÇBLE4.2Ôö¼ÓµÄ£¬ set to one to request LE Secure 
Connection pairing, othe rwise it shall be set to 0 based on the supported features of the 
initiator and responder
				keypress	 Åä¶Ô¹ý³ÌÖÐÊÇ·ñÊ¹ÓÃPasskey Entry 
				rsvd	±£ÁôÎ»
				max_enc_sz	Åä¶Ô¹ý³ÌÖÐ×î´óµÄÃÜ³×³¤¶È£¬The maximum key size shall be in the range 7 to 16 octets.
				init_key	Åä¶Ô¹ý³ÌÖÐ³õÊ¼Õß£¨Ö÷»ú£©·ÖÅäµÄÃÜ³×
				rsp_key	Åä¶Ô¹ý³ÌÖÐ´Ó»ú·ÖÅäµÄÃÜ³×
*ËµÃ÷£º SYD8821µÄ°²È«²ÎÊýÔÚÆô¶¯µÄÊ±ºòµ÷ÓÃapi:gap_s_security_parameters_set½øÐÐÉèÖÃ£¬ÔÚÅä¶Ô¿ªÊ¼µÄÊ±ºò
µ÷ÓÃapi:gap_s_security_req¿ªÊ¼Åä¶ÔÁ÷³Ì£¬ÆäÖÐgap_s_security_reqº¯ÊýÒ²ÓÐflagsºÍmitmÁ½¸ö²ÎÊý£¬ÕâÁ½¸ö²ÎÊý
ºÍ°²È«²ÎÊýÀïµÄ²ÎÊýÓÐ³åÍ»£¬ÕâÀïÓÅÏÈÊ¹ÓÃµ÷ÓÃgap_s_security_req´«ÈëµÄº¯Êý
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
*½á¹¹ÌåÃû£ºgap_connection_param_rsp_pdu
*³ÉÔ±£º Interval_Min	Á¬½Ó²ÎÊýÖÐµÄÁ¬½Ó¼ä¸ô×îÐ¡Öµ
				Interval_Max	Á¬½Ó²ÎÊýÖÐµÄÁ¬½Ó¼ä¸ô×î´óÖµ
				Latency	Á¬½Ó²ÎÊýÖÐµÄlatency
				Timeout	Á¬½Ó²ÎÊýÖÐµÄTimeout
				PeferredPeriodicity	Á¬½Ó²ÎÊýÖÐµÄPreferredPeriodicity
				ReferenceConnEventCount Á¬½Ó²ÎÊýÖÐµÄReferenceConnEventCount
				Offset Á¬½Ó²ÎÊýÖÐµÄOffsetÊý×é
*ËµÃ÷£º BLE4.2Ôö¼ÓÁËLL_CONNECTION_PARAM_REQºÍLL_CONNECTION_PARAM_RSPÁ½¸öLL²ãµÄÊý¾Ý°ü£¬¶ÔÁ¬½Ó²ÎÊý×ö
ÁË¸ü´óµÄÍêÉÆ£¬Á½ÕßµÄ²ÎÊýÊÇÒ»ÑùµÄ¡£
				gap_connection_param_rsp_pdu½á¹¹ÌåµÄ¾ßÌåÊý¾Ý×îºó»áÓÃÔÚLL_CONNECTION_PARAM_RSPÃüÁîÉÏ£¬Èç¹û¶Ô
·½Ê½BLE4.0»òÕß²»Ö§³ÖLL_CONNECTION_PARAMµÄÃüÁî£¬¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ
				½¨ÒéÔÚÐ­ÒéÕ»³õÊ¼»¯º¯ÊýÖÐµ÷ÓÃapi:gap_s_connection_param_set½øÐÐÉèÖÃ
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
*½á¹¹ÌåÃû£ºgap_update_params
*³ÉÔ±£º updateitv_min	Á¬½Ó²ÎÊý¸üÐÂÖÐµÄÁ¬½Ó¼ä¸ô×îÐ¡Öµ
				updateitv_max	Á¬½Ó²ÎÊý¸üÐÂÖÐµÄÁ¬½Ó¼ä¸ô×î´óÖµ
				updatelatency	Á¬½Ó²ÎÊý¸üÐÂÖÐµÄlatency
				updatesvto	Á¬½Ó²ÎÊý¸üÐÂÖÐµÄ³¬Ê±Ê±¼ä
*ËµÃ÷£º SYD8821µÄÁ¬½Ó²ÎÊýÊ¹ÓÃapi:gap_s_connection_update½øÐÐÉèÖÃ£¬ÕâÀï¸üÈ·ÇÐµÄÊÇÁ¬½Ó²ÎÊý¸üÐÂ£¬BLEÖÐ
ÓÐËµÃ÷BLEµÄ´Ó»úÃ»ÓÐÖ±½Ó·¢ËÍLL²ãµÄLL_CONNECTION_UPDATE_REQÃüÁî£¬Ö»ÄÜ¹»Í¨¹ýL2CAPµÄÐÅÁîÍ¨µÀ·¢ËÍ
CONNECTION PARAMETER UPDATE REQUESTÃüÁî¸üÐÂÁ¬½Ó²ÎÊý£¬¸Ã½á¹¹ÌåÎªCONNECTION PARAMETER UPDATE REQUEST
ÃüÁîÖÐµÄ¸÷¸ö²ÎÊý
**************************************************************************************************/
struct gap_update_params {
	uint16_t  updateitv_min;
	uint16_t  updateitv_max;
	uint16_t  updatelatency;
	uint16_t  updatesvto;
};




/*************************************************************************************************
*Ã¶¾ÙÃû£ºGAP_SMART_CONTROL_SET
*³ÉÔ±£º SMART_CONTROL_LATENCY	Ê¹ÄÜlatency×Ô¶¯¿ØÖÆ»úÖÆ
				SMART_CONTROL_UPDATE	Ê¹ÄÜÁ¬½Ó²ÎÊý×Ô¶¯¿ØÖÆ
*ËµÃ÷£º ¸ÃÃ¶¾ÙÊ¹ÓÃbitmaskµÄÐÎ£¬¸ÃÃ¶¾ÙÖ»ÓÃÓÚsmart_update_latency£¬¿ÉÒÔÊ¹ÓÃ»òµÄÐÎÊ½´«Èë£¬±ÈÈç¿É´«Èë£º
				SMART_CONTROL_LATENCY|SMART_CONTROL_UPDATE
**************************************************************************************************/
enum GAP_SMART_CONTROL_SET {
	SMART_CONTROL_LATENCY	= 0x80,
	SMART_CONTROL_UPDATE	= 0x40,
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_smart_update_params
*³ÉÔ±£º updatectrl	ÉèÖÃsmart_update_latencyµÄ¹ÜÀí·¶Î§£¬ÎªÃ¶¾ÙGAP_SMART_CONTROL_SETµÄ³ÉÔ±»òÖµ
				updateadj_num	ÉèÖÃsmart_update_latencyµÄµ÷Õû´ÎÊý£¬Ò»°ã´«Èë±¾ÎÄ¼þµÄºê£ºMAX_UPDATE_ADJ_NUM
				updateitv_target	ÉèÖÃÄ¿±êÁ¬½Ó²ÎÊý¼ä¸ô£¬smart_update_latency»áÈÃBLEµÄÁ¬½Ó²ÎÊý×î´óµÄ½Ó½ü
													¸Ã²ÎÊý
				updatelatency	ÉèÖÃÁ¬½Ó²ÎÊýµÄlatency£¬smart_update_latency»áÖ±½Ó°Ñ¸Ã²ÎÊýÓÃÓÚÁ¬½Ó²ÎÊýÇëÇóÖÐ
				updatesvto ÉèÖÃÁ¬½Ó²ÎÊýµÄ³¬Ê±Ê±¼ä£¬smart_update_latency»áÖ±½Ó°Ñ¸Ã²ÎÊýÓÃÓÚÁ¬½Ó²ÎÊýÇëÇóÖÐ
*ËµÃ÷£º ¸Ã½á¹¹ÌåÖ»ÓÃÓÚsmart_update_latency£¬Ð­ÒéÕ»»á¸ù¾Ý¸Ã½á¹¹Ìå×Ô¶¯¹ÜÀíÁ¬½Ó²ÎÊý£¬¾­¹ýµ÷Õûºó×îÖÕÁ¬½Ó
²ÎÊý¿ÉÄÜ²¢²»ÄÜ¹»ÍêÈ«¸Ã½á¹¹ÌåµÄÒªÇó£¬µ«ÊÇ»á×î´ó³Ì¶ÈµÄ½Ó½ü¸Ã½á¹¹ÌåµÄ²ÎÊý
**************************************************************************************************/
struct gap_smart_update_params {
	uint8_t 	updatectrl;
	uint8_t 	updateadj_num;
	uint16_t  updateitv_target;
	uint16_t  updatelatency;
	uint16_t  updatesvto;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_link_params
*³ÉÔ±£º interval	Ä¿Ç°Á´Â·²ãµÄÁ¬½Ó²ÎÊý¼ä¸ô
				latency		Ä¿Ç°Á´Â·²ãµÄÁ¬½Ó²ÎÊýµÄlatency
				svto	Ä¿Ç°Á´Â·²ãµÄÁ¬½Ó²ÎÊýµÄ³¬Ê±Ê±¼ä
*ËµÃ÷£º SYD8821µÄÐ­ÒéÕ»±£´æÓÐµ±Ç°µÄÁ¬½Ó²ÎÊý£¬¿ÉÍ¨¹ýgap_s_link_parameters_getº¯Êý»ñÈ¡£¬¸Ã²ÎÊý»áÔÚÊÕµ½
LL²ãµÄLL_CONNECTION_UPDATE_REQÃüÁîµÄÊ±ºò½«»á±»¸üÐÂ
**************************************************************************************************/
struct gap_link_params {	
	uint16_t	interval;
	uint16_t	latency;
	uint16_t	svto;
};




/*************************************************************************************************
*Ã¶¾ÙÃû£º_GAP_EVT_
*³ÉÔ±£º GAP_EVT_ADV_END	À¶ÑÀ¹ã²¥Íê³ÉÊÂ¼þ£¬µ±¹ã²¥²ÎÊýÖÐµÄ³¬Ê±Ê±¼äµ½À´ºóÐ­ÒéÕ»ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ADV_REPORT		À¶ÑÀÉ¨ÃèÊÂ¼þ£¬
				GAP_EVT_CONNECTED	À¶ÑÀÁ¬½ÓÊÂ¼þ£¬µ±Á¬½Ó³É¹¦µÄÊ±ºòÐ­ÒéÕ»ÉÏ±¨¸ÃÊÂ¼þ£¬¸ù¾Ý¹æ·¶µÄËµÃ÷£ºÀ¶ÑÀÁ¬½Ó
													³É¹¦µÄ±êÖ¾²¢²»ÊÇÊÕµ½Ö÷»ú·¢ËÍCONNECT_REQµÄÊ±¿Ì£¬¶øÊÇÔÚ½ÓÊÕµ½CONNECT_REQÃü
													Áîºó»¹Òª½»»¥Ò»´ÎÊý¾Ý°ü²ÅËãÉÏÕæÕýµÄÁ¬½Ó³É¹¦
				GAP_EVT_DISCONNECTED	À¶ÑÀ¶ÏÏßÊÂ¼þ£¬Ð­ÒéÕ»»áÍ¬Ê±ÉÏ±¨¶ÏÏßµÄÔ­Òò£¬·ûºÏHCIµÄ¶ÏÏßÔ­Òò
				GAP_EVT_ENC_KEY	À¶ÑÀÅä¶ÔÍê³ÉÊÂ¼þ£¬BLEÅä¶ÔµÄ×îºó»á½øÐÐÖ÷´ÓÃÜ³×µÄ½»»»£¬½»»»Íê³ÉºóÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_PASSKEY_REQ	À¶ÑÀÃÜÂëÊäÈëÊÂ¼þ£¬Èç¹ûBLEÊ¹ÓÃÃÜÂë¼ÓÃÜ£¬ÔÚÐèÒªÊäÈëÃÜÂëµÄÊ±ºòÉÏ±¨¸ÃÊÂ¼þ
														ÇëÇóÊäÈëÃÜÂë
				GAP_EVT_ENC_START	À¶ÑÀ¿ªÊ¼¼ÓÃÜÊÂ¼þ£¬Åä¶ÔÍê³É»òÕß°ó¶¨ÖØÁ¬ºóÖ÷»ú»áÒªÇó¿ªÊ¼¼ÓÃÜ
				GAP_EVT_CONNECTION_EVENT	Á¬½Ó¼ä¸ôÊ±¿ÌÊÂ¼þ£¬·¢ÉúÔÚÁ¬½ÓÊÂ¼þ½áÊøµÄÊ±ºòÉÏ±¨
				GAP_EVT_CONNECTION_UPDATE_RSP	Á¬½Ó²ÎÊý¸üÐÂÏìÓ¦ÊÂ¼þ£¬SYD8821´Ó»ú·¢ËÍCONNECTION PARAMETER 
																		  UPDATE REQUESTºóÖ÷»ú»á×ö³öÏìÓ¦£¬ÊÕµ½ÏìÓ¦ºóÐ­ÒéÕ»ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ATT_READ	Ö÷»ú¶ÔSYD8821·¢ËÍÁË¶Á²Ù×÷£¬Ð­ÒéÕ»½«ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ATT_WRITE	Ö÷»ú¶ÔSYD8821·¢ËÍÁËÐ´²Ù×÷£¬Ð­ÒéÕ»½«ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ATT_PREPARE_WRITE	Ö÷»ú¶ÔSYD8821·¢ËÍÁËÔ¤±¸Ð´²Ù×÷£¬Ð­ÒéÕ»½«ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ATT_EXECUTE_WRITE	Ö÷»ú¶ÔSYD8821·¢ËÍÁËÖ´ÐÐÐ´²Ù×÷£¬Ð­ÒéÕ»½«ÉÏ±¨¸ÃÊÂ¼þ
				GAP_EVT_ATT_HANDLE_CONFIRMATION	À¶ÑÀÔÚÊÕµ½Ö¸Ê¾È·ÈÏÊ±ÉÏ±¨¸ÃÊÂ¼þ£¬±êÖ¾×ÅÖ¸Ê¾²Ù×÷Íê³É
				GAP_EVT_ATT_HANDLE_CONFIGURE	À¶ÑÀÔÚÊÕµ½ÅäÖÃCCCDÒ²¾ÍÊÇÖ÷»úÊ¹ÄÜnotify»òÕßÖ¸Ê¾¹¦ÄÜµÄÊ±ºòÉÏ±¨
																			¸ÃÊÂ¼þ£¬´ú±íÖ÷»úÒª¿ªÆôÏàÓ¦¹¦ÄÜ
*ËµÃ÷£º SYD8821Ð­ÒéÕ»³õÊ¼»¯µÄÊ±ºòÒªµ÷ÓÃapi:gap_s_evt_handler_setÉèÖÃgap_evt_callback½á¹¹Ìå£¬¸Ã½á¹¹Ìå
°üº¬ÁËÐ­ÒéÕ»ÉÏ±¨BLEÊÂ¼þµÄ½Ó¿Ú£¬SYD8821µÄÐ­ÒéÕ»ÔÚ·¢ÏÖBLEµÄ×´Ì¬·¢Éú±ä»¯£¨±ÈÈç·¢ÉúÁËÁ¬½Ó»òÕß¶ÏÏß£©µÄÊ±
ºò£¬Ð­ÒéÕ»½«»áÍùAPP²ã£¨ÓÃ»§´úÂë£©ÉÏ±¨BLEÊÂ¼þ£¬Í¬Ê±»¹»áÉÏ±¨gap_ble_evt½á¹¹Ìå£¬¸Ã½á¹¹ÌåÖÐµÄevt_code±ä
Á¿¾ÍÊÇ_GAP_EVT_Ã¶¾Ù¶¨ÒåµÄ³ÉÔ±
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
*Ã¶¾ÙÃû£ºCONNECTION_UPDATE_RSP_
*³ÉÔ±£º CONN_PARAM_ACCEPTED	Á¬½Ó²ÎÊý¸üÐÂ±»Ö÷»ú½ÓÊÕ
				CONN_PARAM_REJECTED	Á¬½Ó²ÎÊý¸üÐÂ±»Ö÷»ú¾Ü¾ø
				CONN_PARAM_SMART_TIMEROUT	ÖÇÄÜÁ¬½Ó²ÎÊý¹ÜÀíµ÷Õû³¬Ê±ÊÂ¼þ£¬¸ÃÀàÐÍµÄÊÂ¼þ²¢²»ÊÇ±ê×¼µÄBLEÊÂ¼þ£¬¶øÊÇ
																	Ëæ×Åsmart_update¹¦ÄÜ¶øÌí¼ÓµÄ£¬²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
				CONN_PARAM_SMART_SUCCEED	ÖÇÄÜÁ¬½Ó²ÎÊý¹ÜÀíµ÷Õû³É¹¦ÊÂ¼þ£¬¸ÃÀàÐÍµÄÊÂ¼þ²¢²»ÊÇ±ê×¼µÄBLEÊÂ¼þ£¬¶øÊÇ
																	Ëæ×Åsmart_update¹¦ÄÜ¶øÌí¼ÓµÄ£¬²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
				CONN_PARAM_LATENCY_ENABLE	ÖÇÄÜÁ¬½Ó²ÎÊý¹ÜÀíÊ¹ÄÜlatency£¬Ð­ÒéÕ»Ê¹ÄÜlatencyµÄÊ±ºòÉÏ±¨¸ÃÊÂ¼þ£¬
																	¸ÃÀàÐÍµÄÊÂ¼þ²¢²»ÊÇ±ê×¼µÄBLEÊÂ¼þ£¬¶øÊÇËæ×Åsmart_update¹¦ÄÜ¶øÌí¼ÓµÄ£¬
																	²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
				CONN_PARAM_LATENCY_DISABLE	ÖÇÄÜÁ¬½Ó²ÎÊý¹ÜÀíÊ§ÄÜlatency£¬Ð­ÒéÕ»Ê¹ÄÜlatencyµÄÊ±ºòÉÏ±¨¸ÃÊÂ¼þ£¬
																	¸ÃÀàÐÍµÄÊÂ¼þ²¢²»ÊÇ±ê×¼µÄBLEÊÂ¼þ£¬¶øÊÇËæ×Åsmart_update¹¦ÄÜ¶øÌí¼ÓµÄ£¬
																	²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
*ËµÃ÷£º BLE¹æ·¶ÖÐÖ»ÊÇ¶¨ÒåÁËCONN_PARAM_ACCEPTEDºÍCONN_PARAM_REJECTEDÁ½¸öÊÂ¼þ£¬ÒòÎªsmart_update_latency
»úÖÆÒ²ÐèÒªÉÏ±¨Ò»Ð©ÊÂ¼þ¸øÓ¦ÓÃ²ã£¬ËùÒÔÕâÀïÔÚCONNECTION_UPDATE_RSPÖÐÔö¼Ó¼¸¸öÉÏ±¨ÊÂ¼þ£¬Ôö¼ÓµÄÕâÐ©ÊÂ¼þÈç¹û
²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
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
*½á¹¹ÌåÃû£ºgap_disconnected_evt
*³ÉÔ±£º reason	¶ÏÏßÔ­Òò£¬¸ÃÔ­Òò·ûºÏHCIµÄERROR CODE
*ËµÃ÷£º BLE¶ÏÏßµÄÔ­ÒòÓÐºÜ¶à£¬SYD8821ÔÚ·¢Éú¶ÏÏßµÄÊ±ºò»á°Ñ¶ÏÏßµÄÔ­ÒòÉÏ±¨ÉÏÀ´£¬¸ÃÔ­Òò¿É×÷Îª²Î¿¼£¬¾ßÌå
µÄÔµÓÉ¿É¿´¹æ·¶µÄµÚ¶þÕÂµÄPart D ERROR CODESµÄµÚ¶þ½ÚERROR CODE DESCRIPTIONS
**************************************************************************************************/
struct gap_disconnected_evt {
	uint8_t	reason;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_read_evt
*³ÉÔ±£º primary	BLE¶ÁÊÂ¼þµÄÖ÷·þÎñµÄUUID
				uuid	BLE¶ÁÊÂ¼þµÄÌØÐÔUUID
				hdl	BLE¶ÁÊÂ¼þµÄÌØÐÔµÄhdl(¾ä±ú)
				offset	BLE¶ÁÊÂ¼þµÄÆ«ÒÆÖµ£¬¸ÃÏîÔÚÄ³Ð©¶Á²Ù×÷ÊÇÎÞÐ§µÄ
*ËµÃ÷£º BLE¶Á²Ù×÷µÄ¶ÔÏóÊÇBLEµÄÌØÐÔÖµ£¬Ó¦ÓÃ²ã¼È¿ÉÒÔÍ¨¹ýUUIDÀ´ÅÐ¶Ï¶Ô·½Ïë¶ÁÈ¡µÄ¾ßÌåÌØÐÔÖµ£¬Ò²¿ÉÒÔÍ¨¹ý
ÌØÐÔµÄ¾ä±úÀ´ÅÐ¶Ï¾ßÌåµÄÌØÐÔ£¬ÔÚÍ¬Ò»¸öprofileÖÐ¾ä±úÊÇÎ¨Ò»µÄ£¬µ«ÊÇ¿ÉÄÜ²»Í¬µÄprofile¼´Ê¹uuidÏàÍ¬µÄÌØÐÔ
ËûÃÇµÄ¾ä±úÒ²¿ÉÄÜÓÐ²îÒì¡£ÔÚÈÎºÎÒ»¸öprofile£¬ÌØÐÔµÄUUIDÊÇÒ»ÖÂµÄ£¬µ«ÊÇÔÚÍ¬Ò»¸öprofileÖÐ¿ÉÄÜ»áÓÐ¼¸¸öÏà
Í¬UUIDµÄÌØÐÔ£¬ÕâÊ±ºò¾ÍµÃÍ¨¹ýhdlÀ´ÅÐ¶Ïhdl,¶ÔÓÚ¸´ÔÓµÄprofile£¬¿ÉÅäºÏÊ¹ÓÃuuidºÍhdl
**************************************************************************************************/
struct gap_att_read_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	offset;
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_write_evt
*³ÉÔ±£º primary	BLEÐ´ÊÂ¼þµÄÖ÷·þÎñµÄUUID
				uuid	BLEÐ´ÊÂ¼þµÄÌØÐÔUUID
				hdl	BLEÐ´ÊÂ¼þµÄÌØÐÔµÄhdl(¾ä±ú)
				sz	BLEÐ´Êý¾ÝµÄ´óÐ¡
				data	BLEÐ´µÄ¾ßÌåÊý¾Ý
*ËµÃ÷£ºSYD8821µÄMAX_ATT_DATA_SZ£¨MTU)ÊÇ512¸öbyte£¬ÕâÐ©Êý¾ÝÊÇ·Åµ½¶ÑÕ»ÖÐµÄ£¬Õâ¾ÍÔö¼Ó¶ÑÕ»Òç³öµÄ¿ÉÄÜÐÔ£¬
ËùÒÔ²»Í¬µÄ¹¤³Ì£¬²»Í¬µÄÐèÇó£¬¿ÉÊÊµ±ÅäÖÃSYD8821µÄ¶ÑÕ»
		BLEÐ´²Ù×÷µÄ¶ÔÏóÊÇBLEµÄÌØÐÔÖµ£¬Ó¦ÓÃ²ã¼È¿ÉÒÔÍ¨¹ýUUIDÀ´ÅÐ¶Ï¶Ô·½Ïë¶ÁÈ¡µÄ¾ßÌåÌØÐÔÖµ£¬Ò²¿ÉÒÔÍ¨¹ý
ÌØÐÔµÄ¾ä±úÀ´ÅÐ¶Ï¾ßÌåµÄÌØÐÔ£¬ÔÚÍ¬Ò»¸öprofileÖÐ¾ä±úÊÇÎ¨Ò»µÄ£¬µ«ÊÇ¿ÉÄÜ²»Í¬µÄprofile¼´Ê¹uuidÏàÍ¬µÄÌØÐÔ
ËûÃÇµÄ¾ä±úÒ²¿ÉÄÜÓÐ²îÒì¡£ÔÚÈÎºÎÒ»¸öprofile£¬ÌØÐÔµÄUUIDÊÇÒ»ÖÂµÄ£¬µ«ÊÇÔÚÍ¬Ò»¸öprofileÖÐ¿ÉÄÜ»áÓÐ¼¸¸öÏà
Í¬UUIDµÄÌØÐÔ£¬ÕâÊ±ºò¾ÍµÃÍ¨¹ýhdlÀ´ÅÐ¶Ïhdl,¶ÔÓÚ¸´ÔÓµÄprofile£¬¿ÉÅäºÏÊ¹ÓÃuuidºÍhdl
**************************************************************************************************/
struct gap_att_write_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint8_t	sz;
	uint8_t	data[MAX_ATT_DATA_SZ];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_pre_write_evt
*³ÉÔ±£º primary	BLEÔ¤±¸Ð´ÊÂ¼þµÄÖ÷·þÎñµÄUUID
				uuid	BLEÔ¤±¸Ð´ÊÂ¼þµÄÌØÐÔUUID
				hdl	BLEÔ¤±¸Ð´ÊÂ¼þµÄÌØÐÔµÄhdl(¾ä±ú)
				sz	BLEÔ¤±¸Ð´Êý¾ÝµÄ´óÐ¡
				data	BLEÔ¤±¸Ð´µÄ¾ßÌåÊý¾Ý
*ËµÃ÷£ºBLEµÄÔ¤±¸Ð´ºÍÖ´ÐÐÐ´ÊÇÅäÌ×Ê¹ÓÃµÄ£¬ÒªÐ´Èë´óÅúÁ¿µÄÊý¾ÝµÄÊ±ºò¿ÉÄÜ¶Ô·½»áÏÈÍ¨¹ýÔ¤±¸Ð´ÃüÁî°ÑÊý¾Ý·Ö¶Î
·¢ËÍ¸øSYD8821£¬È»ºóÍ¨¹ýÖ´ÐÐÐ´À´°ÑÊäÈëÐ´ÈëÌØÐÔÖµ£¬ÕâÑù±£Ö¤Ð´ÈëÊý¾ÝµÄÔ­×ÓÐÔ
		SYD8821µÄMAX_ATT_DATA_SZ£¨MTU)ÊÇ512¸öbyte£¬ÕâÐ©Êý¾ÝÊÇ·Åµ½¶ÑÕ»ÖÐµÄ£¬Õâ¾ÍÔö¼Ó¶ÑÕ»Òç³öµÄ¿ÉÄÜÐÔ£¬
ËùÒÔ²»Í¬µÄ¹¤³Ì£¬²»Í¬µÄÐèÇó£¬¿ÉÊÊµ±ÅäÖÃSYD8821µÄ¶ÑÕ»
		BLEÔ¤±¸Ð´²Ù×÷µÄ¶ÔÏóÊÇBLEµÄÌØÐÔÖµ£¬Ó¦ÓÃ²ã¼È¿ÉÒÔÍ¨¹ýUUIDÀ´ÅÐ¶Ï¶Ô·½Ïë¶ÁÈ¡µÄ¾ßÌåÌØÐÔÖµ£¬Ò²¿ÉÒÔÍ¨¹ý
ÌØÐÔµÄ¾ä±úÀ´ÅÐ¶Ï¾ßÌåµÄÌØÐÔ£¬ÔÚÍ¬Ò»¸öprofileÖÐ¾ä±úÊÇÎ¨Ò»µÄ£¬µ«ÊÇ¿ÉÄÜ²»Í¬µÄprofile¼´Ê¹uuidÏàÍ¬µÄÌØÐÔ
ËûÃÇµÄ¾ä±úÒ²¿ÉÄÜÓÐ²îÒì¡£ÔÚÈÎºÎÒ»¸öprofile£¬ÌØÐÔµÄUUIDÊÇÒ»ÖÂµÄ£¬µ«ÊÇÔÚÍ¬Ò»¸öprofileÖÐ¿ÉÄÜ»áÓÐ¼¸¸öÏà
Í¬UUIDµÄÌØÐÔ£¬ÕâÊ±ºò¾ÍµÃÍ¨¹ýhdlÀ´ÅÐ¶Ïhdl,¶ÔÓÚ¸´ÔÓµÄprofile£¬¿ÉÅäºÏÊ¹ÓÃuuidºÍhdl
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
*½á¹¹ÌåÃû£ºgap_att_exec_write_evt
*³ÉÔ±£º flags	0x00:Cancel all prepared writes 0x01:Immediately write all pending prepared values
*ËµÃ÷£º µ±Ô¤±¸Ð´½áÊøºó»á·¢ËÍÖ´ÐÐÐ´À´Ö´ÐÐÌØÐÔÖµÐ´ÈëµÄ²Ù×÷£¬¸ÃÃüÁî´øÓÐÒ»¸ö²ÎÊý£¬±íÊ¾ÊÇ·ñÐ´ÈëÖ®Ç°µÄ
Êý¾Ý
**************************************************************************************************/
struct gap_att_exec_write_evt {
	uint8_t	flags;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_handle_configure_evt
*³ÉÔ±£º uuid	BLEÅäÖÃCCCDÊÂ¼þµÄ·þÎñµÄUUID
				hdl	BLEÅäÖÃCCCDÊÂ¼þµÄÃèÊö·ûµÄhdl(¾ä±ú)
				value	¶ÔCCCDµÄÐÐÎª£¬Õâ¸ö±äÁ¿ÊÇbitmaskµÄÐÎÊ½£¬ÎªÃ¶¾ÙBLE_SEND_TYPEµÄ³ÉÔ±»òÕßÆä³ÉÔ±µÄ»òÖµ
*ËµÃ÷£º ¸ù¾Ý¹æ·¶£¬BLEµÄnotify(Í¨Öª£©»òÕßindicate£¨Ö¸Ê¾£©µÄÊ¹ÄÜ»òÕßÊ§ÄÜ¶¼ÊÇÓÉÖ÷»úÒ²¾ÍÊÇmaster¿ØÖÆµÄ£¬
µ±Ö÷»ú¶ÔSYD8821µÄnotify»òÕßindicateµÄ¿ª¹Ø½øÐÐ²Ù×÷µÄÊ±ºò£¬µ×²ãÐ­ÒéÕ»»áÉÏ±¨¸ÃÊÂ¼þ
				handle_configure²Ù×÷µÄ¶ÔÏóÊÇBLEµÄÃèÊö·û£¬Ó¦ÓÃ²ã¼È¿ÉÒÔÍ¨¹ýUUIDÀ´ÅÐ¶Ï¶Ô·½Ïë¶ÁÈ¡µÄ¾ßÌåµÄ·þÎñ
ÃèÊö·û£¬µ±Ê±ÕâÖÖ·½Ê½¶ÔÓÚÒ»¸ö·þÎñÏÂÃæÓÐ¶à¸ö´øÓÐCCCDµÄÃèÊö·û£¬ÕâÖÖ·½Ê½¾Í²»¿ÉÐÐÁË£¬ÕâÖÖÇé¿öÏÂÖ»ÄÜ¹»Í¨
¹ýÃèÊö·ûµÄ¾ä±úÀ´ÅÐ¶Ï¾ßÌåµÄÃèÊö·û,¶ÔÓÚ¸´ÔÓµÄprofile£¬¿ÉÅäºÏÊ¹ÓÃuuidºÍhdl
				Ò»¸öÌØÐÔ¿ÉÄÜ´æÔÚnotifyºÍindicateÁ½ÖÖ£¬ËùÒÔÕâÀïµÃÍ¨¹ýÅÐ¶Ï¾ßÌåµÄÎ»À´ÅÐ¶ÏÊÇ·ñÊ¹ÄÜ»òÕßÊ§ÄÜ¡£±ÈÈç£º
Èç¹ûvalue=0x03£¬ÄÇ¾ÍÊÇÍ¬Ê±Ê¹ÄÜnotifyºÍindicate£¬value=0x01Ôòµ¥µ¥Ê¹ÄÜnotify
**************************************************************************************************/
struct gap_att_handle_configure_evt {
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	value;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_connection_update_rsp_evt
*³ÉÔ±£º result	BLEÁ¬½Ó²ÎÊý¸üÐÂ½á¹û £¬ÎªÃ¶¾ÙCONNECTION_UPDATE_RSP_µÄ³ÉÔ±
*ËµÃ÷£º SYD8821ÊÕµ½CONNECTION PARAMETER UPDATE RESPONSEÏìÓ¦»áÉÏ±¨GAP_EVT_CONNECTION_UPDATE_RSPÊÂ¼þ£¬
°éËæ¸ÃÊÂ¼þ»á´ø×Å¸Ã½á¹¹ÌåµÄ²ÎÊý£¬¿ÉÍ¨¹ý¸Ã²ÎÊýÅÐ¶ÏÖ÷»úÊÇ·ñÍ¬ÒâÁ¬½Ó²ÎÊý¸üÐÂ
				BLE¹æ·¶ÖÐÖ»ÊÇ¶¨ÒåÁËCONN_PARAM_ACCEPTEDºÍCONN_PARAM_REJECTEDÁ½¸öÊÂ¼þ£¬ÒòÎªsmart_update_latency
»úÖÆÒ²ÐèÒªÉÏ±¨Ò»Ð©ÊÂ¼þ¸øÓ¦ÓÃ²ã£¬ËùÒÔÕâÀïÔÚCONNECTION_UPDATE_RSPÖÐÔö¼Ó¼¸¸öÉÏ±¨ÊÂ¼þ£¬Ôö¼ÓµÄÕâÐ©ÊÂ¼þÈç¹û
²»Ê¹ÓÃsmart_update¹¦ÄÜ²»»áÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
struct gap_connection_update_rsp_evt {
	uint16_t result;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_advertising_report_evt
*³ÉÔ±£º type	É¨ÃèÉÏ±¨ÊÂ¼þµÄÀàÐÍ
				peer_dev_addr	¹ã²¥ÕßµØÖ·
				len	É¨Ãèµ½µÄ¹ã²¥Êý¾ÝµÄ³¤¶È
				buf	É¨Ãèµ½µÄ¹ã²¥µÄÊý¾Ý
				rssi	É¨Ãèµ½µÄ¹ã²¥µÄÐÅºÅÇ¿¶È
*ËµÃ÷£º SYD8821ÔÚÉ¨ÃèµÄÊ±ºòÈç¹ûÊÕµ½ÕýÈ·µÄ¹ã²¥ÐÅºÅ»áÉÏ±¨ÏàÓ¦µÄÊÂ¼þ£¬¸ÃÊÂ¼þ°üº¬ÓÐ¸Ã½á¹¹Ìå£¬²»¹ýÓÐÐ©
³ÉÔ±ÔÚ²»Í¬µÄ¹ã²¥ÀàÐÍÏÂÊÇÃ»ÓÐµÄ
**************************************************************************************************/
struct gap_advertising_report_evt {
	uint8_t type;
	struct gap_ble_addr	peer_dev_addr;
	int8_t	len;
	uint8_t	buf[MAX_ADV_DATA_SZ];
	uint8_t	rssi;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_handle_confirmation_evt
*³ÉÔ±£º primary	BLEÖ¸Ê¾È·ÈÏÊÂ¼þµÄÖ÷·þÎñµÄUUID
				uuid	BLEÖ¸Ê¾È·ÈÏÊÂ¼þµÄÌØÐÔUUID
				hdl	BLEÖ¸Ê¾È·ÈÏÊÂ¼þµÄÌØÐÔµÄÌØÐÔÖµµÄhdl(¾ä±ú)
*ËµÃ÷£º BLEµÄÖ¸Ê¾µÄ²½Öè±ÈÍ¨Öª¹¦ÄÜ¶àÁËÒ»¸ö·µ»ØµÄ¹ý³Ì£¬µ±SYD8801ÊÕµ½Ö÷»ú·µ»ØÀ´µÄÏìÓ¦Ê±ÉÏ±¨
GAP_EVT_ATT_HANDLE_CONFIRMATIONÊÂ¼þ£¬¸ÃÊÂ¼þ°üº¬¸Ã½á¹¹Ìå¡£
				Èç¹û°´ÕÕ¹æ·¶£¬GAP_EVT_ATT_HANDLE_CONFIRMATIONÏìÓ¦²»»áÓÐÈÎºÎµÄÊý¾Ý£¬ËùÒÔ¸Ã½á¹¹ÌåµÄÄÚÈÝÊÇ±£´æ
ÔÚÁ´Â·²ãµÄ»º³åÇøÖÐ£¬ÓÉAPP´úÂëÔÚ·¢ËÍÖ¸Ê¾µÄÊ±ºò´«¸øÐ­ÒéÕ»µÄ				
**************************************************************************************************/
struct gap_att_handle_confirmation_evt {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_ble_evt
*³ÉÔ±£º evt_type	Ð­ÒéÕ»ÉÏ±¨ÊÂ¼þµÄÀàÐÍ£¬¶ÔÓÚAPPÖ»»áÉÏ±¨GAPÊÂ¼þ£¬ËùÒÔÖ»»áGAPÊÂ¼þ
				evt_code	Ð­ÒéÕ»ÉÏ±¨ÊÂ¼þµÄ±êÊ¶Âë£¨code)
				evt	Ð­ÒéÕ»ÉÏ±¨ÊÂ¼þµÄ¾ßÌåÄÚÈÝ£¬ÊÇÒ»¸öÁªºÏÌå£¬°üº¬ÓÐ¸÷ÖÖÊÂ¼þµÄÊý¾Ý
					disconn_evt	¶ÏÏßÊÂ¼þÊý¾Ý
					bond_dev_evt	°ó¶¨Éè±¸µÄµØÖ·£¬¸ÃÊý¾ÝÔÚÁ¬ÏßµÄÊ±ºò»áÉÏ±¨¸Ã½á¹¹Ìå£¬Ö¸Ê¾¶Ô·½µÄÉè±¸µØÖ·
					enc_key_evt	ÃÜ³×·ÖÅäÊÂ¼þÊý¾Ý£¬°üº¬ÁËÅä¶Ô¹ý³ÌµÄ¸÷¸öÃÜ³×
					att_read_evt	BLE¶Á²Ù×÷ÊÂ¼þ£¬Ö¸Ê¾¾ßÌåµÄÒª¶ÁÈ¡µÄ¶ÔÏó£¬UUID,hdlµÈ
					att_write_evt	BLEÐ´²Ù×÷ÊÂ¼þ£¬Ö¸Ê¾¾ßÌåµÄÒª²Ù×÷µÄ¶ÔÏóºÍÏàÓ¦µÄÊý¾Ý£¬UUID,hdlÒÔ¼°Êý¾ÝµÈ
					att_pre_write_evt	BLEÔ¤±¸Ð´²Ù×÷ÊÂ¼þ£¬Ö¸Ê¾¾ßÌåµÄÒª²Ù×÷µÄ¶ÔÏóºÍÏàÓ¦µÄÊý¾Ý£¬UUID,hdlÒÔ¼°Êý¾ÝµÈ
					att_exec_write_evt	BLEÖ´ÐÐÐ´²Ù×÷ÊÂ¼þ£¬Ö¸Ê¾¾ßÌåµÄÒª²Ù×÷µÄ¶ÔÏóºÍÏàÓ¦µÄÊý¾Ý£¬UUID,hdlÒÔ¼°Êý¾ÝµÈ
					att_handle_config_evt	BLE¿ª¹Ønotify(Í¨Öª£©»òÕßindicate£¨Ö¸Ê¾£©ÊÂ¼þ£¬
					att_handle_confirmation_evt	BLEÖ¸Ê¾¹¦ÄÜÈ·ÈÏÏìÓ¦ÊÂ¼þ£¬°üº¬Á´Â·²ãµÄÊý¾Ý
					connection_update_rsp_evt	BLEÁ¬½Ó²ÎÊý¸üÐÂÏìÓ¦ÊÂ¼þ¾ßÌåÊý¾Ý
					advertising_report_evt	BLEÉ¨ÃèÉÏ±¨ÊÂ¼þ
*ËµÃ÷£º SYD8821ÔÚÊÕµ½À¶ÑÀÊÂ¼þµÄÊ±ºòÍ¨¹ýp_callbackÉÏ±¨À¶ÑÀÊÂ¼þ£¬gap_ble_evt½á¹¹ÌåÎªÀ¶ÑÀÊÂ¼þµÄ¾ßÌå
Êý¾Ý
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
*½á¹¹ÌåÃû£ºgap_evt_callback
*³ÉÔ±£º evt_mask	ÊÂ¼þÀàÐÍÆÁ±ÎÎ»£¬¸Ã±äÁ¿Ê¹ÓÃbitmaskÐÎÊ½£¬ÎªÃ¶¾Ù_GAP_EVT_µÄÖµ»òÕßÆäÖµµÄ»òÖµ
				p_callback	BLEÊÂ¼þÉÏ±¨µÄ¹¹Ôìº¯Êý£¬ÎªÒ»¸öº¯ÊýÖ¸Õë
*ËµÃ÷£º SYD8821ÔÚÊÕµ½À¶ÑÀÊÂ¼þµÄÊ±ºòÍ¨¹ýp_callbackÉÏ±¨À¶ÑÀÊÂ¼þ£¬gap_ble_evt½á¹¹ÌåÎªÀ¶ÑÀÊÂ¼þµÄ¾ßÌå
Êý¾Ý
**************************************************************************************************/
struct gap_evt_callback {
	uint32_t	evt_mask;
	void 	(*p_callback)(struct gap_ble_evt *p_evt);
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_att_report
*³ÉÔ±£º primary	BLE±¨¸æ½á¹¹ÌåµÄÖ÷·þÎñµÄUUID
				uuid	BLE±¨¸æ½á¹¹ÌåµÄÌØÐÔUUID
				hdl	BLE±¨¸æ½á¹¹ÌåµÄÌØÐÔµÄÌØÕ÷Öµhdl(¾ä±ú val_hdl)
				config	BLE±¨¸æ½á¹¹ÌåµÄÌØÐÔµÄCCCDÃèÊö·ûµÄhandle(¾ä±ú)
				value ¾ßÌåµÄÐÐÎª£¬ÓÐnotifyºÍindicateÁ½ÖÖ
*ËµÃ÷£º SYD8821µÄprofile°ÑnotifyºÍindicateÁ½ÖÖGATT serviceÐÐÎª¼¯ºÏ³ÆÎªreport£¬×¨ÃÅÉèÁ¢½á¹¹Ìå
gap_att_report_handle±£´æÁËreportµÄÏàÓ¦ÊôÐÔÐÅÏ¢£¬Ã¿¸öCCCD¶ÔÓ¦×Å½á¹¹Ìågap_att_report_handleÖÐµÄÒ»¸ö
Êý×é³ÉÔ±¡£°üÀ¨±¾CCCDËùÔÚµÄÎ»ÖÃ£¨ÆäÖ÷Òª·þÎñ£¬ÌØÐÔ£¬val_hdl£©µÈ
**************************************************************************************************/
struct gap_att_report {
	uint16_t	primary;
	uint16_t	uuid;
	uint16_t	hdl;
	uint16_t	config;
	uint16_t	value;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_err_rsp
*³ÉÔ±£º opcode	ATT´íÎóÏìÓ¦ÖÐµÄRequest Opcode In Error²ÎÊý
				hdl	ATT´íÎóÏìÓ¦ÖÐµÄAttribute Handle In Error²ÎÊý
				err	ATT´íÎóÏìÓ¦ÖÐµÄError Code
*ËµÃ÷£º	BLEÔÚ·¢ÏÖÇëÇóÌõ¼þ³ö´íµÄÊ±ºò»ØÓ¦Error ResponseÊÂ¼þ£¬¸ÃATTÃüÁîÖÐ°üº¬ÓÐ´íÎóµÄÔ­Òò»òÕßhdlµÈ
**************************************************************************************************/
struct att_err_rsp{
	uint8_t opcode;
	uint16_t hdl;
	uint8_t err;
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_find_by_type_val_req
*³ÉÔ±£º start_hdl		ATT¿Í»§¶ËFind By Type Value RequestÃüÁîµÄStarting Handle²ÎÊý
				end_hdl		ATT¿Í»§¶ËFind By Type Value RequestÃüÁîµÄEnding Handle²ÎÊý
				att_type	 ATT¿Í»§¶ËFind By Type Value RequestÃüÁîµÄAttribute Type²ÎÊý
				att_val		ATT¿Í»§¶ËFind By Type Value RequestÃüÁîµÄAttribute Value²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅFind By Type Value RequestµÄ¸÷¸ö²ÎÊý
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_find_by_type_val_req{
	uint16_t start_hdl;
	uint16_t end_hdl;
	uint16_t att_type;
	uint8_t att_val[MAX_ATT_DATA_SZ-7];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_mtu_rsp
*³ÉÔ±£º mtu		ATT²ãµÄExchange MTU ResponseÃüÁîµÄServer Rx MTU²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅExchange MTU ResponseµÄ¸÷¸ö²ÎÊý
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ä½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_mtu_rsp{
	uint16_t mtu;
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_find_info_128
*³ÉÔ±£º hdl		HandleÖµ
				uuid		16bitµÄUUIDÖµ
*ËµÃ÷£º	Find Information ResponseÃüÁîµÄformatÎª0x01Ê±£¬ÆäInformation DataÎª±¾½á¹¹Ìå
**************************************************************************************************/
struct att_find_info_16{
	uint16_t hdl;
	uint8_t uuid[2];
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_find_info_128
*³ÉÔ±£º hdl		HandleÖµ
				uuid		128bitµÄUUIDÖµ
*ËµÃ÷£º	Find Information ResponseÃüÁîµÄformatÎª0x02Ê±£¬ÆäInformation DataÎª±¾½á¹¹Ìå
**************************************************************************************************/
struct att_find_info_128{
	uint16_t hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_find_info_payload
*³ÉÔ±£º uuid16		16bituuidÏÂµÄInformation Data²ÎÊý
				uuid128		128bituuidÏÂµÄInformation Data²ÎÊý
*ËµÃ÷£º	±¾ÁªºÏÌå¶ÔÓ¦×ÅFind Information ResponseµÄInformation Data²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªformat²»Ò»Ñù
Information DataÓÐ¿ÉÄÜÊÇhandleºÍ16bit uuidµÄ×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄ×éºÏ
**************************************************************************************************/
union  att_find_info_payload {
	struct att_find_info_16   uuid16[5];
	struct att_find_info_128 uuid128;
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_find_info_rsp
*³ÉÔ±£º format		¸Ã²ÎÊý¾ö¶¨ÁËInformation DataµÄ³¤¶È
				pair		Information Data²ÎÊý
*ËµÃ÷£º	¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_find_info_rsp{
	uint8_t format;
	union att_find_info_payload pair;
};



/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_find_by_type_val_rsp
*³ÉÔ±£º list		Handles Information List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅFind By Type Value ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶listÊÇÒ»¸öFound Attribute HandleºÍ
Group End HandleµÄ×éºÏ
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_find_by_type_val_rsp{
	uint8_t list[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_read_by_type_16
*³ÉÔ±£º hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle²ÎÊý
				property		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊý£¬ÔÚ·¢ÏÖcharacteristicÊ±
										Attribute ValueÖÐµÄCharacteristic Properties
				val_hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊýÔÚ·¢ÏÖcharacteristicÊ±Attribute Value
									ÖÐµÄCharacteristic Value Attribute Handle
				char_uuid		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊýÔÚ·¢ÏÖcharacteristicÊ±Attribute Value
										ÖÐµÄCharacteristic UUID
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type Response²ÎÊýÖÐAttribute Data ListµÄ²ÎÊý
**************************************************************************************************/
struct att_read_by_type_16{
	uint16_t hdl;
	uint8_t property;
	uint16_t val_hdl;
	uint8_t char_uuid[2];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºatt_read_by_type_128
*³ÉÔ±£º hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle²ÎÊý
				property		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊý£¬ÔÚ·¢ÏÖcharacteristicÊ±
										Attribute ValueÖÐµÄCharacteristic Properties
				val_hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊýÔÚ·¢ÏÖcharacteristicÊ±Attribute Value
									ÖÐµÄCharacteristic Value Attribute Handle
				char_uuid		Attribute Data List²ÎÊýÖÐµÄAttribute Value²ÎÊýÔÚ·¢ÏÖcharacteristicÊ±Attribute Value
										ÖÐµÄCharacteristic UUID
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type Response²ÎÊýÖÐAttribute Data ListµÄ²ÎÊý
**************************************************************************************************/
struct att_read_by_type_128{
	uint16_t hdl;
	uint8_t property;
	uint16_t val_hdl;
	uint8_t char_uuid[16];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_payload
*³ÉÔ±£º uuid16		16bituuidÏÂµÄAttribute Data List²ÎÊý
				uuid128		128bituuidÏÂµÄAttribute Data List²ÎÊý
*ËµÃ÷£º	Read By Type ResponseÖÐµÄLength¾ö¶¨ÁËAttribute Data ListµÄ³¤¶È£¬Çø±ð16bit uuidºÍ1258bit uuid
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇcharacteristicµÄÇé¿ö
**************************************************************************************************/
union  att_read_by_type_payload {
	struct att_read_by_type_16   uuid16[3];
	struct att_read_by_type_128 uuid128;
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_rsp
*³ÉÔ±£º length		ATT²ãµÄRead By Type ResponseÃüÁîÏÂµÄLength²ÎÊý
				pair		ATT²ãµÄRead By Type ResponseÃüÁîÏÂµÄAttribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»Ñù£¬ÆäÖÐlength¾ö¶¨ÁË
Attribute Data ListµÄ³¤¶È
				Attribute Data ListÓÐ¿ÉÄÜÊÇhandleºÍ16bit uuidµÄcharacteristic×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄ
characteristic×éºÏ
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇcharacteristicµÄÇé¿ö
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_type_rsp{
	uint8_t length;
	union att_read_by_type_payload pair;
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_include_rsp
*³ÉÔ±£º length		ATT²ãµÄRead By Type ResponseÃüÁîÏÂµÄLength²ÎÊý
				hdl		ATT²ãµÄRead By Type ResponseÃüÁîÏÂµÄAttribute Data List²ÎÊýÖÐµÄAttribute Handle
				buf		ATT²ãµÄRead By Type ResponseÃüÁîÏÂµÄAttribute Data List²ÎÊýÖÐµÄAttribute Value
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»Ñù£¬ÆäÖÐlength¾ö¶¨ÁË
Attribute Data ListµÄ³¤¶È
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇ´ÎÒª·þÎñ£¨include£©µÄÇé¿ö
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_type_include_rsp{
	uint8_t length;
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-2];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_pair_val
*³ÉÔ±£º hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle
				val		Attribute Data List²ÎÊýÖÐµÄAttribute Value
*ËµÃ÷£º	±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇµ¥¸öpairÖµµÄÇé¿ö
**************************************************************************************************/
struct att_read_by_type_pair_val{
	uint16_t hdl;
	uint8_t val[10];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_val_rsp
*³ÉÔ±£º length		Length²ÎÊý
				pair		Attribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»Ñù£¬ÆäÖÐlength¾ö¶¨ÁË
Attribute Data ListµÄ³¤¶È
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇµ¥¸öpairÖµµÄÇé¿ö
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_type_val_rsp{
	uint8_t length;
	struct att_read_by_type_pair_val pair[1];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_service_16
*³ÉÔ±:hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle
			uuid	Attribute Data List²ÎÊýÖÐµÄAttribute Value
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇÖ÷Òª·þÎñµÄÇé¿ö
**************************************************************************************************/
struct att_read_by_type_service_16{
	uint16_t hdl;
	uint8_t uuid[2];
};



/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_service_128
*³ÉÔ±:hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle
			uuid	Attribute Data List²ÎÊýÖÐµÄAttribute Value
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇÖ÷Òª·þÎñµÄÇé¿ö
**************************************************************************************************/
struct att_read_by_type_service_128{
	uint16_t hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_service_payload
*³ÉÔ±:uuid16	16bituuidÏÂµÄAttribute Data List²ÎÊý
			uuid128	128bituuidÏÂµÄAttribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»ÑùAttribute Data ListÓÐ¿É
ÄÜÊÇhandleºÍ16bit uuidµÄÖ÷Òª·þÎñ×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄÖ÷Òª·þÎñ×éºÏ
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇÖ÷Òª·þÎñµÄÇé¿ö
**************************************************************************************************/
union  att_read_by_type_service_payload {
	struct att_read_by_type_service_16   uuid16[3];
	struct att_read_by_type_service_128 uuid128;
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_service_rsp
*³ÉÔ±:length	Length²ÎÊý
			pair	attribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»ÑùAttribute Data ListÓÐ¿É
ÄÜÊÇhandleºÍ16bit uuidµÄÖ÷Òª·þÎñ×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄÖ÷Òª·þÎñ×éºÏ
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇÖ÷Òª·þÎñµÄÇé¿ö
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_type_service_rsp{
	uint8_t length;
	union att_read_by_type_service_payload pair;
};





/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_type_chartextend_rsp
*³ÉÔ±:length	Length²ÎÊý
			hdl		Attribute Data List²ÎÊýÖÐµÄAttribute Handle
			val 	128bituuidÏÂµÄAttribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead By Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength´ú±íAttribute Data ListµÄ³¤¶È
				±¾½á¹¹Ö»ÊÇÓÃÓÚ²Ù×÷¶ÔÏóÊÇÌØÐÔ£¬ºÍatt_read_by_type_rsp²»Í¬£¬ÕâÊÇÒ»¸öÀ©Õ¹µÄ½á¹¹Ìå
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_type_chartextend_rsp{
	uint8_t length;
	uint16_t hdl;
	uint8_t val[MAX_ATT_DATA_SZ-4];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_rsp
*³ÉÔ±:buf	Attribute Value²ÎÊý
*ËµÃ÷£º	¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_rsp{
	uint8_t buf[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_blob_rsp
*³ÉÔ±:buf	Part Attribute Value²ÎÊý
*ËµÃ÷£º	¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_blob_rsp{
	uint8_t buf[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_multiple_rsp
*³ÉÔ±:val	Set Of Values²ÎÊý
*ËµÃ÷£º	¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_multiple_rsp{
	uint8_t val[MAX_ATT_DATA_SZ-1];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_group_type_16
*³ÉÔ±:hdl	16bituuidÏÂµÄAttribute Data ListµÄAttribute Handle²ÎÊý
			end_hdl	16bituuidÏÂµÄAttribute Data ListµÄEnd Group Handle²ÎÊý
			uuid 16bituuidÏÂµÄAttribute Data ListµÄAttribute Value²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead by Group Type ResponseµÄ²ÎÊý
**************************************************************************************************/
struct att_read_by_group_type_16{
	uint16_t hdl;
	uint16_t end_hdl;
	uint8_t uuid[2];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_group_type_128
*³ÉÔ±:hdl	128bituuidÏÂµÄAttribute Data ListµÄAttribute Handle²ÎÊý
			end_hdl	128bituuidÏÂµÄAttribute Data ListµÄEnd Group Handle²ÎÊý
			uuid 128bituuidÏÂµÄAttribute Data ListµÄAttribute Value²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead by Group Type ResponseµÄ²ÎÊý
**************************************************************************************************/
struct att_read_by_group_type_128{
	uint16_t hdl;
	uint16_t end_hdl;
	uint8_t uuid[16];
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_group_type_payload
*³ÉÔ±:uuid16	16bituuidÏÂµÄAttribute Data List²ÎÊý
			uuid128	128bituuidÏÂµÄAttribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead by Group Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»ÑùAttribute Data List
ÓÐ¿ÉÄÜÊÇhandleºÍ16bit uuidµÄÖ÷Òª·þÎñ×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄÖ÷Òª·þÎñ×éºÏ
**************************************************************************************************/
union  att_read_by_group_type_payload {
	struct att_read_by_group_type_16   uuid16[3];
	struct att_read_by_group_type_128 uuid128;
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_read_by_group_type_rsp
*³ÉÔ±:length	Length²ÎÊý
			pair	attribute Data List²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅRead by Group Type ResponseµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶ÒòÎªlength²»Ò»ÑùAttribute Data List
ÓÐ¿ÉÄÜÊÇhandleºÍ16bit uuidµÄÖ÷Òª·þÎñ×éºÏ£¬Ò²ÓÐ¿ÉÄÜÊÇhandleºÍ128bit uuidµÄÖ÷Òª·þÎñ×éºÏ
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_read_by_group_type_rsp{
	uint8_t length;
	union  att_read_by_group_type_payload pair;
};




/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_hdl_val_notifivation
*³ÉÔ±:hdl	Attribute Handle²ÎÊý
			buf	Attribute Value²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×ÅHandle Value NotificationµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶buf×î´óÖµÎªATT_MTU–
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_hdl_val_notifivation{
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-3];
};



/*************************************************************************************************
*ÁªºÏÌåÃû£ºatt_hdl_val_indication
*³ÉÔ±:hdl	Attribute Handle²ÎÊý
			buf	Attribute Value²ÎÊý
*ËµÃ÷£º	±¾½á¹¹Ìå¶ÔÓ¦×Handle Value IndicationµÄ²ÎÊý£¬¸ù¾Ý¹æ·¶buf×î´óÖµÎªATT_MTU–
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
**************************************************************************************************/
struct att_hdl_val_indication{
	uint16_t hdl;
	uint8_t buf[MAX_ATT_DATA_SZ-3];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºattc_ble_evt
*³ÉÔ±£º attc_code	Ð­ÒéÕ»ÉÏ±¨att¿Í»§¶ËÊÂ¼þµÄ±êÊ¶Âë£¨code)
				attc_sz Ð­ÒéÕ»ÉÏ±¨att¿Í»§¶ËÊÂ¼þµÄ³¤¶È
				evt	Ð­ÒéÕ»ÉÏ±¨attÊÂ¼þµÄ¾ßÌåÄÚÈÝ£¬ÊÇÒ»¸öÁªºÏÌå£¬°üº¬ÓÐ¸÷ÖÖÊÂ¼þµÄÊý¾Ý
					AttErrRsp	ATT¿Í»§¶Ë´íÎóÊÂ¼þµÄÊý¾Ý
					AttMtuRsp	ATT¿Í»§¶Ë½»»»MTUÊÂ¼þµÄÊý¾Ý
					AttFindInfoRsp	ATT¿Í»§¶ËÉÏ±¨Find Information ResponseÊÂ¼þµÄÊý¾Ý
					AttFindByTypeValRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý
					AttReadByTypeRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý£¬¸ÃÊý¾ÝÖ»ÊÊÓÃÓÚcharacteristic
					AttReadByTypeIncludeRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý£¬¸ÃÊý¾ÝÖ»ÊÊÓÃÓÚ°üº¬·þÎñ
					AttReadByTypeValRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý£¬¸ÃÊý¾ÝÖ»ÊÊÓÃÓÚµ¥¸öpairÖµ
					AttReadByTypeServiceRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý£¬¸ÃÊý¾ÝÖ»ÊÊÓÃÓÚ·þÎñ
					AttReadByTypeChartExtendRsp	ATT¿Í»§¶ËÉÏ±¨Read By Type ResponseÊÂ¼þµÄÊý¾Ý£¬¸ÃÊý¾ÝÖ»ÊÊÓÃÓÚÀ©Õ¹
																			characteristic
					AttReadRsp	ATT¿Í»§¶ËÉÏ±¨Read ResponseÊÂ¼þµÄÊý¾Ý
					AttReadBlobRsp	ATT¿Í»§¶ËÉÏ±¨Read Blob ResponseÊÂ¼þµÄÊý¾Ý
					AttReadMultipleRsp	ATT¿Í»§¶ËÉÏ±¨Read Multiple ResponseÊÂ¼þµÄÊý¾Ý
					AttReadByGroupTypeRsp	ATT¿Í»§¶ËÉÏ±¨Read by Group Type ResponseÊÂ¼þµÄÊý¾Ý
					AttHdlValNotification	ATT¿Í»§¶ËÉÏ±¨Handle Value NotificationÊÂ¼þµÄÊý¾Ý
					AttHdlValIndication	ATT¿Í»§¶ËÉÏ±¨Handle Value IndicationÊÂ¼þµÄÊý¾Ý
*ËµÃ÷£º SYD8821ÔÚÊÕµ½À¶ÑÀATT¿Í»§¶ËµÄÊÂ¼þµÄÊ±ºòÍ¨¹ýgap_s_att_c_evt_handler_setÉèÖÃµÄ½Ó¿ÚÉÏ±¨À¶ÑÀÊÂ¼þ£¬
attc_ble_evt½á¹¹ÌåÎªÀ¶ATT¿Í»§¶ËÑÀÊÂ¼þµÄ¾ßÌåÊý¾Ý
				¸Ã½á¹¹ÌåÓÃÓÚGATT¿Í»§¶Ë£¬¶ÔÓÚGATTµÄ½ÇÉ«Îª·þÎñÆ÷£¬ÄÇÃ´¸Ã½á¹¹ÌåÊÇÎÞÓÃµÄ¡£SYD8821Ö»ÊÇÌá¹©ATT¿Í»§
¶ËÏàÓ¦µÄAPI£¬²¢Ã»ÓÐÌá¹©GAPµÄ´¦ÀíÁ÷³Ì£¬ÒÔÁô¸øAPP×î´óµÄÁé»îÐÔ
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
*½á¹¹ÌåÃû£ºgap_att_report_handle
*³ÉÔ±£º cnt	BLEµÄreport(notify»òÕßindicate)µÄ¸öÊý£¬Ò²¾ÍÊÇCCCDÃèÊö·ûµÄ¸öÊý
				report	BLEµÄreport(notify»òÕßindicate)µÄ¾ßÌåÄÚÈÝ£¬¸ÃÊý×é×î´óÎª20
*ËµÃ÷£º SYD8821µÄprofile°ÑnotifyºÍindicateÁ½ÖÖGATT serviceÐÐÎª¼¯ºÏ³ÆÎªreport£¬×¨ÃÅÉèÁ¢½á¹¹Ìå
gap_att_report_handle±£´æÁËreportµÄÏàÓ¦ÊôÐÔÐÅÏ¢£¬Ã¿¸öCCCD¶ÔÓ¦×Å½á¹¹Ìågap_att_report_handleÖÐµÄÒ»¸ö
Êý×é³ÉÔ±¡£°üÀ¨±¾CCCDËùÔÚµÄÎ»ÖÃ£¨ÆäÖ÷Òª·þÎñ£¬ÌØÐÔ£¬val_hdl£©µÈ
**************************************************************************************************/
struct gap_att_report_handle {
	uint8_t	cnt;
	struct	gap_att_report report[MAX_ATT_REPORT_HDL];
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_bond_dev
*³ÉÔ±£º addr	BLE°ó¶¨Éè±¸µÄµØÖ·
				key	BLE°ó¶¨ÐÅÏ¢µÄ¾ßÌå²ÎÊý£¨Åä¶ÔºóÉú²úµÄÃÜ³×£©
*ËµÃ÷£º BLEÔÚÅä¶ÔÖ®ºó¸ù¾ÝÐèÒª¿ÉÄÜ»á½øÐÐ°ó¶¨µÄ²Ù×÷£¬ÕâÀï»á°Ñ±¾½á¹¹Ìå´æ½øÐ¾Æ¬ÄÚ²¿µÄflashÖÐ
¿ÉÒÔÍ¨¹ýapi:bm_s_bond_info_get»ñÈ¡°ó¶¨ÐÅÏ¢
**************************************************************************************************/
struct gap_bond_dev {			
	struct gap_ble_addr 		addr;
	struct gap_key_params	key;	
};




/*************************************************************************************************
*½á¹¹ÌåÃû£ºgap_wakeup_config
*³ÉÔ±£º timer_wakeup_en	¶¨Ê±Æ÷ÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´
				gpi_wakeup_en	GPIOÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´
				gpi_wakeup_cfg	ÉèÖÃ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´µÄGPIO
				gpi_wakeup_pol	ÉèÖÃ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´µÄGPIOµÄ¼«ÐÔ
*ËµÃ÷£º ±¾½á¹¹ÌåÓÃÓÚÅäÖÃSYD8821Ë¯ÃßµÄ»½ÐÑÔ´£¬µ«ÊÇÖ»ÊÇ¶Ô¶¨Ê±Æ÷ºÍgpioµÄ»½ÐÑÔ´½øÐÐÅäÖÃ£¬Õû¸öË¯ÃßµÄ»½ÐÑ
Ô´µÄÉèÖÃÒª½áºÏSystemPowerDownÒÔ¼°SystemSleepº¯ÊýµÄµ÷ÓÃ²ÎÊý
				gpi_wakeup_polÓÃÓÚÉèÖÃ»½ÐÑÔ´µÄGPIOµÄ¼«ÐÔ£¬Í¬Ê±GPIOÄ£¿éÒ²ÓÐGPIOINT_POL_SETµÄ¼«ÐÔÉèÖÃ£¬Á½¸ö¼«ÐÔ
ÉèÖÃÊÇ¶ÀÁ¢µÄ£¬»½ÐÑÔ´µÄÉèÖÃÖ»ÊÇºÍgpi_wakeup_polÓÐ¹Ø£¬ºÍGPIOÄ£¿éµÄÉèÖÃÎÞ¹Ø
**************************************************************************************************/
struct gap_wakeup_config {
	uint8_t timer_wakeup_en;
	uint8_t gpi_wakeup_en;
	uint32_t gpi_wakeup_cfg;
	uint32_t gpi_wakeup_pol;
};




/*************************************************************************************************
*Ã¶¾ÙÃû£ºPOWER_SAVING_TYPE
*³ÉÔ±£º POWER_SAVING_RC_ON	Ë¯Ãßºó¸ßÆµRCÊ±ÖÓÒÀ¾É´ò¿ª
				POWER_SAVING_RC_OFF	Ë¯Ãßºó¸ßÆµRCÊ±ÖÓ½«±»¹Ø±Õ
				POWER_SAVING_DSLEEP_LPO_ON_RETAIN	Ë¯ÃßºóµÍÆµRC(LPO)ÒÀ¾É´ò¿ª£¬²¢ÇÒ»½ÐÑºó½Ó×ÅË¯ÃßÇ°µÄÎ»ÖÃÖ´ÐÐ
				POWER_SAVING_DSLEEP_LPO_OFF_RETAIN	Ë¯ÃßºóµÍÆµRC(LPO)±»¹Ø±Õ£¬²¢ÇÒ»½ÐÑºó½Ó×ÅË¯ÃßÇ°µÄÎ»ÖÃÖ´ÐÐ
				POWER_SAVING_DSLEEP_LPO_ON_RESET	Ë¯ÃßºóµÍÆµRC(LPO)ÒÀ¾É´ò¿ª£¬²¢ÇÒ»½ÐÑºó½«»á¸´Î»
				POWER_SAVING_DSLEEP_LPO_OFF_RESET	Ë¯ÃßºóµÍÆµRC(LPO)±»¹Ø±Õ£¬²¢ÇÒ»½ÐÑºó½«»á¸´Î»
				POWER_SAVING_TYPE_NUM	Ê¡µçÄ£Ê½µÄÊýÁ¿
*ËµÃ÷£º Ê¡µçÄ£Ê½ÀàÐÍ¾ö¶¨¸ßÆµRCÕñµ´Æ÷ºÍµÍÆµRCÕñµ´Æ÷ÔÚË¯ÃßºóÊÇ·ñ´ò¿ª£¬Êý×ÖÔ½´óÔ½Ê¡µç£¬²¢ÇÒÉèÖÃLPOµÄ
Ä£Ê½ÏÂRCÕñµ´Æ÷ÊÇ±»¹Ø±ÕµÄ£¬±ÈÈçÉèÖÃPOWER_SAVING_DSLEEP_LPO_ON_RETAINÄÇÃ´Ëæ´ø×ÅÒ²ÉèÖÃÁË
POWER_SAVING_RC_OFF
				´øÓÐ¡°DSLEEP¡±×ÖÑùµÄÒâÎ¶×Å½«»á½øÈëÉî¶ÈË¯ÃßÄ£Ê½
				´øÓÐ¡°RESET¡±×ÖÑùµÄÒâÎ¶×Å»½ÐÑºó½«»á¸´Î»£¬ÄÚ´æµÈ½«²»¸´´æÔÚ
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
*Ã¶¾ÙÃû£ºMODULE_CONTROL_TYPE
*³ÉÔ±£º NO_MODULE	Ë¯Ãßºó²»¹Ø±ÕÈÎºÎÄ£¿é
				PER_MODULE	Ë¯Ãßºó¹Ø±ÕÍâÉèÄ£¿é£¬°üÀ¨GPIO SPI iicµÈ£¬ÉèÖÃ¸ÃÄ£¿é»½ÐÑºó±ØÐëÒªÖØÉèÕâÐ©Ä£¿é
				BLE_MODULE	Ë¯Ãßºó¹Ø±ÕBLEÄ£¿é£¬Á¬½ÓÉÏºó²»ÄÜ¹»ÉèÖÃ£¬µ±Ê±ÔÚ²»¹ã²¥µÄÊ±ºò±ØÐëÉèÖÃ
				FLASH_LDO_MODULE	Ë¯Ãßºó¹Ø±ÕFLASH_LDO,Ò²¾ÍÊÇFLASHµÄµçÔ´
				PB_MODULE	ÎªPER_MODULEºÍBLE_MODULEµÄ¼¯ºÏ
				PF_MODULE	ÎªPER_MODULEºÍFLASH_LDO_MODULEµÄ¼¯ºÏ
				PBF_MODULE	ÎªPER_MODULEºÍFLASH_LDO_MODULEÒÔ¼°FLASH_LDO_MODULEµÄ¼¯ºÏ
*ËµÃ÷£º Ä£¿é¿ØÖÆÀàÐÍÓÃÓÚ¿ØÖÆÔÚË¯ÃßºóÄ³Ð©Ä£¿éµÄ¿ª¹Ø
				FLASH_LDO_MODULEÄ£Ê½ÏÂ»½ÐÑµÄÊ±ºò±ØÐëÒª¸øflash×ã¹»µÄÊÂ¼þ£¬Ò²¾ÍÊÇSystemSleep´«ÈëµÄµÚÈý¸ö²ÎÊý
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
*Ã¶¾ÙÃû£ºPMU_WAKEUP_CONFIG_TYPE
*³ÉÔ±£º PIN_WAKE_EN	GPIOÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´
				TIMER_WAKE_EN	¶¨Ê±Æ÷ÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´
				FSM_SLEEP_EN	À¶ÑÀÊÂ¼þÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´£¬Ö»ÒªÊ¹ÓÃBLE±ØÐë¿ªÆô¸Ã¹¦ÄÜ
				ANA_WAKE_EN	ADC±È½ÏÆ÷ÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´
				RTC_WAKE_EN	RTCÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´,rtcÓÐÈýÖÖ»½ÐÑÔ´
				WDT_WAKE_EN	¿´ÃÅ¹·ÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´,SYD8821µÄ¿ªÃÅ¹»ÓÐÖÐ¶Ï¹¦ÄÜ£¬Ê¹ÄÜÖÐ¶Ï±ØÐë¿ªÆô¸Ã¹¦ÄÜ
				CAPDET_WAKE_EN	SYD8821´¥ÃþÄ£¿éÊÇ·ñ×÷ÎªË¯ÃßºóµÄ»½ÐÑÔ´,
				KEEP_ORIGINAL_WAKEUP	
*ËµÃ÷£º »½ÐÑ¿ØÖÆÀàÐÍÅäÖÃÄÄ¸öÄ£¿éÄÜ¹»×÷Îª»½ÐÑÔ´£¬gap_wakeup_configÒ²ÓÐGPIOºÍTIMERµÄÊ¹ÄÜ£¬ÆäÊÇÎÞÓÃµÄ
ÕæÕýÆðÐ§µÄÊÇSystemSleep´«ÈëµÄ»½ÐÑÔ´
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
*Ã¶¾ÙÃû£ºAMIC_BIAS
*³ÉÔ±£º  AMIC_BIAS_2_94	amicµÄ adc bias Îª2.94V
		AMIC_BIAS_2_5	amicµÄ adc bias Îª2.5V
		AMIC_BIAS_1_4	amicµÄ adc bias Îª1.4V
		AMIC_BIAS_1_25	amicµÄ adc bias Îª1.25V
*ËµÃ÷£º ÉèÖÃAMICµÄbias²ÎÊý£¬amic_set_bias´«ÈëµÄ²ÎÊý
**************************************************************************************************/
typedef enum {
	AMIC_BIAS_2_94	= 0,
	AMIC_BIAS_2_5	= 1,
	AMIC_BIAS_1_4	= 2,
	AMIC_BIAS_1_25	= 3,
} AMIC_BIAS;


#pragma pack()

/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_init
*ÊäÈë²ÎÊý£ºÎÞ
*Êä³ö²ÎÊý£ºuint8_t -->³õÊ¼»¯Ð­ÒéÕ»µÄ½á¹û
						0	-->Ð­ÒéÕ»³õÊ¼»¯Ê§°Ü
						1	-->Ð­ÒéÕ»³õÊ¼»¯³É¹¦
*ËµÃ÷£º 1.Ð­ÒéÕ»³õÊ¼»¯º¯Êý»á°Ñµ×²ãºÍRFÏà¹ØµÄ¼Ä´æÆ÷½øÐÐ³õÊ¼»¯
				2.¸Ãº¯Êý»á°ÑºÍBLEÐ­ÒéÏà¹ØµÄÄÚÈÝ½øÐÐ³õÊ¼»¯£¬±ÈÈçprofile¡¢°²È«²ÎÊý¡¢Á¬½Ó²ÎÊýµÈ
**************************************************************************************************/
extern uint8_t gap_s_ble_init(void);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_disconnect
*ÊäÈë²ÎÊý£ºÎÞ
*Êä³ö²ÎÊý£ºÎÞ
*ËµÃ÷£º ¸Ãº¯Êý»á·¢ÆðLL_TERMINATE_IND,
				¸ù¾ÝÐ­ÒéÖ÷»úÊÕµ½¸ÃÇëÇóºó»á»Ø¸´Ò»¸ö¿Õ°ü£¬´ú±í¶ÏÏß½áÊø£¬¶ÏÏß½áÊøºóÐ­ÒéÕ»»áÉÏ±¨Ò»¸ö¶ÏÏßÊÂ¼þ
**************************************************************************************************/
extern void gap_s_disconnect(void);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_validate_irk
*ÊäÈë²ÎÊý£ºÎÞ
*Êä³ö²ÎÊý£ºÎÞ
*ËµÃ÷£º ÉèÖÃËæ»úË½ÓÐµØÖ·µÄirk
**************************************************************************************************/
extern uint8_t gap_s_validate_irk(uint8_t *irk);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_gen_random_private_address
*ÊäÈë²ÎÊý:struct gap_ble_addr * rpa	·µ»ØµØÖ·µÄÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡Ë½ÓÐËæ»úµØÖ·
**************************************************************************************************/
extern void gap_s_ble_gen_random_private_address(struct gap_ble_addr * rpa);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_address_set
*ÊäÈë²ÎÊý:struct gap_ble_addr* p_dev	ÉèÖÃµØÖ·µÄÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃÉè±¸µØÖ·
			Èç¹ûµ÷ÓÃÁËÕâ¸ö½Ó¿Ú£¬BLEµÄÉèÖÃµØÖ·½«ÒÔ¸ÃµØÖ·Îª×¼£¬Èç¹û²»µ÷ÓÃÕâ¸ö½Ó¿Ú£¬BLEµÄÉè±¸µØÖ·½«ÒÔÅäÖÃÎÄ
			¼þµÄÎª×¼
			×¢Òâ£º¶ÔÓÚÁ¿²úµÄ³ÌÐò£¬Èç¹ûÉèÖÃ¸Ã½Ó¿Ú£¬ÄÇÃ´ÕûÅú²úÆ·µÄµØÖ·½«Ò»Ñù
**************************************************************************************************/
extern void gap_s_ble_address_set(struct gap_ble_addr* p_dev);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_address_get
*ÊäÈë²ÎÊý:struct gap_ble_addr* p_dev	»ñÈ¡µØÖ·µÄÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡Éè±¸µÄµØÖ·
**************************************************************************************************/
extern void gap_s_ble_address_get(struct gap_ble_addr* p_dev);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_feature_set
*ÊäÈë²ÎÊý:uint8_t *p_feature	ÉèÖÃfeatureµÄÖ¸Õë ÒÀ¾Ý¹æ·¶ÒªÇó£¬ÕâÀïµÄ´«ÈëµÄÊý¾Ý³¤¶ÈÓ¦¸ÃÎª8
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃÖ÷»ú¶ËµÄfeature£¬Ïà¶ÔÓÚ¸ÃÏî£¬BLE5.0ÔÚBLE4.0Ö»ÓÐÒ»¸öbitµÄ»ù´¡ÉÏÔö¼Óµ½ÁË16¸öbit
			SYD8821¸ÃÏîµÄÄ¬ÈÏÖµÊÇ0x0000000000000133
			¸Ãº¯ÊýÖ»ÓÐÔÚSYD8821´¦ÓÚÖ÷»úÄ£Ê½²ÅÓÐÓÃ
**************************************************************************************************/
extern void gap_s_ble_feature_set(uint8_t *p_feature);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_feature_get
*ÊäÈë²ÎÊý:uint8_t *p_feature	·µ»Ø±¾»úµÄfeatureµÄÖ¸Õë ÒÀ¾Ý¹æ·¶ÒªÇó£¬ÕâÀï»á·µ»Ø8¸öbyteµÄÊý¾Ý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡Ö÷»ú¶ËµÄfeature£¬Ïà¶ÔÓÚ¸ÃÏî£¬BLE5.0ÔÚBLE4.0Ö»ÓÐÒ»¸öbitµÄ»ù´¡ÉÏÔö¼Óµ½ÁË16¸öbit
			SYD8821¸ÃÏîµÄÄ¬ÈÏÖµÊÇ0x0000000000000133
			¸Ãº¯ÊýÖ»ÓÐÔÚSYD8821´¦ÓÚÖ÷»úÄ£Ê½²ÅÓÐÓÃ
**************************************************************************************************/
extern void gap_s_ble_feature_get(uint8_t *p_feature);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_feature_set
*ÊäÈë²ÎÊý:uint8_t *p_feature	ÉèÖÃfeatureµÄÖ¸Õë ÒÀ¾Ý¹æ·¶ÒªÇó£¬ÕâÀïµÄ´«ÈëµÄÊý¾Ý³¤¶ÈÓ¦¸ÃÎª8
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃ´Ó»ú¶ËµÄfeature£¬Ïà¶ÔÓÚ¸ÃÏî£¬BLE5.0ÔÚBLE4.0Ö»ÓÐÒ»¸öbitµÄ»ù´¡ÉÏÔö¼Óµ½ÁË16¸öbit
			SYD8821¸ÃÏîµÄÄ¬ÈÏÖµÊÇ0x0000000000000133
			¸Ãº¯ÊýÖ»ÓÐÔÚSYD8821´¦ÓÚ´Ó»úÄ£Ê½²ÅÓÐÓÃ
**************************************************************************************************/
extern void gap_s_ble_slave_feature_set(uint8_t *p_feature);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_ble_feature_get
*ÊäÈë²ÎÊý:uint8_t *p_feature	·µ»Ø±¾»úµÄfeatureµÄÖ¸Õë ÒÀ¾Ý¹æ·¶ÒªÇó£¬ÕâÀï»á·µ»Ø8¸öbyteµÄÊý¾Ý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡´Ó»ú¶ËµÄfeature£¬Ïà¶ÔÓÚ¸ÃÏî£¬BLE5.0ÔÚBLE4.0Ö»ÓÐÒ»¸öbitµÄ»ù´¡ÉÏÔö¼Óµ½ÁË16¸öbit
			SYD8821¸ÃÏîµÄÄ¬ÈÏÖµÊÇ0x0000000000000133
			¸Ãº¯ÊýÖ»ÓÐÔÚSYD8821´¦ÓÚ´Ó»úÄ£Ê½²ÅÓÐÓÃ
**************************************************************************************************/
extern void gap_s_ble_slave_feature_get(uint8_t *p_feature);




/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_access_code_set
*ÊäÈë²ÎÊý:uint8_t *p_acc	ÉèÖÃ¹ã²¥Í¨µÀ·ÃÎÊµØÖ·Ö¸Õë£¨4byte£©
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃ¹ã²¥Í¨µÀµÄ·ÃÎÊµØÖ·£¬·ÇÌØÊâÇé¿ö²»½¨ÒéÊ¹ÓÃ
**************************************************************************************************/
extern void gap_s_adv_access_code_set(uint8_t *p_acc);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_access_code_get
*ÊäÈë²ÎÊý:uint8_t *p_acc	·µ»Ø¹ã²¥Í¨µÀ·ÃÎÊµØÖ·Ö¸Õë£¨4byte£©
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡¹ã²¥·ÃÎÊµØÖ·£¬ÔÚ²»µ÷ÓÃgap_s_adv_access_code_setµÄÇé¿öÏÂ·µ»Ø0x8E89BED6,·ÇÌØÊâÇé¿öÏÂ²»½¨Òé
			µ÷ÓÃ¸Ã½Ó¿Ú
**************************************************************************************************/
extern void gap_s_adv_access_code_get(uint8_t *p_acc);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_parameters_set
*ÊäÈë²ÎÊý:struct gap_adv_params * p_adv	¹ã²¥²ÎÊýÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃ¹ã²¥²ÎÊý£¬¾ßÌå²ÎÊýÇë¿´gap_adv_params½á¹¹Ìå
**************************************************************************************************/
extern void gap_s_adv_parameters_set(struct gap_adv_params * p_adv);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_data_set
*ÊäÈë²ÎÊý:uint8_t *p_adv	¹ã²¥Êý¾ÝÖ¸Õë
					uint8_t adv_sz	¹ã²¥Êý¾Ý´óÐ¡
					uint8_t *p_scan	É¨ÃèÏìÓ¦Êý¾ÝÖ¸Õë
					uint8_t sacn_sz	É¨ÃèÏìÓ¦Êý¾Ý´óÐ¡
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÖ»ÊÇ°Ñ¹ã²¥Êý¾ÝµÄÉèÖÃ±£´æµ½Ð­ÒéÕ»µ×²ã»º³åÇø£¬²¢Ã»ÓÐÕæÕýÆðÐ§£¬ÕæÕýÆðÐ§ÊÇÔÚµ÷gap_s_adv_start
			º¯ÊýµÄÊ±ºò
			ÒòÎªÖ÷»ú½øÐÐÉ¨ÃèµÄÊ±ºòÁô¸ø´Ó»úµÄÊ±¼äºÜ¶Ì£¬ËùÒÔÕâÀïÌáÇ°°ÑÊý¾Ý×¼±¸ºÃ£¬¶ø²»ÊÇÉ¨ÃèÇëÇóÀ´ÁËÔÙÌî³äÊý¾Ý
**************************************************************************************************/
extern void gap_s_adv_data_set(uint8_t *p_adv, uint8_t adv_sz,uint8_t *p_scan, uint8_t sacn_sz);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_start
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯Êý¿ªÊ¼¹ã²¥£¬°Ñgap_s_adv_data_setÉèÖÃÏÂÀ´µÄÊý¾Ý·¢ËÍµ½¹ã²¥Í¨µÀ
**************************************************************************************************/
extern void gap_s_adv_start(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_stop
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯ÊýÍ£Ö¹¹ã²¥
**************************************************************************************************/
extern void gap_s_adv_stop(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_scan_parameters_set
*ÊäÈë²ÎÊý:struct gap_scan_params*p_scan	É¨Ãè²ÎÊýÉèÖÃ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃÉÕÃæ²ÎÊý£¬¾ßÌå²ÎÊýÇë¿´gap_scan_params½á¹¹Ìå ×¢Òâ£ºÕâÀïÉ¨ÃèµÄ¶¯×÷ºÍ¹ã²¥²»ÄÜ¹»Í¬Ê±½øÐÐ
**************************************************************************************************/
extern void gap_s_scan_parameters_set(struct gap_scan_params*p_scan);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_scan_start
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯Êý¿ªÊ¼É¨Ãè
**************************************************************************************************/
extern void gap_s_scan_start(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_adv_state_get
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t	À¶ÑÀ¹ã²¥×´Ì¬	¸Ã·µ»ØÖµÎªÃ¶¾Ù_ADV_SCAN_MODE_µÄ³ÉÔ±
*ËµÃ÷:µ÷ÓÃ¸Ãº¯Êý¿É»ñÈ¡µ×²ãÀ¶ÑÀ¹ã²¥µÄ×´Ì¬£¬¹ã²¥×´Ì¬·ÖÎª¿ÕÏÐ×´Ì¬¡¢¹ã²¥×´Ì¬¡¢É¨Ãè×´Ì¬µÈ£¬¾ßÌå¿É¿´£¬Ã¶¾Ù£º
_ADV_SCAN_MODE_
**************************************************************************************************/
extern uint8_t gap_s_adv_state_get(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_scan_stop
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯ÊýÍ£Ö¹¹ã²¥
**************************************************************************************************/
extern void gap_s_scan_stop(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_adv_data_set
*ÊäÈë²ÎÊý:uint8_t advtype	Á¬½Ó×´Ì¬ÏÂ¹ã²¥µÄÀàÐÍ
					uint8_t *buf		Á¬½Ó×´Ì¬ÏÂ¹ã²¥µÄÊý¾ÝÖ¸Õë
					uint8_t sz			Á¬½Ó×´Ì¬ÏÂ¹ã²¥µÄÊý¾ÝµÄ´óÐ¡
					uint8_t *p_addr	Á¬½Ó×´Ì¬ÏÂ¹ã²¥µÄµØÖ·Ö¸Õë
					uint8_t addr_type	Á¬½Ó×´Ì¬ÏÂ¹ã²¥µÄµØÖ·ÀàÐÍ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÏà¶ÔÓÚgap_s_adv_data_setÖÐµÄ²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ£¬Á¬½Ó×´Ì¬ÖÐ·¢ÆðµÄ¹ã²¥
			²»¿ÉÁ¬½Ó£¬ÒòÎªSYD8821µÄ´Ó»úÄ¿Ç°»¹²»Ö§³Ö»ìºÏ×´Ì¬»úµÄ»úÖÆ
**************************************************************************************************/
extern void gap_s_coex_adv_data_set(uint8_t advtype, uint8_t *buf, uint8_t sz, uint8_t *p_addr, uint8_t addr_type);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_scan_rsp_data_set
*ÊäÈë²ÎÊý:uint8_t *buf		Á¬½Ó×´Ì¬ÏÂÉ¨ÃèÏìÓ¦µÄÊý¾ÝÖ¸Õë
					uint8_t sz			Á¬½Ó×´Ì¬ÏÂÉ¨ÃèÏìÓ¦µÄÊý¾ÝµÄ´óÐ¡
					uint8_t *p_addr	Á¬½Ó×´Ì¬ÏÂÉ¨ÃèÏìÓ¦µÄµØÖ·Ö¸Õë
					uint8_t addr_type	Á¬½Ó×´Ì¬ÏÂÉ¨ÃèÏìÓ¦µÄµØÖ·ÀàÐÍ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÏà¶ÔÓÚgap_s_adv_data_setÖÐµÄÉ¨ÃèÏìÓ¦Êý¾Ý²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ
**************************************************************************************************/
extern void gap_s_coex_scan_rsp_data_set(uint8_t *buf, uint8_t sz, uint8_t *p_addr, uint8_t addr_type);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_adv_start
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯Êý¿ªÊ¼¹ã²¥£¬°Ñgap_s_coex_adv_data_setÉèÖÃÏÂÀ´µÄÊý¾Ý·¢ËÍµ½¹ã²¥Í¨µÀ
			¸Ãº¯ÊýÏà¶ÔÓÚgap_s_adv_startµÄ²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ
**************************************************************************************************/
extern void gap_s_coex_adv_start(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_adv_stop
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯ÊýÍ£Ö¹¹ã²¥
			¸Ãº¯ÊýÏà¶ÔÓÚgap_s_adv_startµÄ²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ
**************************************************************************************************/
extern void gap_s_coex_adv_stop(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_scan_req_data_set
*ÊäÈë²ÎÊý:uint8_t *buf		Á¬½Ó×´Ì¬ÏÂÉ¨ÃèÇëÇóµÄÊý¾ÝÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃÁ¬½Ó×´Ì¬ÏÂµÄÉ¨ÃèÇëÇóÊý¾Ý£¬ÉèÖÃµÄÊÇÉÕÃæÇëÇóÖÐµÄ¹ã²¥µØÖ·
**************************************************************************************************/
extern void gap_s_coex_scan_req_data_set(uint8_t *buf);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_scan_start
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯Êý¿ªÊ¼É¨Ãè
			¸Ãº¯ÊýÏà¶ÔÓÚgap_s_scan_startµÄ²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ
**************************************************************************************************/
extern void gap_s_coex_scan_start(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_coex_scan_stop
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:µ÷ÓÃ¸Ãº¯ÊýÍ£Ö¹É¨Ãè
			¸Ãº¯ÊýÏà¶ÔÓÚgap_s_scan_stopµÄ²»Í¬Ö®´¦ÔÚÓÚ¸Ãº¯ÊýÊÇÔÚÁ¬½Ó×´Ì¬ÖÐµ÷ÓÃ
**************************************************************************************************/
extern void gap_s_coex_scan_stop(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_profiles_set
*ÊäÈë²ÎÊý:struct gap_profile_struct *p_gatt_profile profileÔªËØ(Èç:·þÎñ¡¢ÌØÐÔ¡¢ÖµÒÔ¼°report_handle)
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:Ö¸¶¨profile¸÷¸öÔªËØµÄÎ»ÖÃ£¬µ×²ãÐ­ÒéÕ»»áÈ¥p_gatt_profile½á¹¹ÌåÖ¸¶¨µÄÎ»ÖÃÈ¥»ñÈ¡profile
**************************************************************************************************/
extern void gap_s_gatt_profiles_set(struct gap_profile_struct *p_gatt_profile);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_security_parameters_set
*ÊäÈë²ÎÊý:struct gap_pairing_req *p_sec_params À¶ÑÀ°²È«²ÎÊý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃÀ¶ÑÀ°²È«²ÎÊý£¬ÕâÐ©²ÎÊýÓ°Ïìµ½°ó¶¨Åä¶ÔµÄ¹ý³Ì
**************************************************************************************************/
extern void gap_s_security_parameters_set(struct gap_pairing_req *p_sec_params);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_security_parameters_get
*ÊäÈë²ÎÊý:struct gap_pairing_req *p_sec_params À¶ÑÀ°²È«²ÎÊý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡À¶ÑÀ°²È«²ÎÊý
**************************************************************************************************/
extern void gap_s_security_parameters_get(struct gap_pairing_req *p_sec_params);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_security_req
*ÊäÈë²ÎÊý:uint8_t flag -->ÊÇ·ñ°ó¶¨
						0	-->	Åä¶Ôºó²»°ó¶¨
						1	-->	Åä¶Ôºó°ó¶¨
					uint8_t mitm	-->ÖÐ¼äÈË±£»¤
						0	-->	Ã»ÓÐÖÐ¼äÈË±£»¤
						1	-->	ÐèÒªÖÐ¼äÈË±£»¤£¨ÊäÈëÃÜÂëµÈ£©
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÆäÖÐÖÐ¼äÈË±£»¤³ýÁËÊäÈëÃÜÂë»¹ÓÐODBÕâÖÖ·½Ê½
**************************************************************************************************/
extern void gap_s_security_req(uint8_t flag, uint8_t mitm);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_connection_param_set
*ÊäÈë²ÎÊý:struct gap_connection_param_rsp_pdu *p_connection_params	ÉèÖÃÁ¬½ÓÇëÇóÏìÓ¦²ÎÊý½á¹¹ÌåÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃÁ¬½ÓÇëÇóÏìÓ¦²ÎÊý£¬Õâ¸öÊÇBLE4.1ÒÔÉÏµÄ°æ±¾Ö§³ÖµÄ½Ó¿Ú
			¸Ãº¯ÊýÖ»ÊÇ°ÑÁ¬½Ó²ÎÊýµÄÉèÖÃ±£´æµ½Ð­ÒéÕ»µ×²ã»º³åÇø£¬²¢Ã»ÓÐÕæÕýÆðÐ§£¬ÕæÕýÆðÐ§ÊÇÔÚÖ÷»ú·¢Æð
			LL_CONNECTION_PARAM_REQµÄÊ±ºò
**************************************************************************************************/
extern void gap_s_connection_param_set(struct gap_connection_param_rsp_pdu *p_connection_params);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_connection_param_get
*ÊäÈë²ÎÊý:struct gap_connection_param_rsp_pdu *p_connection_params	·µ»ØÁ¬½Ó²ÎÊý½á¹¹ÌåÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡Á¬½ÓÇëÇóÏìÓ¦²ÎÊý£¬Õâ¸öÊÇBLE4.1ÒÔÉÏµÄ°æ±¾Ö§³ÖµÄ½Ó¿Ú
**************************************************************************************************/
extern void gap_s_connection_param_get(struct gap_connection_param_rsp_pdu *p_connection_params);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_connection_update
*ÊäÈë²ÎÊý:struct gap_update_params *p_update_params	·µ»ØÁ¬½Ó²ÎÊýÇëÇó½á¹¹ÌåÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÉèÖÃÀ¶ÑÀÁ¬½Ó²ÎÊý£¬¾ßÌåÇë¿´gap_update_params½á¹¹Ìå
			×¢Òâ£ºÖ»ÓÐÔÚ´Ó»úÁ¬½Ó×´Ì¬ÏÂ²ÅÄÜ¹»ÉèÖÃÁ¬½Ó²ÎÊý
			¸Ãº¯ÊýÖ»ÊÇ·¢ËÍÁ¬½Ó²ÎÊý¸üÐÂÇëÇó£¬²¢Ã»ÓÐÉèÁ¢ÏàÓ¦µÄ»úÖÆÀ´ÅÐ¶ÏÕâÐ©Á¬½Ó²ÎÊý¸üÐÂÊÇ·ñÕæµÄÄÜ¹»ÆðÐ§£¬
ËäÈ»¸Ãº¯Êý£¬
			ËäÈ»Á¬½Ó²ÎÊý°üº¬ÁËlatencyÕâ¸ö²ÎÊý£¬µ«ÊÇSYD8821µÄÐ­ÒéÕ»²¢Ã»ÓÐ¶Ôlatency½øÐÐ¹ÜÀí£¬ºÎÊ±Ê¹ÄÜºÎÊ±
Ê§ÄÜ²¢Ã»ÓÐÔÚgap_s_connection_updateº¯ÊýµÄ¹ÜÀí·¶Î§Ö®ÄÚ
			SYD8821¶ÔÓÚÁ¬½Ó²ÎÊýµÄ¹ÜÀí×¨ÃÅ³ÉÁ¢ÁË×¨ÃÅµÄÁ¬½Ó²ÎÊý¹ÜÀí»úÖÆ£ºsmart_update_latency
**************************************************************************************************/
extern void gap_s_connection_update(struct gap_update_params *p_update_params);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_link_parameters_get
*ÊäÈë²ÎÊý:struct gap_link_params* p_link	·µ»ØÁ´Â·²ÎÊý½á¹¹ÌåÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:»ñÈ¡À¶ÑÀ±¾µØÁ´Â·²ÎÊý£¬¾ßÌåÇë¿´gap_link_params½á¹¹Ìå
			Á´Â·²ÎÊýÊÇË«·½¶ÔÁ¬½Ó²ÎÊýÉÌÁ¿µÃµ½µÄ½á¹û£¬ÊÇÎïÀí²ãÉÏÕýÔÚÆð×÷ÓÃµÄÁ¬½Ó²ÎÊý
**************************************************************************************************/
extern void gap_s_link_parameters_get(struct gap_link_params* p_link);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_passkey_set
*ÊäÈë²ÎÊý:uint32_t passkey	Åä¶Ô¹ý³ÌÖÐµÄÃÜÂë£¬¸ÃÃÜÂëÎªÊ®½øÖÆÊý¾Ý£¬±ÈÈçÕâÀï´«Èëpasskey=123456£¬
														ÄÇÃ´ÔÚÖ÷»ú¶ËÓ¦ÊäÈë×Ö·û¡°123456¡±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:º¬ÓÐÖÐ¼äÈË±£»¤µÄÅä¶Ô¹ý³ÌÔÚ¿ªÊ¼Åä¶ÔµÄÊ±ºò»áÉÏ±¨GAP_EVT_SHOW_PASSKEY_REQÊÂ¼þ£¬app´úÂëÒªÔÚ¸ÃÊÂ
			¼þÖÐµ÷ÓÃgap_s_passkey_set½Ó¿ÚÉèÖÃÃÜÂë
**************************************************************************************************/
extern void gap_s_passkey_set(uint32_t passkey);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_evt_handler_set
*ÊäÈë²ÎÊý:struct gap_evt_callback* p_callback	ÉèÖÃÐ­ÒéÕ»ÉÏ±¨½á¹¹Ìå
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÎªapp´úÂëÉèÖÃÐ­ÒéÕ»£¨GAP£©ÉÏ±¨À¶ÑÀÊÂ¼þµÄ½Ó¿Ú£¬Ò²ÊÇGAP²ãÉÏºÍAPP½»»¥µÄ½Ó¿Ú
			gap_evt_callbackÖÐÓÐÁ½¸ö³ÉÔ±£¬ÆäÖÐevt_mask³ÉÔ±ÄÜ¹»ÆÁ±Îµô²»ÐèÒªµÄÊÂ¼þ£¬ÉèÖÃÏàÓ¦µÄbitmaskÄÜ¹»
			×èµ²GAPµÄÉÏ±¨£¬
			gap_evt_callbackµÄp_callback³ÉÔ±ÉèÖÃÁËGAPÉÏ±¨ÊÂ¼þ¸øAPPµÄ½Ó¿ÚÖ¸Õë£¬¸Ã½Ó¿ÚµÄ²ÎÊýÊÇÒ»¸ö
			gap_ble_evt½á¹¹Ìå£¬µ±Ð­ÒéÕ»·¢ÏÖÀ¶ÑÀ×´Ì¬ÓÐ±ä»¯Ê±£¬
			GAP²ã»áµ÷ÓÃ¸Ãº¯ÊýÍ¬Ê±·µ»ØÒ»¸ögap_ble_evt½á¹¹Ìå£¬APPµÄp_callbackº¯ÊýÖÐ¸ù¾Ýgap_ble_evt½á¹¹Ìå´¦
			Àí¸÷ÀàÀ¶ÑÀÊÂ¼þ£¬
**************************************************************************************************/
extern void gap_s_evt_handler_set(struct gap_evt_callback* p_callback);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_att_c_evt_handler_set
*ÊäÈë²ÎÊý:void* p_callback	ÉèÖÃGatt²ãClientÊÂ¼þÉÏ±¨µÄº¯ÊýÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÎªapp´úÂëÉèÖÃGattµÄClientÉÏ±¨attcÊÂ¼þµÄ½Ó¿Ú£¬ÒòÎª´Ó»úÒ»°ãÇé¿öÏÂÊÇserver¶Ë£¬ËùÒÔGATTµÄÊÂ¼þ²¢
²»¹éÀàµ½GAPµÄÊÂ¼þÖÐ£¬¶øÊÇµ¥¶À³ÉÁ¢Ò»¸ö½Ó¿Ú
			p_callbackÖ¸Õë²ÎÊýÊÇÒ»¸öattc_ble_evt½á¹¹Ìå£¬µ±Ð­ÒéÕ»·¢ÏÖGATTµÄClientÓÐ±ä»¯Ê±£¨±ÈÈçÖ÷»ú¶Ë¶ÔÓÚÄ³Ìõ
ATTÖ¸Áî×÷³öÁËÏìÓ¦£©»áµ÷ÓÃ¸Ãº¯Êý£¬Í¬Ê±·µ»ØÒ»¸öattc_ble_evt½á¹¹Ìå£¬APPµÄp_callbackº¯ÊýÖÐ¸ù¾Ý¸öattc_ble_evt
½á¹¹Ìå´¦Àí¸÷ÀàÀ¶ÑÀÊÂ¼þ£¬
**************************************************************************************************/
extern void gap_s_att_c_evt_handler_set(void* p_callback);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_report_handle_get
*ÊäÈë²ÎÊý:struct gap_att_report_handle** p_hdl	·µ»ØprofileÖÐreport_handleµÄ¶þ¼¶Ö¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:notify»òÕßindicateµÄÐÐÎª¶¼ÐèÒªÍùÏÂ´«gap_att_report½á¹¹Ìå£¬¸Ã½á¹¹ÌåÖ¸¶¨ÁË·¢ËÍµÄuuidºÍhdlµÈ£¬¶øÕâÐ©
ÐÅÏ¢¶¼´æ´¢ÔÚ_gatt_database_report_handle½á¹¹ÌåÖÐ£¬
			µ±È»µ÷ÓÃ¸Ãº¯ÊýºÍÖ±½Ó¶ÁÈ¡_gatt_database_report_handleÐ§¹ûÊÇÒ»ÑùµÄ
**************************************************************************************************/
extern void gap_s_gatt_report_handle_get(struct gap_att_report_handle** p_hdl);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_read_rsp_set
*ÊäÈë²ÎÊý:uint8_t len	Ö¸¶¨¶ÁÏìÓ¦µÄ³¤¶È
					uint8_t *p_data ¶ÁÏìÓ¦µÄ¾ßÌåÊý¾Ý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃÁË¶ÁÏìÓ¦µÄ·µ»ØÊý¾Ý£¬Ò²¾ÍÊÇÖ÷»ú¶Ë¶ÁÈ¡ÏàÓ¦ÌØÐÔµÃµ½µÄÊý¾Ý
			µ÷ÓÃ¸Ãº¯ÊýµÄÊ±ºòÐ­ÒéÕ»¿ÉÄÜ²¢²»ÄÜ¹»Á¢¼´½«Êý¾Ý·¢ËÍ³öÈ¥£¬Ö»ÊÇ°ÑÊý¾Ý·Åµ½µ×²ã·¢ËÍ¶ÓÁÐÖÐ£¬Èç¹ûµ×²ã»º³åÇø
»¹ÓÐÊý¾Ý´ý·¢ËÍ£¬ÕâÀï»áÒòÎªÒªµÈ´ýÖ®Ç°µÄÊý¾Ý·¢ËÍÍê³É
**************************************************************************************************/
extern void gap_s_gatt_read_rsp_set(uint8_t len,uint8_t *p_data);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_write_err_rsp_set
*ÊäÈë²ÎÊý:uint16_t hdl	 attÖÐError ResponseÏìÓ¦µÄAttribute Handle In Error²ÎÊý
					uint8_t err attÖÐError ResponseÏìÓ¦µÄError Code²ÎÊý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÖ÷ÒªµÄÄ¿µÄÊÇÈÃAPPÓµÓÐÈÃÐ­ÒéÕ»µ×²ã»Ø¸´´íÎóµÄÄÜÁ¦£¬µ±appÊý¾ÝÓÐ´íÊ±¿ÉÒÔµ÷ÓÃ¸Ã½Ó¿Ú
**************************************************************************************************/
extern void gap_s_gatt_write_err_rsp_set(uint16_t hdl, uint8_t err);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_check_fifo_sz
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint16_t	Ö¸Ê¾µ×²ãµÄfifoµÄÊ£Óà´óÐ¡
*ËµÃ÷:¶ÔÓÚSYD8821£¬·¢ËÍÊý¾Ý¶¼½«´æ·ÅÔÚµ×²ãµÄfifoÖÐ£¬¸Ãº¯ÊýÄÜ¹»»ñÈ¡µ×²ãfifoÖÐÊ£ÓàµÄ´óÐ¡
**************************************************************************************************/
extern uint16_t gap_s_check_fifo_sz(void);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_data_send
*ÊäÈë²ÎÊý:uint8_t type Êý¾Ý·¢ËÍµÄÀàÐÍ£¬ÎªBLE_SEND_TYPEÃ¶¾Ù³ÉÔ±£¬¿ÉÒÔÊÇnotifyÐÎÊ½£¨BLE_GATT_NOTIFICATION£©
											£¬Ò²¿ÉÒÔÊÇÖ¸Ê¾ÐÎÊ½£¨BLE_GATT_INDICATION£©
					struct gap_att_report* p_report	Ö¸¶¨Òª·¢ËÍµÄÍ¨µÀ£¨¾ßÌåµÄÄ³¸öÌØÐÔ£©µÄ·¢ËÍ½á¹¹Ìå£¬´Ë½á¹¹ÌåÎª
																					_gatt_database_report_handleµÄÄ³¸ö³ÉÔ±
					uint8_t len	Òª·¢ËÍµÄÊý¾ÝµÄ³¤¶È
					uint8_t *p_data	Òª·¢ËÍµÄ¾ßÌåÊý¾Ý
*Êä³ö²ÎÊý:uint8_t	·µ»ØµÄ·¢ËÍ½á¹û£¬µ±µ×²ãµÄfifoÒÑ¾­ÌîÂú£¬»òÕß¹ý¶àµÄindicate»¹Ã»ÓÐµÈµ½·µ»ØµÄÊ±ºò£¬¸ÃÖµÎª0£¬
·ñÔòÎª1´ú±í·¢ËÍ³É¹¦
*ËµÃ÷:notifyÊÇ²»ÐèÒª·µ»ØµÄ·¢ËÍÄ£Ê½£¬¸ÃÄ£Ê½¿ÉÄÜ»á´æÔÚÖ÷»úµÄÉÏ²ã´¦Àí²»¼°Ê±¶ø¶ªµôÊý¾ÝµÄÇé¿ö£¬¶øindicateÊÇ
ÐèÒª·µ»ØÈ·ÈÏµÄ·¢ËÍÐÎÊ½£¬
			ËùÒÔÕâÀïÍê³ÉºóÐèÒªµÈ´ýÐ­ÒéÕ»ÉÏ±¨GAP_EVT_ATT_HANDLE_CONFIRMATIONÊÂ¼þ
**************************************************************************************************/
extern uint8_t gap_s_gatt_data_send(uint8_t type, struct gap_att_report* p_report, uint8_t len, uint8_t *p_data);







/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_gatt_data_send_report_confirmation_handle
*ÊäÈë²ÎÊý:uint8_t type Êý¾Ý·¢ËÍµÄÀàÐÍ£¬ÎªBLE_SEND_TYPEÃ¶¾Ù³ÉÔ±£¬¿ÉÒÔÊÇnotifyÐÎÊ½£¨BLE_GATT_NOTIFICATION£©
											£¬Ò²¿ÉÒÔÊÇÖ¸Ê¾ÐÎÊ½£¨BLE_GATT_INDICATION£©
					struct gap_att_report* p_report	Ö¸¶¨Òª·¢ËÍµÄÍ¨µÀ£¨¾ßÌåµÄÄ³¸öÌØÐÔ£©µÄ·¢ËÍ½á¹¹Ìå£¬´Ë½á¹¹ÌåÎª
																					_gatt_database_report_handleµÄÄ³¸ö³ÉÔ±
					uint8_t len	Òª·¢ËÍµÄÊý¾ÝµÄ³¤¶È
					uint8_t *p_data	Òª·¢ËÍµÄ¾ßÌåÊý¾Ý
*Êä³ö²ÎÊý:gap_s_gatt_data_send_report_confirmation_handleºÍgap_s_gatt_data_sendÓÃ·¨Ò»ÖÂ£¬Î¨Ò»µÄÇø±ðÊÇ
½øÐÐindicateµ÷ÓÃgap_s_gatt_data_send_report_confirmation_handleºóÐ­ÒéÕ»ÉÏ±¨GAP_EVT_ATT_HANDLE_CONFIRMATIONÊÂ¼þ
µÄÊ±ºò»á°Ñ·¢ËÍÊý¾ÝµÄÌØÐÔµÄhandleµÈÍ¬Ê±ÉÏ±¨ÉÏÀ´£¡
uint8_t	·µ»ØµÄ·¢ËÍ½á¹û£¬µ±µ×²ãµÄfifoÒÑ¾­ÌîÂú£¬»òÕß¹ý¶àµÄindicate»¹Ã»ÓÐµÈµ½·µ»ØµÄÊ±ºò£¬¸ÃÖµÎª0£¬
·ñÔòÎª1´ú±í·¢ËÍ³É¹¦
*ËµÃ÷:notifyÊÇ²»ÐèÒª·µ»ØµÄ·¢ËÍÄ£Ê½£¬¸ÃÄ£Ê½¿ÉÄÜ»á´æÔÚÖ÷»úµÄÉÏ²ã´¦Àí²»¼°Ê±¶ø¶ªµôÊý¾ÝµÄÇé¿ö£¬¶øindicateÊÇ
ÐèÒª·µ»ØÈ·ÈÏµÄ·¢ËÍÐÎÊ½£¬
			ËùÒÔÕâÀïÍê³ÉºóÐèÒªµÈ´ýÐ­ÒéÕ»ÉÏ±¨GAP_EVT_ATT_HANDLE_CONFIRMATIONÊÂ¼þ
**************************************************************************************************/
extern uint8_t gap_s_gatt_data_send_report_confirmation_handle(uint8_t type, struct gap_att_report* p_report, uint8_t len, uint8_t *p_data);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_connection_latency_mode
*ÊäÈë²ÎÊý:uint8_t en	-->ÊÇ·ñÊ¹ÄÜlatency
										0-->¹Ø±Õlatency
										1-->´ò¿ªlatency
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:À¶ÑÀ½øÈëÁ¬½ÓºóÖ÷»ú»áÒÔÁ¬½Ó²ÎÊýÖÐµÄinterval×÷Îª¼ä¸ôÓë´Ó»ú½øÐÐÒ»´Î½»»¥£¬¶ø¹æ·¶ÖÐ¹æ¶¨´Ó»úÄÜ¹»ºöÂÔµôÒ»¶¨
ÊýÁ¿µÄinterval¶ø´ïµ½½ÚÊ¡¹¦ºÄµÄÄ¿µÄ£¬
			ÄÜ¹»ºöÂÔintervalµÄ¸öÊý¾ÍÊÇlatency£¬¶ø¸Ãº¯ÊýÖ¸Ê¾Ð­ÒéÕ»µ×²ãÊ±ºòÊ¹ÄÜlatency
**************************************************************************************************/
extern void gap_s_connection_latency_mode(uint8_t en);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_profile_data_read
*ÊäÈë²ÎÊý:uint16_t addr	Æ«ÒÆµØÖ·£¬ÓÐÐ§·¶Î§ÊÇ0x0-0xfff
					uint16_t len	¶ÁÈ¡Êý¾ÝµÄ³¤¶È£¬ÓÐÐ§·¶Î§ÊÇ0x0-0xfff
					uint8_t *p_buf	·µ»ØÊý¾ÝµÄÖ¸Õë£¬SYD8821µÄflashÒªÇó¸ÃÖ¸ÕëÖ¸ÏòµÄÄÚ´æ±ØÐëÊÇ4×Ö½Ú¶ÔÆäµÄ
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾¶ÁÈ¡Êý¾ÝµÄ½á¹û
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:SYD8821ÄÚ²¿µÄflash³ýÁË´æ´¢´úÂëºÍÅäÖÃÎÄ¼þÍâ»¹µ¥¶ÀÁô³ö4KµÄÇøÓò¸øapp´æ´¢ÖØÒªÊý¾Ý£¬ÒòÎªÕâÐ©Êý¾ÝÊÇ´æ´¢ÔÚ
flashÖÐµÄ£¬ËùÒÔµôµçºóÒ²²»»á¶ªÊ§
			µ±´«ÈëµÄ²ÎÊý·Ç·¨Ê±£¬¸Ãº¯Êý·µ»Ø´íÎó£¬ÕâÀïÖØµã×¢ÒâÊý¾ÝÖ¸Õë±ØÐëÊÇËÄ×Ö½Ú¶ÔÆä£¬·ñÔò»á³ö´í
**************************************************************************************************/
extern uint8_t gap_s_profile_data_read(uint16_t addr , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_profile_data_write
*ÊäÈë²ÎÊý:uint16_t addr	Æ«ÒÆµØÖ·£¬ÓÐÐ§·¶Î§ÊÇ0x0-0xfff
					uint16_t len	Ð´Êý¾ÝµÄ³¤¶È£¬ÓÐÐ§·¶Î§ÊÇ0x0-0xfff
					uint8_t *p_buf	Ð´ÈëÊý¾ÝµÄÖ¸Õë£¬SYD8821µÄflashÒªÇó¸ÃÖ¸ÕëÖ¸ÏòµÄÄÚ´æ±ØÐëÊÇ4×Ö½Ú¶ÔÆäµÄ
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾Ð´ÈëÊý¾ÝµÄ½á¹û
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:SYD8821ÄÚ²¿µÄflash³ýÁË´æ´¢´úÂëºÍÅäÖÃÎÄ¼þÍâ»¹µ¥¶ÀÁô³ö4KµÄÇøÓò¸øapp´æ´¢ÖØÒªÊý¾Ý£¬ÒòÎªÕâÐ©Êý¾ÝÊÇ´æ´¢ÔÚ
flashÖÐµÄ£¬ËùÒÔµôµçºóÒ²²»»á¶ªÊ§
			µ±´«ÈëµÄ²ÎÊý·Ç·¨Ê±£¬¸Ãº¯Êý·µ»Ø´íÎó,ÕâÀïÖØµã×¢ÒâÊý¾ÝÖ¸Õë±ØÐëÊÇËÄ×Ö½Ú¶ÔÆä£¬·ñÔò»á³ö´í
			ÒòÎª¸Ãº¯ÊýÔÚÐ´flashÖ®Ç°»¹ÓÐ²Á³ýflashÉÈÇø£¬ËùÒÔ½¨Òé²»ÒªÆµ·±µ÷ÓÃ¸Ãº¯Êý£¬ÒÔÃâÔì³Éflash³ö´í
			¸Ãº¯ÊýÖ»ÊÇ°ÑÊý¾Ý·Åµ½µ×²ã»º³åÇøÖÐ£¬ÕýÔÚÖ´ÐÐflashµÄ²Ù×÷µÄÊÇÔÚble_sched_executeº¯ÊýÖÐ
**************************************************************************************************/
extern uint8_t gap_s_profile_data_write(uint16_t addr , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_mtureq
*ÊäÈë²ÎÊý:uuint16_t mtu	ÇëÇóÉèÖÃµÄmtu´óÐ¡
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ Exchange MTU RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬SYD8821ÍêÈ«Ö§³Ö¹æ·¶ÖÐl2cap²ãµÄ¹¦ÄÜ£¬ÕâÀïÖ§³Öapp
ÇëÇó¸Ä±ämtu£¬ÔÚÐèÒª´«Êä´óÁ¿Êý¾ÝµÄÊ±ºò¿ÉÒÔ°ÑMTUµ÷´ó
			µ±¶Ô·½ÊÕµ½Exchange MTU Requestºó»áÏàÓ¦·¢ËÍExchange MTU ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄ
Ê±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_mtureq(uint16_t mtu);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_findinforeq
*ÊäÈë²ÎÊý:uint16_t start_hdl ·¢ËÍATTµÄFind Information RequestÖ¸ÁîµÄStarting Handle²ÎÊý
					uint16_t end_hdl	·¢ËÍATTµÄFind Information RequestÖ¸ÁîµÄEnding Handle²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ Find Information RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ 
Find Information RequestÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½att_c_findinforeqºó»áÏàÓ¦·¢ËÍFind Information ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄ
Ê±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_findinforeq(uint16_t start_hdl, uint16_t end_hdl);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_findbytypevaluereq
*ÊäÈë²ÎÊý:uint16_t start_hdl ·¢ËÍATTµÄFind By Type Value RequestÖ¸ÁîµÄStarting Handle²ÎÊý
					uint16_t end_hdl	·¢ËÍATTµÄFind By Type Value RequestÖ¸ÁîµÄEnding Handle²ÎÊý
					uint16_t type	·¢ËÍATTµÄFind By Type Value RequestÖ¸ÁîµÄAttribute Type²ÎÊý
					uint8_t val_sz	·¢ËÍATTµÄFind By Type Value RequestÖ¸ÁîµÄAttribute Value²ÎÊýµÄ³¤¶È
					uint8_t *p_val	·¢ËÍATTµÄFind By Type Value RequestÖ¸ÁîµÄAttribute Value²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ Find By Type Value RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ 
Find By Type Value RequestÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½Find By Type Value Requestºó»áÏàÓ¦·¢ËÍFind By Type Value ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½
¸ÃÊý¾Ý°üµÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_findbytypevaluereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type, uint8_t val_sz, uint8_t *p_val);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_readbytypereq
*ÊäÈë²ÎÊý:uint16_t start_hdl ·¢ËÍATTµÄRead By Type RequestÖ¸ÁîµÄStarting Handle²ÎÊý
					uint16_t end_hdl	·¢ËÍATTµÄRead By Type RequestÖ¸ÁîµÄEnding Handle²ÎÊý
					uint16_t type_sz	·¢ËÍATTµÄRead By Type RequestÖ¸ÁîµÄAttribute Type²ÎÊýµÄ³¤¶È£¬¸ù¾Ý¹æ·¶ÕâÀï
														Ö»ÄÜ¹»Ìî³ä0»òÕß16
					uint8_t *p_type	·¢ËÍATTµÄRead By Type RequestÖ¸ÁîµÄAttribute Type²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ Read By Type RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Read By Type Request
ÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½Read By Type Requestºó»áÏàÓ¦·¢ËÍRead By Type ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄ
Ê±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_readbytypereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type_sz, uint8_t *p_type);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_readreq
*ÊäÈë²ÎÊý:uint16_t hdl ·¢ËÍATTµÄRead RequestÖ¸ÁîµÄAttribute Handle²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ Read RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍRead RequestÃüÁî£¬¹ØÓÚ
¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½Read Requestºó»áÏàÓ¦·¢ËÍRead ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã
»áÏògap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_readreq(uint16_t hdl);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_readblobreq
*ÊäÈë²ÎÊý:uint16_t hdl ·¢ËÍATTµÄRead Blob RequestÖ¸ÁîµÄAttribute Handle²ÎÊý
					uint16_t offset ·¢ËÍATTµÄRead Blob RequestÖ¸ÁîµÄValue Offset²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  Read Blob RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Read Blob RequestÃüÁî£¬
¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ Read Blob Requestºó»áÏàÓ¦·¢ËÍRead Blob ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄÊ±ºòÐ­Òé
Õ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_readblobreq(uint16_t hdl, uint16_t offset);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_readmultiplereq
*ÊäÈë²ÎÊý:uint8_t hdl_sz ·¢ËÍATTµÄRead Multiple RequestÖ¸ÁîµÄSet Of Handles²ÎÊýµÄ³¤¶È£¬
												ÒÀ¾Ý¹æ·¶¸Ã²ÎÊý²»µÃÐ¡ÓÚ4
					uint8_t *p_hdl ·¢ËÍATTµÄRead Multiple RequestÖ¸ÁîµÄSet Of Handles²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  Read Multiple RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Read Multiple Request
ÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ ÄRead Multiple Requestºó»áÏàÓ¦·¢ËÍRead Multiple ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°ü
µÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_readmultiplereq(uint8_t hdl_sz, uint8_t *p_hdl);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_readbygrouptypereq
*ÊäÈë²ÎÊý:uint16_t start_hdl ·¢ËÍATTµÄRead by Group Type RequestÖ¸ÁîµÄStarting Handle²ÎÊý
					uint16_t end_hdl ·¢ËÍATTµÄRead by Group Type RequestÖ¸ÁîµÄEnding Handle²ÎÊý
					uint16_t type_sz ·¢ËÍATTµÄRead by Group Type RequestÖ¸ÁîµÄAttribute Group Type²ÎÊýµÄ´óÐ¡£¬
													ÒÀ¾Ý¹æ·¶¸Ã´óÐ¡±ØÐëÎª2»òÕß16
					uint8_t *p_type	·¢ËÍATTµÄRead by Group Type RequestÖ¸ÁîµÄAttribute Group Type²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  Read by Group Type RequesÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ 
Read by Group Type RequestÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ Read by Group Type Requestºó»áÏàÓ¦·¢ËÍRead by Group Type ResponseÊý¾Ý°ü£¬µ±SYD8821
ÊÕµ½¸ÃÊý¾Ý°üµÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_readbygrouptypereq(uint16_t start_hdl, uint16_t end_hdl, uint16_t type_sz, uint8_t *p_type);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_writereq
*ÊäÈë²ÎÊý:uint16_t hdl ·¢ËÍATTµÄWrite RequestÖ¸ÁîµÄAttribute Handle²ÎÊý
					uint16_t sz ·¢ËÍATTµÄWrite RequestÖ¸ÁîµÄAttribute Value²ÎÊýµÄ´óÐ¡
					uint8_t *p_buf	·¢ËÍATTµÄWrite RequestÖ¸ÁîµÄAttribute Value²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  Read Blob RequestÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Write RequestÃüÁî£¬¹Ø
ÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ Write Requestºó»áÏàÓ¦·¢ËÍWrite ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°üµÄÊ±ºòÐ­ÒéÕ»µÄ
GATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_writereq(uint16_t hdl, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_writecmdreq
*ÊäÈë²ÎÊý:uint16_t hdl ·¢ËÍATTµÄWrite CommandÖ¸ÁîµÄAttribute Handle²ÎÊý
					uint16_t sz ·¢ËÍATTµÄWrite CommandÖ¸ÁîµÄAttribute Value²ÎÊýµÄ´óÐ¡
					uint8_t *p_buf	·¢ËÍATTµÄWrite CommandÖ¸ÁîµÄAttribute Value²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  Write CommandÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Write CommandÃüÁî£¬¹ØÓÚ
¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			ÒòÎªWrite CommandÃüÁî²¢Ã»ÓÐÏàÓ¦Êý¾Ý°ü£¬ËùÒÔÕâÀï²»»áÉÏ±¨ÈÎºÎÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_writecmdreq(uint16_t hdl, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_preparewritereq
*ÊäÈë²ÎÊý:uint16_t hdl ·¢ËÍATTµÄPrepare Write RequestÖ¸ÁîµÄAttribute Handle²ÎÊý
					uint16_t offset ·¢ËÍATTµÄPrepare Write RequestÖ¸ÁîµÄValue Offset²ÎÊý
					uint16_t sz ·¢ËÍATTµÄPrepare Write RequestÖ¸ÁîµÄPart Attribute Value²ÎÊýµÄ´óÐ¡
					uint8_t *p_buf	·¢ËÍATTµÄWrite CommandÖ¸ÁîµÄPart Attribute Value²ÎÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  att_c_preparewritereqÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ Prepare Write Request
ÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ Prepare Write Requestºó»áÏàÓ¦·¢ËÍWPrepare Write ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°ü
µÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_preparewritereq(uint16_t hdl, uint16_t offset, uint16_t sz, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_executewritereq
*ÊäÈë²ÎÊý:uint8_t flags ·¢ËÍATTµÄExecute Write RequestÖ¸ÁîµÄFlags²ÎÊý 
										0x00 -->Cancel all prepared writes
										0x01 --> Immediately write all pending prepared values
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  att_c_preparewritereqÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ 
Execute Write RequestÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			µ±¶Ô·½ÊÕµ½ Execute Write Requestºó»áÏàÓ¦·¢ËÍExecute Write ResponseÊý¾Ý°ü£¬µ±SYD8821ÊÕµ½¸ÃÊý¾Ý°ü
µÄÊ±ºòÐ­ÒéÕ»µÄGATT²ã»áÏò
			gap_s_att_c_evt_handler_setº¯ÊýÖ¸¶¨µÄ½Ó¿ÚÉÏ±¨¸ÃÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_executewritereq(uint8_t flags);





/*************************************************************************************************
*º¯ÊýÃû£ºatt_c_confirmation
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾  att_c_preparewritereqÖ¸ÁîÊÇ·ñ·¢ËÍ³É¹¦ 
									0	-->Ê§°Ü
									1	-->³É¹¦
*ËµÃ÷:¸ÃÖ¸ÁîÖ»ÓÐÔÚµ±Ç°µÄGATTÊÇ´¦ÔÚclientÄ£Ê½ÏÂ²Å¿ÉÊ¹ÓÃ£¬µ÷ÓÃ¸Ã½Ó¿ÚSYD8821½«»á·¢ËÍ 
			Handle Value ConfirmationÃüÁî£¬¹ØÓÚ¸ÃÃüÁîµÄ¾ßÌåÄÚÈÝ¿É¿´¹æ·¶ÖÐATTµÄÏà¹ØÕÂ½Ú
			ÒòÎªWrite CommandÃüÁî²¢Ã»ÓÐÏàÓ¦Êý¾Ý°ü£¬ËùÒÔÕâÀï²»»áÉÏ±¨ÈÎºÎÊÂ¼þ
**************************************************************************************************/
extern uint8_t att_c_confirmation(void);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_manager_idx_set
*ÊäÈë²ÎÊý:uint8_t idx	ÉèÖÃ°ó¶¨ÐÅÏ¢´æ´¢Î»ÖÃÒýË÷
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯ÊýÉèÖÃÊ¹ÓÃÄÄ¸öÎ»ÖÃµÄ°ó¶¨ÐÅÏ¢£¬Í¨¹ý¸Ãº¯Êý¿ÉÒÔÊµÏÖ°ó¶¨Éè±¸
µÄÇÐ»»
			ÒòÎªÄ¿Ç°SYD8821×÷Îª´Ó»úÖ»ÄÜ¹»Á¬½ÓÒ»¸öÖ÷»ú£¬ËùÒÔÍ¬Ò»Ê±¿ÌÖ»ÄÜ¹»ÓÐÒ»¸ö°ó¶¨ÐÅÏ¢
**************************************************************************************************/
extern void bm_s_bond_manager_idx_set(uint8_t idx);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_manager_idx_get
*ÊäÈë²ÎÊý:uint8_t idx	»ñÈ¡°ó¶¨ÐÅÏ¢´æ´¢Î»ÖÃÒýË÷
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯Êý»ñÈ¡µ±Ç°ÕýÔÚÊ¹ÓÃÄÄ¸öÎ»ÖÃµÄ°ó¶¨ÐÅÏ¢£¬
			ÒòÎªÄ¿Ç°SYD8821×÷Îª´Ó»úÖ»ÄÜ¹»Á¬½ÓÒ»¸öÖ÷»ú£¬ËùÒÔÍ¬Ò»Ê±¿ÌÖ»ÄÜ¹»ÓÐÒ»¸ö°ó¶¨ÐÅÏ¢
**************************************************************************************************/
extern void bm_s_bond_manager_idx_get(uint8_t *p_idx);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_info_get
*ÊäÈë²ÎÊý:struct gap_bond_dev *p_device	°ó¶¨ÐÅÏ¢½á¹¹Ìå
*Êä³ö²ÎÊý:uint8_t Ö¸Ê¾°ó¶¨ÐÅÏ¢ÊÇ·ñ»ñÈ¡³É¹¦ 
										0	-->Ê§°Ü
										1	-->³É¹¦
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯Êý»ñÈ¡bm_s_bond_manager_idx_setÖ¸¶¨µÄÒýË÷Î»ÖÃµÄ¾ßÌå°ó¶¨
ÐÅÏ¢ÄÚÈÝ
			°ó¶¨ÐÅÏ¢µÄÄÚÈÝÇë²Î¿¼gap_bond_dev½á¹¹Ìå
**************************************************************************************************/
extern uint8_t bm_s_bond_info_get(struct gap_bond_dev *p_device);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_info_add
*ÊäÈë²ÎÊý:struct gap_bond_dev *p_device	°ó¶¨ÐÅÏ¢½á¹¹Ìå
*Êä³ö²ÎÊý:uint8_t Ö¸Ê¾Ìí¼Ó°ó¶¨ÐÅÏ¢ÊÇ·ñ³É¹¦
					0	-->Ê§°Ü
					1	-->³É¹¦
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯ÊýÔö¼ÓÒ»¸ö°ó¶¨Éè±¸£¬ÒòÎªÔö¼Ó°ó¶¨µÄ²Ù×÷¶¼ÊÇÔÚÅä¶ÔÍê³ÉºóÓÉ
Ð­ÒéÕ»Ôö¼Ó£¬ËùÒÔ¸Ãº¯Êý·ÇÌØÊâÇé¿ö²»½¨ÒéappÊ¹ÓÃ
			°ó¶¨ÐÅÏ¢µÄÄÚÈÝÇë²Î¿¼gap_bond_dev½á¹¹Ìå
**************************************************************************************************/
extern uint8_t bm_s_bond_info_add(struct gap_bond_dev *p_device);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_info_delete_all
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t Ö¸Ê¾É¾³ý°ó¶¨ÐÅÏ¢ÊÇ·ñ³É¹¦
					0	-->Ê§°Ü
					1	-->³É¹¦
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯Êý½«É¾³ýËùÓÐµÄ°ó¶¨ÐÅÏ¢£¬Ö®ºóµÄÉè±¸¶¼½«ÖØÐÂ°ó¶¨
**************************************************************************************************/
extern uint8_t bm_s_bond_info_delete_all(void);





/*************************************************************************************************
*º¯ÊýÃû£ºbm_s_bond_info_delete
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t Ö¸Ê¾É¾³ý°ó¶¨ÐÅÏ¢ÊÇ·ñ³É¹¦
					0	-->Ê§°Ü
					1	-->³É¹¦
*ËµÃ÷:SYD8821ÁôÓÐÎå¸ö´æ´¢°ó¶¨ÐÅÏ¢µÄÎ»ÖÃ£¬¸Ãº¯Êý½«É¾³ýbm_s_bond_manager_idx_setÖ¸¶¨µÄÒýË÷Î»ÖÃµÄ°ó¶¨ÐÅÏ¢
**************************************************************************************************/
extern uint8_t bm_s_bond_info_delete(void);





/*************************************************************************************************
*º¯ÊýÃû£ºecdh_public_key_get
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:
*ËµÃ÷:
**************************************************************************************************/
extern void ecdh_public_key_get(uint8_t *p_x, uint8_t *p_y);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_mcu_clock_get
*ÊäÈë²ÎÊý:uint8_t *p_sel  ·µ»Øµ±Ç°Ê±ÖÓÔ´µÄÖ¸Õë£¬ÆäÊôÓÚÃ¶¾Ù_MCU_CLOCK_SEL_µÄ³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡µ±Ç°mcuÊ±ÖÓÔ´£¬
**************************************************************************************************/
extern void sys_mcu_clock_get(uint8_t *p_sel);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_mcu_clock_set
*ÊäÈë²ÎÊý:uint8_t  sel  ÉèÖÃMCUÊ±ÖÓÔ´£¬ÆäÊôÓÚÃ¶¾Ù_MCU_CLOCK_SEL_µÄ³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃmcuÊ±ÖÓÔ´£¬ÒòÎªMCUµÄÊ±ÖÓ±ØÐëÒª¾­¹ýÐ£×¼²ÅÄÜ¹»Ê¹ÓÃ£¬ËùÒÔµ÷ÓÃÁË¸Ãº¯Êýºó±ØÐë½ô½Ó×Åµ÷ÓÃ
sys_mcu_rc_calibrationº¯Êý
**************************************************************************************************/
extern void sys_mcu_clock_set(uint8_t  sel);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_mcu_clock_div_set
*ÊäÈë²ÎÊý:
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:
**************************************************************************************************/
extern void sys_mcu_clock_div_set(uint8_t  div);
extern void sys_mcu_clock_div_get(uint8_t *p_div);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_32k_clock_get
*ÊäÈë²ÎÊý:uint8_t *p_sel  ·µ»Øµ±Ç°Ê±ÖÓÔ´µÄÖ¸Õë£¬ÆäÊôÓÚÃ¶32K_CLOCK_SEL_³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡µ±Ç°32.768KHzµÍÆµÊ±ÖÓÔ´£¬ÒòÎª¸ßÆµÊ±ÖÓÔËÐÐ»á´øÀ´ºÜ¸ßµÄ¹¦ºÄ£¬ËùÒÔSYD8821µÄÏµÍ³¼ÈÓÐ¸ßÆµµÄ
Ê±ÖÓÔ´¸øMCUÊ¹ÓÃ£¬Ò²ÓÐµÍÆµµÄÊ±ÖÓÔ´¸øtimerÊ¹ÓÃ
**************************************************************************************************/
extern void sys_32k_clock_get(uint8_t *p_sel);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_32k_clock_set
*ÊäÈë²ÎÊý:uint8_t sel  ·µ»Øµ±Ç°Ê±ÖÓÔ´µÄÖ¸Õë£¬ÆäÊôÓÚÃ¶32K_CLOCK_SEL_³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃ32.768KHzµÍÆµÊ±ÖÓÔ´£¬ÒòÎª¸ßÆµÊ±ÖÓÔËÐÐ»á´øÀ´ºÜ¸ßµÄ¹¦ºÄ£¬ËùÒÔSYD8821µÄÏµÍ³¼ÈÓÐ¸ßÆµµÄÊ±ÖÓÔ´
¸øMCUÊ¹ÓÃ£¬Ò²ÓÐµÍÆµµÄÊ±ÖÓÔ´¸øtimerÊ¹ÓÃ
**************************************************************************************************/
extern void sys_32k_clock_set(uint8_t sel);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_32k_lpo_calibration
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t
					0	-->Ê§°Ü
					1	-->³É¹¦
*ËµÃ÷:¸Ãº¯ÊýÓÃÓÚÐ£×¼Ð¾Æ¬ÄÚ²¿µÍÆµÊ±ÖÓÔ´(Ò²³ÆÎªLPO£¬ÎªÄÚ²¿32.768KHz RCÕñµ´Æ÷)£¬ÒòÎªRCÕñµ´Æ÷ÓÐÊÜÎÂ¶ÈÓ°ÏìµÄ
ÌØÐÔ£¬
			ËùÒÔAPPÈç¹ûÊ¹ÓÃÄÚ²¿RCÕñµ´Æ÷µÄ»°Òª×öµ½ÒÔÒ»¶¨µÄ¼ä¸ôÈ¥Ð£×¼ÄÚ²¿RC¾§Õñ£¬ÕâÀïÍÆ¼öµÄ¼ä¸ôÊÇ3-10·ÖÖÓ£¬½¨Òé
Ê¹ÓÃ3·ÖÖÓ
			²»Ð£×¼µÄÄÚ²¿RCÕñµ´Æ÷»á´æÔÚºÜ´óµÄÆ«²î£¬BLEÁ¬½Ó»á³öÏÖ¶ÏÏßÒÔ¼°Á¬½Ó²»ÉÏµÄÎÊÌâ
**************************************************************************************************/
extern uint8_t sys_32k_lpo_calibration(void);





/*************************************************************************************************
*º¯ÊýÃû£ºsys_mcu_rc_calibration
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÓÃÓÚÐ£×¼Ð¾Æ¬ÄÚ¸ßÆµÊ±ÖÓÔ´£¬ÒòÎª¸ßËÙRCÕñµ´Æ÷ÔÚÉÏµçµÄÊ±ºò´æÔÚÆ«²î±È½Ï´óµÄÇé¿ö£¬
			ËùÒÔAPPÈç¹ûÊ¹ÓÃÄÚ²¿RCÕñµ´Æ÷µÄ»°Òª×öµ½ÔÚ¿ª»úµÄÑ¡ÔñÍêMCUÊ±ÖÓÖ®ºóÐ£×¼Ò»´Î¸ßÆµÊ±ÖÓÔ´
**************************************************************************************************/
extern void sys_mcu_rc_calibration(void);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_wakeup_config
*ÊäÈë²ÎÊý:struct gap_wakeup_config *p_cfg	»½ÐÑÔ´ÅäÖÃ½á¹¹Ìå
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÎªÁË½ÚÊ¡¹¦ºÄ£¬BLEÀ¶ÑÀ²¢²»ÊÇÊ±Ê±¿Ì¿Ì¹¤×÷µÄ£¬Ó¦¸ÃËµºÜ³¤µÄÒ»¶ÎÊ±¼ä¶¼ÊÇË¯Ãß×´Ì¬£¬ÄÇ×ÔÈ»¾Í»á²úÉú¹ØÓÚË¯Ãß
µÄÉèÖÃ£¬Ä¿Ç°SYD8821ÄÜ¹»»½ÐÑMCUµÄÓÐÈý¸ö£º¶¨Ê±Æ÷¡¢GPIO¡¢À¶ÑÀÖÐ¶Ï
			¹ØÓÚGPIOµÄÏà¹ØÅäÖÃÒª½÷É÷£¬ÆäÓÐÁ½¸öÑ¡Ïîgpi_wakeup_enºÍgpi_wakeup_cfgºÜ¹Ø¼ü£¬Ç°Õß´ú±íÊÇ·ñÊ¹ÄÜGPIO»½
ÐÑMCU£¬ºóÕß´ú±íÄÄÐ©¹Ü½Å»½ÐÑMCU£¬gpi_wakeup_cfgÊÇbit_mask
			±íÊ¾µÄ£¬±ÈÈçgpi_wakeup_cfg=0x00000900´ú±íGPIO11ºÍGPIO8ÄÜ¹»»½ÐÑ
**************************************************************************************************/
extern void pmu_wakeup_config(struct gap_wakeup_config *p_cfg);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_mcu_off
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý½«¹Ø±ÕMCU£¬ÄÚ´æºÍÍâÉèµÄ×´Ì¬»¹ÔÚ
**************************************************************************************************/
extern void pmu_mcu_off(void);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_system_off
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯Êý½«¹Ø±ÕÐ¾Æ¬ËùÓÐÄ£¿é£¬ÄÚ´æºÍÍâÉèµÈ¶¼»á±»¹Ø±Õ
**************************************************************************************************/
extern void pmu_system_off(void);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_reset
*ÊäÈë²ÎÊý:uint32_t reset_type	¸´Î»ÀàÐÍ,ÆäÊôÓÚÃ¶¾ÙPMU_RESET_FLAG_TYPEµÄ³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÕâÀïÔÚMCU_RESETÄ£Ê½ÏÂÖ»ÊÇ¸´Î»MCUºÍÒ»Ð©Êý×ÖµçÂ·£¬¶øGPIO,TIMER,RTC£¬pem_led£¬watchdogÕâÐ©ÍâÉèÊÇÃ»ÓÐ
±»¸´Î»µôµÄ,SYSTEM_RESET»á°ÑËùÓÐÄ£¿é¶¼¸´Î»µô
**************************************************************************************************/
extern void pmu_reset(uint32_t reset_type);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_system_reset
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÏµÍ³¸´Î»£¬Ïàµ±ÓÚ£ºpmu_reset(SYSTEM_RESET);
**************************************************************************************************/
extern void pmu_system_reset(void);





/*************************************************************************************************
*º¯ÊýÃû£ºpmu_mcu_reset
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:MCU¸´Î»£¬Ïàµ±ÓÚ£ºpmu_reset(MCU_RESET);
**************************************************************************************************/
extern void pmu_mcu_reset(void);





/*************************************************************************************************
*º¯ÊýÃû£ºble_flash_erase
*ÊäÈë²ÎÊý:uint32_t address  ²Á³ýµÄµØÖ·£¬ÕâÊÇÎïÀíµØÖ·
					uint8_t num	Òª²Á³ýµÄÉÈÇøÊý
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ²Á³ý³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:ble_flash_erase¡¢ble_flash_readÒÔ¼°ble_flash_writeÈý¸öº¯Êý¹¹³ÉÁËÐ¾Æ¬ÄÚ²¿flashµÄÈý¸ö²Ù×÷º¯Êý£¬
app¿ÉÒÔÊ¹ÓÃÕâÈý¸öº¯Êý¶ÔÐ¾Æ¬ÄÚ²¿flash½øÐÐ²Ù×÷
			ÔÚµ÷ÓÃble_flash_writeÖ®Ç°±ØÐëµ÷ÓÃble_flash_erase½øÐÐ²Á³ý£¬²¢ÇÒ×öºÃ±¸·Ý£¬ÒòÎªble_flash_erase²Á³ý
µÄµ¥Î»ÊÇÉÈÇø£¬Ò²¾ÍÊÇ4096¸öbyte£¬
			ËùÒÔµ±ble_flash_writeµÄµØÖ··¶Î§ÊÇÃ»ÓÐÐ´¹ý£¬ÄÇ¾Í²»ÐèÒªÔÙ´Îµ÷ÓÃble_flash_eraseº¯Êý£¬±ÈÈç²Á³ýÁËµÚÎå
¸öÉÈÇø£¨0x4000)£¬µÚÒ»´ÎÐ´0x4000-0x4100,
			µÚ¶þ´ÎÐ´0x4100-0x4200¾Í²»ÐèÒªÔÙ´Î²Á³ýÁË£¬ÒòÎªËûÃÇµÄµØÖ·²¢Ã»ÓÐÖØµþ¶øÇÒ»¹ÔÚÍ¬Ò»¸öÉÈÇø¡£
			SYD8821¶ÔÓÚ²Á³ýflash¿Õ¼äµÄ´ÎÊýÊÇÓÐÏÞÖÆµÄ£¨Ò»°ãÎª10Íò´Î£©,ËùÒÔ²»½¨ÒéÆµ·±µÄ²Á³ý
**************************************************************************************************/
extern uint8_t ble_flash_erase(uint32_t address,uint8_t num);





/*************************************************************************************************
*º¯ÊýÃû£ºble_flash_read
*ÊäÈë²ÎÊý:uint32_t address  Òª¶ÁÈ¡µÄµØÖ·£¬ÕâÊÇÎïÀíµØÖ·
					uint16_t len	Òª¶ÁÈ¡µÄ³¤¶È
					uint8_t *pbuf	·µ»ØÊý¾ÝµÄÖ¸Õë
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ²Ù×÷³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:¸Ãº¯Êý¸ºÔð´ÓÄÚ²¿flashÖÐ¶ÁÈ¡Êý¾Ý£¬
			ÎªÁË·ÀÖ¹¶ÁÈ¡µ½´úÂëµÄÇøÓò£¬ËùÒÔÕâÀïÒªÇó´úÂëÇøµÄÇ°Ãæ48Kbyte¿Õ¼ä²»¿É¶Á
**************************************************************************************************/
extern uint8_t ble_flash_read(uint32_t address,uint16_t len, uint8_t *pbuf);





/*************************************************************************************************
*º¯ÊýÃû£ºble_flash_write
*ÊäÈë²ÎÊý:uint32_t address  ÒªÐ´ÈëµÄµØÖ·£¬ÕâÊÇÎïÀíµØÖ·
					uint16_t len	ÒªÐ´ÈëµÄ³¤¶È
					uint8_t *pbuf	Ð´ÈëÊý¾ÝµÄÖ¸Õë
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ²Ù×÷³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:¸Ãº¯Êý¸ºÔðÐ´ÈëÊý¾Ýµ½ÄÚ²¿falsh¿Õ¼äÖÐ£¬SYD8821ÄÚ²¿µÄflash¿Õ¼äÒ²ÊÇÍ¨ÓÃµÄflash£¬ËùÒÔ×ñÑ­flash²Ù×÷µÄ
¹æ·¶£¬ÔÚµ÷ÓÃÐ´º¯ÊýÖ®Ç°±ØÐëµ÷ÓÃ²Á³ýº¯Êý
			ÒòÎªµ÷ÓÃÁË²Á³ýº¯ÊýÖ®ºó£¬flashÄÚ²¿¿Õ¼äµÄÊý¾Ý¾Í»Ø¸´³ÉÄ¬ÈÏÖµÁË£¬ËùÒÔÔÚ²Á³ýÖ®Ç°Ò»¶¨Òªµ÷ÓÃ¶ÁÈ¡º¯ÊýÏÈ°Ñ
Õû¸öÉÈÇø¶ÁÈ¡µ½ÄÚ´æ¿Õ¼äµÄÊý×éÖÐ£¬
			È»ºóÐÞ¸ÄÄÚ´æµÄÊý×éµÄÄÚÈÝ£¬ËæÖ®µ÷ÓÃ²Á³ýº¯Êý£¬ÔÙ°ÑÄÚ´æÖÐµÄÊý×éÐ´ÈëflashÖÐ
**************************************************************************************************/
extern uint8_t ble_flash_write(uint32_t address,uint16_t len, uint8_t *pbuf);





/*************************************************************************************************
*º¯ÊýÃû£ºble_flash_write_burst
*ÊäÈë²ÎÊý:uint32_t address  ÒªÐ´ÈëµÄµØÖ·£¬ÕâÊÇÎïÀíµØÖ·
					uint16_t len	ÒªÐ´ÈëµÄ³¤¶È
					uint8_t *pbuf	Ð´ÈëÊý¾ÝµÄÖ¸Õë
					uint8_t flush	´«Èë0
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ²Ù×÷³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:¸Ãº¯Êý¸ºÔðÐ´ÈëÊý¾Ýµ½ÄÚ²¿falsh¿Õ¼äÖÐ£¬SYD8821ÄÚ²¿µÄflash¿Õ¼äÒ²ÊÇÍ¨ÓÃµÄflash£¬ËùÒÔ×ñÑ­flash²Ù×÷µÄ
¹æ·¶£¬ÔÚµ÷ÓÃÐ´º¯ÊýÖ®Ç°±ØÐëµ÷ÓÃ²Á³ýº¯Êý
			ÒòÎªµ÷ÓÃÁË²Á³ýº¯ÊýÖ®ºó£¬flashÄÚ²¿¿Õ¼äµÄÊý¾Ý¾Í»Ø¸´³ÉÄ¬ÈÏÖµÁË£¬ËùÒÔÔÚ²Á³ýÖ®Ç°Ò»¶¨Òªµ÷ÓÃ¶ÁÈ¡º¯ÊýÏÈ°Ñ
Õû¸öÉÈÇø¶ÁÈ¡µ½ÄÚ´æ¿Õ¼äµÄÊý×éÖÐ£¬
			È»ºóÐÞ¸ÄÄÚ´æµÄÊý×éµÄÄÚÈÝ£¬ËæÖ®µ÷ÓÃ²Á³ýº¯Êý£¬ÔÙ°ÑÄÚ´æÖÐµÄÊý×éÐ´ÈëflashÖÐ
**************************************************************************************************/
extern uint8_t ble_flash_write_burst(uint32_t address,uint16_t len, uint8_t *pbuf,uint8_t flush);





/*************************************************************************************************
*º¯ÊýÃû£ºota_code_erase
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ²Á³ý³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:SYD8821µÄflashÖÐ»®·ÖÁËÁ½¸ö´úÂëÇø£¬Ò»¸ö×÷Îªµ±Ç°´úÂëµÄÔËÐÐÇø£¬Ò»¸ö×÷Îªota¹ý³ÌÖÐ´æ´¢´úÂëµÄ±¸·ÝÈ¥£¬ÕâÑù
¼´Ê¹ÔÚOTA¹ý³ÌÖÐ³ö´íÒÀ¾ÉÄÜ¹»¼ÌÐøÔËÐÐ´úÂëÇøµÄ´úÂë£¬
			²»ÖÁÓÚËÀ»ú£¬Èç¹ûotaµÄÊý¾ÝÊÇÕýÈ·µÄ£¬¾Í°Ñ±¸·ÝÇø×÷ÎªÔËÐÐÇø£¬¶øÔ­À´µÄÔËÐÐÇø¾Í×ÔÈ»±ä³ÉÁË±¸·ÝÇø£¬OTA·ÖÎªÈý
¸ö½×¶Î£¬²Á³ý£¬Ð´ÒÔ¼°¼ìÑé£¬
			µ÷ÓÃota_code_eraseº¯Êý½«»á²Á³ýflashÄÚ²¿µÄ±¸·Ý´úÂëÇøµÄÊý¾Ý
			¸Ãº¯ÊýÖ»ÊÇ×÷ÓÃÓÚÄÚ²¿ÓÐÁ½¸ö´úÂëÇøµÄÐÎÊ½£¬Èç¹û°ÑÕû¸öflash¿Õ¼ä¶¼×÷ÎªÒ»¸ö´úÂëÇø£¬ÄÇ¸Ãº¯Êý½«²»ÔÙÊÊÓÃ
**************************************************************************************************/
extern uint8_t ota_code_erase(void);





/*************************************************************************************************
*º¯ÊýÃû£ºota_code_write
*ÊäÈë²ÎÊý:uint32_t offset	µ±Ç°Êý¾Ý°üÖÐµÄflashÊý¾ÝÏà¶ÔÓÚota¿ªÊ¼Êý¾ÝµÄÆ«ÒÆ
					uint16_t len	±¾Êý¾Ý°üÖÐµÄÓÐÐ§flashÊý¾ÝµÄ³¤¶È
					uint8_t *p_buf	´Ë´ÎÊý¾Ý°üÖÐflashÊý¾ÝµÄÖ¸Õë
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñÐ´Êý¾Ý³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:SYD8821µÄflashÖÐ»®·ÖÁËÁ½¸ö´úÂëÇø£¬Ò»¸ö×÷Îªµ±Ç°´úÂëµÄÔËÐÐÇø£¬Ò»¸ö×÷Îªota¹ý³ÌÖÐ´æ´¢´úÂëµÄ±¸·ÝÈ¥£¬ÕâÑù
¼´Ê¹ÔÚOTA¹ý³ÌÖÐ³ö´íÒÀ¾ÉÄÜ¹»¼ÌÐøÔËÐÐ´úÂëÇøµÄ´úÂë£¬²»ÖÁÓÚËÀ»ú£¬Èç¹ûotaµÄÊý¾ÝÊÇÕýÈ·µÄ£¬¾Í°Ñ±¸·ÝÇø×÷ÎªÔËÐÐÇø£¬
¶øÔ­À´µÄÔËÐÐÇø¾Í×ÔÈ»±ä³ÉÁË±¸·ÝÇø£¬OTA·ÖÎªÈý¸ö½×¶Î£¬²Á³ý£¬Ð´ÒÔ¼°¼ìÑé£¬
			µ÷ÓÃota_code_eraseº¯Êý½«»á²Á³ýflashÄÚ²¿µÄ±¸·Ý´úÂëÇøµÄÊý¾Ý
			¸Ãº¯ÊýÖ»ÊÇ×÷ÓÃÓÚÄÚ²¿ÓÐÁ½¸ö´úÂëÇøµÄÐÎÊ½£¬Èç¹û°ÑÕû¸öflash¿Õ¼ä¶¼×÷ÎªÒ»¸ö´úÂëÇø£¬ÄÇ¸Ãº¯Êý½«²»ÔÙÊÊÓÃ
			SYD8801µÄota»úÖÆÊÇ°ÑotaÎÄ¼þ·Ö³ÉºÜ¶à¸öÊý¾Ý°ü£¬È»ºóÖðÌõÏÂ·¢µÄÐÎÊ½
**************************************************************************************************/
extern uint8_t ota_code_write(uint32_t offset , uint16_t len, uint8_t *p_buf);





/*************************************************************************************************
*º¯ÊýÃû£ºota_code_update
*ÊäÈë²ÎÊý:uint8_t *p_desc	ota´úÂëµÄÃèÊö
					uint8_t *p_ver	ota´úÂëµÄ°æ±¾
					uint32_t sz	ota´úÂëµÄ´óÐ¡
					uint16_t checksum	ota´úÂëµÄ¼ìÑéºÍ
*Êä³ö²ÎÊý:uint8_t	Ö¸Ê¾ÊÇ·ñ¸üÐÂ£¨Ð£Ñé£©³É¹¦
									0 --> Ê§°Ü
									1	-->	³É¹¦
*ËµÃ÷:SYD8821µÄflashÖÐ»®·ÖÁËÁ½¸ö´úÂëÇø£¬Ò»¸ö×÷Îªµ±Ç°´úÂëµÄÔËÐÐÇø£¬Ò»¸ö×÷Îªota¹ý³ÌÖÐ´æ´¢´úÂëµÄ±¸·ÝÈ¥£¬
ÕâÑù¼´Ê¹ÔÚOTA¹ý³ÌÖÐ³ö´íÒÀ¾ÉÄÜ¹»¼ÌÐøÔËÐÐ´úÂëÇøµÄ´úÂë£¬²»ÖÁÓÚËÀ»ú£¬Èç¹ûotaµÄÊý¾ÝÊÇÕýÈ·µÄ£¬¾Í°Ñ±¸·ÝÇø×÷ÎªÔËÐÐÇø£¬
¶øÔ­À´µÄÔËÐÐÇø¾Í×ÔÈ»±ä³ÉÁË±¸·ÝÇø£¬OTA·ÖÎªÈý¸ö½×¶Î£¬²Á³ý£¬Ð´ÒÔ¼°¼ìÑé£¬
			¸Ãº¯Êý¸ºÔð¼ÆËãflashÖÐµÄÊý¾ÝµÄ¼ìÑéÖµÊÇ·ñµÈÓÚ²ÎÊýchecksum£¬Èç¹û²»µÈÓÚÔò·µ»ØÊ§°Ü£¬ÕâÊ±¸´Î»ÔËÐÐµÄ³ÌÐòÊÇ
Ô­À´Î´OTAµÄ³ÌÐò£¬p_descºÍp_ver²»»áÆð×÷ÓÃ£¬
			Èç¹û¼ìÑéÖµÏàµÈ£¬¸Ãº¯Êý½«¸üÐÂÅäÖÃÎÄ¼þÖÐµÄp_descºÍp_ver£¬¶øºó·µ»Ø³É¹¦£¡
			×¢Òâ£ºota_code_updateº¯ÊýÖÐ²¢²»»á·¢Æð¸´Î»²Ù×÷£¬appÔÚµ÷ÓÃota_code_updateºó¿ÉÒÔ·¢Æð¸´Î»µÄ²Ù×÷£¬Èç¹û
¼ìÑé³É¹¦£¬ÔòOTA³É¹¦£¬·ñÔòOTAÊ§°Ü
**************************************************************************************************/
extern uint8_t ota_code_update(uint8_t *p_desc, uint8_t *p_ver, uint32_t sz, uint16_t checksum);





/*************************************************************************************************
*º¯ÊýÃû£ºble_sched_execute
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÎªbleµ÷¶Èº¯Êý£¬ËùÎ½µÄµ÷¶Èº¯Êý¾ÍÊÇËµ¸Ãº¯Êý»áÈ¥ÅÐ¶ÏÀ¶ÑÀµÄ×´Ì¬¶øÖ´ÐÐÒ»Ð©ÐÐÎª£¬ÒòÎªflashµÄ²Ù×÷±È
½Ï³¬Ê±£¬Èç¹ûÔÚÀ¶ÑÀÐ­ÒéÕ»ÐèÒª½øÐÐ´óÁ¿¶øÇÒÖØÒªµÄ¹¤×÷µÄÊ±ºò
		 £¨±ÈÈçÅä¶Ô³É¹¦ºóºó±£´æ°ó¶¨ÐÅÏ¢µÄ²Ù×÷£©²Ù×÷flashÊÆ±Ø»á×èÈûÀ¶ÑÀÐ­ÒéÕ»£¬ËùÒÔÕâÀï»á°ÑÒ»Ð©flashµÄ²Ù×÷·Å
µ½¸Ãº¯ÊýÀ´ÔËÐÐ£¬Í¬Ê±gap_s_profile_data_writeÒ²ÊÇÔÚ¸Ãº¯ÊýÖÐÆä×÷ÓÃ£¬µ«ÊÇble_flash_erase£¬ble_flash_read£¬
ble_flash_write£¬ota_code_erase£¬ota_code_write£¬ota_code_updateº¯Êý¶¼ÊÇÁ¢¼´ÆðÐ§µÄ
			±ØÐëÒªÔÚappµÄwhile£¨1£©Ö÷Ñ­»·ÖÐµ÷ÓÃ¸Ãº¯Êý£¬
**************************************************************************************************/
extern void ble_sched_execute(void);





/*************************************************************************************************
*º¯ÊýÃû£ºBBRFWrite
*ÊäÈë²ÎÊý:uint8_t addr	BBR¼Ä´æÆ÷µÄµØÖ·Æ«ÒÆ
					uint8_t data	ÒªÐ´ÈëµÄÊý¾Ý
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:BBR¼Ä´æÆ÷ÊÇSYD8821ÀïÃæ±È½ÏÌØÊâµÄÒ»Ð©¼Ä´æÆ÷£¬¸Ãº¯Êý¸ºÔðÐ´BBR¼Ä´æÆ÷
			BBR¼Ä´æÆ÷Éæ¼°µ½Õû¸öÐ¾Æ¬µÄÐÐÎª£¬²»ÕýÈ·µÄÐ´·¨ÓÐ¿ÉÄÜÔì³ÉÐ¾Æ¬´íÂÒ£¬ËùÒÔÕâÐ©¼Ä´æÆ÷µÄ²Ù×÷±ØÐëÒªÔÚ¹Ù·½µÄÕý
È·ËµÃ÷Ö¸µ¼ÏÂ½øÐÐ
**************************************************************************************************/
extern void BBRFWrite(uint8_t addr, uint8_t data);





/*************************************************************************************************
*º¯ÊýÃû£ºBBRFRead
*ÊäÈë²ÎÊý:uint8_t addr	BBR¼Ä´æÆ÷µÄµØÖ·Æ«ÒÆ
					uint8_t data	Òª¶Á³öµÄÊý¾ÝÖ¸Õë
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:BBR¼Ä´æÆ÷ÊÇSYD8821ÀïÃæ±È½ÏÌØÊâµÄÒ»Ð©¼Ä´æÆ÷£¬¸Ãº¯Êý¸ºÔð¶Á³öBBR¼Ä´æÆ÷µÄÄÚÈÝ
			BBR¼Ä´æÆ÷Éæ¼°µ½Õû¸öÐ¾Æ¬µÄÐÐÎª£¬²»ÕýÈ·µÄÐ´·¨ÓÐ¿ÉÄÜÔì³ÉÐ¾Æ¬´íÂÒ£¬ËùÒÔÕâÐ©¼Ä´æÆ÷µÄ²Ù×÷±ØÐëÒªÔÚ¹Ù·½µÄ
ÕýÈ·ËµÃ÷Ö¸µ¼ÏÂ½øÐÐ
**************************************************************************************************/
extern void BBRFRead(uint8_t addr, uint8_t* data);





/*************************************************************************************************
*º¯ÊýÃû£ºota_code_update_496kb
*ÊäÈë²ÎÊý:uint8_t *p_desc	ota´úÂëµÄÃèÊö
					uint8_t *p_ver	ota´úÂëµÄ°æ±¾
					uint16_t sz	ota´úÂëµÄ´óÐ¡
					uint16_t checksum	ota´úÂëµÄ¼ìÑéºÍ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:SYD8821µÄflash³ýÁËÕý³£µÄÁ½¸ö248KB´úÂëÇøµÄÄ£Ê½Íâ£¬ÒÀ¾ÉÖ§³ÖÒ»¸ö´óµÄ496µÄ´úÂëÇøµÄÐÎÊ½£¬µ«ÊÇÔÚ496kB
µÄÄ£Ê½ÏÂOTA±ØÐëÒªÔÚÐ¾Æ¬Íâ²¿Ôö¼ÓÒ»¸öflashÐ¾Æ¬£¬ota¹ý³ÌÖÐ°Ñ´úÂë·Åµ½¸ÃÐ¾Æ¬ÖÐ£¬ÔÚÏÂÔØ´úÂë½øÍâ²¿flashÍê³Éºó
£¬¶þ´ÎBootLoader´úÂë£¨¶þ´ÎBootLoaderÊÇÔÚSYD8821µÄ»ù´¡ÉÏÔÙ½¨Á¢Ò»¸ö¶ÀÁ¢ÓÚROMÇøÓòµÄ´úÂë£¬¸Ã´úÂë¿ÉÒÔ·ÅÔÚ
´úÂëÖÐ£¬ÔÚÐèÒªµÄÊ±ºò¿½±´ÈëÄÚ´æÔËÐÐ£¬Ò²¿ÉÒÔÖ±½Ó·ÅÔÚÐ¾Æ¬ÄÚ²¿falshÖÐ£¬µ«ÊÇÒªÇø·ÖºÃÇøÓò£©¸ºÔð°ÑÍâ²¿flashµÄ
´úÂë¿½±´µ½ÄÚ²¿flashÖÐ
			¸Ãº¯Êý¸ºÔð°Ñ¸÷¸ö²ÎÊýÖ¸¶¨µÄÄÚÈÝÐ´Èëµ½Ð¾Æ¬ÄÚ²¿µÄÅäÖÃÇø£¬Ö®ºó¸´Î»ÔËÐÐ²ÅÄÜ¹»ÔËÐÐµ½ÕýÈ·µÄ496KB´úÂë
			×¢Òâ£ºota_code_updateº¯ÊýÖÐ²¢²»»á·¢Æð¸´Î»²Ù×÷£¬appÔÚµ÷ÓÃota_code_updateºó¿ÉÒÔ·¢Æð¸´Î»µÄ²Ù×÷£¬Èç¹û
¼ìÑé³É¹¦£¬ÔòOTA³É¹¦£¬·ñÔòOTAÊ§°Ü
**************************************************************************************************/
extern uint8_t ota_code_update_496kb(uint8_t *p_desc, uint8_t *p_ver, uint32_t sz, uint16_t checksum);




/*************************************************************************************************
*º¯ÊýÃû£º_checksum_cache_496kb
*ÊäÈë²ÎÊý:uint32_t adr 496kb´úÂë´æÔÚÄÚ²¿flashµÄÎ»ÖÃ£¬´ËÎªÎïÀíÎ»ÖÃ
					uint16_t sz	ota´úÂëµÄ´óÐ¡
					uint16_t checksum	ota´úÂëµÄ¼ìÑéºÍ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:SYD8821µÄflash³ýÁËÕý³£µÄÁ½¸ö248KB´úÂëÇøµÄÄ£Ê½Íâ£¬ÒÀ¾ÉÖ§³ÖÒ»¸ö´óµÄ496µÄ´úÂëÇøµÄÐÎÊ½£¬µ«ÊÇÔÚ496kB
µÄÄ£Ê½ÏÂOTA±ØÐëÒªÔÚÐ¾Æ¬Íâ²¿Ôö¼ÓÒ»¸öflashÐ¾Æ¬£¬ota¹ý³ÌÖÐ°Ñ´úÂë·Åµ½¸ÃÐ¾Æ¬ÖÐ£¬ÔÚÏÂÔØ´úÂë½øÍâ²¿flashÍê³Éºó
£¬¶þ´ÎBootLoader´úÂë£¨¶þ´ÎBootLoaderÊÇÔÚSYD8821µÄ»ù´¡ÉÏÔÙ½¨Á¢Ò»¸ö¶ÀÁ¢ÓÚROMÇøÓòµÄ´úÂë£¬¸Ã´úÂë¿ÉÒÔ·ÅÔÚ
´úÂëÖÐ£¬ÔÚÐèÒªµÄÊ±ºò¿½±´ÈëÄÚ´æÔËÐÐ£¬Ò²¿ÉÒÔÖ±½Ó·ÅÔÚÐ¾Æ¬ÄÚ²¿falshÖÐ£¬µ«ÊÇÒªÇø·ÖºÃÇøÓò£©¸ºÔð°ÑÍâ²¿flashµÄ
´úÂë¿½±´µ½ÄÚ²¿flashÖÐ
			¸Ãº¯Êý¸ºÔð¼ÆËãflashÖÐµÄÊý¾ÝµÄ¼ìÑéÖµÊÇ·ñµÈÓÚ²ÎÊýchecksum£¬Èç¹û²»µÈÓÚÔò·µ»ØÊ§°Ü
			Ïà¶ÔÓÚota_code_updateº¯Êý£¬¸Ãº¯ÊýÓµÓÐ¼ìÑéÕû¸ö496kb´úÂëµÄÄÜÁ¦
**************************************************************************************************/
extern uint8_t _checksum_cache_496kb(uint32_t adr,uint32_t sz, uint16_t checksum);



/*************************************************************************************************
*º¯ÊýÃû£ºgap_s_smart_update_latency
*ÊäÈë²ÎÊý:struct gap_smart_update_params *p_smart_params  ÖÇÄÜÁ¬½Ó²ÎÊý½á¹¹Ìå
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:ÒòÎª¹æ·¶ÖÐ²¢Ã»ÓÐ¹ØÓÚlatencyµÄ¹ÜÀíµÄÏà¹ØËµÃ÷£¬ËùÒÔÕâÀïSYD8821Ôö¼ÓÒ»¸öÖÇÄÜ¹ÜÀíÁ¬½Ó²ÎÊýºÍlatency
µÄ»úÖÆ£¬½Ð×ösmart_update_latency£¬¸Ãº¯Êý¸ºÔð¹ÜÀílatencyµÄ¿ªÊ¼£¬ºÍÁ¬½Ó²ÎÊýµÄ¸üÐÂ²Ù×÷£¬ÔÚÁ¬½Ó²ÎÊýÇëÇó
±»Ö÷»ú¾Ü¾øµÄÊ±ºò£¬¸Ãº¯Êý¸ºÔð¶¯Ì¬µÄµ÷ÕûÁ¬½Ó²ÎÊýÔÙ´ÎÇëÇó¸üÐÂ£¬µ±ÓÐÊý¾Ý½øÐÐ½»»¥µÄÊ±ºò¸Ãº¯Êý¶¯Ì¬µÄ¿ªÆô
»òÕß¹Ø±ÕlatencyÒÔÊµÏÖ¹¦ºÄµÄÆ¥Åä¡£
			µ±È»£¬¸Ãº¯ÊýÊÇ¶ÀÁ¢ÓÚÔ­À´µÄÐ­ÒéÕ»»úÖÆµÄ£¬Ò²¾ÍÊÇËµ²»µ÷ÓÃ¸Ãº¯Êý£¬¶ÔÔ­À´µÄÐ­ÒéÕ»µÄÐÐÎªÊÇÃ»ÓÐÈÎºÎ
Ó°ÏìµÄ
			ÒòÎªlatencyºÍÁ¬½Ó²ÎÊýµÈ¸üÐÂÉæ¼°µ½À¶ÑÀµÄÎÈ¶¨ÐÔºÍ¼æÈÝÐÔµÈ£¬ËùÒÔÕâÀï½¨ÒéÊ¹ÓÃÖÇÄÜ¹ÜÀí»úÖÆ
			smart_update_latencyºÍÐ­ÒéÕ»±¾ÉíÊÇ¶ÀÁ¢µÄ£¬ËùÒÔÈç¹û²»µ÷ÓÃ¸Ãº¯Êý¶ÔÓÚÐ­ÒéÕ»µÄ¹¦ÄÜÊÇÃ»ÓÐÈÎºÎµÄÓ°Ïì
**************************************************************************************************/
extern uint8_t gap_s_smart_update_latency(struct gap_smart_update_params *p_smart_params);




/*************************************************************************************************
*º¯ÊýÃû£ºSystemSleep
*ÊäÈë²ÎÊý:POWER_SAVING_TYPE mode	µçÔ´Ê¡µçÀàÐÍ£¬ÎªPOWER_SAVING_TYPEÃ¶¾Ù³ÉÔ±
					MODULE_CONTROL_TYPE c Ä£¿é¿ØÖÆÀàÐÍ£¬ÎªMODULE_CONTROL_TYPEÃ¶¾ÙÀàÐÍ
					uint32_t ldo_delay	MCU»½ÐÑºó¡°nop"Ö´ÐÐÖ´ÐÐµÄ´ÎÊý
					PMU_WAKEUP_CONFIG_TYPE	»½ÐÑÔ´Ê¹ÄÜ¿ØÖÆ
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÊ¹SYD8821½øÈëÇ³¶ÈË¯ÃßÄ£Ê½£¬µÍ¹¦ºÄµÄ³Ì¶ÈÓÉmodeºÍcÁ½¸ö±äÁ¿¿ØÖÆ£¬¶ÔÓÚ¸Ãº¯Êý²»¿É´«Èë´ø¡°RESET¡±
µÄ±äÁ¿£¬
			½¨Òéµ÷ÓÃ·½Ê½£ºSystemSleep(POWER_SAVING_RC_OFF, FLASH_LDO_MODULE, 11000 , 
												(PMU_WAKEUP_CONFIG_TYPE)(FSM_SLEEP_EN|PIN_WAKE_EN|TIMER_WAKE_EN|RTC_WAKE_EN));
**************************************************************************************************/
extern uint8_t SystemSleep(POWER_SAVING_TYPE mode, MODULE_CONTROL_TYPE c,uint32_t ldo_delay,PMU_WAKEUP_CONFIG_TYPE w);







/*************************************************************************************************
*gap_s_att_mtu_get
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint16_t µ±Ç°att²ãµÄMTU
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡µ½µ±Ç°att²ãµÄmtu£¬¸ÃmtuÎªÖ÷»úºÍ´Ó»úÉÌÁ¿µÃµ½µÄ½á¹û£¬Ó¦ÓÃ³ÌÐò·¢ËÍµÄ×î´óµÄÊý¾Ý°ü²»ÄÜ¹»
´óÓÚ¸Ãº¯Êý·µ»ØµÄmtu´óÐ¡
			½¨Òéµ÷ÓÃ·½Ê½£ºmtu_now=gap_s_att_mtu_get;
**************************************************************************************************/
extern uint16_t gap_s_att_mtu_get(void);



/*************************************************************************************************
*ble_SetTxPower
*ÊäÈë²ÎÊý:BLE_TX_POWER value RF·¢Éä¹¦ÂÊÖµ£¬ÎªBLE_TX_POWER¾Ù³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃRF·¢Éä¹¦ÂÊ£¬ÉÏµçºóble_initÖ®ºóÉèÖÃÓÐÐ§£¬RF·¢Éä¹¦ÂÊÄ¬ÈÏÎª0dbm
**************************************************************************************************/
extern void ble_SetTxPower(BLE_TX_POWER value);

/*************************************************************************************************
*gap_s_att_mtu_get
*ÊäÈë²ÎÊý:ÎÞ
*Êä³ö²ÎÊý:uint8_t µ±Ç°Ó²¼þTX buffer×´Ì¬
*ËµÃ÷:¸Ãº¯Êý»ñÈ¡µ½µ±Ç°TXÓ²¼þbufferµÄ×´Ì¬ÊÇ·ñÎª¿Õ
			·µ»Ø1£ºÎÞÊý¾Ý£»·µ»Ø0£ºÓÐÊý¾Ý
**************************************************************************************************/			
extern uint8_t BBCheckTXFIFOEmpty(void);

/*************************************************************************************************
*ble_SetTxPower
*ÊäÈë²ÎÊý:uint8_t timer Á¬½ÓÊÂ¼þÖÐ×îºóÒ»¸öÊý¾Ý°üµ½ÏÂÒ»¸öÁ¬½ÓÊÂ¼þµ½À´µÄÊ±¼ä£¬µ¥Î»ÊÇ30.25us
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýµÄ²ÎÊý£¨¼ò³ÆTH_LAST£©¾ö¶¨ÁËÒ»¸öÁ¬½ÓÊÂ¼þÖÐ×îºóÒ»¸ömoredataÊý¾Ý°üµ½ÏÂÒ»¸öÁ¬½ÓÊÂ¼þµÄÊ±¼ä£¬
µ¥Î»Îª30.5us£¬Ò²¾ÍÊÇËµ¸Ãº¯ÊýµÄ²ÎÊýÒ²µ½£¬ÄÇÃ´Ò»¸öÁ¬½ÓÊÂ¼þÄÚµÄÊý¾Ý°üÊýÄ¿Ô½Ð¡£¬
**************************************************************************************************/
extern void ll_set_replying_packet_timer(uint8_t timer);

/*************************************************************************************************
*amic_set_bias
*ÊäÈë²ÎÊý:AMIC_BIAS value amicÄ£¿éµÄbiasÖµ£¬ÎªAMIC_BIAS¾Ù³ÉÔ±
*Êä³ö²ÎÊý:ÎÞ
*ËµÃ÷:¸Ãº¯ÊýÉèÖÃamicÄ£¿éµÄbiasÖµ£¬ÉÏµçºóble_initÖ®ºóÉèÖÃÓÐÐ§£¬amicÄ£¿éµÄbiasÖµÄ¬ÈÏÎª1.4V
**************************************************************************************************/
extern void amic_set_bias(AMIC_BIAS value);

#endif
