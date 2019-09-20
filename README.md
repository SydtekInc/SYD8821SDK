# SYD8821SDK    
## 本程序版权属于成都盛芯微科技SYDTEK，更多内容请看官网：<a href="http://www.sydtek.com/" title="Title">http://www.sydtek.com/</a>
  
2018-04-20  
Initial version;  
<br/><br/><br/>



2018-05-07 完善SDK；  
1.更新ADC例程，修复之前的ADC dma的bug  
2.在《SYD8821_misc》目录下增加《cache_496KB_BootLoader_twice》例程，介绍使用“SYD8821_FLASH_CACHE_BootLoader.FLM”插件  
3.在《SYD8821_misc》目录下增加《TIMER4》例程，介绍使用timer4，timer4不同于其他timer之处在于计数器位数的不同  
4.在《SYD8821_misc》目录下增加《rtt》的例程  
5.更新了《Include》目录下的《ble_service.h》文件，增加了对BLE各个函数的注释，增加了att的相关宏  
6.更新了《lib》目录下的《ble_slve.lib》文件，实现《ble_service.h》的相关函数  
7.修改所有的《startup_ARMCM0.s》文件，把“Stack_Size      EQU     0x00000400”改为“Stack_Size      EQU     0x00001000”  
8.增加I2S驱动  
9.增加I2S RX的驱动！  
10.增加fast_pwm_xosc  
11.增加7816, IIC_SLAVE驱动  
12.增加AMIC驱动  
<br/><br/><br/>



2018-09-15  
1.更新LIB(syd8821_ble_slave2018_11_12_V1.lib)，更新4K文件(SYD8821_Cache_0db_Setting_2018-11-08.bin)；  
--新版IC修复内部32k rc无法使用的bug；  
2.更新lib《syd8821_ble_slave20181120.lib》，增加smart_update_latency机制，以达到和之前的lib兼容！  
3.增加ble_flash_write_burst函数API  
4.修改ADC驱动  
<br/><br/><br/>



2018-12-01 新增FreeRTOS操作系统  
1.在《Source Code》中增加《SYD8821_freeRTOS》目录，并在《SYD8821_freeRTOS》文件夹中增加8个SYD8821的freeRTOS使用示例  
分别是：  
《1.FreeRTOS_Task》最简单的串口任务打印  
《2.FreeRTOS_Timer_led》任务延时和软定时器  
《3.FreeRTOS_Queue》任务之间的消息队列使用  
《4.FreeRTOS_Key_IRQ_Queue》中断与任务的消息队列使用  
《5.FreeRTOS_Ble_Queue》用手机往UUID（0002） 发送一个数据，然后用消息队列转接到任务处理  
《6.FreeRTOS_Ble_Key_Timer_misc》集合以上几种功能  
2.更新/tools/SYDTEK Studios下的三个SWD下载插件，更新文件分别是  
《SYD8821_FLASH_CACHE_248KB.FLM》  
《SYD8821_FLASH_CACHE_496KB.FLM》  
《SYD8821_FLASH_XROM_128KB.FLM》  
3.删除掉头文件《ble_slave.h》中的SystemPowerDown函数，并且协议栈的lib中已经没有该函数了  
4.增加gap_s_att_mtu_get函数  
5.使用最新的协议栈的lib  
<br/><br/><br/>



2018-12-15  完善FreeRTOS系统  
1.删除《SYD8821_misc》中《cache》的工程，该工程在其他工程中都有体现！  
2.删除《SYD8821_misc》《cache_496KB_BootLoader_twice》工程，因为该工程和《cache_496KB_BootLoader》一样  
3.该版本SDK有涉及到所有工程的修改：  
A:原来在《Include》目录下的config.h文件挪到各个工程目录下，和main.c同一个目录，比如“\Source Code\SYD8821\ADC”目录  
B.原来在《Include》目录下的《ble_slave.h》文件挪到《lib》目录下，并且所有工程都修改了引索！  
4.增加《BLE》目录，并且在该目录中放入OTA的C文件和H文件  
5.增加《SYD8821_ble_peripheral》目录，并逐步增加BLE相关工程，目前增加《1.SYD8821_BLE_UART》透传工程！  
6、优化调整SDK中的《Source Code》结构目录  
  a、原先《SYD8821》目录更名为《SYD8821_peripheral》，存放SYD8821相关外设驱动例程  
  b、原先《SYD8821_misc》目录更名为《SYD8821_peripheral_misc》，存放SYD8821相关特殊外设例程  
  c、增加《Source_FreeRTOS》目录，存放FreeRTOS相关的源代码  
  d、增加《SYD8821_OTA》目录，存放OTA相关例程  
  e、在《Lib》目录下，增加《syd8821_ancs_lib.lib》和《ancs.h》文件  
7、新增BLE相关例程  
  a、在《SYD8821_ble_peripheral》目录下，新增三个工程,分别是：  
  《2.SYD8821_BLE_ANCS》   ANCS推送例程  
   《3.SYD8821_BLE_Wechat》 微信运动例程  
  《4.SYD8821_BLE_HID》    HID例程  
  b、在《SYD8821_freeRTOS》目录下，新增三个工程,分别是：  
  《7.FreeRTOS_Ble_ANCS》   FreeRTOS的ANCS推送例程  
  《8.FreeRTOS_Ble_Wechat》 FreeRTOS的微信运动例程  
  《9.FreeRTOS_Ble_HID》    FreeRTOS的HID例程  
  《10.FreeRTOS_Ble_ANCS_Wechat_HID》 FreeRTOS的ANCS推送+微信运动+HID集合例程  
8、使用最新的协议栈的lib（更新ENC_KEY事件上报）  
<br/><br/><br/>



2019-1-2 增加OTAV3.0协议的OTA程序  
1.在《SYD8821_OTA》目录下增加《SYD8821_BLE_OTAV30_Double》工程  
2.在《\Source Code\BLE》目录下增加《otaV30.c》和《otaV30.h》文件  
<br/><br/><br/>



2019-1-23   
 1、在《SYD8821_freeRTOS》目录下增加《11.FreeRTOS_Ble_ADC》    FreeRTOS的HID例程  
 2、修改了ble_lib,在《ble_slave.h》增加如下接口  
 		extern uint16_t gap_s_att_mtu_get(void);  
 		extern void ble_SetTxPower(BLE_TX_POWER value);  
 		extern uint8_t BBCheckTXFIFOEmpty(void);	  
 		extern void ll_set_replying_packet_timer(uint8_t timer);  
 		extern void amic_set_bias(AMIC_BIAS value);  
 3、更新4K文件《SYD8821_Cache_0db_Setting_2018-12-27.bin》  
 4、更新下载工具《SYDTEK Studio release2019-1-15》  
 5、在《SYD8821_freeRTOS》目录下增加《12.FreeRTOS_Ble_RTC》    FreeRTOS的RTC例程   
 6、增加security_connection的例程，工程在“Source Code\SYD8821_ble_peripheral\5.SYD8821_BLE_security_connection”,该工程从同目录的“1.SYD8821_BLE_UART”工程修改，除了关于安全连接的修改，还打开了SWD的RTT调试，修改了睡眠的算法！  
要使用安全连接必须下载工程目录下面的4K_SETTING:《SYD8821_Cache_0db_10PF_10PPM_Setting_security_connection_2019-01-09.bin》  
<br/><br/><br/>



2019-3-11  
1.使用新的lib（syd8821_ble_slave_20190308.lib），主要差异在flash擦除函数！  
2.在《\Source Code\SYD8821_ble_peripheral》下增加《6.SYD8821_BLE_SCAN》工程  
3."SYD8821_SDK\Source Code\SYD8821_ble_peripheral\4.SYD8821_BLE_HID"中把广播的UUID修改为0x1812  
4.修改《SYDTEK Studio  release20190610v3.5.0 》  
   这里务必使用最新的tool，原来的tool在某种情况下有可能有问题！  
<br/><br/><br/>


