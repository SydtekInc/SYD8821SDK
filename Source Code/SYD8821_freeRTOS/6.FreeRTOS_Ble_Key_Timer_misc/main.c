/* BLE */
#include "ARMCM0.h"
#include "config.h"
#include "ble_slave.h"
#include "SYD_ble_service_Profile.h"
#include "ble.h"

/* freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

/*drive*/
#include "debug.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "led_key.h"

#define LED_TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
#define LED_TIMER_PERIOD      1000           /**< Task delay. Delays a LED0 task for 200 ms */

QueueHandle_t Ble_RX_Queue =NULL;
#define BLE_RX_QUEUE_LEN  10 /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define BLE_RX_QUEUE_SIZE sizeof(struct queue_att_write_data) /* ������ÿ����Ϣ��С���ֽڣ� */


TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED1 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED2 toggling FreeRTOS timer. */

xQueueHandle KeyMsgQueue;
#define KEY_QUEUE_LEN  10 /* ���еĳ��ȣ����ɰ������ٸ���Ϣ */
#define KEY_QUEUE_SIZE 4 /* ������ÿ����Ϣ��С���ֽڣ� */

uint32_t send_data1 = 0;

void key1_irq_handle(void)
{
	BaseType_t xReturn = pdPASS;
	BaseType_t xHigherPriorityTaskWoken;
	uint32_t ulReturn;

	//xReturn = xQueueSendFromISR(Test_Queue,send_data1,NULL);
	ulReturn = taskENTER_CRITICAL_FROM_ISR();
	
	xReturn = xQueueSendFromISR(KeyMsgQueue,&send_data1,&xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	
	if (pdPASS == xReturn) 
	{
		dbg_printf("\r\nkey1 irq send queue success! data:%d\r\n",send_data1);
		send_data1 ++;
	}
	else
	{	
		dbg_printf("\r\nkey1 irq send queue error!\r\n");		
	}
	taskEXIT_CRITICAL_FROM_ISR(ulReturn);	
}

void gpio_callback(GPIO_BANK_TYPE type, uint32_t bitmask)
{
	switch(type)
	{
		case GPIO_BANK_0: //GPIO_0 - GPIO_31
			switch (bitmask)
			{
				case U32BIT(KEY1):key1_irq_handle();
					break;
				case U32BIT(KEY2):gpo_toggle(LED4);
					break;
				case U32BIT(KEY3):gpo_toggle(LED4);
					break;
				case U32BIT(KEY4):gpo_toggle(LED4);
					break;
				default:
					break;
			}
			break;
		case GPIO_BANK_1: // GPIO 32 ~ 38
			break;
		default:break;
	}
}


void board_init(void)
{
	
#ifdef CONFIG_UART_ENABLE
	//Uart init GPIO 20 21
	pad_mux_write(GPIO_20, 7);
	pad_mux_write(GPIO_21, 7);

	dbg_init();
	DBGPRINTF(("FreeRTOS BLE %s:%s ...\r\n",__DATE__ ,__TIME__));
#endif
	
	//led 1 2 3 4 --> GPIO25 23 34 10
	pad_mux_write(LED1,0);
	pad_mux_write(LED2,0);
	pad_mux_write(LED3,0);
	pad_mux_write(LED4,0);
	
	gpo_config(LED1,1);
	gpo_config(LED2,1);
	gpo_config(LED3,1);
	gpo_config(LED4,1);
	
	
	//KEY 1 2 3 4 -->GPIO14 15 16 17
	pad_mux_write(KEY1,0);
	pad_mux_write(KEY2,0);
	pad_mux_write(KEY3,0);
	pad_mux_write(KEY4,0);
	gpi_config(KEY1,PULL_UP);
	gpi_config(KEY2,PULL_UP);
	gpi_config(KEY3,PULL_UP);
	gpi_config(KEY4,PULL_UP);
	
	//config KEY 1 
	gpi_irq_set_cb(gpio_callback);
	gpi_enable_int(KEY1, EDGE_TRIGGER,POL_FALLING_LOW);
}


/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
	
    while (true)
    {
        gpo_toggle(LED1);
        /* Delay a task for a given number of ticks */
        vTaskDelay(LED_TASK_DELAY);
        /* Tasks must be implemented to never return... */
    }
}

/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (void * pvParameter)
{
	gpo_toggle(LED2);
}


//BLE RX Task
void BLE_Queue_Receive_Task( void *pvParameters )
{
	BaseType_t xReturn = pdTRUE;
	struct queue_att_write_data r_data = {0};
	
	for( ;; )
	{
		//xReturn = xQueueReceive( Test_Queue, &r_queue,portMAX_DELAY);//��ʱ����ʹ�����ַ�ʽ
		
		xReturn = xQueueReceive(Ble_RX_Queue,(void*)&r_data,0);
		//xReturn = xQueueReceive( MsgQueue, &ReceiveNum,portMAX_DELAY);
		if (pdTRUE == xReturn)
		{
			DBGPRINTF(("BLE Queue Recevie\r\n"));
			DBGPRINTF(("UUID:%04x\r\n",r_data.uuid));
			DBGHEXDUMP("data\r\n",r_data.data,r_data.sz);
			DBGPRINTF(("\r\n"));
		}
		vTaskDelay(10);//10ms
	}	
}


//key irq queue  handle task
void Key_IRQ_Queue_Receive_Task( void *pvParameters )
{
	BaseType_t xReturn = pdTRUE;
	uint32_t r_data = 0;
	
	for( ;; )
	{
		//xReturn = xQueueReceive( Test_Queue, &r_queue,portMAX_DELAY);//��ʱ����ʹ�����ַ�ʽ
		
		xReturn = xQueueReceive(KeyMsgQueue,(void*)&r_data,0);
		//xReturn = xQueueReceive( MsgQueue, &ReceiveNum,portMAX_DELAY);
		if (pdTRUE == xReturn)
		{
			DBGPRINTF(("key1 irq queue receive data:%d\r\n",r_data));
		}
		vTaskDelay(10);//10ms
	}	
}


void vStartThreadTasks( void )
{
	BaseType_t xReturn = pdPASS;/* Create Result*/
	
	DBGPRINTF(("Init task and queue\r\n"));
	/* Create task for LED1 blinking with priority set to 2 */
	xReturn = xTaskCreate(led_toggle_task_function, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, &led_toggle_task_handle);
	if(pdPASS == xReturn) DBGPRINTF(("Create LED1 Task Success!\r\n"));
	
	/* Start timer for LED2 blinking */
	led_toggle_timer_handle = xTimerCreate( "LED2", LED_TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
	xReturn = xTimerStart(led_toggle_timer_handle, 0);
	if(pdPASS == xReturn) DBGPRINTF(("Create LED2 Softtimer Task Success!\r\n"));
	
		
	xReturn = xTaskCreate( BLE_Queue_Receive_Task, "BLE_R", configMINIMAL_STACK_SIZE, NULL, 3, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create BLE_Queue_Receive_Task Success!\r\n"));
	
	
	xReturn = xTaskCreate( Key_IRQ_Queue_Receive_Task, "KEY_R", configMINIMAL_STACK_SIZE, NULL, 2, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create Key_IRQ_Queue_Receive_Task Success!\r\n"));
	
	
	Ble_RX_Queue = xQueueCreate((UBaseType_t)BLE_RX_QUEUE_LEN, (UBaseType_t)BLE_RX_QUEUE_SIZE);	
	if (NULL != Ble_RX_Queue) DBGPRINTF(("Create BLE_RX_QUEUE_LEN Success!\r\n"));
	
	KeyMsgQueue = xQueueCreate((UBaseType_t)KEY_QUEUE_LEN, (UBaseType_t)KEY_QUEUE_SIZE);	
	if (NULL != KeyMsgQueue) DBGPRINTF(("Create KEY_QUEUE_LEN Success!\r\n"));
	
	DBGPRINTF(("\r\nSYD8821 freeRTOS Start\r\n"));
	DBGPRINTF(("\r\n"));
	
}


/*-----------------------------------------------------------*/
int main( void )
{
	__disable_irq();
	
	//ble stack init
	ble_init();
	
	// RC bumping
	sys_mcu_rc_calibration();	
	// Select External XO
	sys_32k_clock_set(SYSTEM_32K_CLOCK_XO);
	// Set MCU Clock 64M
	sys_mcu_clock_set(MCU_CLOCK_64_MHZ);
	
	
	board_init();
	DBGPRINTF(("board hardware init\r\n"));
	gap_s_adv_start();
	DBGPRINTF(("start ble adv\r\n\r\n"));

	__enable_irq();	

	/* Start user demo tasks. */
	vStartThreadTasks();

	/* Start the scheduler. */
	vTaskStartScheduler();
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}


