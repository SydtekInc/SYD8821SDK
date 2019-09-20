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
TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED1 toggling FreeRTOS task. */


xQueueHandle TaskMsgQueue;
#define Task_QUEUE_LEN  10 /* 队列的长度，最大可包含多少个消息 */
#define Task_QUEUE_SIZE 4 /* 队列中每个消息大小（字节） */


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

void Queue_Send_Task( void *pvParameters )
{
	BaseType_t xReturn = pdTRUE;
	uint32_t t_data = 0;
	
	for( ;; )
	{
		//xReturn = xQueueReceive( Test_Queue, &r_queue,portMAX_DELAY);//暂时不能使用这种方式
		
		xReturn = xQueueSend(TaskMsgQueue,(void*)&t_data,0);
		if (pdTRUE == xReturn)
		{
			DBGPRINTF(("\r\nqueue send data:%d\r\n",t_data));
			t_data ++;
		}
		vTaskDelay(1000);//1s
	}	
}


void Queue_Receive_Task( void *pvParameters )
{
	BaseType_t xReturn = pdTRUE;
	uint32_t r_data = 0;
	
	for( ;; )
	{
		//xReturn = xQueueReceive( Test_Queue, &r_queue,portMAX_DELAY);//暂时不能使用这种方式
		
		xReturn = xQueueReceive(TaskMsgQueue,(void*)&r_data,0);
		//xReturn = xQueueReceive( MsgQueue, &ReceiveNum,portMAX_DELAY);
		if (pdTRUE == xReturn)
		{
			DBGPRINTF(("queue receive data:%d\r\n",r_data));
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

	xReturn = xTaskCreate( Queue_Send_Task, "TASKS", configMINIMAL_STACK_SIZE, NULL, 3, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create Queue_Send_Task Success!\r\n"));
	
	xReturn = xTaskCreate( Queue_Receive_Task, "TASKR", configMINIMAL_STACK_SIZE, NULL, 4, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create Queue_Receive_Task Success!\r\n"));
	
	
	TaskMsgQueue = xQueueCreate((UBaseType_t)Task_QUEUE_LEN, (UBaseType_t)Task_QUEUE_SIZE);	
	if (NULL != TaskMsgQueue) DBGPRINTF(("Create TaskMsgQueue Success!\r\n"));
	
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

