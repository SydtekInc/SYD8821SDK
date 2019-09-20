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
//#include "uart_0_debug.h"
#include "gpio.h"
#include "pad_mux_ctrl.h"
#include "led_key.h"


void board_init(void)
{
#ifdef CONFIG_UART_ENABLE
	//Uart init GPIO 20 21
	pad_mux_write(GPIO_20, 7);
	pad_mux_write(GPIO_21, 7);

	dbg_init();
	DBGPRINTF(("FreeRTOS BLE %s:%s ...\r\n",__DATE__ ,__TIME__));
#endif
}


//Task1
void Task1( void *pvParameters )
{
	for( ;; )
	{		
		vTaskDelay(500);//500ms
		DBGPRINTF(("\r\nTask1 vTaskDelay\r\n"));
	}	
}

//Task2
void Task2( void *pvParameters )
{
	portTickType xLastWakeTime;
	 xLastWakeTime = xTaskGetTickCount();
	for( ;; )
	{		
		vTaskDelayUntil( &xLastWakeTime, 3000 );
		DBGPRINTF(("\r\nTask2 vTaskDelayUntil\r\n"));
	}	
}



void vStartThreadTasks( void )
{
	BaseType_t xReturn = pdPASS;/* Create Result*/
	
	DBGPRINTF(("Init task\r\n"));

	xReturn = xTaskCreate( Task1, "Task1", configMINIMAL_STACK_SIZE, NULL, 4, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create Task1 Success!\r\n"));
	
	xReturn = xTaskCreate( Task2, "Task2", configMINIMAL_STACK_SIZE, NULL, 3, ( xTaskHandle * ) NULL );
	if(pdPASS == xReturn) DBGPRINTF(("Create Task2 Success!\r\n"));

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

	__enable_irq();	

	vStartThreadTasks();
	
	/* Start the scheduler. */
	vTaskStartScheduler();
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}

