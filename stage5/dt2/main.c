/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c
  * @author  MDS
  * @date    04022015
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_lightbar.h"
#include "s4353096_sysmon.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//struct Timer *tim_l;
//struct Timer *tim_r;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void TaskTimerLeft(void);
void TaskTimerRight(void);
int main (void) {
	//struct Timer timerleft;
	//struct Timer timerright;
	//tim_l = &timerleft;
	//tim_r = &timerright;
	//tim_l->count = 0;
	//tim_r->count = 0;
	BRD_init();
	Hardware_init();
	xTaskCreate( (void *) &TaskTimerLeft, (const signed char *) "TaskTimerLeft", mainLA_CHAN0TASK1_STACK_SIZE, NULL,  mainLA_CHAN0TASK1_PRIORITY, NULL );
  xTaskCreate( (void *) &TaskTimerRight, (const signed char *) "TaskTimerRight", mainLA_CHAN1TASK2_STACK_SIZE, NULL,  mainLA_CHAN1TASK2_PRIORITY, NULL );
	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}
void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts

	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
	s4353096_lightbar_init();
  s4353096_sysmon_init();
	portENABLE_INTERRUPTS();	//Enable interrupts

}
void TaskTimerLeft(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime1;
  const TickType_t xFrequency1 = 1000 / portTICK_PERIOD_MS;;
  xLastWakeTime1 = xTaskGetTickCount();
	struct Timer TimerLeft;
	TimerLeft.count = 0; /*Might possibly need to adjust this depending on if Task fires at 0 or 1 first*/
	for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
		TimerLeft.count++;
		/*Send Count via Queue here*/
		if (s4353096_QueueLightBar != NULL) {	/* Check if queue exists */
			if( xQueueSendToBack(s4353096_QueueLightBar, ( void * ) &TimerLeft, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
    vTaskDelayUntil( &xLastWakeTime1, xFrequency1 );                //Extra Task Delay of 3ms
    S4353096_LA_CHAN0_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

void TaskTimerRight(void) {
  S4353096_LA_CHAN1_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime2;
  const TickType_t xFrequency2 = 100 / portTICK_PERIOD_MS;;
  xLastWakeTime2 = xTaskGetTickCount();
	struct Timer TimerRight;
	TimerRight.count = -1;
	for (;;) {
    S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
		TimerRight.count++;
		/*if (timeright.count == 10) {
			timeright.count = 0;
		} else {

		}*/
		/*Adjust count for sending via queue*/
		TimerRight.count = ((TimerRight.count & 0x1F) << 5);
		/*Send Count via Queue here*/
		if (s4353096_QueueLightBar != NULL) {	/* Check if queue exists */
			if( xQueueSendToBack(s4353096_QueueLightBar, ( void * ) &TimerRight, ( portTickType ) 10 ) != pdPASS ) {
				debug_printf("Failed to post the message, after 10 ticks.\n\r");
			}
		}
		TimerRight.count = ((TimerRight.count & 0x3E0) >> 5);
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2 );                //Extra Task Delay of 3ms
    S4353096_LA_CHAN1_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

/*void s4353096_TaskLightBar(void) {
  S4353096_LA_CHAN2_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime3;
  const TickType_t xFrequency3 = 200 / portTICK_PERIOD_MS;;
  xLastWakeTime3 = xTaskGetTickCount();
  for (;;) {
    S4353096_LA_CHAN2_SET();*/      //Set LA Channel 0
    /*Do Stuff Here*/
		/*lightbar_value = ((tim_l->count & 0xF) ^ ((tim_r->count & 0xF) << 4));

    vTaskDelayUntil( &xLastWakeTime3, xFrequency3 );               //Extra Task Delay of 3ms
    S4353096_LA_CHAN2_CLR();
    vTaskDelay(1);                // Mandatory Delay
  }
}*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*void vApplicationTickHook( void ) {

	BRD_LEDOff();
}*/
