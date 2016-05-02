/**
  ******************************************************************************
  * @file    stage5/dt1/main.c
  * @author  Steffen Mitchell
  * @date    04022015
  * @brief   FreeRTOS sysmon tester.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_sysmon.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void Task1_Task(void);
void Task2_Task(void);
void Task3_Task(void);
int main (void) {
	BRD_init();
	Hardware_init();
	xTaskCreate( (void *) &Task1_Task, (const signed char *) "TASK1", mainLA_CHAN0TASK1_STACK_SIZE, NULL,  mainLA_CHAN0TASK1_PRIORITY, NULL );
  xTaskCreate( (void *) &Task2_Task, (const signed char *) "TASK2", mainLA_CHAN1TASK2_STACK_SIZE, NULL,  mainLA_CHAN1TASK2_PRIORITY, NULL );
  xTaskCreate( (void *) &Task3_Task, (const signed char *) "TASK3", mainLA_CHAN2TASK3_STACK_SIZE, NULL,  mainLA_CHAN2TASK3_PRIORITY, NULL );
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
  s4353096_sysmon_init();
	portENABLE_INTERRUPTS();	//Enable interrupts

}
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
void Task1_Task(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0
  for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    vTaskDelay(3);               //Extra Task Delay of 3ms
    S4353096_LA_CHAN0_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

void Task2_Task(void) {
  S4353096_LA_CHAN1_CLR();        //Clear LA Channel 0
  for (;;) {
    S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    vTaskDelay(3);               //Extra Task Delay of 3ms
    S4353096_LA_CHAN1_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

void Task3_Task(void) {
  S4353096_LA_CHAN2_CLR();        //Clear LA Channel 0
  //TickType_t xLastWakeTime3;
  //const TickType_t xFrequency3 = 30 / portTICK_PERIOD_MS;;
  //xLastWakeTime3 = xTaskGetTickCount();
  for (;;) {
    S4353096_LA_CHAN2_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    //vTaskDelayUntil( &xLastWakeTime3, xFrequency3 );               //Extra Task Delay of 3ms
    S4353096_LA_CHAN2_CLR();
    vTaskDelay(1);                // Mandatory Delay
  }
}
/*void vApplicationTickHook( void ) {

	BRD_LEDOff();
}*/
