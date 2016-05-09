/**
  ******************************************************************************
  * @file    stage6/main.c
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
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "s4353096_accelerometer.h"
#include "s4353096_sysmon.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();

int main (void) {
	BRD_init();
	Hardware_init();
	xTaskCreate( (void *) &s4353096_TaskAccelerometer, (const signed char *) "s4353096_TaskAccelerometer", mainTASKACC_STACK_SIZE, NULL,  mainTASKACC_PRIORITY, NULL );
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

/*Initialise Hardware (i.e Lightbar, Pushbutton, sysmon)*/
void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts
	GPIO_InitTypeDef GPIO_InitStructure;
	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
	s4353096_sysmon_init();
	s4353096_accelerometer_init();
	BRD_LEDToggle();
  portENABLE_INTERRUPTS();	//Disable interrupts
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
