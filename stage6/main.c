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
#include <string.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "s4353096_sysmon.h"
#include "s4353096_pantilt.h"
#include "s4353096_cli.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();

int main (void) {
	BRD_init();
	Hardware_init();
	s4353096_SemaphoreLaser = xSemaphoreCreateBinary();
	s4353096_SemaphorePanLeft = xSemaphoreCreateBinary();
	s4353096_SemaphorePanRight = xSemaphoreCreateBinary();

	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xPan);
	//FreeRTOS_CLIRegisterCommand(&xTilt);
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
	s4353096_pantilt_init();
  s4353096_sysmon_init();
	__LASER_GPIO_CLK();
	//Set up Pin behaviour
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //Output Mode
	GPIO_InitStructure.Pull = GPIO_PULLUP; //Pull up resistor
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST; //Pun latency
	/*GPIO Pins D0-D9 are configured to the above specifications in the space
	bellow*/
	GPIO_InitStructure.Pin = LASER_PIN;
	HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStructure);
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
