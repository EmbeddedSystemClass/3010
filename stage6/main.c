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

int main (void) {
	BRD_init();
	Hardware_init();
	xTaskCreate( (void *) &TaskTimerLeft, (const signed char *) "TaskTimerLeft", mainLA_CHAN0TASK1_STACK_SIZE, NULL,  mainLA_CHAN0TASK1_PRIORITY, NULL );
  xTaskCreate( (void *) &TaskTimerRight, (const signed char *) "TaskTimerRight", mainLA_CHAN1TASK2_STACK_SIZE, NULL,  mainLA_CHAN1TASK2_PRIORITY, NULL );
	PBLeftSemaphore = xSemaphoreCreateBinary();
	//PBRightSemaphore = xSemaphoreCreateBinary();
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
	s4353096_lightbar_init();
  s4353096_sysmon_init();
  portENABLE_INTERRUPTS();	//Disable interrupts
}
