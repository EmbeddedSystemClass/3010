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
TIM_HandleTypeDef TIM_Init;
void Hardware_init();
void tim_2_init(void);
void tim2_irqhandler (void);
extern void GetRunTimeStats( void );
//volatile unsigned long ulHighFrequencyTimerTicks;
int main (void) {
	BRD_init();
	Hardware_init();
	xTaskCreate( (void *) &s4353096_TaskAccelerometer, (const signed char *) "s4353096_TaskAccelerometer", mainTASKACC_STACK_SIZE, NULL,  mainTASKACC_PRIORITY, NULL );
	//xTaskCreate( (void *) &s4353096_TaskAccelerometer, (const signed char *) "s4353096_", mainTASKACC_STACK_SIZE, NULL,  mainTASKACC_PRIORITY, NULL );
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
extern void GetRunTimeStats( void ) {
TaskStatus_t *pxTaskStatusArray;
volatile UBaseType_t uxArraySize, x;
unsigned long ulTotalRunTime, ulStatsAsPercentage;
char* state;

   /* Make sure the write buffer does not contain a string. */

   /* Take a snapshot of the number of tasks in case it changes while this
   function is executing. */
   uxArraySize = uxTaskGetNumberOfTasks();

   /* Allocate a TaskStatus_t structure for each task.  An array could be
   allocated statically at compile time. */
   pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

   if( pxTaskStatusArray != NULL )
   {
      /* Generate raw status information about each task. */
      uxArraySize = uxTaskGetSystemState( pxTaskStatusArray,
                                 uxArraySize,
                                 &ulTotalRunTime );

      /* For percentage calculations. */
      ulTotalRunTime /= 100UL;

      /* Avoid divide by zero errors. */
      //if( ulTotalRunTime > 0 )
      //{
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
				 debug_printf("Task Name \t\tTask #\tPrioriy\t\tState    \tRunning Time\n");
				 debug_printf("--------------------------------------------------------------------------------------\n");
         for( x = 0; x < uxArraySize; x++ )
         {
					 	/*Want to print in the below fashion*/
						/* NAME | NUMBER | PRIORITY | STATE | RUNNING TIME |*/
						/* Before this we have to work out the States appropriate char value*/
						if (pxTaskStatusArray[x].eCurrentState == eReady) {
							state = "Ready";
						} else if (pxTaskStatusArray[x].eCurrentState == eBlocked) {
							state = "Blocked";
						} else if (pxTaskStatusArray[x].eCurrentState == eSuspended) {
							state = "Suspended";
						} else if (pxTaskStatusArray[x].eCurrentState == eRunning) {
							state = "Running";
						} else {
							debug_printf("No State Recieved");
						}

						//debug_printf("%s\t\t%d\t\t%d\t\t%c\t\t%d",pxTaskStatusArray[x].pcTaskName,
						//pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, /*State value here*/, /*RunTime Here*/);
						debug_printf("%-11s\t\t%-2.0d\t%-2.0d\t\t%-9s\t%-lu\n",pxTaskStatusArray[x].pcTaskName,
						pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, state, pxTaskStatusArray[x].ulRunTimeCounter);

						/* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage =
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            /*if( ulStatsAsPercentage > 0UL )
            {
               debug_printf("%s\t\t%lu\t\t%lu%%\r\n",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                                 ulStatsAsPercentage );
            }
            else
            {*/
               /* If the percentage is zero here then the task has
               consumed less than 1% of the total run time. */
            /*   debug_printf("%s\t\t%lu\t\t<1%%\r\n",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter );
            }*/
         }
				 debug_printf("\n");
      //}

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
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
