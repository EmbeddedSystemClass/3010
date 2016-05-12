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
#include "s4353096_sysmon.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>


/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*Initialise the system monitor pins*/
void s4353096_sysmon_init(void) {
  /*Enable Clocks*/
  __LA_CHAN0_GPIO_CLK();
  __LA_CHAN1_GPIO_CLK();
  __LA_CHAN2_GPIO_CLK();

  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //Output Mode
  GPIO_InitStructure.Pull = GPIO_PULLUP; //Pull up resistor
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST; //Pun latency

  GPIO_InitStructure.Pin = LA_CHAN0_PIN;
  HAL_GPIO_Init(LA_CHAN0_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = LA_CHAN1_PIN;
  HAL_GPIO_Init(LA_CHAN1_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = LA_CHAN2_PIN;
  HAL_GPIO_Init(LA_CHAN2_GPIO_PORT, &GPIO_InitStructure);
}

extern void GetTopList( void ) {
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

      /* Avoid divide by zero errors. */
      //if( ulTotalRunTime > 0 )
      //{
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
				 debug_printf("Task Name \t\t\t\tTask #\tPrioriy\t\tState    \tRunning Time\n");
				 debug_printf("--------------------------------------------------------------------------------------\n");
         for( x = 0; x < uxArraySize; x++ )
         {
					 	/*Want to print in the below fashion*/
						/* NAME | NUMBER | PRIORITY | STATE | RUNNING TIME |*/
						/* Before this we have to work out the States appropriate char value*/
            //TaskValues.TaskHandles[x] = pxTaskStatusArray[x].xHandle;
						if (pxTaskStatusArray[x].eCurrentState == eReady) {
							state = "Ready";
              debug_printf( ANSI_COLOR_YELLOW "%-25s\t\t%-2.0d\t%-2.0d\t\t%-9s\t%-lu\n" ANSI_COLOR_RESET , pxTaskStatusArray[x].pcTaskName,
  						pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, state, pxTaskStatusArray[x].ulRunTimeCounter);
						} else if (pxTaskStatusArray[x].eCurrentState == eBlocked) {
							state = "Blocked";
              debug_printf( ANSI_COLOR_RED "%-25s\t\t%-2.0d\t%-2.0d\t\t%-9s\t%-lu\n" ANSI_COLOR_RESET , pxTaskStatusArray[x].pcTaskName,
  						pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, state, pxTaskStatusArray[x].ulRunTimeCounter);

						} else if (pxTaskStatusArray[x].eCurrentState == eSuspended) {
							state = "Suspended";
              debug_printf( ANSI_COLOR_BLUE "%-25s\t\t%-2.0d\t%-2.0d\t\t%-9s\t%-lu\n" ANSI_COLOR_RESET , pxTaskStatusArray[x].pcTaskName,
  						pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, state, pxTaskStatusArray[x].ulRunTimeCounter);

						} else if (pxTaskStatusArray[x].eCurrentState == eRunning) {
							state = "Running";
              debug_printf( ANSI_COLOR_GREEN "%-25s\t\t%-2.0d\t%-2.0d\t\t%-9s\t%-lu\n" ANSI_COLOR_RESET ,pxTaskStatusArray[x].pcTaskName,
  						pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, state, pxTaskStatusArray[x].ulRunTimeCounter);

						} else {
							debug_printf("No State Recieved\n");
						}

						//debug_printf("%s\t\t%d\t\t%d\t\t%c\t\t%d",pxTaskStatusArray[x].pcTaskName,
						//pxTaskStatusArray[x].xTaskNumber, pxTaskStatusArray[x].uxCurrentPriority, /*State value here*/, /*RunTime Here*/);


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
				 debug_printf("\n\n");
      //}

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
}
