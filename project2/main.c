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
void GetRunTimeStats( void );
volatile unsigned long ulHighFrequencyTimerTicks;
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
void GetRunTimeStats( void ) {
TaskStatus_t *pxTaskStatusArray;
volatile UBaseType_t uxArraySize, x;
unsigned long ulTotalRunTime, ulStatsAsPercentage;

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
      if( ulTotalRunTime > 0 )
      {
         /* For each populated position in the pxTaskStatusArray array,
         format the raw data as human readable ASCII data. */
         for( x = 0; x < uxArraySize; x++ )
         {
            /* What percentage of the total run time has the task used?
            This will always be rounded down to the nearest integer.
            ulTotalRunTimeDiv100 has already been divided by 100. */
            ulStatsAsPercentage =
                  pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

            if( ulStatsAsPercentage > 0UL )
            {
               debug_printf("%s\t\t%lu\t\t%lu%%\r\n",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter,
                                 ulStatsAsPercentage );
            }
            else
            {
               /* If the percentage is zero here then the task has
               consumed less than 1% of the total run time. */
               debug_printf("%s\t\t%lu\t\t<1%%\r\n",
                                 pxTaskStatusArray[ x ].pcTaskName,
                                 pxTaskStatusArray[ x ].ulRunTimeCounter );
            }
         }
      }

      /* The array is no longer needed, free the memory it consumes. */
      vPortFree( pxTaskStatusArray );
   }
}

void tim_2_init(void) {
	unsigned short PrescalerValue;
	/* Timer 2 clock enable */
__TIM2_CLK_ENABLE();

/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/1000000) - 1;		//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

	/* Time base configuration */
TIM_Init.Instance = TIM2;				//Enable Timer 2
	TIM_Init.Init.Period = 1000000/1000;			//Set period count to be 1ms, so timer interrupt occurs every 1ms.
	TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	TIM_Init.Init.ClockDivision = 0;			//Set clock division
TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
	TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

/* Initialise Timer */
HAL_TIM_Base_Init(&TIM_Init);

/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

/* Enable timer update interrupt and interrupt vector for Timer  */
NVIC_SetVector(TIM2_IRQn, (uint32_t)&tim2_irqhandler);
NVIC_EnableIRQ(TIM2_IRQn);

/* Start Timer */
HAL_TIM_Base_Start_IT(&TIM_Init);
}

/**
* @brief  Timer 2 Interrupt handler
* @param  None.
* @retval None
*/
void tim2_irqhandler (void) {

//Clear Update Flag
__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
ulHighFrequencyTimerTicks++;		//increment counter, when the interrupt occurs


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
