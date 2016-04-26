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
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t mode = 1;
//struct Timer *tim_l;
//struct Timer *tim_r;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void TaskTimerLeft(void);
void TaskTimerRight(void);
void exti_pb_irqhandler(void);
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
	PBLeftSemaphore = xSemaphoreCreateBinary();
	PBRightSemaphore = xSemaphoreCreateBinary();
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
	GPIO_InitTypeDef GPIO_InitStructure;
	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
	s4353096_lightbar_init();
  s4353096_sysmon_init();
	/* Enable PB clock */
  	__BRD_PB_GPIO_CLK();

	/* Set priority of PB Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(BRD_PB_EXTI_IRQ, 4, 0);	//Set Main priority ot 10 and sub-priority ot 0.

	//Enable PB interrupt and interrupt vector for pin DO
	NVIC_SetVector(BRD_PB_EXTI_IRQ, (uint32_t)&exti_pb_irqhandler);
	NVIC_EnableIRQ(BRD_PB_EXTI_IRQ);

  	/* Configure PB pin as pull down input */
	GPIO_InitStructure.Pin = BRD_PB_PIN;				//Pin
  	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;		//interrupt Mode
  	GPIO_InitStructure.Pull = GPIO_PULLUP;			//Enable Pull up, down or no pull resister
  	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  	HAL_GPIO_Init(BRD_PB_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
	portENABLE_INTERRUPTS();	//Enable interrupts

}
void TaskTimerLeft(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime1;
  const TickType_t xFrequency1 = 1000 / portTICK_PERIOD_MS;;
  xLastWakeTime1 = xTaskGetTickCount();
	struct dualtimer_msg TimerLeft;
	TimerLeft.timer_value = 0; /*Might possibly need to adjust this depending on if Task fires at 0 or 1 first*/
	for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
		if (PBLeftSemaphore != NULL) {	/* Check if semaphore exists */

			/* See if we can obtain the PB semaphore. If the semaphore is not available
           	wait 10 ticks to see if it becomes free. */
			if( xSemaphoreTake( PBLeftSemaphore, 10 ) == pdTRUE ) {
            	/* We were able to obtain the semaphore and can now access the shared resource. */

            	/* Invert mode to stop or start LED flashing */
				mode = ~mode & 0x01;
			}
		}
		if (mode == 1) {
			TimerLeft.timer_value++;
			TimerLeft.type = 'l';
			if (s4353096_QueueLightBar != NULL) {	/* Check if queue exists */
				if( xQueueSendToBack(s4353096_QueueLightBar, ( void * ) &TimerLeft, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
		} else {

		}
		//TimerLeft.timer_value++;
		/*Send Count via Queue here*/
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
	struct dualtimer_msg TimerRight;
	TimerRight.timer_value = -1;
	for (;;) {
    S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
		if (PBLeftSemaphore != NULL) {	/* Check if semaphore exists */

			/* See if we can obtain the PB semaphore. If the semaphore is not available
           	wait 10 ticks to see if it becomes free. */
			if( xSemaphoreTake( PBLeftSemaphore, 10 ) == pdTRUE ) {
            	/* We were able to obtain the semaphore and can now access the shared resource. */

            	/* Invert mode to stop or start LED flashing */
				mode = ~mode & 0x01;
			}
		}
		if (mode == 1) {
			TimerRight.timer_value++;
			TimerRight.type = 'r';
			/*Adjust count for sending via queue*/
			//TimerRight.timer_value = ((TimerRight.timer_value & 0x1F) << 5);
			/*Send Count via Queue here*/
			if (s4353096_QueueLightBar != NULL) {	/* Check if queue exists */
				if( xQueueSendToBack(s4353096_QueueLightBar, ( void * ) &TimerRight, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
			//TimerRight.timer_value = ((TimerRight.timer_value & 0x3E0) >> 5);
		} else {

		}

		//TimerRight.timer_value++;
		/*if (timeright.count == 10) {
			timeright.count = 0;
		} else {

		}*/
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
void exti_pb_irqhandler(void) {

	BaseType_t xHigherPriorityTaskWoken;

    /* Is it time for another Task() to run? */
    xHigherPriorityTaskWoken = pdFALSE;

	/* Check if Pushbutton external interrupt has occured */
  	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);				//Clear D0 pin external interrupt flag

	if (PBLeftSemaphore != NULL) {	/* Check if semaphore exists */
		xSemaphoreGiveFromISR( PBLeftSemaphore, &xHigherPriorityTaskWoken );		/* Give PB Semaphore from ISR*/
		debug_printf("Triggered \n\r");    //Print press count value
	}

	/* Perform context switching, if required. */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
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
/*void vApplicationTickHook( void ) {

	BRD_LEDOff();
}*/
