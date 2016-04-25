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


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void s4353096_sysmon_init(void) {
  xTaskCreate( (void *) &Task1_Task, (const signed char *) "TASK1", mainLA_CHAN0TASK1_STACK_SIZE, NULL,  mainLA_CHAN0TASK1_PRIORITY, NULL );
  xTaskCreate( (void *) &Task2_Task, (const signed char *) "TASK2", mainLA_CHAN1TASK2_STACK_SIZE, NULL,  mainLA_CHAN1TASK2_PRIORITY, NULL );
  xTaskCreate( (void *) &Task3_Task, (const signed char *) "TASK3", mainLA_CHAN2TASK3_STACK_SIZE, NULL,  mainLA_CHAN2TASK3_PRIORITY, NULL );
}
void Task1_Task(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0

  for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    vTaskDelay(3);                //Extra Task Delay of 3ms
    S4353096_LA_CHAN0_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

void Task2_Task(void) {
  S4353096_LA_CHAN1_CLR();        //Clear LA Channel 0

  for (;;) {
    S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    vTaskDelay(3);                //Extra Task Delay of 3ms
    S4353096_LA_CHAN1_CLR();
    vTaskDelay(1);                // Mandatory Delay


  }
}

void Task3_Task(void) {
  S4353096_LA_CHAN2_CLR();        //Clear LA Channel 0

  for (;;) {
    S4353096_LA_CHAN2_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    //vTaskDelay(3);                //Extra Task Delay of 3ms
    S4353096_LA_CHAN2_CLR();

  }
}
