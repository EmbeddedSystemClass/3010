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
GPIO_InitTypeDef  GPIO_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

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
  /*xTaskCreate( (void *) &Task1_Task, (const signed char *) "TASK1", mainLA_CHAN0TASK1_STACK_SIZE, NULL,  mainLA_CHAN0TASK1_PRIORITY, NULL );
  xTaskCreate( (void *) &Task2_Task, (const signed char *) "TASK2", mainLA_CHAN1TASK2_STACK_SIZE, NULL,  mainLA_CHAN1TASK2_PRIORITY, NULL );
  xTaskCreate( (void *) &Task3_Task, (const signed char *) "TASK3", mainLA_CHAN2TASK3_STACK_SIZE, NULL,  mainLA_CHAN2TASK3_PRIORITY, NULL );
*/
}
void Task1_Task(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime1;
  const TickType_t xFrequency1 = 30 / portTICK_PERIOD_MS;;
  xLastWakeTime1 = xTaskGetTickCount();
  for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    //vTaskDelay(xDelay1);
     vTaskDelayUntil( &xLastWakeTime1, xFrequency1 );                //Extra Task Delay of 3ms
    S4353096_LA_CHAN0_CLR();
    vTaskDelay(10);                // Mandatory Delay


  }
}

void Task2_Task(void) {
  S4353096_LA_CHAN1_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime2;
  const TickType_t xFrequency2 = 30 / portTICK_PERIOD_MS;;
  xLastWakeTime2 = xTaskGetTickCount();
  for (;;) {
    S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    //vTaskDelay(xDelay2 );
    vTaskDelayUntil( &xLastWakeTime2, xFrequency2 );                //Extra Task Delay of 3ms
    S4353096_LA_CHAN1_CLR();
    vTaskDelay(10);                // Mandatory Delay


  }
}

void Task3_Task(void) {
  S4353096_LA_CHAN2_CLR();        //Clear LA Channel 0
  TickType_t xLastWakeTime3;
  const TickType_t xFrequency3 = 30 / portTICK_PERIOD_MS;;
  xLastWakeTime3 = xTaskGetTickCount();
  for (;;) {
    S4353096_LA_CHAN2_SET();      //Set LA Channel 0
    /*Do Stuff Here*/
    //vTaskDelayUntil( &xLastWakeTime3, xFrequency3 );               //Extra Task Delay of 3ms
    S4353096_LA_CHAN2_CLR();
    vTaskDelay(10);                // Mandatory Delay
  }
}
