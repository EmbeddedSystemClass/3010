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
