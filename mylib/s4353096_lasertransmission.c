/**
  ******************************************************************************
  * @file    ex12_hamming.c
  * @author  MDS & KB
  * @date    04022015
  * @brief   Hamming encoder example.
  *			 Bytes received from the VCP are Hamming encoded and displayed.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_lasertransmission.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*t_ represents transmission*/
/*struct Transmission_Variables {
  int t_currentbit;
  int t_
};*/
/* Private function prototypes -----------------------------------------------*/
/*extern void s4353096_laser_transmitt_init(void) {
  __SQR_WAVE_GEN_1_GPIO_CLK();
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pin = SQR_WAVE_GEN_1_PIN;
	HAL_GPIO_Init(SQR_WAVE_GEN_1_GPIO_PORT, &GPIO_InitStructure);*/
  /* Timer 2 clock enable */
	//__TIM2_CLK_ENABLE();
	/* Compute the prescaler value for 50Khz */
  //PrescalerValue = (uint16_t) ((SystemCoreClock /2)/500000) - 1;
	/* Time base configuration */
	//TIM_Init.Instance = TIM2;				//Enable Timer 2
	//Set period count to be 1ms, so timer interrupt occurs every (1ms)*0.2.
  /*TIM_Init.Init.Period = (50000/1000)*0.5;//*0.18;
  TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.*/
	/* Initialise Timer */
	//HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	//HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	/*NVIC_SetVector(TIM2_IRQn, (uint32_t)&s4353096_laser_transmitt_irqhandler);
	NVIC_EnableIRQ(TIM2_IRQn);*/

	/* Start Timer */
	/*HAL_TIM_Base_Start_IT(&TIM_Init);
}
extern void s4353096_laser_start(void) {

}

extern void s4353096_laser_stop(void) {

}
*/
