/**
  ******************************************************************************
  * @file    stage1/main.c
  * @author  Steffen Mitchell
  * @date    10-January-2015
  * @brief   Prac 1 Template C main file - BCD timer and press counter.
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE.
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_lightbar.h"
#include "s4353096_joystick.h"		////////CHANGE THIS//////////

/* Private typedef -----------------------------------------------------------*/
#define SQR_WAVE_GEN_1_PIN BRD_D1_PIN
#define SQR_WAVE_GEN_1_GPIO_PORT BRD_D1_GPIO_PORT
#define __SQR_WAVE_GEN_1_GPIO_CLK() __BRD_D1_GPIO_CLK()
#define SQR_WAVE_GEN_1_EXTI_IRQ BRD_D1_EXTI_IRQ
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;
int count_interrupt = 99;
float x_value;
int duty_cycle;
int led_value;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void tim2_irqhandler (void);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */
void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	HAL_Delay(7000);
	/* Main processing loop */
  while (1) {
		//s4353096_joystick_x_read();
		//s4353096_joystick_y_read();
		//s4353096_joystick_z_read();
		x_value = (s4353096_joystick_x_read() / 4095.00);
		lightbar_percentage();
	}
}

/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	unsigned short PrescalerValue;

	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
	/* Initialise LEDBar
       Call
	   sxxxxxx_ledbar_init();

	*/
	s4353096_lightbar_init();
	s4353096_joystick_init();
	/* Configure D1 for output of square wave signal */
	__SQR_WAVE_GEN_1_GPIO_CLK();
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pin = SQR_WAVE_GEN_1_PIN;
	HAL_GPIO_Init(SQR_WAVE_GEN_1_GPIO_PORT, &GPIO_InitStructure);
	/* Timer 2 clock enable */
	__TIM2_CLK_ENABLE();
	/* Compute the prescaler value for 50Khz */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;
	/* Time base configuration */
	TIM_Init.Instance = TIM2;				//Enable Timer 2
	//Set period count to be 1ms, so timer interrupt occurs every (1ms)*0.2.
  TIM_Init.Init.Period = (50000/1000)*0.18;
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
void lightbar_percentage(void) {
	for(int i = 0; i <= 10; i++) {
		if (i == (duty_cycle/10)) {
			led_value = ~(((1023 << i) & 1023));
			s4353096_lightbar_write(led_value);
		}
	}
	/*switch (duty_cycle/10) {
		case 0:
			s4353096_lightbar_write(1023 >> 10);
			break;
		case 1:
			s4353096_lightbar_write(1023 >> 9);
			break;
			case 2:
				s4353096_lightbar_write(1023 >> 8);
				break;
	}*/
}
void tim2_irqhandler (void) {

	//Clear Update Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);

	//Check whether to read joystick values (Every 20ms)
	if (count_interrupt == 100) {
		duty_cycle = x_value * 100;
		count_interrupt = 0;
	}

	if (count_interrupt < (duty_cycle - 1)) {
		HAL_GPIO_WritePin(SQR_WAVE_GEN_1_GPIO_PORT, SQR_WAVE_GEN_1_PIN, 1);
	} else {
		HAL_GPIO_WritePin(SQR_WAVE_GEN_1_GPIO_PORT, SQR_WAVE_GEN_1_PIN, 0);
	}
	count_interrupt++;
	/*
	//Geneate Signal on D1 (Every 1ms)
	if (count_interrupt <= 10) {
		HAL_GPIO_WritePin(SQR_WAVE_GEN_1_GPIO_PORT, SQR_WAVE_GEN_1_PIN, 1);
		count_interrupt++;
	} else if (count_interrupt == 20) {
		HAL_GPIO_WritePin(SQR_WAVE_GEN_1_GPIO_PORT, SQR_WAVE_GEN_1_PIN, 0);
		count_interrupt = 0;
	} else {
		HAL_GPIO_WritePin(SQR_WAVE_GEN_1_GPIO_PORT, SQR_WAVE_GEN_1_PIN, 0);
		count_interrupt++;
	}*/
}
/**
  * @brief  exti_a2 GPIO Interrupt handler
  * @param  None.
  * @retval None
  */
//void exti_joystick_z_interrupt_handler(void) {
//	HAL_GPIO_EXTI_IRQHandler(JOYSTICK_Z_PIN);				//Clear A2 pin external interrupt flag

	/* Speed up the counter by reducing the delay value */
//	press_counter_val++;
//	if (press_counter_val == 1) {
//		counter_divider = counter_divider * 2;
		//debug_printf("Triggered - %d\n\r", press_counter_val);
//	} else {
//		press_counter_val = 0;
//	}
//	HAL_Delay(100);
//}
