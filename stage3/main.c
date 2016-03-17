/**
  ******************************************************************************
  * @file    stage3/main.c
  * @author  Steffen Mitchell
  * @date    10-January-2015
  * @brief   Prac 1 Template C main file - BCD timer and press counter.
  *
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_pantilt.h"
#include "s4353096_joystick.h"		////////CHANGE THIS//////////

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;
int count_interrupt = 199;
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
		//x_value = (s4353096_joystick_x_read() / 4095.00);
		//lightbar_percentage();
		//BRD_LEDToggle();	//Toggle 'Alive' LED on/off
		//HAL_Delay(1000);	//Delay for 1s
		//BRD_LEDToggle();
		//s4353096_pantilt_angle_write(1, 90);
		HAL_Delay(500);
		BRD_LEDToggle();
		s4353096_pantilt_angle_write(1, 85);
		HAL_Delay(500);
		BRD_LEDToggle();
		s4353096_pantilt_angle_write(1, (-85));
		HAL_Delay(500);
		BRD_LEDToggle();
		s4353096_pantilt_angle_write(1, 60);
		HAL_Delay(500);
		BRD_LEDToggle();
		BRD_LEDToggle();
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
	/* Initialise Joystick */
	s4353096_joystick_init();
	/* Configure D1 for output of square wave signal */
	s4353096_pantilt_init();
}
