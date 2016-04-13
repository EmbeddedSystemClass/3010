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
int y_value;
int x_value;
int duty_cycle;
char RxChar;
//int last_button_state = 0;
//int button_state;
//int last_Debounce_Time = 0;
int set_angle_pan = 0;
int set_angle_tilt = 0;
int write_angles = 0;
int direction_multiplier = 1;
//int q = 0;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void s4353096_pantilt_irqhandler(void);

void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	HAL_Delay(3000);
	/* Main processing loop */
  while (1) {
		//x_value = (s4353096_joystick_x_read() / 4095.00);
		//lightbar_percentage();
		//BRD_LEDToggle();	//Toggle 'Alive' LED on/off
		//HAL_Delay(1000);	//Delay for 1s
		//BRD_LEDToggle();
		//s4353096_pantilt_angle_write(1, 85);
		//s4353096_joystick_z_read();
		if (write_angles == 1) {
			s4353096_pantilt_angle_write(0, 60);
			s4353096_pantilt_angle_write(1, 60);
			write_angles = 0;
		}
		if (((HAL_GetTick()/1000) % 5) == 0) { /*Delay for 1 second or setup to delay for 0.2 seconds and set angle to += or -= 1 each time*/

			y_value = s4353096_joystick_y_read();
			if ((y_value > 2300) && (set_angle_pan < 76)) {
				set_angle_pan += 1;
			} else if ((y_value < 1750) && (set_angle_pan > -76)) {
				set_angle_pan -= 1;
			} else { //Joystick is stationary, no input

			}
			x_value = s4353096_joystick_x_read();
			if ((x_value > 2300) && (set_angle_tilt < 70)) {
				set_angle_tilt += 1;
			} else if ((x_value < 1750) && (set_angle_tilt > -70)) {
				set_angle_tilt -= 1;
			} else {

			}
			//s4353096_pantilt_angle_write(1, set_angle_pan);
			//s4353096_pantilt_angle_write(0, set_angle_tilt);
			//HAL_Delay(20);
			//debug_printf("Pan: %d Tlit: %d\n", set_angle_pan, set_angle_tilt);
		}
		//BRD_LEDToggle();
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
	__PANTILT_IR_TIMER_CLK();
	/* Compute the prescaler value for 50Khz */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;
	/* Time base configuration */
	TIM_Init.Instance = PANTILT_IR_TIM;				//Enable Timer 2
	//Set period count to be 1ms, so timer interrupt occurs every (1ms)*0.2.
  TIM_Init.Init.Period = (50000/1000)*20;//*0.18;
  TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.
	/* Initialise Timer */
	HAL_TIM_Base_Init(&TIM_Init);

	/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
	HAL_NVIC_SetPriority(PANTILT_TIM_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

	/* Enable timer update interrupt and interrupt vector for Timer  */
	NVIC_SetVector(PANTILT_TIM_IRQn, (uint32_t)&s4353096_pantilt_irqhandler);
	NVIC_EnableIRQ(PANTILT_TIM_IRQn);
	/*Start the timer*/
	HAL_TIM_Base_Start_IT(&TIM_Init);
}
void s4353096_pantilt_irqhandler(void) {
  TIM_Init.Instance = PANTILT_IR_TIM;
  __HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
  write_angles = 1;
}
/*void exti_a2_interrupt_handler(void) {
	HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);
}*/
	//int button_state = s4353096_joystick_z_read();
	//q = 0;
	//while (q = 0) {
	//BRD_LEDToggle();
		/*if (reading != last_button_state) {
			last_Debounce_Time = HAL_GetTick()/1000;
		}*/
	/*	if ((HAL_GetTick()/1000 - last_Debounce_Time) > 10) {
				if (button_state == 1) {
					set_angle = set_angle + direction_multiplier*10;
					debug_printf("%d\n", set_angle);
					q = 1;
				//HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);
				}
		}
		//last_button_state = reading;
	}
}*/
	/* Speed up the counter by reducing the delay value */
//}
