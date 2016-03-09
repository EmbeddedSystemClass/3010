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
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;
int counter_divider = 1;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void exti_joystick_z_interrupt_handler(void);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */
void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	int unsigned adc_value_x;
	HAL_Delay(7000);
	/*debug_printf("Timer Value: %d\n", counter_value);
	s4353096_lightbar_write(counter_value);
	HAL_Delay(1000/counter_divider);*/
	/* Main processing loop */
  	while (1) {
		//debug_printf("LED Toggle %d\n\r", i);	//Print debug message
		/*if (counter_value == 0) {
			counter_value = 64;
		} else {
			counter_value--;	//Increment counter
		}
		s4353096_lightbar_write(0);
		s4353096_lightbar_write(counter_value);
		debug_printf("Timer Value: %d\n", counter_value);*/
		/****************** Display counter. ***************/
		/* First, turn off each LED light bar segment
			write 0 to D0
			Write 0 to D1
			....
			Write 0 to D9

			Call sxxxxxx_ledbar_set(0)

			then call
	*/
			//s4353096_lightbar_write(counter_value);
		//*/

		/* Toggle 'Keep Alive Indicator' BLUE LED */
		s4353096_joystick_x_read();
		s4353096_joystick_y_read();
		s4353096_joystick_z_read();
		BRD_LEDToggle();
    HAL_Delay(1000/counter_divider);		//Delay for 1s (1000ms)

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
	/* Timer 2 clock enable */
	__TIM2_CLK_ENABLE();
	/* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1;
	//Set clock prescaler to 50kHz - SystemCoreClock is the system clock frequency.

	/* Configure the GPIO_D1 pin

	 	....

		Configure the GPIO_D9 pin */

	/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */
	//Initialise Interrupt, Priority set to 10



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
