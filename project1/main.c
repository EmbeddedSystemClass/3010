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
#define S4353096_JOYSTICK 1
#define S4353096_TERMINAL 0
TIM_HandleTypeDef TIM_Init;
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;
int count_interrupt = 199;
int y_value;
int x_value;
int duty_cycle;
char RxChar;
int mode = S4353096_TERMINAL;
//int last_button_state = 0;
//int button_state;
//int last_Debounce_Time = 0;
int set_angle_pan = 0;
int set_angle_tilt = 0;
int direction_multiplier = 1;
//int q = 0;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void tim2_irqhandler (void);
void Pb_init(void);
void exti_pb_irqhandler(void);

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
		if (((HAL_GetTick()/10000) % 5) == 0) { /*Delay for 1 second or setup to delay for 0.2 seconds and set angle to += or -= 1 each time*/
			if (mode == S4353096_JOYSTICK) {
				y_value = s4353096_joystick_y_read();
				if ((y_value > 2150) && (set_angle_pan < 80)) {
					set_angle_pan += 1;
				} else if ((y_value < 1900) && (set_angle_pan > -80)) {
					set_angle_pan -= 1;
				} else { //Joystick is stationary, no input

				}
				x_value = s4353096_joystick_x_read();
				if ((x_value > 2150) && (set_angle_tilt < 76)) {
					set_angle_tilt += 1;
				} else if ((x_value < 1900) && (set_angle_tilt > -76)) {
					set_angle_tilt -= 1;
				} else {

				}
			} else if (mode == S4353096_TERMINAL) {
				/* Receive characters using getc */
				RxChar = debug_getc();
				/* Check if character is not Null */
				if (RxChar != '\0') {
					switch (RxChar) {
						case 'w':
							set_angle_tilt += 1;
							break;
						case 's':
							set_angle_tilt -= 1;
							break;
						case 'a':
							set_angle_pan += 1;
							break;
						case 'd':
							set_angle_pan -= 1;
							break;
						default:
							break;
					}
						//debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
						debug_flush();
				}
				switch (set_angle_pan) {
					case 77:
						set_angle_pan = 76;
						break;
					case -77:
						set_angle_pan = -76;
						break;
					default:
						break;
				}
				switch (set_angle_tilt) {
					case 77:
						set_angle_tilt = 76;
						break;
					case -77:
						set_angle_tilt = -76;
						break;
					default:
						break;
				}
			}
			s4353096_pantilt_angle_write(1, set_angle_pan);
			s4353096_pantilt_angle_write(0, set_angle_tilt);
			debug_printf("Pan: %d Tlit: %d\n", set_angle_pan, set_angle_tilt);
				//BRD_LEDToggle();
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
	Pb_init();
}
void Pb_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
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
}
void exti_pb_irqhandler(void) {
	//debug_printf("Hey\n");
	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);
	mode = !mode;
	BRD_LEDToggle();
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
