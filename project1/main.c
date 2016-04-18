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
#include "s4353096_joystick.h"
#include "s4353096_hamming.h"
//#include "s4353096_radio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define S4353096_JOYSTICK 1
#define S4353096_TERMINAL 0
#define S4353096_LASER_TRANSMIT 2
#define LASER_WAVE_GEN_1_PIN BRD_D1_PIN
#define LASER_WAVE_GEN_1_GPIO_PORT BRD_D1_GPIO_PORT
#define __LASER_WAVE_GEN_1_GPIO_CLK() __BRD_D1_GPIO_CLK()
#define LASER_WAVE_GEN_1_EXTI_IRQ BRD_D1_EXTI_IRQ

TIM_HandleTypeDef TIM_Init;
TIM_IC_InitTypeDef  TIM_ICInitStructure;
uint16_t counter_value = 64;
uint16_t press_counter_val = 0;
static uint16_t PrescalerValue = 0;
int count_interrupt = 199;
int duty_cycle;
char RxChar;
int mode = S4353096_LASER_TRANSMIT;
struct Variables {
	int count;
	int bit_half;
	int bit_count;
	int encoded_bit;
	int encoded_bit_count;
	uint16_t encoded_char;
	int transmit_frequency;
	float period_multiplyer;
	unsigned int recieve[44];
	int recieve_decoded[24];
	int recieve_element;
	int recieve_period;
	int current_bit;
};
struct Variables *vars;
unsigned int x_value;
unsigned int y_value;
int direction_multiplier = 1;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void s4353096_pantilt_irqhandler(void);
void Pb_init(void);
void exti_pb_irqhandler(void);
void s4353096_general_irqhandler(void);
void s4353096_laser_transmit_bit(int bit);
void tim3_irqhandler (void);
int approximately_equal(unsigned int number, unsigned int check_number);
void manchester_decode(void);

void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();
	HAL_Delay(3000);
		//Initalise hardware modules

	/* Main processing loop */
	struct PanTilt pantiltvars;
	pantilt = &pantiltvars;
	pantilt->write_angles = 0;
	pantilt->read_angles = 0;
	pantilt->set_angle_pan = 0;
	pantilt->set_angle_tilt = 0;
	struct Variables variables;
	vars = &variables;
	vars->count = 0;
	vars->bit_half = 1;
	vars->bit_count = 0;
	vars->encoded_bit_count = -1;
	vars->encoded_bit = 0;
	vars->encoded_char = hamming_byte_encoder('<');
	vars->transmit_frequency = 1000;
	vars->period_multiplyer = ((1.000000/vars->transmit_frequency)*500)*0.998;
	vars->recieve_element = 0;
	debug_printf("\n%f\n", variables.period_multiplyer);
	Pb_init();
	//HAL_Delay(1000);
	for(int i=0; i<16; i++) {
		debug_printf("%d", !!((vars->encoded_char >> i) & 0x1));
	}
  while (1) {
		if (pantilt->write_angles == 1) {
			s4353096_pantilt_angle_write(1, pantilt->set_angle_pan);
			s4353096_pantilt_angle_write(0, pantilt->set_angle_tilt);
			pantilt->write_angles = 0;
		}
		if (((HAL_GetTick()/10000) % 100) == 0) {
			debug_printf("\nPan: %d Tlit: %d\n", pantilt->set_angle_pan, pantilt->set_angle_tilt);
			for (int i = 0; i < vars->recieve_element; i++) {
				debug_printf("Recieve edge %d : %d\n", i, vars->recieve[i]);
			}
			manchester_decode();
			//BRD_LEDToggle();
		}//
		if (pantilt->read_angles == 1) { /*Delay for 1 second or setup to delay for 0.2 seconds and set angle to += or -= 1 each time*/
			if (mode == S4353096_JOYSTICK) {
				y_value = s4353096_joystick_y_read();
				if ((y_value > 2500) && (pantilt->set_angle_pan < 76)) {
					pantilt->set_angle_pan += 1;
				} else if ((y_value < 1550) && (pantilt->set_angle_pan > -76)) {
					pantilt->set_angle_pan -= 1;
				} else { //Joystick is stationary, no input
				}
				x_value = s4353096_joystick_x_read();
				if ((x_value > 2500) && (pantilt->set_angle_tilt < 76)) {
					pantilt->set_angle_tilt += 1;
				} else if ((x_value < 1550) && (pantilt->set_angle_tilt > -76)) {
					pantilt->set_angle_tilt -= 1;
				} else {
				}
			} else if (mode == S4353096_TERMINAL) {
				/* Receive characters using getc */
				RxChar = debug_getc();
				/* Check if character is not Null */
				if (RxChar != '\0') {
					switch (RxChar) {
						case 'w':
							pantilt->set_angle_tilt += 1;
							break;
						case 's':
							pantilt->set_angle_tilt -= 1;
							break;
						case 'a':
							pantilt->set_angle_pan += 1;
							break;
						case 'd':
							pantilt->set_angle_pan -= 1;
							break;
						case 't':
							mode = S4353096_LASER_TRANSMIT;
							break;
						default:
							break;
					}
					debug_flush();
				}
				s4353096_terminal_angle_check();

			}
			pantilt->read_angles = 0;
		}
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
	//
	//Pb_init();
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

		/*Set up Interrupt timer for laser transmit and pan tilt*/
	  __PANTILT_IR_TIMER_CLK();
		/* Compute the prescaler value for 50Khz */
	  PrescalerValue = (uint16_t) ((SystemCoreClock /2)/1000000) - 1;
		/* Time base configuration */
		TIM_Init.Instance = PANTILT_IR_TIM;				//Enable Timer 2
		//Set period count to be 1ms, so timer interrupt occurs every (1ms)*0.2.
	  TIM_Init.Init.Period = (1000000/1000)*(vars->period_multiplyer); //10 = 1ms; 5 = 0.5ms 1khz half bit period for 100khz
	  TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
	  TIM_Init.Init.ClockDivision = 0;			//Set clock division
		TIM_Init.Init.RepetitionCounter = 0;	// Set Reload Value
	  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

		__LASER_WAVE_GEN_1_GPIO_CLK();
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Pin = LASER_WAVE_GEN_1_PIN;
		HAL_GPIO_Init(LASER_WAVE_GEN_1_GPIO_PORT, &GPIO_InitStructure);
		//HAL_Delay(1000);
		/* Initialise Timer */
		HAL_TIM_Base_Init(&TIM_Init);

		/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
		/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
		HAL_NVIC_SetPriority(PANTILT_TIM_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.

		/* Enable timer update interrupt and interrupt vector for Timer  */
		NVIC_SetVector(PANTILT_TIM_IRQn, (uint32_t)&s4353096_general_irqhandler);
		NVIC_EnableIRQ(PANTILT_TIM_IRQn);
		/*Start the timer*/
		HAL_TIM_Base_Start_IT(&TIM_Init);
		/*Set up Laser transmit pins*/
		/*__LASER_WAVE_GEN_1_GPIO_CLK();
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Pin = LASER_WAVE_GEN_1_PIN;
		HAL_GPIO_Init(LASER_WAVE_GEN_1_GPIO_PORT, &GPIO_InitStructure);*/

		/*Input capture*/
		__TIM3_CLK_ENABLE();

		/* Enable the D0 Clock */
		__BRD_D0_GPIO_CLK();

		/* Configure the D0 pin with TIM3 input capture */
	GPIO_InitStructure.Pin = BRD_D0_PIN;				//Pin
		GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
		GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM3;	//Set alternate function to be timer 2
		HAL_GPIO_Init(BRD_D0_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
		PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	/* Configure Timer 3 settings */
	TIM_Init.Instance = TIM3;					//Enable Timer 3
		TIM_Init.Init.Period = 2*500000;			//Set for 100ms (10Hz) period
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

	/* Configure TIM3 Input capture */
		TIM_ICInitStructure.ICPolarity = TIM_ICPOLARITY_RISING;			//Set to trigger on rising edge
		TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
		TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.ICFilter = 0;

	/* Set priority of Timer 3 Interrupt [0 (HIGH priority) to 15(LOW priority)] */
	HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0.

	//Enable Timer 3 interrupt and interrupt vector
	NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim3_irqhandler);
	NVIC_EnableIRQ(TIM3_IRQn);

	/* Enable input capture for Timer 3, channel 2 */
	HAL_TIM_IC_Init(&TIM_Init);
	HAL_TIM_IC_ConfigChannel(&TIM_Init, &TIM_ICInitStructure, TIM_CHANNEL_2);

	/* Start Input Capture */
	HAL_TIM_IC_Start_IT(&TIM_Init, TIM_CHANNEL_2);
	//HAL_GPIO_WritePin(LASER_WAVE_GEN_1_GPIO_PORT, LASER_WAVE_GEN_1_PIN, 0);
}
void exti_pb_irqhandler(void) {
	//debug_printf("Hey\n");
	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);
	mode = !mode;
}

void s4353096_general_irqhandler(void) {
  TIM_Init.Instance = PANTILT_IR_TIM;
  __HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
	vars->count++;
	if (mode == S4353096_LASER_TRANSMIT) {
		if (vars->bit_count == 0 || vars->bit_count == 1 || vars->bit_count == 11 ||
			vars->bit_count == 12) {
			s4353096_laser_transmit_bit(1);
		} else if (vars->bit_count == 10 || vars->bit_count == 21) {
			s4353096_laser_transmit_bit(0);
			if ((vars->bit_count == 22) && (vars->bit_half == 1)) {
				//Stop transmitting and reset values
				vars->bit_count = 0;
				vars->encoded_bit_count = -1;
				mode = S4353096_TERMINAL;
			} else {
				//s4353096_laser_transmit_bit(0);
			}
			//HAL_Delay(10);
		} else {
			if (vars->bit_half == 1) {
				vars->encoded_bit_count++;
				vars->encoded_bit = !!((vars->encoded_char >> vars->encoded_bit_count) & 0x1);
			}
			s4353096_laser_transmit_bit(vars->encoded_bit);
		}
	}
	if (vars->count == (vars->transmit_frequency)*0.2 && (mode == S4353096_TERMINAL || mode == S4353096_JOYSTICK)) {
		pantilt->read_angles = 1;
  	pantilt->write_angles = 1;
		vars->count = 0;
	}
}
void s4353096_laser_transmit_bit(int bit) {
  if (vars->bit_half == 1) {
		HAL_GPIO_WritePin(LASER_WAVE_GEN_1_GPIO_PORT, LASER_WAVE_GEN_1_PIN, !(bit));
		vars->bit_half++;
	} else if (vars->bit_half == 2) {
		HAL_GPIO_WritePin(LASER_WAVE_GEN_1_GPIO_PORT, LASER_WAVE_GEN_1_PIN, bit);
		vars->bit_count++;
		vars->bit_half--;
	}
}
void tim3_irqhandler (void) {
	unsigned int input_capture_value;
	TIM_Init.Instance = TIM3;					//Enable Timer 3
		TIM_Init.Init.Period = 2*500000;			//Set for 100ms (10Hz) period
		TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
		TIM_Init.Init.ClockDivision = 0;			//Set clock division
	TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
		TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.
	//Clear Input Capture Flag
	__HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_TRIGGER);

	/* Toggle LED */
	//BRD_LEDToggle();

  	/* Read and display the Input Capture value of Timer 3, channel 2 */
  	input_capture_value = HAL_TIM_ReadCapturedValue(&TIM_Init, TIM_CHANNEL_2);
		vars->recieve[vars->recieve_element] = input_capture_value;
		vars->recieve_element++;
		__TIM3_CLK_DISABLE();
		HAL_TIM_IC_DeInit(&TIM_Init);
		__TIM3_CLK_ENABLE();
		/* Set priority of Timer 3 Interrupt [0 (HIGH priority) to 15(LOW priority)] */
		HAL_NVIC_SetPriority(TIM3_IRQn, 10, 0);	//Set Main priority ot 10 and sub-priority ot 0.
		//Enable Timer 3 interrupt and interrupt vector
		NVIC_SetVector(TIM3_IRQn, (uint32_t)&tim3_irqhandler);
		NVIC_EnableIRQ(TIM3_IRQn);
		/* Enable input capture for Timer 3, channel 2 */
		HAL_TIM_IC_Init(&TIM_Init);
		HAL_TIM_IC_ConfigChannel(&TIM_Init, &TIM_ICInitStructure, TIM_CHANNEL_2);
		/* Start Input Capture */
		HAL_TIM_IC_Start_IT(&TIM_Init, TIM_CHANNEL_2);
  	//debug_printf("Input capture 2: %d RE = %d\n\r", input_capture_value);

}
int approximately_equal(unsigned int number, unsigned int check_number) {
	if ((number < ((check_number)*1.2)) && (number > ((check_number)*0.8) )) {
		return 1;
	} else {
		return 0;
	}
}
void manchester_decode(void) {
	int j = 0;
	vars->current_bit = 1;
	vars->recieve_period = (vars->recieve[0])*4;
	/*If the first bit doesn't have an input capture of approximately_equal to any other input capture then first bit was not one and an error occured*/
	for (int i = 0; i < 22; i++) {
		if (i == 0) {
			vars->recieve_decoded[0] = 1;
			debug_printf("A %d %d\n", vars->recieve_decoded[0],i);
		} else if ((vars->current_bit == 1) && (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.75) == 1)) {
			/*The bit is !current bit, set current bit to !current bit*/
			vars->recieve_decoded[i] = !vars->current_bit;
			debug_printf("B %d %d\n", vars->recieve_decoded[i],i);
			vars->current_bit = !vars->current_bit;
		} else if ((vars->current_bit == 0) && (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.75) == 1)) {
			/*The bit is current bit + !current_bit, set current bit to !current bit*/
			vars->recieve_decoded[i] = vars->current_bit;
			debug_printf("C %d %d\n", vars->recieve_decoded[i],i);
			i++;
			vars->recieve_decoded[i] = !vars->current_bit;
			debug_printf("D %d %d\n", vars->recieve_decoded[i],i);
			vars->current_bit = !vars->current_bit;

		} else if (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.5) == 1) {
			/*The bit is current bit*/
			vars->recieve_decoded[i] = vars->current_bit;
			debug_printf("E %d %d\n", vars->recieve_decoded[i],i);
		} else if (approximately_equal(vars->recieve[j], (vars->recieve_period)) == 1) {
			/*The bit is !current bit + current bit*/
			vars->recieve_decoded[i] = !vars->current_bit;
			debug_printf("F %d %d\n", vars->recieve_decoded[i],i);
			i++;
			vars->recieve_decoded[i] = vars->current_bit;
			debug_printf("G %d %d\n", vars->recieve_decoded[i],i);
		}
		j++;
	}
	debug_printf("\n");
}
/*void s4353096_pantilt_irqhandler(void) {
  TIM_Init.Instance = PANTILT_IR_TIM;
  __HAL_TIM_CLEAR_IT(&TIM_Init, TIM_IT_UPDATE);
	//interrupt_time = HAL_GetTick();
	pantilt->read_angles = 1;
  pantilt->write_angles = 1;
}*/
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
