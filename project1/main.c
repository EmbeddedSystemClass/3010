/**
******************************************************************************
* @file    project1/main.c
* @author  Steffen Mitchell
* @date    10-January-2015
* @brief   Project 1 main file.
*
*			 REFERENCES: 
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_pantilt.h"
#include "s4353096_joystick.h"
#include "s4353096_hamming.h"
#include "s4353096_radio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define S4353096_JOYSTICK 1
#define S4353096_TERMINAL 0
#define S4353096_LASER_TRANSMIT 2
#define S4353096_RADIO 3
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
	int recieve_flag;
};
struct Variables *vars;
unsigned int x_value;
unsigned int y_value;
int direction_multiplier = 1;
char RxChar;
int s4353096_keystroke = 0;
int s4353096_payload_length = 0;
unsigned char s4353096_tx_addr[] = {0x22, 0x91, 0x54, 0x43, 0x00};
unsigned char s4353096_rx_addr[] = {0x98, 0x74, 0x56, 0x43, 0x00};;
unsigned char s4353096_chan = 47;
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
	vars->recieve_flag = 0;
	Pb_init();
	s4353096_radio_setchan(s4353096_chan);
	s4353096_radio_settxaddress(s4353096_tx_addr);
	s4353096_radio_setrxaddress(s4353096_rx_addr);
  while (1) {
		s4353096_radio_setfsmrx();
		s4353096_radio_fsmprocessing();
		if (vars->recieve_flag == 1) {
			manchester_decode();
			vars->recieve_flag = 0;
		}
		if (mode == S4353096_RADIO) {
		/*Processes the current fsm state*/
		RxChar = debug_getc();
		if (RxChar != '\0') {
			s4353096_keystroke = 1;
			while(s4353096_keystroke == 1){
				if (RxChar == '\r' || s4353096_payload_length == 7) {
					for (int j = s4353096_payload_length; j < 7; j++) {
						s4353096_payload_buffer[j] = '-';
					}
					s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
					s4353096_radio_fsmprocessing();
					s4353096_radio_fsmcurrentstate = S4353096_TX_STATE;
					debug_printf("\n");
					/*Compiles the transmit packet. Transmits packet if in TX state*/
					s4353096_radio_sendpacket(s4353096_radio_getchan(), s4353096_addr_get, s4353096_payload_buffer);
					s4353096_radio_fsmprocessing();
					s4353096_radio_setfsmrx();
					s4353096_radio_fsmprocessing();
					s4353096_payload_length = 0;
					s4353096_keystroke = 0;
				} else if (RxChar != '\0') {
					s4353096_payload_buffer[s4353096_payload_length] = RxChar;
					s4353096_payload_length++;
					debug_putc(RxChar);				//reflect byte using putc - puts character into buffer
					debug_flush();					//Must call flush, to send character		//reflect byte using printf - must delay before calling printf again.
				} else {

				}
				HAL_Delay(125);
				RxChar = debug_getc();
			}
		} else {

		}
		s4353096_radio_fsmprocessing();
		if (s4353096_radio_getrxstatus() == 1) { //Checks if packet has been recieved
				/*Prints recieved packet to console*/
			s4353096_radio_getpacket(s4353096_rx_buffer);
		} else {

		}
	}
		if (pantilt->write_angles == 1) {
			s4353096_pantilt_angle_write(1, pantilt->set_angle_pan);
			s4353096_pantilt_angle_write(0, pantilt->set_angle_tilt);
			pantilt->write_angles = 0;
		}
		if (((HAL_GetTick()/10000) % 100) == 0) {
			debug_printf("\nPan: %d Tlit: %d\n", pantilt->set_angle_pan, pantilt->set_angle_tilt);
		}
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

/*Initialise relevent hardware*/
void Hardware_init(void) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	unsigned short PrescalerValue;
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
	/* Initialise Joystick */
	s4353096_joystick_init();
	/* Configure D1 for output of square wave signal */
	s4353096_pantilt_init();
	s4353096_radio_init();
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
		/*Initialise Laser Transmit Port*/
		__LASER_WAVE_GEN_1_GPIO_CLK();
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLDOWN;
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
		GPIO_InitStructure.Pin = LASER_WAVE_GEN_1_PIN;
		HAL_GPIO_Init(LASER_WAVE_GEN_1_GPIO_PORT, &GPIO_InitStructure);
		HAL_Delay(500);
		/* Initialise General interrupt Timer  */
		HAL_TIM_Base_Init(&TIM_Init);
		/* Set priority of Timer 2 update Interrupt [0 (HIGH priority) to 15(LOW priority)] */
		/* 	DO NOT SET INTERRUPT PRIORITY HIGHER THAN 3 */
		HAL_NVIC_SetPriority(PANTILT_TIM_IRQn, 10, 0);		//Set Main priority ot 10 and sub-priority ot 0.
		/* Enable timer update interrupt and interrupt vector for Timer  */
		NVIC_SetVector(PANTILT_TIM_IRQn, (uint32_t)&s4353096_general_irqhandler);
		NVIC_EnableIRQ(PANTILT_TIM_IRQn);
		/*Start the timer*/
		HAL_TIM_Base_Start_IT(&TIM_Init);
		/*Initialise Input capture*/
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
	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 500Khz clock */
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
}
/*Interrupt handler for on board push button*/
void exti_pb_irqhandler(void) {
	HAL_GPIO_EXTI_IRQHandler(BRD_PB_PIN);
	mode = !mode;
}
/*General Interrupt handler which handles interrupts for Laser trasnmit and
	writing angles to pan and tilt. The interrupt is set to occur every half
	manchester bit period for the purposes of transmitting manchester using GPIO*/
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
		} else if ((vars->bit_count == 22) && (vars->bit_half == 1)) {
			//Stop transmitting and reset values
			HAL_GPIO_WritePin(LASER_WAVE_GEN_1_GPIO_PORT, LASER_WAVE_GEN_1_PIN, 0);
			vars->bit_count = 0;
			vars->encoded_bit_count = -1;
			mode = S4353096_TERMINAL;
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
/*Handles the transmission of bits in a manchester encoding*/
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
/*The Input Capture interrupt handler which is used to recieve a laser transmission*/
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
  	/* Read and display the Input Capture value of Timer 3, channel 2 */
		vars->recieve_flag = 1;
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
}
/*Checks whether an unsigned int is approximately equal to another unsigned int
	by a margin of approximately 20%, returns True if it is, else returns false*/
int approximately_equal(unsigned int number, unsigned int check_number) {
	if ((number < ((check_number)*1.2)) && (number > ((check_number)*0.8) )) {
		return 1;
	} else {
		return 0;
	}
}
/*Converts the recieved input captures into bits which are then passed through
	the hamming_byte_decoder to extract data bits and correct/notify of any bit errors*/
void manchester_decode(void) {
	int j = 0;
	vars->current_bit = 1;
	vars->recieve_period = (vars->recieve[0])*4;
	uint8_t upper_byte = 0x00;
	uint8_t lower_byte = 0x00;
	/*If the first bit doesn't have an input capture of approximately_equal to any other input capture then first bit was not one and an error occured*/
	if (approximately_equal(vars->recieve[1], vars->recieve_period*0.5) == 1) {
		for (int l = 0; l < 21; l += 12) {
			for (int i = 0; i < 11; i++) {
				if ((i == 0) && (l==0)) {
					vars->recieve_decoded[i] = 1;
					debug_printf("%d", vars->recieve_decoded[i]);
				} else if ((vars->current_bit == 1) && (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.75) == 1)) {
					/*The bit is !current bit, set current bit to !current bit*/
					vars->recieve_decoded[(i+l)] = !vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
					vars->current_bit = !vars->current_bit;
				} else if ((vars->current_bit == 0) && (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.75) == 1)) {
					/*The bit is current bit + !current_bit, set current bit to !current bit*/
					vars->recieve_decoded[(i+l)] = vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
					i++;
					vars->recieve_decoded[(i+l)] = !vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
					vars->current_bit = !vars->current_bit;
				} else if (approximately_equal(vars->recieve[j], (vars->recieve_period)*0.5) == 1) {
					/*The bit is current bit*/
					vars->recieve_decoded[(i+l)] = vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
				} else if (approximately_equal(vars->recieve[j], (vars->recieve_period)) == 1) {
					/*The bit is !current bit + current bit*/
					vars->recieve_decoded[(i+l)] = !vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
					i++;
					vars->recieve_decoded[(i+l)] = vars->current_bit;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
				} else if ((i == 10) && (l == 12)) {
					vars->recieve_decoded[(i+l)] = 0;
					debug_printf("%d", vars->recieve_decoded[(i+l)]);
				}
				j++;
			}
			if (vars->recieve_decoded[10+l] != 0) {
				/*There is no stop bit, send error back to radio and cancel current recieve*/
				debug_printf("ERROR1\n");
			}
		}
		debug_printf("\n");
	} else {
	/*Start bits are not equal to 1, send error back to radio and cancel current recieve*/
	debug_printf("ERROR2\n");
	}
	/*lower half byte*/
	/*Convert recieve_decoded to MSB and remove start and stop bits*/
	for (int k = 0; k < 8; k++) {
		if (k == 7) {
			lower_byte = (lower_byte ^ vars->recieve_decoded[9-k]);
		} else {
			lower_byte = (lower_byte ^ vars->recieve_decoded[9-k]) << 1;
		}
	}
	/*Checks if start bits for second recieved byte is present*/
	if ((vars->recieve_decoded[11] == 1) && (vars->recieve_decoded[12] == 1)) {//Then start bits for upper are present
		/*Convert recieve_decoded to MSB and remove start and stop bits*/
		for (int m = 0; m < 8; m++) {
			if (m == 7) {
				upper_byte = (upper_byte ^ vars->recieve_decoded[20-m]);
			} else {
				upper_byte = (upper_byte ^ vars->recieve_decoded[20-m]) << 1;
			}
		}
	} else {
		debug_printf("ERROR3\n");
		//ERROR No start bits recieved
	}	/*Convert recieve_decoded to MSB and remove start and stop bits*/
	hamming_byte_decoder(lower_byte, upper_byte);
}
