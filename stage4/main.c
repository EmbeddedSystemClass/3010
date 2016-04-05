/**
  ******************************************************************************
  * @file    stage4/main.c
  * @author  Steffen Mitchell
  * @date    4-May-2016
  * @brief   Stage 4 main file
  *
  *			 REFERENCES: ex13_radio, ex11_console, ex10_SPI
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_radio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);

void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	HAL_Delay(3000); //Delay Mainprogram start
	/* Main processing loop */
  while (1) {
		/*Processes the current fsm state*/
		s4353096_radio_fsmprocessing();
		if (s4353096_radio_getrxstatus() == 1) { //Checks if packet has been recieved
			/*Prints recieved packet to console*/
			s4353096_radio_getpacket(s4353096_rx_buffer);
		}
		/*Compiles the transmit packet. Transmits packet if in TX state*/
		s4353096_radio_sendpacket(s4353096_radio_getchan(), s4353096_addr_get, s4353096_payload_buffer);
	}
}

void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
	s4353096_radio_init(); //Initialises the radio_fsm and associated ports
}
