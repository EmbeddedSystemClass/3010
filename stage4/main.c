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
#include "s4353096_radio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
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
	HAL_Delay(3000);
	unsigned char s4353096_addr[] = {0x78, 0x56, 0x34, 0x12};
	s4353096_chan = 51;
	s4353096_radio_setchan(s4353096_chan);
	s4353096_radio_settxaddress(s4353096_addr);
	/* Main processing loop */
  while (1) {
		s4353096_radio_fsmprocessing();
		HAL_Delay(100);
		s4353096_radio_sendpacket(s4353096_radio_getchan(), s4353096_addr, s4353096_rx_buffer);
	}
}

/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
	s4353096_radio_init();
}
