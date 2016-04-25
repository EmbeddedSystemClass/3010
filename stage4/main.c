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
char RxChar;
int s4353096_keystroke = 0;
int s4353096_payload_length = 0;
unsigned char s4353096_tx_addr[] = {0x22, 0x91, 0x54, 0x43, 0x00};
unsigned char s4353096_rx_addr[] = {0x98, 0x74, 0x56, 0x43, 0x00};
unsigned char s4353096_chan = 47;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);

void main(void) {

	BRD_init();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	HAL_Delay(3000); //Delay Mainprogram start
	/* Main processing loop */
	s4353096_radio_setchan(s4353096_chan);
	s4353096_radio_settxaddress(s4353096_tx_addr);
	s4353096_radio_setrxaddress(s4353096_rx_addr);
  while (1) {
		s4353096_radio_setfsmrx();
		s4353096_radio_fsmprocessing();
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
}
//}

void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	BRD_LEDInit();		//Initialise Blue LED
	BRD_LEDOff();		//Turn off Blue LED
	s4353096_radio_init(); //Initialises the radio_fsm and associated ports
}
