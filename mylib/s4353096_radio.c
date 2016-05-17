/**
 ******************************************************************************
 * @file mylib/s4353096_radio.c
 * @author Steffen Mitchell - 43530960
 * @date 04042016
 * @brief Radio peripheral driver
 * REFERENCE: ex10_SPI, ex11_console, ex13_radio
 ******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4353096_radio_init() - Initialise radio (GPIO, SPI, etc)
 *
 * s4353096_radio_fsmprocessing() - Radio	FSM	processing	loop (internal	and
 *                                  external function) Called	from	main()
 *                                  and	mylib	files, whenever the	radio FSM
 *                                  must change	state.
 *
 * s4353096_radio_setchan(unsigned char	chan) - Set the channel of the radio
 *
 * s4353096_radio_settxaddress(unsigned	char *addr) - Set	the	transmit address
 *                                                    of the radio
 *
 * unsigned	char s4353096_radio_getchan() - Get	the	channel	of the radio
 *
 * s4353096_radio_gettxaddress(unsigned	char *addr) - Get	the	transmit	address
 *                                                    of the radio. (i.e get txaddress and put it in an unsigned char *addr)
 *
 * s4353096_radio_sendpacket(char	chan,	unsigned char *addr,
 *                          unsigned char *txpacket) - Function to send a packet
 *
 * s4353096_radio_setfsmrx() - Set Radio FSM into	RX mode
 *
 * int s4353096_radio_getrxstatus() - Function to check when packet is recieved.
 *                                    Returns value of s4353096_radio_rxstatus
 *
 * s4353096_radio_getpacket(unsigned char *rxpacket)
 *                                        - function to recieve a packet, when
 *                                          s4353096_radio_status is 1. Must
 *                                          ONLY be called when
 *                                          s4353096_radio_getrxstatus() == 1.
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "FreeRTOS_CLI.h"
#include "s4353096_radio.h"
#include "s4353096_hamming.h"
static SPI_HandleTypeDef SpiHandle;

unsigned char s4353096_txpacket[32];
/*Initialises Relevent GPIO/SPI Ports and sets both FSM states to IDLE*/

/*The main function for the Radio Task*/
void s4353096_TaskRadio (void) {
  unsigned char s4353096_tx_addr[] = {0x22, 0x91, 0x54, 0x43, 0x00};
  unsigned char s4353096_rx_addr[] = {0x07, 0x35, 0x22, 0x11, 0x00};
  unsigned char s4353096_chan = 50;
  s4353096_radio_setchan(s4353096_chan);
	s4353096_radio_settxaddress(s4353096_tx_addr);
	s4353096_radio_setrxaddress(s4353096_rx_addr);
  /*Main loop for Radio Task*/
  for(;;) {

    if (s4353096_SemaphoreTracking != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
            wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake(s4353096_SemaphoreTracking, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Check the format of the input to hamenc*/
        xSemaphoreGive(s4353096_SemaphoreTracking);
        s4353096_radio_setfsmrx();
		    s4353096_radio_fsmprocessing();
        s4353096_radio_fsmprocessing();

        if (s4353096_radio_getrxstatus() == 1) { //Checks if packet has been recieved
			      /*Prints recieved packet to console*/
	          s4353096_radio_getRAEpacket(s4353096_rx_buffer);
            /*Print the raw packet*/
            debug_printf("\nRaw Packet Recieved: ");

            /*Increment through raw packet and print each byte*/
            for(int j = 0; j < 32; j++) {
              debug_printf("%x", s4353096_rx_buffer[j]);
            }
            debug_printf("\n");

		    } else {

		    }
      }
    }
  }
  vTaskDelay(10);
}
/*Print out values*/
extern void s4353096_radio_getRAEpacket(unsigned char *rxpacket) {
  uint16_t crc_output;
  uint8_t hamming_decoded_bytes[10];
  uint16_t payload[5];
  uint16_t crc_recieved;
  int current_x;
  int current_y;
  int velocity_x;
  int velocity_y;
  int velocity_x_decimal;
  int velocity_y_decimal;
  int l = 0;
  debug_printf("RECV:");
  /*Type*/
  debug_printf("\nType: ");
  for (int j = 0; j < 1; j++) {
    debug_printf("%x", rxpacket[j]);
  }
  /*To Address*/
  debug_printf("\nTo Address: ");
  for (int j = 1; j < 5; j++) {
    debug_printf("%x", rxpacket[j]);
  }
  /*From Address*/
  debug_printf("\nFrom Address: ");
  for (int j = 5; j < 9; j++) {
    debug_printf("%x", rxpacket[j]);
  }
  /*Sequence*/
  debug_printf("\nSequence: ");
  for (int j = 9; j < 10; j++) {
    debug_printf("%x", rxpacket[j]);
  }
  /*CRC*/
  /*Warning, CRC is currently in LSB*/
  crc_output = crc_calculation(rxpacket);
  debug_printf("\nCalculated CRC: %x\n", crc_output);
  crc_recieved = (rxpacket[31]) ^ (rxpacket[30] << 8);
  if (crc_output != crc_recieved) {
    debug_printf("CRC ERROR Detected\n");
  }
  /*Hamming decode*/
  for(int p = 10; p < 30; p+=2) {
    hamming_decoded_bytes[l] = hamming_byte_decoder(rxpacket[p], rxpacket[p+1]);
    /*Change to MSB*/
    if ((l % 2) == 1) {
      payload[l/2] = (hamming_decoded_bytes[l] << 8) ^ hamming_decoded_bytes[l-1];
    }
    l++;
  }
  /*Calculate the Velocity*/
  current_x = payload[1];
  current_y = payload[2];
  /*Calculate X Velocity*/
  velocity_x_decimal = ((current_x - previous_x)*1000)/((current_recieved_time - previous_recieved_time)/1000);
  velocity_x = velocity_x_decimal/1000;
  velocity_x_decimal = velocity_x_decimal - velocity_x*1000;
  /*Calculate Y Velocity*/
  velocity_y_decimal = ((current_y - previous_y)*1000)/((current_recieved_time - previous_recieved_time)/1000);
  velocity_y = velocity_y_decimal/1000;
  velocity_y_decimal = velocity_y_decimal - velocity_y*1000;

  debug_printf("\n\nTIME: %lu", current_recieved_time);
  /*Marker ID*/
  debug_printf("\n\nMarker ID: %hd\nX Coordinate: %hd\nY Coordinate: %hd\nWidth of marker: %hd\nHeight of marker: %hd\n", payload[0], payload[1],payload[2],payload[3],payload[4]);
  previous_recieved_time = current_recieved_time;
  previous_y = current_y;
  previous_x = current_x;
  /*Velocity of X*/
  debug_printf("\nVelocity in X: %d.%03d pixels per millisecond\n", velocity_x, velocity_x_decimal);
  /*Velocity of Y*/
  debug_printf("\nVelocity in Y: %d.%03d pixels per millisecond\n", velocity_y, velocity_y_decimal);

  s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
  s4353096_radio_fsmprocessing();
}
extern void s4353096_radio_init(void) {
  GPIO_InitTypeDef GPIO_spi;
  /*Initialise radio FSM*/
  radio_fsm_init();
  /*Set radio FSM sate to IDLE*/
  radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
  s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
}

/*Processes the current state*/
extern void s4353096_radio_fsmprocessing(void) {
  //Receiving FSM
  switch(s4353096_radio_fsmcurrentstate) {
    case S4353096_IDLE_STATE:	//Idle state for reading current channel
      radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
      /* Get current channel , if radio FSM is in IDLE State */
      if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {
        s4353096_radio_rxstatus = 0;
      } else {
          /* if error occurs, set state back to IDLE state */
          debug_printf("ERROR: Radio FSM not in Idle state\n\r");
          radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
      }
      break;
    case S4353096_TX_STATE:	//TX state for writing packet to be sent.
        /* Put radio FSM in TX state, if radio FSM is in IDLE state */
      if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {
        if (radio_fsm_setstate(RADIO_FSM_TX_STATE) == RADIO_FSM_ERROR) {
            debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
            //HAL_Delay(100);
        } else {
            /*Has just been put into TX State and has yet to transmit the packet
            Toggle the LED to indicate about to transmit*/
            radio_fsm_write(s4353096_txpacket);
            BRD_LEDToggle();
        }
      } else {
        /* if error occurs, set state back to IDLE state */
        debug_printf("ERROR: Radio FSM not in Idle state\n\r");
        radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
      }
      break;
    case S4353096_RX_STATE:
        /* Put radio FSM in RX state, if radio FSM is in IDLE or in waiting state */
      if ((radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) || (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE)) {
        if (radio_fsm_setstate(RADIO_FSM_RX_STATE) == RADIO_FSM_ERROR) {
          debug_printf("ERROR: Cannot set Radio FSM RX state\n\r");
          //HAL_Delay(100);
        } else {
          /*The below line is possibly redundant*/
          radio_fsm_setstate(RADIO_FSM_RX_STATE);
          s4353096_radio_fsmcurrentstate = S4353096_WAITING_STATE;		//set next state as Waiting state
        }
      } else {
        /* if error occurs, set state back to IDLE state */
        debug_printf("ERROR: Radio FSM not in Idle state\n\r");
        radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
      }
      break;
    case S4353096_WAITING_STATE:	//Waiting state for reading received packet.
      /* Check if radio FSM is in WAITING STATE */
      if (radio_fsm_getstate() == RADIO_FSM_WAIT_STATE) {
        if (radio_fsm_read(s4353096_rx_buffer) == RADIO_FSM_DONE) {
          s4353096_radio_rxstatus = 1;
          current_recieved_time = HAL_GetTick();
        } else {
          s4353096_radio_rxstatus = 0;
        }
        //HAL_Delay(10);
      } else {

      }
      break;
  }
}

/*Constructs the transmission packet and transmits it if the FSM's are in the
  TX state*/
extern void s4353096_radio_sendpacket(char	chan,	unsigned char *addr,
  unsigned char *txpacket) {
    unsigned char s4353096_student_number[] = {0x43, 0x53, 0x09, 0x60};
    /*Construction of transmittion packet*/
    for (int i = 0; i < 16; i++) {
      if (i == 0) {
        s4353096_txpacket[i] = 0x20;
      } else if (i < 5) {
        s4353096_txpacket[i] = addr[(i-1)];
      } else if (i < 9) {
        s4353096_txpacket[i] = s4353096_student_number[(i-5)];
      } else if (i < 16) {
        s4353096_txpacket[i] = txpacket[(i-9)];
      } else {
        debug_printf("ERROR with Packaging\n");
      }
    }
      /*Debug Statement to check format of packet sent*/
      #ifdef DEBUG
        for (int j = 0; j < 16; j++) {
          debug_printf("%x",s4353096_txpacket[j]);
        }
        debug_printf("\n");
      #endif
}

/*Retrieves the channel from the RF_CHAN Register on the Transciever*/
extern unsigned char s4353096_radio_getchan(void) {
  unsigned char radio_channel;
  radio_fsm_register_read(NRF24L01P_RF_CH, &radio_channel);
  return radio_channel;
}

/*Writes the channel to the RF_CHAN Register on the Transciever*/
extern void s4353096_radio_setchan(unsigned char chan) {
  radio_fsm_register_write(NRF24L01P_RF_CH, &chan);
}

/*Retrieves the tx address from the TX_ADDR Register on the Transciever*/
extern void s4353096_radio_gettxaddress(unsigned char *addr) {
  radio_fsm_buffer_read(NRF24L01P_TX_ADDR, addr, 5);
}

/*Writes the tx address to the TX_ADDR Register on the Transciever*/
extern void s4353096_radio_settxaddress(unsigned char *addr) {
  radio_fsm_buffer_write(NRF24L01P_TX_ADDR, addr, 5);
}

/*Writes the rx address to the RX_ADDR Register on the Transciever*/
extern void s4353096_radio_setrxaddress(unsigned char *addr) {
  radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, addr, 5);
}
/*Sets s4353096_fsm and RADIO_FSM to rx state*/
extern void s4353096_radio_setfsmrx(void) {
  s4353096_radio_fsmcurrentstate = S4353096_RX_STATE;
  radio_fsm_setstate(RADIO_FSM_RX_STATE);
}
/*Check if a packet has been recieved and if it has put it in the rx_buffer.
  Returns 1 for recieved and 0 for not recieved.*/
extern int s4353096_radio_getrxstatus(void) {
  return s4353096_radio_rxstatus;
}

/*Prints the recieved packet to the console*/
extern void s4353096_radio_getpacket(unsigned char *rxpacket) {
  debug_printf("RECV:");
  for (int j = 5; j < 9; j++) {
    debug_printf("%x", rxpacket[j]);
  }
  debug_printf(">");
  for (int i = 9; i < 16; i++) {
    debug_printf("%c", rxpacket[i]);
  }
  debug_printf("\n");
  s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
  s4353096_radio_fsmprocessing();
}
