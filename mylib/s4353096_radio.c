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
 *                                    Returns value of radio_vars.s4353096_radio_rxstatus
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
#include "s4353096_rover.h"
#include <stdio.h>
#include <string.h>
static SPI_HandleTypeDef SpiHandle;



/*The main function for the Radio Task*/
void s4353096_TaskRadio (void) {
  unsigned char orb_addr[] = {0x31, 0x34, 0x22, 0x11, 0x00};
  unsigned char rover_addr[] = {0x48, 0x33, 0x22, 0x11, 0x00};
  memcpy(radio_vars.s4353096_rx_addr_orb, orb_addr, sizeof(orb_addr));
  memcpy(radio_vars.s4353096_rx_addr_rover, rover_addr, sizeof(rover_addr));
  memcpy(radio_vars.s4353096_tx_addr, rover_addr, sizeof(rover_addr));
  radio_vars.s4353096_chan_rover = 48;
  radio_vars.s4353096_chan_orb = 43;
  radio_vars.next_sequence = 0x00;
  radio_vars.passkey = 0x00;
  int p = 0;
  s4353096_radio_setchan(radio_vars.s4353096_chan_orb);
	s4353096_radio_settxaddress(radio_vars.s4353096_tx_addr);
	s4353096_radio_setrxaddress(radio_vars.s4353096_rx_addr_orb);
  /*Main loop for Radio Task*/
  for(;;) {
    p = 0;
    if (s4353096_SemaphoreTracking != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
            wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake(s4353096_SemaphoreTracking, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Check the format of the input to hamenc*/
        xSemaphoreGive(s4353096_SemaphoreTracking);
        if (s4353096_QueueRoverTransmit != NULL) {	/* Check if queue exists */
                /* Check for item received - block atmost for 10 ticks */
          if (xQueueReceive(s4353096_QueueRoverTransmit, &radio_side_communication, 10 )) {
            radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
            s4353096_radio_fsmprocessing();
            s4353096_radio_setrxaddress(radio_vars.s4353096_rx_addr_rover);
            s4353096_radio_settxaddress(radio_vars.s4353096_tx_addr);
            s4353096_radio_setchan(radio_vars.s4353096_chan_rover);
            /*Transmit the Packet*/
            radio_vars.s4353096_radio_fsmcurrentstate = S4353096_TX_STATE;
            /*Set Transmit Packet as Rover Packet*/
            memcpy(radio_vars.s4353096_tx_packet, radio_side_communication.s4353096_tx_packet, sizeof(radio_side_communication.s4353096_tx_packet));
            s4353096_radio_fsmprocessing();
            debug_printf("Waiting for Recieve\n");

            /*Wait for packet from rover*/
            while(s4353096_radio_getrxstatus() == 0 && (p < 80000)) {
              /*Loop until a packet has been recieved*/
              p++;
              s4353096_radio_setfsmrx();
              s4353096_radio_fsmprocessing();
              //vTaskDelay(50);
              s4353096_radio_fsmprocessing();
              if (s4353096_radio_getrxstatus() == 1) {
                memcpy(radio_side_communication.s4353096_rx_buffer, radio_vars.s4353096_rx_buffer, sizeof(radio_vars.s4353096_rx_buffer));
                if (s4353096_QueueRoverRecieve != NULL) {	/* Check if queue exists */
                  /*Send the recieved packet to a Queue*/
                  if( xQueueSendToBack(s4353096_QueueRoverRecieve, ( void * ) &radio_side_communication, ( portTickType ) 10 ) != pdPASS ) {
                    debug_printf("BFailed to post the message, after 10 ticks.\n\r");
                  }
                }
              }
            }
            radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
            s4353096_radio_fsmprocessing();
            debug_printf("Finished Recieve\n");
            s4353096_radio_setchan(radio_vars.s4353096_chan_orb);
            s4353096_radio_setrxaddress(radio_vars.s4353096_rx_addr_orb);
            vTaskDelay(500);
          }
        }
        switch(radio_vars.orb_rover_fsmcurrentstate) {
          case ROVERS_RECIEVE:
              s4353096_radio_setrxaddress(radio_vars.s4353096_rx_addr_rover);
              s4353096_radio_setchan(radio_vars.s4353096_chan_rover);

              /*Loop until a packet has been recieved*/
              while(s4353096_radio_getrxstatus() == 0) {
                /*Loop until a packet has been recieved*/
                p++;
                s4353096_radio_setfsmrx();
                s4353096_radio_fsmprocessing();
                /*In Wait state here*/
                s4353096_radio_fsmprocessing();
                /*After wait state here*/
                if (s4353096_radio_getrxstatus() == 1) {
                  memset(radio_side_communication.s4353096_rx_buffer, 0x00, 32);
                  memcpy(radio_side_communication.s4353096_rx_buffer, radio_vars.s4353096_rx_buffer, sizeof(radio_vars.s4353096_rx_buffer));
                  if (s4353096_QueueRoverRecieve != NULL) {	/* Check if queue exists */
                  /*Send the recieved packet to a Queue*/
                    if( xQueueSendToBack(s4353096_QueueRoverRecieve, ( void * ) &radio_side_communication, ( portTickType ) 10 ) != pdPASS ) {
                      debug_printf("BFailed to post the message, after 10 ticks.\n\r");
                    }
                  }
                }
              }
              radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
              s4353096_radio_fsmprocessing();
              s4353096_radio_setrxaddress(radio_vars.s4353096_rx_addr_orb);
              s4353096_radio_setchan(radio_vars.s4353096_chan_orb);
            break;
          case ORB_RECIEVE:
            vTaskDelay(100);
            s4353096_radio_setfsmrx();
            s4353096_radio_fsmprocessing();
            /*Wait state here*/
            vTaskDelay(100);
            s4353096_radio_fsmprocessing();
            /*After wait state*/
            if (s4353096_radio_getrxstatus() == 1) { //Checks if packet has been recieved
              /*Prints recieved packet to console*/
              s4353096_radio_getRAEpacket(radio_vars.s4353096_rx_buffer);
            }
            break;

          default:
            break;
        }

      }
    }
  }
  vTaskDelay(10);
}

/*Constructs the transmission packet and transmits it if the FSM's are in the
  TX state*/
extern void s4353096_radio_sendpacket(char	chan,	unsigned char *addr,
  unsigned char *txpacket) {
    unsigned char s4353096_student_number[] = {0x43, 0x53, 0x09, 0x60};

    /*Construction of transmittion packet*/
    for (int i = 0; i < 16; i++) {

      if (i == 0) {
        txpacket[i] = 0x20;
      } else if (i < 5) {
        txpacket[i] = addr[(i-1)];
      } else if (i < 9) {
        txpacket[i] = s4353096_student_number[(i-5)];
      } else if (i < 16) {
        txpacket[i] = txpacket[(i-9)];
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

/*Print out values*/
extern void s4353096_radio_getRAEpacket(unsigned char *rxpacket) {
  uint16_t crc_output; //Calculated CRC of the Recieved Packet
  int hamming_decoded_bytes[10];
  int payload[5];
  uint16_t crc_recieved; //CRC recieved in the packet
  int current_x;
  int current_y;
  int velocity_x;
  int velocity_y;
  int velocity_x_decimal;
  int velocity_y_decimal;
  int l = 0;
  #ifdef DEBUGRADIO
  debug_printf("RECV:");
  /*Print the recieved Type*/
  debug_printf("\nType: %x", rxpacket[0]);

  /*Increment through and print the Destination address*/
  debug_printf("\nTo Address: ");
  for (int j = 4; j >= 1; j--) {
    debug_printf("%x", rxpacket[j]);
  }

  /*Increment through and print the Transmission address*/
  debug_printf("\nFrom Address: ");
  for (int j = 8; j >= 5; j--) {
    debug_printf("%x", rxpacket[j]);
  }
  /*Print the recieved Type Sequence*/
  debug_printf("\nSequence: %x", rxpacket[9]);
  #endif
  /*CRC*/
  /*Warning, CRC is currently in LSB*/
  crc_output = crc_calculation(rxpacket); //Calculate the crc of the 30byte packet
  /*Print out the calculated CRC*/
  crc_recieved = (rxpacket[31]) ^ (rxpacket[30] << 8);

  /*Compare the Calculated CRC to the Recieved CRC*/
  if (crc_output != crc_recieved) {
    debug_printf("CRC ERROR Detected\n");
  }

  /*Increment through the rx payload and hamming decode each 2 bytes*/
  for(int p = 10; p < 30; p+=2) {
    hamming_decoded_bytes[l] = hamming_byte_decoder(rxpacket[p], rxpacket[p+1]);

    /*Change order of bytes to MSB*/
    if ((l % 2) == 1) {
      payload[l/2] = (hamming_decoded_bytes[l] << 8) | hamming_decoded_bytes[l-1];
    }
    l++;
  }

  /*Calculate the Velocity*/
  current_x = payload[1];
  current_y = payload[2];
  if(payload[0] == rover.rover_id) {
    rover.rover_current_x = current_x;
    rover.rover_current_y = current_y;
  } else if (payload[0] == rover.marker_id) {
    rover.marker_current_x = current_x;
    rover.marker_current_y = current_y;
  }
  /*Calculate X Velocity*/
  velocity_x_decimal = ((current_x - previous_x)*1000)/((current_recieved_time - previous_recieved_time)/1000);
  velocity_x = velocity_x_decimal/1000;
  velocity_x_decimal = velocity_x_decimal - velocity_x*1000;

  /*Calculate Y Velocity*/
  velocity_y_decimal = ((current_y - previous_y)*1000)/((current_recieved_time - previous_recieved_time)/1000);
  velocity_y = velocity_y_decimal/1000;
  velocity_y_decimal = velocity_y_decimal - velocity_y*1000;

  /*Print out the Marker Id, Marker Width and height, Marker Coordinates*/
  //debug_printf("\n\nMarker ID: %d\nX Coordinate: %d\nY Coordinate: %hd\nWidth of marker: %hd\nHeight of marker: %hd\n", payload[0], payload[1],payload[2],payload[3],payload[4]);
  /*Set current time and position so that the next calculation uses these values*/
  previous_recieved_time = current_recieved_time;
  previous_y = current_y;
  previous_x = current_x;
  /*Set FSM back to IDLE STATE and process this new fsm state*/
  radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
  s4353096_radio_fsmprocessing();
}

/*Initialise radio (GPIO, SPI, etc)*/
extern void s4353096_radio_init(void) {
  GPIO_InitTypeDef GPIO_spi;
  /*Initialise radio FSM*/
  radio_fsm_init();
  /*Set radio FSM sate to IDLE*/
  radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
  radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
  radio_vars.s4353096_radio_fsmcurrentstate = ORB_RECIEVE;
}

/*Processes the current state*/
extern void s4353096_radio_fsmprocessing(void) {

  //Receiving FSM
  switch(radio_vars.s4353096_radio_fsmcurrentstate) {
    case S4353096_IDLE_STATE:	//Idle state for reading current channel
      radio_fsm_setstate(RADIO_FSM_IDLE_STATE);

      /* Get current channel , if radio FSM is in IDLE State */
      if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {
        radio_vars.s4353096_radio_rxstatus = 0;
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
            debug_printf("ERROR: Cannot set Radio FSM TX state\n\r");
        } else {
            /*Has just been put into TX State and has yet to transmit the packet
            Toggle the LED to indicate about to transmit*/
            radio_fsm_write(radio_vars.s4353096_tx_packet);
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
        } else {
          /*The below line is possibly redundant*/
          radio_fsm_setstate(RADIO_FSM_RX_STATE);
          radio_vars.s4353096_radio_fsmcurrentstate = S4353096_WAITING_STATE;		//set next state as Waiting state
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

        if (radio_fsm_read(radio_vars.s4353096_rx_buffer) == RADIO_FSM_DONE) {
          radio_vars.s4353096_radio_rxstatus = 1;
          current_recieved_time = HAL_GetTick();
        } else {
          radio_vars.s4353096_radio_rxstatus = 0;
        }
      } else {

      }
      break;
  }
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

/*Writes the rx address to the RX_ADDR_P1 Register on the Transciever*/
extern void s4353096_radio_setrxaddress(unsigned char *addr) {
  radio_fsm_buffer_write(NRF24L01P_RX_ADDR_P0, addr, 5);
}
/*Sets s4353096_fsm and RADIO_FSM to rx state*/
extern void s4353096_radio_setfsmrx(void) {
  radio_vars.s4353096_radio_fsmcurrentstate = S4353096_RX_STATE;
  radio_fsm_setstate(RADIO_FSM_RX_STATE);
}
/*Check if a packet has been recieved and if it has put it in the rx_buffer.
  Returns 1 for recieved and 0 for not recieved.*/
extern int s4353096_radio_getrxstatus(void) {
  return radio_vars.s4353096_radio_rxstatus;
}

/*Prints the recieved packet to the console*/
extern void s4353096_radio_getpacket(unsigned char *rxpacket) {
  debug_printf("RECV:");

  /*Print the Trasmit Address Maybe*/
  for (int j = 5; j < 9; j++) {
    debug_printf("%x", rxpacket[j]);
  }

  debug_printf(">");
  /*Print the payload*/
  for (int i = 9; i < 16; i++) {
    debug_printf("%c", rxpacket[i]);
  }
  debug_printf("\n");
  radio_vars.s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
  s4353096_radio_fsmprocessing();
}
