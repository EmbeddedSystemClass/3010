/**
 ******************************************************************************
 * @file mylib/s4353096_radio.c
 * @author Steffen Mitchell - 43530960
 * @date 09032015
 * @brief Radio peripheral driver
 * REFERENCE:
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
                                            s4353096_radio_status is 1. Must
                                            ONLY be called when
                                            s4353096_radio_getrxstatus() == 1.
 ******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "s4353096_radio.h"
static SPI_HandleTypeDef SpiHandle;

extern void s4353096_radio_init(void) {
  GPIO_InitTypeDef GPIO_spi;

  /*Initialise radio FSM*/
  radio_fsm_init();
  /*Set radio FSM sate to IDLE*/
  radio_fsm_setstate(S4353096_IDLE_STATE);
  s4353096_radio_fsmcurrentstate = radio_fsm_getstate();
}
extern void s4353096_radio_fsmprocessing(void) {
  //Receiving FSM
  switch(s4353096_radio_fsmcurrentstate) {

    case S4353096_IDLE_STATE:	//Idle state for reading current channel

      /* Get current channel , if radio FSM is in IDLE State */
      if (radio_fsm_getstate() == RADIO_FSM_IDLE_STATE) {
        if (((HAL_GetTick()/100000) % 50) == 0) {
					//debug_printf("%d\n",HAL_GetTick()/10000);
					s4353096_radio_fsmcurrentstate = S4353096_TX_STATE;
        }
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
            HAL_Delay(100);
          } else {
            //Has just been put into TX State and has yet to transmit the packet
            radio_fsm_setstate(RADIO_FSM_TX_STATE);
            BRD_LEDToggle();
          }
        } else {

            /* if error occurs, set state back to IDLE state */
            debug_printf("ERROR: Radio FSM not in Idle state\n\r");
            radio_fsm_setstate(RADIO_FSM_IDLE_STATE);
        }

        break;

    }
}
extern void s4353096_radio_sendpacket(char	chan,	unsigned char *addr,
  unsigned char *txpacket) {
    unsigned char s4353096_student_number[] = {0x60, 0x09, 0x53, 0x43};
    unsigned char s4353096_payload[] = {'s','t','e','f','f','e','n'};
    for (int i = 0; i < 16; i++) {
      if (i == 0) {
        txpacket[i] = 0x20;
      } else if (i < 5) {
        txpacket[i] = addr[(i-1)];
      } else if (i < 9) {
        txpacket[i] = s4353096_student_number[(i-5)];
      } else if (i < 16) {
        txpacket[i] = s4353096_payload[(i-9)];
      } else {
        debug_printf("ERROR with Packaging\n");
      }
    }
    /*Only Sends the Packet if the FSM is in TX State*/
    if (s4353096_radio_fsmcurrentstate == S4353096_TX_STATE) {
      radio_fsm_write(txpacket); //This function moves FSM back to IDLE STATE
      BRD_LEDToggle();
      s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
      /*for (int j = 0; j < 16; j++) {
        debug_printf("%x",txpacket[j]);
      }
      debug_printf("\n");*/
    }
}
extern unsigned char s4353096_radio_getchan(void) {
  unsigned char radio_channel;
  radio_fsm_register_read(NRF24L01P_RF_CH, &radio_channel);
  //debug_printf("The Radio Channel is: %d\n",radio_channel);
  return radio_channel;
}
extern void s4353096_radio_setchan(unsigned char chan) {
  radio_fsm_register_write(NRF24L01P_RF_CH, &chan);
}

extern void s4353096_radio_gettxaddress(unsigned char *addr) {
  radio_fsm_buffer_read(NRF24L01P_TX_ADDR, addr, 5);
}
extern void s4353096_radio_settxaddress(unsigned char *addr) {
  radio_fsm_buffer_write(NRF24L01P_TX_ADDR, addr, 5);
}
