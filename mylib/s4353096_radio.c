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


extern void s4353096_radio_init(void) {
  /*Initialise radio FSM*/
  radio_fsm_init();
  /*Set radio FSM sate to IDLE*/
  radio_fsm_setstate(S4353096_IDLE_STATE);
  s4353096_radio_fsmcurrentstate = S4353096_IDLE_STATE;
}
extern void s4353096_radio_fsmprocessing() {
  while(1){


  }
}
