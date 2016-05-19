/**
 ******************************************************************************
 * @file mylib/s4353096_radio.h
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
 *                                                    of the radio
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
 *
 * s4353096_TaskRadio() - The main function for the Radio Task
 ******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
 #include "board.h"
 #include "stm32f4xx_hal_conf.h"
 #include "debug_printf.h"
 #include "radio_fsm.h"
 #include "nrf24l01plus.h"

 #include "FreeRTOS.h"
 #include "task.h"
 #include "queue.h"
 #include "semphr.h"
 /* Private typedef -----------------------------------------------------------*/
 /* Private define ------------------------------------------------------------*/
 #define S4353096_IDLE_STATE 0
 #define S4353096_RX_STATE 1
 #define S4353096_TX_STATE 2
 #define S4353096_WAITING_STATE 3
 /* Task Priorities ------------------------------------------------------------*/
 #define mainTASKRADIO_PRIORITY					( tskIDLE_PRIORITY + 1 )
 /* Task Stack Allocations -----------------------------------------------------*/
 #define mainTASKRADIO_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )
 /*Global Variables for radio*/
 struct Radio {
   int s4353096_radio_fsmcurrentstate;
   int s4353096_radio_rxstatus;
   unsigned char s4353096_rx_buffer[32];
   unsigned char s4353096_tx_packet[32];
   unsigned char s4353096_rx_addr_orb[] = {0x32, 0x34, 0x22, 0x11, 0x00};
   unsigned char s4353096_rx_addr_rover[] = {0x46, 0x33, 0x22, 0x11, 0x00};
   unsigned char s4353096_tx_addr[] = s4353096_rx_addr_rover;
   unsigned char s4353096_chan_rover = 46;
   unsigned char s4353096_chan_orb = 43;

};
struct Packet {
  unsigned char s4353096_rx_buffer[32];
  unsigned char s4353096_tx_packet[32];
  unsigned char s4353096_chan;
};


struct Radio radio_vars;
   int previous_x;
   int previous_y;
   unsigned long previous_recieved_time;
   unsigned long current_recieved_time;
   unsigned char s4353096_payload_buffer[8];
   unsigned char s4353096_addr_get[4];
/*Semephore Definitions*/
SemaphoreHandle_t s4353096_SemaphoreTracking; //CLI Semephore to enable/disable Radio Recieve
SemaphoreHandle_t s4353096_SemaphoreRadioState; //Tracks whether the Radio has been enabled or disabled for Suspension upon key recieve

/*Function Prototypes*/
void s4353096_TaskRadio (void);
extern void s4353096_radio_init(void);
extern void s4353096_radio_fsmprocessing();
extern void s4353096_radio_sendpacket(char	chan,	unsigned char *addr,
  unsigned char *txpacket);
extern unsigned char s4353096_radio_getchan(void);
extern void s4353096_radio_setchan(unsigned char chan);
extern void s4353096_radio_gettxaddress(unsigned char *addr);
extern void s4353096_radio_settxaddress(unsigned char *addr);
extern void s4353096_radio_setfsmrx(void);
extern int s4353096_radio_getrxstatus(void);
extern void s4353096_radio_getpacket(unsigned char *rxpacket);
extern void s4353096_radio_setrxaddress(unsigned char *addr);
extern void s4353096_radio_getRAEpacket(unsigned char *rxpacket);
