/**
 ******************************************************************************
 * @file mylib/s4353096_pantilt.c
 * @author Steffen Mitchell - 43530960
 * @date 16032015
 * @brief Servo Pan and Tilt peripheral driver
 * REFERENCE:
 ******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4353096_pantilt_init() - Initialise servo (GPIO, PWM, Timer, etc)
 * s4353096_pantilt_angle(type, angle) - Write the pan or tilt servo to an angle
 * s4353096_terminal_angle_check () - Checks  angle setting values and adjusts
 * their values accordingly.
 ******************************************************************************
*/
/* Includes */
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "s4353096_rover.h"

extern void s4353096_TaskRover(void) {
  struct Packet rover_communication;
  for(;;) {
  vTaskDelay(10);
  }
}
/*Task Rover TXRX Communication*/

/*Recieve Rover Packet Decode*/
extern recieve_rover_packet (uint8_t *recieved_packet) {
/*Check CRC's*/

/*Hamming Decode Payload*/

/*Make sure we were the intended recipient*/


}
/*Transmit Packet Encode*/
extern void send_rover_packet (uint8_t *payload) {
  /*Type from CLI command via a queue*/

  /*To Address from radio struct*/

  /*From Address fromm radio struct*/

  /*Sequence generated Here, increment byte from 0x00 to 0xFF and back round again*/

  /*Pass Key here from rover struct*/

  /*Payload is a parameter given to the function*/

  /*Hamming encode the payload and put it into the trasnmit packet*/

  /*Calculate the crc and place it at the end*/

  /*Place the transmit packet in a Queue to the radio task*/

}
