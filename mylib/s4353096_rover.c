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
#include <stdlib.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "s4353096_rover.h"
#include "s4353096_radio.h"
#include "s4353096_hamming.h"

struct Packet rover_communication;

extern void s4353096_TaskRadioProcessing(void) {
  uint8_t getpass[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t motor[] = {100, 100, 0x5A};
  for(;;) {
    if (s4353096_QueueRoverRecieve != NULL) {	/* Check if queue exists */
              /* Check for item received - block atmost for 10 ticks */
      if (xQueueReceive(s4353096_QueueRoverRecieve, &rover_communication, 10 )) {
        /*Process the Recieved packet*/
        recieve_rover_packet(rover_communication.s4353096_rx_buffer);
        vTaskDelay(20);
      }
    }
    if (s4353096_SemaphoreGetSensor != NULL) {
      if( xSemaphoreTake(s4353096_SemaphoreGetSensor, 10 ) == pdTRUE ) {
        send_rover_packet (getpass, 0x31);
      }
    }
    //debug_printf("%x", radio_vars.s4353096_rx_addr_rover[0]);
    if (s4353096_SemaphoreGetPassKey != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
            wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake(s4353096_SemaphoreGetPassKey, 10 ) == pdTRUE ) {
        send_rover_packet (getpass, 0x30);
      }
    }
    if (s4353096_SemaphoreSendMotor != NULL) {
      if( xSemaphoreTake(s4353096_SemaphoreSendMotor, 10 ) == pdTRUE ) {
        send_rover_packet( motor, 0x32);
      }
    }
  vTaskDelay(500);
  }
}
/*Task Rover TXRX Communication*/

/*Recieve Rover Packet Decode*/
extern void recieve_rover_packet (uint8_t *recieved_packet) {
int l = 0;
uint8_t hamming_decoded_bytes[10];
/*Print the raw Packet*/
debug_printf("\n\nRecieved from Rover: ");
for (int k = 0; k < 32; k++) {
  debug_printf("%x-", recieved_packet[k]);
}
debug_printf("\n");
/*Check CRC's*/

/*Hamming Decode Payload*/

/*for(int p = 10; p < 30; p+=2) {
  hamming_decoded_bytes[l] = hamming_byte_decoder(recieved_packet[p+1], recieved_packet[p+2]);
  debug_printf("%d\n", p);*//*Change order of bytes to MSB*/
  /*if ((l % 2) == 1) {
    payload[l/2] = (hamming_decoded_bytes[l] << 8) ^ hamming_decoded_bytes[l-1];
  }*/
  //l++;
//}
//debug_printf("Sequence: %d\n", recieved_packet[9]);
/*if (recieved_packet[0] == 0x30) {
  radio_vars.passkey = hamming_decoded_bytes[0];
}
debug_printf("Decoded Hamming\n\n");
for (int n = 0; n < 10; n++) {
  debug_printf("%x --%d\n", hamming_decoded_bytes[n], n);
}
debug_printf("\n\n");*/
/*Make sure we were the intended recipient*/


}
/*Transmit Packet Encode*/
extern void send_rover_packet (uint8_t *payload, uint8_t packet_type) {
  unsigned char s4353096_student_number[] = {0x43, 0x53, 0x09, 0x60};
  //unsigned char hamming_encoded_payload_passkey[20];
  uint16_t hamming_encoded_byte;
  uint16_t crc_calculated;
  /*Type from CLI command via a queue*/
  rover_communication.s4353096_tx_packet[0] = packet_type;
  /*To Address from radio struct*/
  for (int j = 1; j < 5; j++) {
    rover_communication.s4353096_tx_packet[j] = radio_vars.s4353096_rx_addr_rover[j-1];
  }
  /*From Address fromm radio struct*/
  for (int j = 5; j < 9; j++) {
    rover_communication.s4353096_tx_packet[j] = s4353096_student_number[j-5];
  }
  /*Sequence generated Here, increment byte from 0x00 to 0xFF and back round again*/
  rover_communication.s4353096_tx_packet[9] = radio_vars.next_sequence;
  radio_vars.next_sequence = radio_vars.next_sequence + 0x01;

  for (int h = 11; h < 30; h+=2) {
    hamming_encoded_byte = hamming_byte_encoder(payload[(h/2) - 5]);
    //debug_printf("\nPayload: %x, Hamming Byte: %x", payload[(h/2) - 5], hamming_encoded_byte);
    /*LSB Format*/
    rover_communication.s4353096_tx_packet[h] = (hamming_encoded_byte & 0x00FF);
    rover_communication.s4353096_tx_packet[h+1] = (hamming_encoded_byte & 0xFF00) >> 8;
  }
  /*Hamming encode the payload and put it into the trasnmit packet*/
  /*for (int i = 0; i < 40; i += 2) {
    if (i == 0) {*/
    /*Pass Key here from rover struct*/
/*hamming_encoded_byte = 0x00;//hamming_byte_encoder(radio_vars.passkey);
    } else {
      hamming_encoded_byte = hamming_byte_encoder(payload[(i/2) - 1]);
    }

  }*/
  rover_communication.s4353096_tx_packet[10] = radio_vars.passkey;
  /*Calculate the crc and place it at the end of tx packet*/
  crc_calculated = crc_calculation(rover_communication.s4353096_tx_packet);
  rover_communication.s4353096_tx_packet[30] = (crc_calculated & 0xFF00) >> 8;
  rover_communication.s4353096_tx_packet[31] = (crc_calculated & 0x00FF);
  radio_vars.orb_rover_fsmcurrentstate = ROVER_TRANSCIEVE;
  debug_printf("\nRAW TRANSMIT: ");
  for (int y = 0; y < 32; y++) {
    debug_printf("%x-", rover_communication.s4353096_tx_packet[y]);
  }
  debug_printf("\n");
  /*Place the transmit packet in a Queue to the radio task*/
  if (s4353096_QueueRoverTransmit != NULL) {	/* Check if queue exists */

    if( xQueueSendToBack(s4353096_QueueRoverTransmit, ( void * ) &rover_communication, ( portTickType ) 10 ) != pdPASS ) {
      debug_printf("AFailed to post the message, after 10 ticks.\n\r");
    } else {
      radio_vars.orb_rover_fsmcurrentstate = ROVER_TRANSCIEVE;
    }
  }
}

extern void s4353096_TaskRover(void) {

  for(;;){
    /*if Motor Forward or backward*/
  }
}
/*Initialise velocity_values*/
extern void calibration_velocity_init(void) {
  Calibrate.velocity[0] = 0;//
  Calibrate.velocity[1] = 96;//30;//
  Calibrate.velocity[2] = 97;//50;
  Calibrate.velocity[3] = 98;//75;
  Calibrate.velocity[4] = 99;// 100;//100; //60
  Calibrate.velocity[5] = 100;
  Calibrate.velocity[6] = 101;
  Calibrate.velocity[7] = 102;
  Calibrate.velocity[8] = 103;
  Calibrate.velocity[9] = 104;
}
/*Largest speed is 80*/
extern void calibration_velocity_calculation(void) {
  int velocity_element;
  velocity_element = (Calibrate.testing_speed - 40)/5;
  Calibrate.velocity[velocity_element] = Calibrate.testing_distance/Calibrate.testing_duration;
}

/*For determining the duration and speed required to best atain a given distance*/
extern void speed_duration_calculation(int distance) {
  int distance_difference;
  float calculated_distance;
  int number_of_speeds = 9;
  Calibrate.closest_difference = 1000; //Something large that will be imediatly changed on first item
  Calibrate.closest_distance = 0; //Initialise the closest distance
  Calibrate.motor_left = 100;
  Calibrate.motor_right = 100;
  /*Loop to increment speed*/
  for (int i = 0; i < number_of_speeds; i++){
    /*Loop to increment duration from 0 - 7.5*/
    for (int j = 0; j < 16; j++) {
      calculated_distance = (0.5*j*Calibrate.velocity[i]);
      distance_difference = abs(distance - calculated_distance);
      if (distance_difference <= Calibrate.closest_difference) {
        /*If the calculated distance better matches the desired distance*/
        Calibrate.closest_speed = (i*5) + 40; //Calculates the value of speed for the associated velocity
        Calibrate.closest_duration = j;
        Calibrate.closest_difference = distance_difference;
        Calibrate.closest_distance = calculated_distance;
      }
    }
  }
  Calibrate.motor_payload[0] = (Calibrate.closest_speed*Calibrate.motor_left)/100;
  Calibrate.motor_payload[1] = (Calibrate.closest_speed*Calibrate.motor_right)/100;
  /*Duration*/
  Calibrate.motor_payload[2] = (Calibrate.closest_duration);
  /*Direction*/
  /*Done in another function*/
  debug_printf("Closest D: %d, S: %d, Du: %d, Dif: %d\n", Calibrate.closest_distance, Calibrate.closest_speed, Calibrate.closest_duration/2, Calibrate.closest_difference);
}
