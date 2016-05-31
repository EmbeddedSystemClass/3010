/**
 ******************************************************************************
 * @file mylib/s4353096_rover.c
 * @author Steffen Mitchell - 43530960
 * @date 15052016
 * @brief Rover Communications driver
 * REFERENCE:
 ******************************************************************************
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
#include "s4353096_pantilt.h"
#include "s4353096_lightbar.h"
#include "s4353096_sysmon.h"
struct Packet rover_communication;

/*Task which handles things recieved by the rover and ocassionaly packets to be sent to the rover*/
extern void s4353096_TaskRover(void) {
  uint8_t getpass[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t motor[] = {100, 100, 0x5A};

  /*Main loop of the Rover Task*/
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
        send_rover_packet(motor, 0x32);
      }
    }
  vTaskDelay(500);
  }
}

/*Recieve Rover Packet Decode*/
extern void recieve_rover_packet (uint8_t *recieved_packet) {
  int l = 0;
  uint8_t hamming_decoded_bytes[10];
  uint8_t payload[10];
  uint16_t crc_recieved;
  uint16_t crc_output;

  /*Print the raw Packet*/
  debug_printf("\n\nRecieved from Rover: ");
  for (int k = 0; k < 32; k++) {
  debug_printf("%x-", recieved_packet[k]);
  }
  debug_printf("\n");

  /*Check CRC's*/
  crc_output = crc_calculation(recieved_packet); //Calculate the crc of the 30byte packet
  /*Print out the calculated CRC*/
  crc_recieved = (recieved_packet[31]) ^ (recieved_packet[30] << 8);
  /*Compare the Calculated CRC to the Recieved CRC*/
  if (crc_output != crc_recieved) {
    debug_printf("CRC ERROR Detected\n");
  }

  /*Hamming Decode Payload*/
  for(int p = 10; p < 30; p+=2) {
    hamming_decoded_bytes[l] = hamming_byte_decoder(recieved_packet[p+1], recieved_packet[p+2]);
    /*Change order of bytes to MSB*/
    if ((l % 2) == 1) {
      payload[l/2] = (hamming_decoded_bytes[l] << 8) ^ hamming_decoded_bytes[l-1];
    }
    l++;
  }
  debug_printf("Sequence: %d\n", recieved_packet[9]);

  if (recieved_packet[0] == 0x30) {
    radio_vars.passkey = hamming_decoded_bytes[0];
    get_system_time();
    debug_printf("Time: %d.%02d Passkey: %x", TaskValues.system_time, TaskValues.system_time_decimal, radio_vars.passkey);
  } else if (recieved_packet[0] == 0x31) {
    get_system_time();
    debug_printf("Time: %d.%02d  Line sensor Value: %x\n", TaskValues.system_time, TaskValues.system_time_decimal, payload[0]);
    s4353096_lightbar_write(((0x00 << 8)^ payload[0]));
  }
  debug_printf("\n");
}

/*Transmit Packet Encode*/
extern void send_rover_packet (uint8_t *payload, uint8_t packet_type) {
  unsigned char s4353096_student_number[] = {0x43, 0x53, 0x09, 0x60};
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

/*Hamming encode the payload*/
  for (int h = 11; h < 30; h+=2) {
    hamming_encoded_byte = hamming_byte_encoder(payload[(h/2) - 5]);
    /*LSB Format*/
    rover_communication.s4353096_tx_packet[h] = (hamming_encoded_byte & 0x00FF);
    rover_communication.s4353096_tx_packet[h+1] = (hamming_encoded_byte & 0xFF00) >> 8;
  }

  rover_communication.s4353096_tx_packet[10] = radio_vars.passkey;

  /*Calculate the crc and place it at the end of tx packet*/
  crc_calculated = crc_calculation(rover_communication.s4353096_tx_packet);
  rover_communication.s4353096_tx_packet[30] = (crc_calculated & 0xFF00) >> 8;
  rover_communication.s4353096_tx_packet[31] = (crc_calculated & 0x00FF);
  radio_vars.orb_rover_fsmcurrentstate = ROVER_TRANSCIEVE;

  /*Print the packet to be transmitted*/
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

/*Initialise values for rover control including base calibrations*/
extern void rover_init(void) {
  unsigned char r46[] = {0x46, 0x33, 0x22, 0x11, 0x00};
  unsigned char r47[] = {0x47, 0x33, 0x22, 0x11, 0x00};
  unsigned char r48[] = {0x48, 0x33, 0x22, 0x11, 0x00};
  unsigned char r49[] = {0x49, 0x33, 0x22, 0x11, 0x00};
  unsigned char r51[] = {0x51, 0x33, 0x22, 0x11, 0x00};
  unsigned char r52[] = {0x52, 0x33, 0x22, 0x11, 0x00};
  unsigned char r53[] = {0x53, 0x33, 0x22, 0x11, 0x00};
  memcpy(rover_coms.ROVER46ADDR, r46, sizeof(r46));
  memcpy(rover_coms.ROVER47ADDR, r47, sizeof(r47));
  memcpy(rover_coms.ROVER48ADDR, r48, sizeof(r48));
  memcpy(rover_coms.ROVER49ADDR, r49, sizeof(r49));
  memcpy(rover_coms.ROVER51ADDR, r51, sizeof(r51));
  memcpy(rover_coms.ROVER52ADDR, r52, sizeof(r52));
  memcpy(rover_coms.ROVER53ADDR, r53, sizeof(r53));
  servo_control.orb_c[0][0] = 0;        //top corner x value
  servo_control.orb_c[1][0] = 0;        //top corner y value
  servo_control.orb_c[0][1] = 284;      //bottom corner x value
  servo_control.orb_c[1][1] = 192;      //bottom corner y value
  servo_control.display_c[0][0] = 20;   //top corner pan value
  servo_control.display_c[1][0] = 44;   //top corner tilt value
  servo_control.display_c[0][1] = -22;  //bottom corner pan value
  servo_control.display_c[1][1] = 63;   //bottom corner tilt value
  rover.rover_current_x = 0;
  rover.rover_current_y = 0;
  calculate_display_ratios();
}
/*Initialise velocity_values*/
extern void calibration_velocity_init(void) {
  Calibrate.velocity[0] = 0;//
  Calibrate.velocity[1] = 0;//30;//
  Calibrate.velocity[2] = 40;//50;
  Calibrate.velocity[3] = 0;//75;
  Calibrate.velocity[4] = 0;// 100;//100; //60
  Calibrate.velocity[5] = 0;
  Calibrate.velocity[6] = 0;
  Calibrate.velocity[7] = 0;
  Calibrate.velocity[8] = 0;
  Calibrate.velocity[9] = 0;
  Calibrate.cal_velocity[0][0][0] = 50;
  Calibrate.cal_velocity[0][1][0] = 55;
  Calibrate.cal_velocity[0][2][0] = 60;
  Calibrate.cal_velocity[0][3][0] = 65;
  Calibrate.cal_velocity[0][4][0] = 70;
  Calibrate.cal_velocity[0][5][0] = 75;
  Calibrate.cal_velocity[0][6][0] = 80;
  Calibrate.cal_velocity[0][7][0] = 85;
  Calibrate.cal_velocity[1][0][0] = 50;
  Calibrate.cal_velocity[1][1][0] = 55;
  Calibrate.cal_velocity[1][2][0] = 60;
  Calibrate.cal_velocity[1][3][0] = 65;
  Calibrate.cal_velocity[1][4][0] = 70;
  Calibrate.cal_velocity[1][5][0] = 75;
  Calibrate.cal_velocity[1][6][0] = 80;
  Calibrate.cal_velocity[1][7][0] = 85;
  /*Note Present code will only function correctly with one speed's velocity given as rovers are wildly unpredictable at different speeds
  in terms of wheter they travel straight or not*/
  Calibrate.cal_velocity[0][0][1] = 0;  //speed 50
  Calibrate.cal_velocity[0][1][1] = 0;  //speed 55
  Calibrate.cal_velocity[0][2][1] = 0;  //speed 60
  Calibrate.cal_velocity[0][3][1] = 0;  //speed 65
  Calibrate.cal_velocity[0][4][1] = 0;  //speed 70
  Calibrate.cal_velocity[0][5][1] = 0;  //speed 75
  Calibrate.cal_velocity[0][6][1] = 0;  //speed 80
  Calibrate.cal_velocity[0][7][1] = 0;  //speed 85
  Calibrate.cal_velocity[1][0][1] = 0;  //speed 50
  Calibrate.cal_velocity[1][1][1] = 0;  //speed 55
  Calibrate.cal_velocity[1][2][1] = 0;  //speed 60
  Calibrate.cal_velocity[1][3][1] = 0;  //speed 65
  Calibrate.cal_velocity[1][4][1] = 0;  //speed 70
  Calibrate.cal_velocity[1][5][1] = 0;  //speed 75
  Calibrate.cal_velocity[1][6][1] = 0;  //speed 80
  Calibrate.cal_velocity[1][7][1] = 0;  //speed 85
  Calibrate.motor_left_forward = 100;
  Calibrate.motor_right_forward =100;
  Calibrate.motor_left_reverse = 100;
  Calibrate.motor_right_reverse =100;
}

/*Old velocity calculation function*/
extern void calibration_velocity_calculation(void) {
  int velocity_element;
  velocity_element = (Calibrate.testing_speed - 40)/5;
  Calibrate.velocity[velocity_element] = Calibrate.testing_distance/Calibrate.testing_duration;
}

/*Calculate the velocity of a speed based on how far it traveled at a given speed over a set duration*/
extern void calibration_velocity_other_calculation(int mode, int speed, int distance, int duration) {
  int velocity;
  velocity = distance/duration;

  /*increment through array storing velocities associated with speeds*/
  for(int i = 0; i < 10; i++) {
    if (Calibrate.cal_velocity[mode][i][0] == speed){
      Calibrate.cal_velocity[mode][i][1] = velocity;
      debug_printf("\nVelocity: %d, Speed =  %d\n", velocity, speed);
    }
  }
}

/*Calculate the motor payload required to reach the input angle*/
extern void angle_duration_calculation(int angle, int direction) {
  uint8_t duration;

  /*If direction is clockwise, positive*/
  if (direction == 0) {
    duration = ((2*angle)/Calibrate.angle_clock_velocity);
    Calibrate.motor_payload[0] = (Calibrate.angle_clock_speed*Calibrate.motor_left_reverse)/100;
    Calibrate.motor_payload[1] = (Calibrate.angle_clock_speed*Calibrate.motor_right_forward)/100;
    Calibrate.motor_payload[2] = duration;
    Calibrate.closest_duration = duration;

  /*If direction is anticlockwise, negative*/
  } else if (direction == 1) {
    duration = ((2*angle)/Calibrate.angle_aclock_velocity);
    Calibrate.motor_payload[0] = (Calibrate.angle_aclock_speed*Calibrate.motor_left_forward)/100;
    Calibrate.motor_payload[1] = (Calibrate.angle_aclock_speed*Calibrate.motor_right_reverse)/100;
    Calibrate.motor_payload[2] = duration;
    Calibrate.closest_duration = duration;
  }
}

/*Direction 0 if forward, 1 if backward*/
/*Calculate the motor payload required to reach the input distance*/
extern void direction_duration_calculation_send(int distance, int direction) {
  int distance_difference;
  float calculated_distance;
  int number_of_speeds = 8;
  int remaining_distance;
  TickType_t xDelayMove;
  Calibrate.closest_difference = 1000; //Something large that will be imediatly changed on first item
  Calibrate.closest_distance = 0; //Initialise the closest distance
  /*Loop to increment speed*/
  for (int i = 0; i < number_of_speeds; i++){
    /*Loop to increment duration from 0 - 7.5*/
    for (int j = 0; j < 16; j++) {
      calculated_distance = (0.5*j*Calibrate.cal_velocity[direction][i][1]);
      distance_difference = abs(distance - calculated_distance);
      if (distance_difference <= Calibrate.closest_difference) {
        /*If the calculated distance better matches the desired distance*/
        Calibrate.closest_speed = Calibrate.cal_velocity[direction][i][0]; //Calculates the value of speed for the associated velocity
        Calibrate.closest_duration = j;
        Calibrate.closest_difference = distance_difference;
        Calibrate.closest_distance = calculated_distance;
      }
    }
  }
  if (direction == 0) {
    Calibrate.motor_payload[0] = (Calibrate.closest_speed*Calibrate.motor_left_forward)/100;
    Calibrate.motor_payload[1] = (Calibrate.closest_speed*Calibrate.motor_right_forward)/100;
  } else if (direction == 1) {
    Calibrate.motor_payload[0] = (Calibrate.closest_speed*Calibrate.motor_left_reverse)/100;
    Calibrate.motor_payload[1] = (Calibrate.closest_speed*Calibrate.motor_right_reverse)/100;

  }
  Calibrate.motor_payload[2] = (Calibrate.closest_duration);
  
  /*Determines whether the process needs to be repeated to achieve the distance desired*/
  remaining_distance = distance - Calibrate.closest_distance;
  if (remaining_distance < 60) {
    Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT ^ (FORWARDLEFT));
    /*Perform Transmit Forward here*/
    send_rover_packet(Calibrate.motor_payload, 0x32);
    xDelayMove = 7700 / portTICK_PERIOD_MS;
    vTaskDelay(xDelayMove);
    direction_duration_calculation_send(remaining_distance, 0);
  } else {

  }
  debug_printf("Closest D: %d, S: %d, Du: %d, Dif: %d\n", Calibrate.closest_distance, Calibrate.closest_speed, Calibrate.closest_duration/2, Calibrate.closest_difference);
}

/*Calculates the ratio for converting between ORB Co-ords and Display Pan/Tilt angles*/
extern void calculate_distance_ratios(void) {
  int ratio_x;
  int ratio_y;
  rover.ratio_x = fabsf(900 - 0)/fabsf(servo_control.orb_c[0][1] - servo_control.orb_c[0][0]);
  rover.ratio_y = fabsf(600 - 0)/fabsf(servo_control.orb_c[1][1] - servo_control.orb_c[1][0]);
  ratio_x = rover.ratio_x;
  ratio_y = rover.ratio_y;
  debug_printf("\nPanR: %d, TiltR %d", ratio_x, ratio_y);
}
/*Calculates the pan and tilt angles for the current rover position*/
extern void calculate_rover_distance_pos(void) {
  int current_x_mm;
  int current_y_mm;
  current_x_mm = (rover.rover_current_x*rover.ratio_x);
  current_y_mm = (rover.rover_current_y*rover.ratio_y);
  debug_printf("\nDistance from starting edge: %d\n", current_x_mm);
}
/*Used to perform one follower_function*/
extern void FollowerTask(void) {
  int current_x;
  int current_y;
  int after_move_x;
  int after_move_y;
  int marker_x;
  int marker_y;
  float rover_orientation_angle;
  float marker_angle;
  int angle_change;
  int distance_to_marker;
  int change_y;
  int change_x;
  TickType_t xDelayMove;

  /*Main Loop for the follower task*/
  for (;;) {
    if (s4353096_SemaphoreFollower != NULL) {
      if( xSemaphoreTake(s4353096_SemaphoreFollower, 3 ) == pdTRUE ) {
        xSemaphoreGive(s4353096_SemaphoreFollower);
        xDelayMove = 1100 / portTICK_PERIOD_MS;
        current_x = rover.rover_current_x;
        current_y = rover.rover_current_y;
        direction_duration_calculation_send(40, 0);
        Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT ^ (FORWARDLEFT));
        /*Move forward 1 second*/
        send_rover_packet(Calibrate.motor_payload, 0x32);
        vTaskDelay(xDelayMove);
        after_move_x = rover.rover_current_x;
        after_move_y = rover.rover_current_y;
        /*Work out rover's orientation angle*/
        rover_orientation_angle = angle_calculation(current_x, current_y, after_move_x, after_move_y);
        /*Work out angle of the marker from the rover*/
        marker_x = rover.marker_current_x;
        marker_y = rover.marker_current_y;
        marker_angle = angle_calculation(current_x, current_y, marker_x, marker_y);
        /*Work out required angle change to face marker*/
        angle_change = marker_angle - rover_orientation_angle;
        /*Turn to angle, can try to implement half of angle turn to cautomaticaly calibrate the angle*/
        if (angle_change < 0) {
          angle_duration_calculation(angle_change, 1);
          Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT);
        } else {
          angle_duration_calculation(angle_change, 0);
          Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDLEFT);
        }
        send_rover_packet(Calibrate.motor_payload, 0x32);
        xDelayMove = ((500*Calibrate.closest_duration)+100) / portTICK_PERIOD_MS;
        vTaskDelay(xDelayMove);
        change_x = marker_x - rover.rover_current_x;
        change_y = marker_y - rover.rover_current_y;
        /*Calculate the approximate distance to the target*/
        distance_to_marker = sqrt(((change_x*change_x) + (change_y*change_y)));
        /*Move forward to the target*/
        direction_duration_calculation_send(distance_to_marker, 0);
        Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT ^ (FORWARDLEFT));
        send_rover_packet(Calibrate.motor_payload, 0x32);
        xDelayMove = ((500*Calibrate.closest_duration)+100) / portTICK_PERIOD_MS;
        vTaskDelay(xDelayMove);
      }
    }
  }
  vTaskDelay(400);
}

/*Calculates the angle between to points based on the x_y axis where top hald is +180 and bottom half is -180*/
extern float angle_calculation(int center_x, int center_y, int point_x, int point_y) {
  float change_y;
  float change_x;
  float val = 180.0 / PI;
  float theta;
  float target_angle;
  change_y = point_y - center_y;
  change_x = point_x - center_x;
  theta = atan2f(change_y, change_x);
  if (center_x < point_x) {
    if (center_y < point_y) {
      target_angle = theta;
    } else if (center_y > point_y) {
      target_angle = -1*theta;
    } else {
      target_angle = 0;
    }
  } else if (center_x > point_x) {
    if (center_y < point_y) {
      target_angle = 180 - theta;
    } else if (center_y > point_y) {
      target_angle = -180 + theta;
    } else {
      target_angle = 0;
    }
  } else {
    target_angle = 0;
  }
  return target_angle;
}
