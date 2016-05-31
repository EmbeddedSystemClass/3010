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
#include <math.h>

/*#ifdef ROVERDEFINES
#define ROVER46CHAN 46
unsigned char ROVER46ADDR[] = {0x46, 0x33, 0x22, 0x11, 0x00};
#define ROVER47CHAN 47
unsigned char ROVER47ADDR[] = {0x47, 0x33, 0x22, 0x11, 0x00};
#define ROVER48CHAN 48
unsigned char ROVER48ADDR[] = {0x48, 0x33, 0x22, 0x11, 0x00};
#define ROVER49CHAN 49
unsigned char ROVER49ADDR[] = {0x49, 0x33, 0x22, 0x11, 0x00};
#define ROVER51CHAN 51
unsigned char ROVER51ADDR[] = {0x51, 0x33, 0x22, 0x11, 0x00};
#define ROVER52CHAN 52
unsigned char ROVER52ADDR[] = {0x52, 0x33, 0x22, 0x11, 0x00};
#define ROVER53CHAN 53
unsigned char ROVER53ADDR[] = {0x53, 0x33, 0x22, 0x11, 0x00};
#endif*/
#define ROVER46CHAN 46
#define ROVER47CHAN 47
#define ROVER48CHAN 48
#define ROVER49CHAN 49
#define ROVER51CHAN 51
#define ROVER52CHAN 52
#define ROVER53CHAN 53

#define FORWARDLEFT 0x04
#define BACKWARDLEFT 0x08
#define FORWARDRIGHT 0x01
#define BACKWARDRIGHT 0x02

struct Rover {
  int velocity[10];
  int desired_distance;
  float testing_duration; //Start with 1.5
  int testing_speed;
  int testing_distance; //so that division results in a float
  int motor_left_forward;
  int motor_right_forward;
  int motor_left_reverse;
  int motor_right_reverse;
  int battery_calibrate;
  int closest_distance;
  int closest_difference;
  uint8_t motor_payload[10];
  int closest_speed;
  int closest_duration;

  int rover_current_x;
  int rover_current_y;

  int marker_current_x;
  int marker_current_y;
  int rover_id;
  int marker_id;
  int angle_multiplier;
  /*Forward/Backward | array for each speed | speed/velocity value*/
  int cal_velocity[2][10][2];
  float ratio_x;
  float ratio_y;
};
struct Rovers {
  unsigned char ROVER46ADDR[5];
  unsigned char ROVER47ADDR[5];
  unsigned char ROVER48ADDR[5];
  unsigned char ROVER49ADDR[5];
  unsigned char ROVER51ADDR[5];
  unsigned char ROVER52ADDR[5];
  unsigned char ROVER53ADDR[5];
};
struct Rover Calibrate;
struct Rover rover;
struct Rovers rover_coms;
SemaphoreHandle_t s4353096_SemaphoreGetPassKey;
SemaphoreHandle_t s4353096_SemaphoreGetSensor;
SemaphoreHandle_t s4353096_SemaphoreSendMotor;
SemaphoreHandle_t s4353096_SemaphoreRecieveRovers;
QueueHandle_t s4353096_QueueRoverTransmit;
QueueHandle_t s4353096_QueueRoverRecieve;

extern void rover_init(void);
extern void recieve_rover_packet (uint8_t *recieved_packet);
extern void send_rover_packet (uint8_t *payload, uint8_t packet_type);
extern void s4353096_TaskRadioProcessing(void);
extern void calibration_velocity_init(void);
extern void calibration_velocity_calculation(void);
extern void calibration_velocity_other_calculation(int mode, int speed, int distance, int duration);
extern void speed_duration_calculation(int distance);
extern void direction_duration_calculation_send(int distance, int direction);
extern void calculate_distance_ratios(void);
extern void calculate_rover_distance_pos(void);
extern void angle_duration_calculation(int angle);
