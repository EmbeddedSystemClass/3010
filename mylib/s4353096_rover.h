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
#define FORWARD 0x10
#define BACKWARD 0x01
struct Rover {
  int velocity[10];
  int desired_distance;
  float testing_duration; //Start with 1.5
  int testing_speed;
  int testing_distance; //so that division results in a float
  int motor_left;
  int motor_right;
  int battery_calibrate;
  int closest_distance;
  int closest_difference;
  uint8_t motor_payload[3];
  int closest_speed;
  int closest_duration;
};
struct Rover Calibrate;
SemaphoreHandle_t s4353096_SemaphoreGetPassKey;
SemaphoreHandle_t s4353096_SemaphoreGetSensor;
SemaphoreHandle_t s4353096_SemaphoreSendMotor;
QueueHandle_t s4353096_QueueRoverTransmit;
QueueHandle_t s4353096_QueueRoverRecieve;

extern void recieve_rover_packet (uint8_t *recieved_packet);
extern void send_rover_packet (uint8_t *payload, uint8_t packet_type);
extern void s4353096_TaskRadioProcessing(void);
extern void calibration_velocity_init(void);
extern void calibration_velocity_calculation(void);
extern void speed_duration_calculation(int distance);
