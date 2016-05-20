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
SemaphoreHandle_t s4353096_SemaphoreGetPassKey;
SemaphoreHandle_t s4353096_SemaphoreGetSensor;
QueueHandle_t s4353096_QueueRoverTransmit;
QueueHandle_t s4353096_QueueRoverRecieve;
extern void recieve_rover_packet (uint8_t *recieved_packet);
extern void send_rover_packet (uint8_t *payload, uint8_t packet_type);
extern void s4353096_TaskRover(void);
