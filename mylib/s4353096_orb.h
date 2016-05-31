/**
 ******************************************************************************
 * @file mylib/s4353096_orb.h
 * @author Steffen Mitchell - 43530960
 * @date 20052016
 * @brief ORB Definitions Driver
 * REFERENCE:
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


#define ORB1CHAN 43
unsigned char ORB1ADDR[] = {0x31, 0x34, 0x22, 0x11, 0x00};
#define ORB2CHAN 43
unsigned char ORB2ADDR[] = {0x32, 0x34, 0x22, 0x11, 0x00};
#define ORB3CHAN 44
unsigned char ORB3ADDR[] = {0x43, 0x34, 0x22, 0x11, 0x00};
#define ORB4CHAN 44
unsigned char ORB4ADDR[] = {0x44, 0x34, 0x22, 0x11, 0x00};
#define ORB5CHAN 45
unsigned char ORB5ADDR[] = {0x55, 0x34, 0x22, 0x11, 0x00};
#define ORB6CHAN 45
unsigned char ORB6ADDR[] = {0x56, 0x34, 0x22, 0x11, 0x00};
#define ORB7CHAN 50
unsigned char ORB7ADDR[] = {0x07, 0x35, 0x22, 0x11, 0x00};
