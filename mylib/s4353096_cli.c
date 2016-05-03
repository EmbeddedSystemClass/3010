/**
  ******************************************************************************
  * @file    mylib/s4353096_cli.c
  * @author  Steffen Mitchell
  * @date    9-May-2014
  * @brief   FreeRTOS CLI program.Creates a task to implement the CLI and flash
  *			 the onboard Blue LED.
  *
  *			 Implements the echo command
  *			 See the FreeRTOSPlus CLI API for more information
  *			 http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_CLI
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
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

//#include "s4353096_cli.h"
#include "s4353096_pantilt.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
struct PanTilt SendPosition;

extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreLaser != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreLaser);		/* Give PB Semaphore from ISR*/
		//debug_printf("Triggered \n\r");    //Print press count value
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
  /* Set the semaphore as available if the semaphore exists*/
	if (strcmp(pcWriteBuffer,"left") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphorePanLeft);
  } else if (strcmp(pcWriteBuffer,"right") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphorePanRight);
  } else {
    /*Check if value is a valid integer and if it is send it to the queue*/
    if ((atoi(pcWriteBuffer) != 0) || (pcWriteBuffer[0] == '0')) {
      /*Valid integer, send to queue*/
      SendPosition.set_angle_pan = atoi(pcWriteBuffer);
      if (s4353096_QueuePan != NULL) {	/* Check if queue exists */
				if( xQueueSendToBack(s4353096_QueuePan, ( void * ) &SendPosition, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
    } else {
      /*Not a valid integer*/
    }
  }
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
  /* Set the semaphore as available if the semaphore exists*/
	if (strcmp(pcWriteBuffer,"up") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreTiltUp);
  } else if (strcmp(pcWriteBuffer,"down") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreTiltDown);
  } else {
    /*Check if value is a valid integer and if it is send it to the queue*/
    if ((atoi(pcWriteBuffer) != 0) || (pcWriteBuffer[0] == '0')) {
      /*Valid integer, send to queue*/
      SendPosition.set_angle_tilt = atoi(pcWriteBuffer);
      if (s4353096_QueueTilt != NULL) {	/* Check if queue exists */
				if( xQueueSendToBack(s4353096_QueueTilt, ( void * ) &SendPosition, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
    } else {
      /*Not a valid integer*/
    }
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	/* Write command echo output string to write buffer. */
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n");
  /* Set the semaphore as available if the semaphore exists*/
  xSemaphoreGive(s4353096_SemaphoreBox);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
