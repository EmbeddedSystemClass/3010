/**
  ******************************************************************************
  * @file    ex18_fr_cli/main.c
  * @author  MDS
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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
static BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

CLI_Command_Definition_t xLaser = {	/* Structure that defines the "echo" command line command. */
	"laser",
	"laser: Change laser to specified on or off state\r\n",
	prvLaserCommand,
	1
};

CLI_Command_Definition_t xPan = {	/* Structure that defines the "echo" command line command. */
	"pan",
	"pan: Adjust pan servo by increments of 5 degrees using ""left"" or ""right"" or to a given angle\r\n",
	prvPanCommand,
	1
};

CLI_Command_Definition_t xTilt = {	/* Structure that defines the "echo" command line command. */
	"tilt",
	"tilt: Adjust tilt servo by increments of 5 degrees using ""up"" or ""down"" or to a given angle\r\n",
	prvTiltCommand,
	1
};
