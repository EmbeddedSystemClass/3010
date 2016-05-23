/**
  ******************************************************************************
  * @file    mylib/s4353096_cli.h
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

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "FreeRTOS_CLI.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Task Priorities ------------------------------------------------------------*/
#define mainTASKCLI_PRIORITY					( tskIDLE_PRIORITY + 1 )
/* Task Stack Allocations -----------------------------------------------------*/
#define mainTASKCLI_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )

/* Private function prototypes -----------------------------------------------*/
extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvTop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvAcc(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvHamenc(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvHamdec(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvTracking(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvSuspend(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvResume(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvCRC(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvGetPassKey(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvGetSensor(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvSendMotor(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvRFChanSet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
extern BaseType_t prvGetTime(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
void CLI_Task(void);



/*CLI Command Definitions*/
CLI_Command_Definition_t xRFChanSet = {	/* Structure that defines the "crc" command line command. */
	"rfchanset",
	"rfchanset: Set the ORB or Rover Channel & Addresses\r\n",
	prvRFChanSet,
	2
};
CLI_Command_Definition_t xGetTime = {	/* Structure that defines the "crc" command line command. */
	"gettime",
	"gettime: 32-bit value (in hex) or an ASCII String\r\n",
	prvGetTime,
	0
};
CLI_Command_Definition_t xSendMotor = {	/* Structure that defines the "crc" command line command. */
	"sendmotor",
	"sendmotor: 32-bit value (in hex) or an ASCII String\r\n",
	prvSendMotor,
	0
};
CLI_Command_Definition_t xGetSensor = {	/* Structure that defines the "crc" command line command. */
	"getsensor",
	"getsensor: 32-bit value (in hex) or an ASCII String\r\n",
	prvGetSensor,
	0
};

CLI_Command_Definition_t xGetPassKey = {	/* Structure that defines the "crc" command line command. */
	"getpasskey",
	"getpasskey: 32-bit value (in hex) or an ASCII String\r\n",
	prvGetPassKey,
	0
};
CLI_Command_Definition_t xCRC = {	/* Structure that defines the "crc" command line command. */
	"crc",
	"crc: 32-bit value (in hex) or an ASCII String\r\n",
	prvCRC,
	1
};

CLI_Command_Definition_t xResume = {	/* Structure that defines the "resume" command line command. */
	"resume",
	"resume: Resumes the task associated with the given task name\r\n",
	prvResume,
	1
};
CLI_Command_Definition_t xSuspend = {	/* Structure that defines the "suspend" command line command. */
	"suspend",
	"suspend: Suspends the task associated with the given task name\r\n",
	prvSuspend,
	1
};
CLI_Command_Definition_t xTracking = {	/* Structure that defines the "tracking" command line command. */
	"tracking",
	"tracking: Toggles tracking based on input ""on"" or ""off""\r\n",
	prvTracking,
	1
};
CLI_Command_Definition_t xHamenc = {	/* Structure that defines the "hamenc" command line command. */
	"hamenc",
	"hamenc: Hamming encoded value of a 8 bit Hex value\r\n",
	prvHamenc,
	1
};
CLI_Command_Definition_t xHamdec = {	/* Structure that defines the "hamdec" command line command. */
	"hamdec",
	"hamdec: Hamming decoded value of a 16bit Hex or Decimal value\r\n",
	prvHamdec,
	1
};
CLI_Command_Definition_t xTop = {	/* Structure that defines the "top" command line command. */
	"top",
	"top: List of the current tasks running in the format of NAME | NUMBER | PRIORITY | STATE | RUNNING TIME\r\n",
	prvTop,
	0
};
CLI_Command_Definition_t xAcc = {	/* Structure that defines the "acc" command line command. */
	"acc",
	"acc: ""raw"" displays raw X,Y,Z Accelerometer values and ""pl"" the orientation of the accelerometer\r\n",
	prvAcc,
	1
};
CLI_Command_Definition_t xLaser = {	/* Structure that defines the "laser" command line command. */
	"laser",
	"laser: Change laser to specified on or off state\r\n",
	prvLaserCommand,
	1
};

CLI_Command_Definition_t xPan = {	/* Structure that defines the "pan" command line command. */
	"pan",
	"pan: Adjust pan servo by increments of 5 degrees using ""left"" or ""right"" or to a given angle\r\n",
	prvPanCommand,
	1
};

CLI_Command_Definition_t xTilt = {	/* Structure that defines the "tilt" command line command. */
	"tilt",
	"tilt: Adjust tilt servo by increments of 5 degrees using ""up"" or ""down"" or to a given angle\r\n",
	prvTiltCommand,
	1
};

CLI_Command_Definition_t xBox = {	/* Structure that defines the "box" command line command. */
	"box",
	"box: draws a 10cm by 10cm box with laser\r\n",
	prvBoxCommand,
	0
};
