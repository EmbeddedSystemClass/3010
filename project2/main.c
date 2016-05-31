/**
  ******************************************************************************
  * @file    milestone/main.c
  * @author  Steffen Mitchell
  * @date    04022015
  * @brief   Main file for milestone.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
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
/* My Lib Includes*/
#include "s4353096_accelerometer.h"
#include "s4353096_sysmon.h"
#include "s4353096_cli.h"
#include "s4353096_hamming.h"
#include "s4353096_radio.h"
#include "s4353096_rover.h"
#include "s4353096_pantilt.h"
#include "s4353096_lightbar.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
TIM_HandleTypeDef TIM_Init;
void Hardware_init();
int main (void) {
	BRD_init();
	Hardware_init();

	/*Initialise all Semaphores*/
	s4353096_SemaphoreAccRaw = xSemaphoreCreateBinary();
	s4353096_SemaphoreAccPl = xSemaphoreCreateBinary();
	s4353096_SemaphoreTracking = xSemaphoreCreateBinary();
	s4353096_SemaphoreRadioState = xSemaphoreCreateBinary();
	s4353096_SemaphoreGetPassKey = xSemaphoreCreateBinary();
	s4353096_SemaphoreGetSensor = xSemaphoreCreateBinary();
	s4353096_SemaphoreSendMotor = xSemaphoreCreateBinary();
	s4353096_SemaphoreGetTime = xSemaphoreCreateBinary();
	s4353096_SemaphoreRecieveRovers = xSemaphoreCreateBinary();
	s4353096_SemaphoreLaser = xSemaphoreCreateBinary();
	s4353096_SemaphorePanLeft = xSemaphoreCreateBinary();
	s4353096_SemaphorePanRight = xSemaphoreCreateBinary();
	s4353096_SemaphoreTiltUp = xSemaphoreCreateBinary();
	s4353096_SemaphoreTiltDown = xSemaphoreCreateBinary();
	s4353096_SemaphoreBox = xSemaphoreCreateBinary();
	s4353096_SemaphoreCalibrate = xSemaphoreCreateBinary();

	s4353096_QueueRoverTransmit = xQueueCreate(10, sizeof(radio_side_communication));
	s4353096_QueueRoverRecieve = xQueueCreate(10, sizeof(radio_side_communication));

	/*Create All Tasks*/
	xTaskCreate( (void *) &s4353096_TaskAccelerometer, (const signed char *) "s4353096_TaskAccelerometer", mainTASKACC_STACK_SIZE, NULL,  mainTASKACC_PRIORITY, &xHandleAccelerometer);
	xTaskCreate( (void *) &CLI_Task, (const signed char *) "CLI_Task", mainTASKCLI_STACK_SIZE, NULL,  mainTASKCLI_PRIORITY +3, &xHandleCLI);
	xTaskCreate( (void *) &s4353096_TaskRadio, (const signed char *) "s4353096_TaskRadio", mainTASKRADIO_STACK_SIZE, NULL,  mainTASKRADIO_PRIORITY + 3, &xHandleRadio);
	xTaskCreate( (void *) &s4353096_TaskRadioProcessing, (const signed char *) "s4353096_TaskRadioProcessing", mainTASKRADIO_STACK_SIZE, NULL,  mainTASKRADIO_PRIORITY + 3, &xHandleRadioProcessing);
	/*Assign the task handles to their respective string values in an array*/
	SetNameHandle();

	/*Initialise all CLI commands*/
	FreeRTOS_CLIRegisterCommand(&xTop);
	FreeRTOS_CLIRegisterCommand(&xAcc);
	FreeRTOS_CLIRegisterCommand(&xHamenc);
	FreeRTOS_CLIRegisterCommand(&xHamdec);
	FreeRTOS_CLIRegisterCommand(&xTracking);
	FreeRTOS_CLIRegisterCommand(&xResume);
	FreeRTOS_CLIRegisterCommand(&xSuspend);
	FreeRTOS_CLIRegisterCommand(&xCRC);
	FreeRTOS_CLIRegisterCommand(&xGetPassKey);
	FreeRTOS_CLIRegisterCommand(&xGetSensor);
	FreeRTOS_CLIRegisterCommand(&xSendMotor);
	FreeRTOS_CLIRegisterCommand(&xRFChanSet);
	FreeRTOS_CLIRegisterCommand(&xGetTime);
	FreeRTOS_CLIRegisterCommand(&xForward);
	FreeRTOS_CLIRegisterCommand(&xRecieveRovers);
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xPan);
	FreeRTOS_CLIRegisterCommand(&xTilt);
	FreeRTOS_CLIRegisterCommand(&xBox);
	FreeRTOS_CLIRegisterCommand(&xDisplayCalibrate);
	FreeRTOS_CLIRegisterCommand(&xORBCalibrate );
	FreeRTOS_CLIRegisterCommand(&xDebugSetRoverPosition);
	FreeRTOS_CLIRegisterCommand(&xCalibrateMarkerId);
	FreeRTOS_CLIRegisterCommand(&xAngle);
	FreeRTOS_CLIRegisterCommand(&xReverse);
	FreeRTOS_CLIRegisterCommand(&xCalibrationRover);
	FreeRTOS_CLIRegisterCommand(&xTestDistance);
	FreeRTOS_CLIRegisterCommand(&xDistance);
	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler. */
  return 0;
}

/*Initialise Hardware (i.e Lightbar, Pushbutton, sysmon)*/
void Hardware_init( void ) {

	portDISABLE_INTERRUPTS();	//Disable interrupts
	GPIO_InitTypeDef GPIO_InitStructure;
	BRD_LEDInit();				//Initialise Blue LED
	BRD_LEDOff();				//Turn off Blue LED
	s4353096_sysmon_init();
	rover_init();
	s4353096_lightbar_init();
	s4353096_accelerometer_init();
	s4353096_radio_init();
	calibration_velocity_init();
	s4353096_pantilt_init();
	__LASER_GPIO_CLK();
	//Set up Pin behaviour
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //Output Mode
	GPIO_InitStructure.Pull = GPIO_PULLUP; //Pull up resistor
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST; //Pun latency
	/*GPIO Pins D0-D9 are configured to the above specifications in the space
	bellow*/
	GPIO_InitStructure.Pin = LASER_PIN;
	HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStructure);
	BRD_LEDToggle();
  portENABLE_INTERRUPTS();	//Disable interrupts
}

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	BRD_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
