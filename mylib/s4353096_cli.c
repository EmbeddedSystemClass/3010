/**
  ******************************************************************************
  * @file    mylib/s4353096_cli.c
  * @author  Steffen Mitchell
  * @date    2-May-2016
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

#include "s4353096_pantilt.h"
#include "s4353096_sysmon.h"
#include "s4353096_accelerometer.h"
#include "s4353096_hamming.h"
#include "s4353096_radio.h"
#define ROVERDEFINES 1
#include "s4353096_rover.h"
#include "s4353096_orb.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
struct PanTilt SendPosition;


extern BaseType_t prvWaypoint(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
	if (atoi(pcWriteBuffer) != 0) {
			rover.marker_id = atoi(pcWriteBuffer);
	} else {
			rover.marker_id = 0;
	}
	/*Give Semaphore*/
	xSemaphoreGive(s4353096_SemaphoreWaypoint);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
/*Toggles on and off the follower task*/
extern BaseType_t prvFollower(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

	if (strcmp(pcWriteBuffer,"on") == 0) {
		/*Give Semaphore*/
		xSemaphoreGive(s4353096_SemaphoreFollower);
	} else if (strcmp(pcWriteBuffer,"off") == 0) {
		/*Give Semaphore*/
		xSemaphoreTake(s4353096_SemaphoreFollower, 1);
	} else {

	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

extern BaseType_t prvAccelerometerControl(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

	if (strcmp(pcWriteBuffer,"on") == 0) {
		/*Give Semaphore*/
		xSemaphoreGive(s4353096_SemaphoreAccControl);
	} else if (strcmp(pcWriteBuffer,"off") == 0) {
		/*Give Semaphore*/
		xSemaphoreTake(s4353096_SemaphoreAccControl, 10);
	} else {

	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
/*Calculates and displays the distance of the rover from the starting edge*/
extern BaseType_t prvDistance(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
		calculate_distance_ratios();
		calculate_rover_distance_pos();
		/* Return pdFALSE, as there are no more strings to return */
		/* Only return pdTRUE, if more strings need to be printed */
		return pdFALSE;
}

/*Used to test the rovers speed at a fixed duration to calculate it's velocity*/
extern BaseType_t prvTestDistance(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string1; //Mode selection
	const char *cCmd_string2; //int speed value
	const char *cCmd_string3; //duration
	int speed;
	int duration;
	/* Get parameters from command string */
	cCmd_string3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &lParam_len);
	/* Write command echo output string to write buffer. */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string3, lParam_len);

	/*Grab the duration from the input string*/
	if (atoi(pcWriteBuffer) != 0) {
			duration = atoi(pcWriteBuffer);
	} else {
			duration = 0;
	}
	/* Get parameters from command string */
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string2, lParam_len);

	/*Grab the speed value from the input string*/
	if (atoi(pcWriteBuffer) != 0) {
			speed = atoi(pcWriteBuffer);
	} else {
			speed = 0;
	}

	/* Get parameters from command string */
	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string1, lParam_len);

	/*Determine the mode set by the input string*/
	if(strcmp(pcWriteBuffer, "forward") == 0) {
		Calibrate.motor_payload[0] = (speed*Calibrate.motor_left_forward)/100;
		Calibrate.motor_payload[1] = (speed*Calibrate.motor_right_forward)/100;
		Calibrate.motor_payload[2] = (0x06 << 4) | (FORWARDRIGHT ^ (FORWARDLEFT)); //run for 3 seconds
		send_rover_packet(Calibrate.motor_payload, 0x32);

	} else if(strcmp(pcWriteBuffer, "reverse") == 0) {
		Calibrate.motor_payload[0] = (speed*Calibrate.motor_left_reverse)/100;
		Calibrate.motor_payload[1] = (speed*Calibrate.motor_right_reverse)/100;
		Calibrate.motor_payload[2] = (0x06 << 4) | (BACKWARDRIGHT ^ (BACKWARDLEFT)); //run for 3 seconds
		send_rover_packet(Calibrate.motor_payload, 0x32);

	} else if(strcmp(pcWriteBuffer, "angleaclock") == 0) {
		Calibrate.motor_payload[0] = (speed*Calibrate.motor_left_reverse)/100;
    Calibrate.motor_payload[1] = (speed*Calibrate.motor_right_forward)/100;
    Calibrate.motor_payload[2] = (0x06 << 4) | (FORWARDRIGHT);
		send_rover_packet(Calibrate.motor_payload, 0x32);

	} else if(strcmp(pcWriteBuffer, "angleclock") == 0) {
		Calibrate.motor_payload[0] = (speed*Calibrate.motor_left_forward);
    Calibrate.motor_payload[1] = (speed*Calibrate.motor_right_reverse);
    Calibrate.motor_payload[2] = (0x06 << 4) | (FORWARDLEFT);
		send_rover_packet(Calibrate.motor_payload, 0x32);

	} else {
		debug_printf("Invalid mode\n");
	}
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Used to calibrate the rover by inputing the speed, duration and distance moved so a velocity can be associated with that speed*/
extern BaseType_t prvCalibrationRover(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string1; //Mode selection
	const char *cCmd_string2; //int speed value
	const char *cCmd_string3; //duration set
	const char *cCmd_string4; //distance moved
	int speed;
	int duration;
	int distance;
	int p1;
	int p2;
	int angle;

	/* Get parameters from command string */
	cCmd_string4 = FreeRTOS_CLIGetParameter(pcCommandString, 4, &lParam_len);

	/*Grab distance moved from input string*/
	if(atoi(cCmd_string4) != 0) {
		distance = atoi(cCmd_string4);
	} else {
		distance = 0;
	}
	angle = distance;

	/* Get parameters from command string */
	cCmd_string3 = FreeRTOS_CLIGetParameter(pcCommandString, 3, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string3, lParam_len);

	/*Grab duration from input string*/
	if (atoi(pcWriteBuffer) != 0) {
			duration = atoi(pcWriteBuffer);
	} else {
			duration = 0;
	}
	p2 = duration;

	/* Get parameters from command string */
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string2, lParam_len);

	/*Grab the speed value from input string*/
	if (atoi(pcWriteBuffer) != 0) {
			speed = atoi(pcWriteBuffer);
	} else {
			speed = 0;
	}
	p1 = speed;

	/* Get parameters from command string */
	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	/* Write command echo output string to write buffer. */
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string1, lParam_len);

	/*Check the mode set in the input string and perform the appropriate calibrations*/
	if(strcmp(pcWriteBuffer, "forward") == 0) {
		calibration_velocity_other_calculation(0, speed, distance, duration);

	} else if(strcmp(pcWriteBuffer, "reverse") == 0) {
			calibration_velocity_other_calculation(1, speed, distance, duration);

	} else if(strcmp(pcWriteBuffer, "angleclock") == 0) {
			Calibrate.angle_clock_velocity = angle/duration;
			Calibrate.angle_clock_speed = speed;
			debug_printf("\nang mul: %d\n", Calibrate.angle_clock_velocity);

	} else if(strcmp(pcWriteBuffer, "angleaclock") == 0) {
			Calibrate.angle_aclock_velocity = angle/duration;
			Calibrate.angle_aclock_speed = speed;
			debug_printf("\nang velocity: %d\n", Calibrate.angle_aclock_velocity);

	} else if(strcmp(pcWriteBuffer, "msforward") == 0) {
			Calibrate.motor_left_forward = p1;
			Calibrate.motor_right_forward = p2;
			debug_printf("\nForward| L: %d R: %d\n", Calibrate.motor_left_forward, Calibrate.motor_right_forward);

	} else if(strcmp(pcWriteBuffer, "msreverse") == 0) {
			Calibrate.motor_left_reverse = p1;
			Calibrate.motor_right_reverse = p2;
			debug_printf("\nReverse| L: %d R: %d\n", Calibrate.motor_left_reverse, Calibrate.motor_right_reverse);

	} else {
		debug_printf("Invalid parameter\n");
	}
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Calibrate the marker_id of the rover and the follower marker*/
extern BaseType_t prvCalibrateMarkerId(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string1;
	const char *cCmd_string2;
	int value;

	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len);

	/*Get the marker_id from the input string*/
	if(atoi(cCmd_string1) != 0) {
		value = atoi(cCmd_string1);
	} else {
		value = 0;
	}

	/* Get parameters from command string */
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string2, lParam_len);

	/*Set the acquired marker_id to the rover or the follower marker*/
	if (strcmp(pcWriteBuffer,"rover") == 0) {
			rover.rover_id = value;
	} else if (strcmp(pcWriteBuffer,"marker") == 0) {
			rover.marker_id = value;
	} else {
		debug_printf("\nInvalid parameters\n");
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*A debug command used to set the rover's current x,y position for testing without a ORB tracking co-ords*/
extern BaseType_t prvDebugSetRoverPosition(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string1;
	const char *cCmd_string2;
	int rover_x;
	int rover_y;

	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len);

	/*Sets the 2nd Param as the rover's current y*/
	if(atoi(cCmd_string1) != 0) {
		rover_y = atoi(cCmd_string1);
	} else {
		rover_y = 0;
	}

	/* Get parameters from command string */
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string2, lParam_len);

	/*Sets the 1st Param as the rover's current x*/
	if (atoi(pcWriteBuffer) != 0) {
		rover_x = atoi(pcWriteBuffer);
	} else {
		rover_x = 0;
	}
	rover.rover_current_x = rover_x;
	rover.rover_current_y = rover_y;
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Calibrates the ORB values for the top and bottom corners of the sandpit*/
extern BaseType_t prvORBCalibrate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
		long lParam_len;
		const char *cCmd_string;

		/* Get parameters from command string */
		cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

		/* Write command echo output string to write buffer. */
		sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

		/*Calibrates the ORB values for the top and bottom corners of the sandpit*/
		if (strcmp(pcWriteBuffer,"tc") == 0) {
	    /*Grab current marker location and set it as top corner of orb*/
			/*Format: [x/y][min\max]*/
			servo_control.orb_c[0][0] = rover.marker_current_x;
			servo_control.orb_c[1][0] = rover.marker_current_y;

	  } else if (strcmp(pcWriteBuffer,"bc") == 0) {
	    /*Grab current marker location and set it as the bottom corner of display*/
			/*Format: [x/y][min\max]*/
			servo_control.orb_c[0][1] = rover.marker_current_x;
			servo_control.orb_c[1][1] = rover.marker_current_y;

		} else {
			debug_printf("\nInvalid parameters\n");
		}
		calculate_display_ratios();

		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		/* Return pdFALSE, as there are no more strings to return */
		/* Only return pdTRUE, if more strings need to be printed */
		return pdFALSE;
}

/*Calibrate the Pan/Tilt values for the top and bottom corners of the display grid, also sets pan/tilt into display calibration mode*/
extern BaseType_t prvDisplayCalibrate(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
		long lParam_len;
		const char *cCmd_string;
		/* Get parameters from command string */
		cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

		/* Write command echo output string to write buffer. */
		sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

		/*Calibrate the Pan/Tilt values for the top and bottom corners of the display grid*/
		if (strcmp(pcWriteBuffer,"tc") == 0) {
	    /*Grab current pan tilt values and set them as the top corner of display*/
			/*Format: [pan/tilt][min\max]*/
			servo_control.display_c[0][0] = servo_control.set_angle_pan;
			servo_control.display_c[1][0] = servo_control.set_angle_tilt;
	  } else if (strcmp(pcWriteBuffer,"bc") == 0) {
	    /*Grab current pan tilt values and set them as the bottom corner of display*/
			/*Format: [pan/tilt][min\max]*/
			servo_control.display_c[0][1] = servo_control.set_angle_pan;
			servo_control.display_c[1][1] = servo_control.set_angle_tilt;

		} else if (strcmp(pcWriteBuffer,"on") == 0) {
			/*Set's the servo's into display calibration mode where pan and tilt can to move the servo and the ORB output is ignored*/
			xSemaphoreGive(s4353096_SemaphoreCalibrate);

		} else if (strcmp(pcWriteBuffer,"off") == 0) {
			/*Turns off display calibration mode*/
			xSemaphoreTake(s4353096_SemaphoreCalibrate, 1);

	  } else {
			debug_printf("\nInvalid parameters\n");
		}
		calculate_display_ratios();
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		/* Return pdFALSE, as there are no more strings to return */
		/* Only return pdTRUE, if more strings need to be printed */
		return pdFALSE;
}

/*A debuging command that sets the radio to constantly recieve commands sent to and from the current set rover*/
extern BaseType_t prvRecieveRovers(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
		long lParam_len;
		const char *cCmd_string;

		/* Get parameters from command string */
		cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

		/* Write command echo output string to write buffer. */
		sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

		/*Turns off and on Reception from rovers by setting the orb rover fsm state accordingly, default mode is to recieve from the orb*/
		if (strcmp(pcWriteBuffer,"on") == 0) {
			radio_vars.orb_rover_fsmcurrentstate = ROVERS_RECIEVE;

		} else if (strcmp(pcWriteBuffer,"off") == 0) {
			radio_vars.orb_rover_fsmcurrentstate = ORB_RECIEVE;

		} else {
			debug_printf("\nInvalid parameters\n");
		}
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		/* Return pdFALSE, as there are no more strings to return */
		/* Only return pdTRUE, if more strings need to be printed */
		return pdFALSE;
}

/*Move the rover to the input angle*/
extern BaseType_t prvAngle(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	int angle;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
	debug_printf("%s", cCmd_string);

	/*Grab the angle from the input string*/
	if(atoi(cCmd_string) != 0) {
		angle = atoi(cCmd_string);
	} else {
		angle = 0;
	}

	/*Negative/anticlockwise angle*/
	/*Currently configured to only use one motor to turn, i.e no reversing*/
	if(atoi(pcWriteBuffer) <  0) {
		angle_duration_calculation(angle, 1); //calculate the duration and speed to best achieve the specified angle*/
		Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT);

	} else {
		/*Clockwise angle*/
		angle_duration_calculation(angle, 0);
		Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDLEFT);
	}

	/*Transmit Angle Here*/
	send_rover_packet(Calibrate.motor_payload, 0x32);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Move the rover the given distance in mm in reverse*/
extern BaseType_t prvReverse(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	int distance;
	uint8_t motor_payload;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	debug_printf("%s", cCmd_string);

	/*Grab the distance from the input string*/
	if(atoi(cCmd_string) != 0) {
		distance = atoi(cCmd_string);

	} else {
		distance = 0;
	}
	/*Designs a motor payload to send to the rover to acheive the wanted distance*/
	direction_duration_calculation_send(distance, 1);
	Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (BACKWARDRIGHT ^ (BACKWARDLEFT));

	/*Perform Transmit Reverse here*/
	send_rover_packet(Calibrate.motor_payload, 0x32);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Move the rover the given distance in mm forward*/
extern BaseType_t prvForward(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
	long lParam_len;
	const char *cCmd_string;
	int distance;
	uint8_t motor_payload;
	TickType_t xDelayMove;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/*Grab the distance from the input string*/
	if(atoi(cCmd_string) != 0) {
		distance = atoi(cCmd_string);
	} else {
		distance = 0;
	}

	direction_duration_calculation_send(distance, 0);
	Calibrate.motor_payload[2] = (Calibrate.motor_payload[2] << 4) | (FORWARDRIGHT ^ (FORWARDLEFT));
	/*Perform Transmit Forward here*/
	send_rover_packet(Calibrate.motor_payload, 0x32);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
/*Set the channel and addresses associated with given Rover's and orbs*/
extern BaseType_t prvRFChanSet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string1;
	const char *cCmd_string2;
	int value;

	cCmd_string1 = FreeRTOS_CLIGetParameter(pcCommandString, 2, &lParam_len);

	/*Grab the ID of the Rover or ORB to set to*/
	if(atoi(cCmd_string1) != 0) {
		value = atoi(cCmd_string1);
	} else {
		value = 0;
	}

	/* Get parameters from command string */
	cCmd_string2 = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat( pcWriteBuffer, cCmd_string2, lParam_len);

	/*Set the associated ID to the retrieved device from the input string, i.e the rover or the orb*/
	if (strcmp(pcWriteBuffer,"rover") == 0) {
		switch(value) {
			case 46:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER46ADDR, sizeof(rover_coms.ROVER46ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER46ADDR, sizeof(rover_coms.ROVER46ADDR));
				radio_vars.s4353096_chan_rover = ROVER46CHAN;
				break;
			case 47:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER47ADDR, sizeof(rover_coms.ROVER47ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER47ADDR, sizeof(rover_coms.ROVER47ADDR));
				radio_vars.s4353096_chan_rover = ROVER47CHAN;
				break;
			case 48:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER48ADDR, sizeof(rover_coms.ROVER48ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER48ADDR, sizeof(rover_coms.ROVER48ADDR));
				radio_vars.s4353096_chan_rover = ROVER48CHAN;
				break;
			case 49:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER49ADDR, sizeof(rover_coms.ROVER49ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER49ADDR, sizeof(rover_coms.ROVER49ADDR));
				radio_vars.s4353096_chan_rover = ROVER49CHAN;
				break;
			case 51:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER51ADDR, sizeof(rover_coms.ROVER51ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER51ADDR, sizeof(rover_coms.ROVER51ADDR));
				radio_vars.s4353096_chan_rover = ROVER51CHAN;
				break;
			case 52:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER52ADDR, sizeof(rover_coms.ROVER52ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER52ADDR, sizeof(rover_coms.ROVER52ADDR));
				radio_vars.s4353096_chan_rover = ROVER52CHAN;
				break;
			case 53:
				memcpy(radio_vars.s4353096_rx_addr_rover, rover_coms.ROVER53ADDR, sizeof(rover_coms.ROVER53ADDR));
				memcpy(radio_vars.s4353096_tx_addr, rover_coms.ROVER53ADDR, sizeof(rover_coms.ROVER53ADDR));
				radio_vars.s4353096_chan_rover = ROVER53CHAN;
				break;
			default:
				debug_printf("\nInvalid Rover ID\n");
				break;
		}

	} else if (strcmp(pcWriteBuffer,"orb") == 0) {
			switch(value) {
				case 1:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB1ADDR, sizeof(ORB1ADDR));
					radio_vars.s4353096_chan_orb = ORB1CHAN;
					break;
				case 2:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB2ADDR, sizeof(ORB2ADDR));
					radio_vars.s4353096_chan_orb = ORB2CHAN;
					break;
				case 3:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB3ADDR, sizeof(ORB3ADDR));
					radio_vars.s4353096_chan_orb = ORB3CHAN;
					break;
				case 4:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB4ADDR, sizeof(ORB4ADDR));
					radio_vars.s4353096_chan_orb = ORB4CHAN;
					break;
				case 5:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB5ADDR, sizeof(ORB5ADDR));
					radio_vars.s4353096_chan_orb = ORB5CHAN;
					break;
				case 6:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB6ADDR, sizeof(ORB6ADDR));
					radio_vars.s4353096_chan_orb = ORB6CHAN;
					break;
				case 7:
					memcpy(radio_vars.s4353096_rx_addr_orb, ORB7ADDR, sizeof(ORB7ADDR));
					radio_vars.s4353096_chan_orb = ORB7CHAN;
					break;
				default:
					debug_printf("\nInvalid ORB ID\n");
					break;
			}

	} else {
		debug_printf("\nInvalid parameters\n");
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Get and print the current system time*/
extern BaseType_t prvGetTime(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	/*Get's the current system time and then prints it*/
	get_system_time();
	debug_printf("Current Time: %d.%02d\n", TaskValues.system_time, TaskValues.system_time_decimal);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Debug command, uses a raw packet specified in the Rover Task and sends this as a Motor Command to the Rover*/
extern BaseType_t prvSendMotor(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreSendMotor != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreSendMotor);		/* Give PB Semaphore from ISR*/
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Begins the process to get the sensor values from the rover*/
extern BaseType_t prvGetSensor(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreGetSensor != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreGetSensor);		/* Give PB Semaphore from ISR*/
	}
	get_system_time();
	debug_printf("Current Time: %d.%02d\n", TaskValues.system_time, 		TaskValues.system_time_decimal);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Begins the process to get a passkey from the rover*/
extern BaseType_t prvGetPassKey(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreGetPassKey != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreGetPassKey);		/* Give PB Semaphore from ISR*/
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Turns the Laser on or off*/
extern BaseType_t prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreLaser != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreLaser);		/* Give PB Semaphore from ISR*/
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Sets the pan servo to the given pan value*/
extern BaseType_t prvPanCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (strcmp(pcWriteBuffer,"left") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphorePanLeft);
  } else if (strcmp(pcWriteBuffer,"right") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphorePanRight);
  } else {

    /*Check if value is a valid integer and if it is send it to the queue*/
    if ((atof(pcWriteBuffer) != 0) || (pcWriteBuffer[0] == '0')) {
      /*Valid integer, send to queue*/
      SendPosition.set_angle_pan = atof(pcWriteBuffer);

      if (s4353096_QueuePan != NULL) {	/* Check if queue exists */

				if( xQueueSendToBack(s4353096_QueuePan, ( void * ) &SendPosition, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
    } else {
      /*Not a valid integer*/
    }
  }

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Sets the pan servo to the given pan value*/
extern BaseType_t prvTiltCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (strcmp(pcWriteBuffer,"up") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreTiltUp);
  } else if (strcmp(pcWriteBuffer,"down") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreTiltDown);
  } else {

    /*Check if value is a valid integer and if it is send it to the queue*/
    if ((atof(pcWriteBuffer) != 0) || (pcWriteBuffer[0] == '0')) {
      /*Valid integer, send to queue*/
      SendPosition.set_angle_tilt = atof(pcWriteBuffer);

      if (s4353096_QueueTilt != NULL) {	/* Check if queue exists */

				if( xQueueSendToBack(s4353096_QueueTilt, ( void * ) &SendPosition, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
    } else {
      /*Not a valid integer*/
    }
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Returns the hamming encoded value of the given input*/
extern BaseType_t prvHamenc(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;
	char *ptr;
	long encode_input_long;
	uint8_t encode_input;
	uint16_t encode_output;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (strlen(pcWriteBuffer) == 1) {
		/*if the input is char*/
		encode_input = pcWriteBuffer[0];
		encode_output = hamming_byte_encoder(encode_input);
		debug_printf("Encoded Value: 0x%x\n",encode_output);

	} else if ((pcWriteBuffer[0] == '0') && (pcWriteBuffer[1] == 'x')) {
		/*If the input is of the format 0x..*/
		encode_input_long = strtoul(pcWriteBuffer, &ptr, 16);
		encode_input = encode_input_long;

		/*Check if Hex value is less than 8bits*/
		if (encode_input_long <= 0xFF) {
			encode_output = hamming_byte_encoder(encode_input);
			debug_printf("Encoded Value: 0x%x\n",encode_output);
		}
	} else {
		/*Check if input is a valid hex byte value*/
		encode_input_long = strtoul(pcWriteBuffer, &ptr, 16);
		encode_input = encode_input_long;

		/*Check if Hex value is less than 8bits*/
		if (encode_input_long <= 0xFF) {
			encode_output = hamming_byte_encoder(encode_input);
			debug_printf("Encoded Value: 0x%x\n",encode_output);
		}
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Returns the hamming decoded value of the given input*/
extern BaseType_t prvHamdec(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;
	long decode_input_long;
	char *ptr;
	uint8_t decode_input_lower;
	uint8_t decode_input_upper;
	uint8_t decode_output;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (strlen(pcWriteBuffer) == 2) {
		/*if the input is char*/
		decode_input_upper = (pcWriteBuffer[0]);
		decode_input_lower = (pcWriteBuffer[1]);
		decode_output = hamming_byte_decoder(decode_input_lower, decode_input_upper);
		debug_printf("Decoded Output: %c\n",decode_output);
	} else if ((pcWriteBuffer[0] == '0') && (pcWriteBuffer[1] == 'x')) {
		/*If the input is a valid Hex*/
		decode_input_long = strtoul(pcWriteBuffer, &ptr, 16);
		decode_input_upper = (decode_input_long) >> 8;
		decode_input_lower = decode_input_long;

		/*Check if Hex value is less than 16bits*/
		if (decode_input_long <= 0xFFFF) {
			decode_output = hamming_byte_decoder(decode_input_lower, decode_input_upper);
			debug_printf("Decoded Output: %x\n",decode_output);
		} else {
			debug_printf("Invalid Parameter Given\n");
		}
	} else {
		/*Check if input is a valid decimal*/
		decode_input_long = strtoul(pcWriteBuffer, &ptr, 10);
		decode_input_upper = (decode_input_long) >> 8;
		decode_input_lower = decode_input_long;

		/*Check if decimal value is less than 16bits*/
		if (decode_input_long <= 0xFFFF) {
			decode_output = hamming_byte_decoder(decode_input_lower, decode_input_upper);
			debug_printf("Decoded Output: %c\n",decode_output);
		} else {
			debug_printf("Invalid Parameter Given\n");
		}
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Used to resume a task which is suspended*/
extern BaseType_t prvResume(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;
	volatile UBaseType_t uxArraySize, x;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
  uxArraySize = uxTaskGetNumberOfTasks();

	/*Increment through the array of task names and find the matching task name to the parameter
		input and resume that task handle*/
	for(x = 0; x < uxArraySize; x++) {

		if(strcmp(TaskValues.TaskNames[x], pcWriteBuffer) == 0) {
			vTaskResume(TaskValues.TaskHandles[x]);

			if (strcmp(pcWriteBuffer,"s4353096_TaskRadio") == 0) {
				xSemaphoreTake(s4353096_SemaphoreRadioState,10);
			}
		}
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Suspends a the given task*/
extern BaseType_t prvSuspend(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;
	volatile UBaseType_t uxArraySize, x;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);
  uxArraySize = uxTaskGetNumberOfTasks();

	/*Increment through the array of task names and find the matching task name to the parameter
		input and suspend that task handle*/
	for(x = 0; x < uxArraySize; x++) {

		if(strcmp(TaskValues.TaskNames[x], pcWriteBuffer) == 0) {
			vTaskSuspend(TaskValues.TaskHandles[x]);

			if (strcmp(pcWriteBuffer,"s4353096_TaskRadio") == 0) {
				xSemaphoreGive(s4353096_SemaphoreRadioState);
			}
		}
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Used to initiate the laser to draw a box*/
extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "\n");
  /* Set the semaphore as available if the semaphore exists*/
  xSemaphoreGive(s4353096_SemaphoreBox);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Used to implement the top command which gives a list of tasks running and associated information on them*/
extern BaseType_t prvTop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	sprintf((char *) pcWriteBuffer, "\n");
	GetTopList();
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Used to get the current accelerometer values*/
extern BaseType_t prvAcc(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

  /* Set the semaphore as available if the semaphore exists*/
	if (strcmp(pcWriteBuffer,"raw") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreAccRaw);
  } else if (strcmp(pcWriteBuffer,"pl") == 0) {
    /*Give Semaphore*/
    xSemaphoreGive(s4353096_SemaphoreAccPl);
  } else {

	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*Enables and disables radio communication*/
extern BaseType_t prvTracking(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {
		long lParam_len;
		const char *cCmd_string;

		/* Get parameters from command string */
		cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

		/* Write command echo output string to write buffer. */
		sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

		if (strcmp(pcWriteBuffer,"on") == 0) {
	    /*Give Semaphore*/
	    xSemaphoreGive(s4353096_SemaphoreTracking);
	  } else if (strcmp(pcWriteBuffer,"off") == 0) {
	    /*Give Semaphore*/
	    xSemaphoreTake(s4353096_SemaphoreTracking, 1);
	  } else {

		}
		/* Return pdFALSE, as there are no more strings to return */
		/* Only return pdTRUE, if more strings need to be printed */
		return pdFALSE;
}

/*Calculates the crc of a given input string*/
extern BaseType_t prvCRC(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;
	char *ptr;
	long crc_input_long;
	int number_crc_updates;
	uint8_t crc_hex_input[4];
	uint16_t crc_output = 0x0000;
	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "%s", cCmd_string);

	if ((pcWriteBuffer[0] == '0') && (pcWriteBuffer[1] == 'x')) {
		/*If the input is of the format 0x..*/
		crc_input_long = strtoul(pcWriteBuffer, &ptr, 16);
		crc_hex_input[0] = ((crc_input_long & 0xFF000000) >> 24);
		crc_hex_input[1] = ((crc_input_long & 0x00FF0000) >> 16);
		crc_hex_input[2] = ((crc_input_long & 0x0000FF00) >> 8);
		crc_hex_input[3] = (crc_input_long & 0x000000FF);
		number_crc_updates = (strlen(pcWriteBuffer) - 2)/2;

		if (number_crc_updates == 4) { /*If input is less/equal than 32-bits*/

			/*Update crc*/
			for (int j = 0; j < number_crc_updates; j++) {
				crc_output = crc_update(crc_output, crc_hex_input[j]);
			}
		}
	} else {
			/*Input is an ASCII String*/
			number_crc_updates = strlen(pcWriteBuffer);

			/*Update crc*/
			for (int j = 0; j < number_crc_updates; j++) {
				crc_output = crc_update(crc_output, pcWriteBuffer[j]);
			}
	}
	debug_printf("CRC Value: %x\n",crc_output);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

/*CLI Task handles all console input and commands*/
void CLI_Task(void) {
	char cRxedChar;
	char cInputString[100];
	int InputIndex = 0;
	char *pcOutputString;
	BaseType_t xReturned;
	short radio_task_state;
	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	/*Main loop for CLI Task*/
	for (;;) {
		/* Receive character */
		cRxedChar = debug_getc();

		/* Process if chacater if not Null */
		if (cRxedChar != '\0') {

			/*If Radio Task is currently Suspended*/
			if (xSemaphoreTake(s4353096_SemaphoreRadioState, 10)) {
				radio_task_state = 0;
				xSemaphoreGive(s4353096_SemaphoreRadioState);
			} else {
				/*Suspend the radio task temporarily until input has been returned*/
				radio_task_state = 1;
				vTaskSuspend(xHandleRadio);
			}

			/*If input is recieved and no input has been previously recieved*/
			if(InputIndex == 0) {
				debug_printf("\n");
			}
			/* Put byte into USB buffer */
			debug_putc(cRxedChar);

			/* Process only if return is received. */
			if (cRxedChar == '\r') {
				//Put new line and transmit buffer
				debug_putc('\n');
				debug_flush();

				/* Put null character in command input string. */
				cInputString[InputIndex] = '\0';

				/*Resume Radio Task if it was resumed before cli input began*/
				if (radio_task_state == 1) {
					vTaskResume(xHandleRadio);
				}
				xReturned = pdTRUE;

				/* Process command input string. */
				while (xReturned != pdFALSE) {
					/* Returns pdFALSE, when all strings have been returned */
					/* Display CLI output string */
					xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
					/*Display input parameter*/
					debug_printf("\n%s\n\r",pcOutputString);
					vTaskDelay(5);	//Must delay between debug_printfs.
				}
				memset(cInputString, 0, sizeof(cInputString));
				InputIndex = 0;

			} else {

				if( cRxedChar == '\r' ) {

					/* Ignore the character. */
				} else if( cRxedChar == 127 ) {

					/* Backspace was pressed.  Erase the last character in the
					 string - if any.*/
					if( InputIndex > 0 ) {
						InputIndex--;
						cInputString[ InputIndex ] = '\0';
						debug_printf("\33[2K\r%s", cInputString);
					} else {
						debug_printf("\33[2K\r%s", cInputString);
					}

				} else {
					debug_flush();		//Transmit USB buffer

					/* A character was entered.  Add it to the string
						 entered so far.  When a \n is entered the complete
						 string will be passed to the command interpreter. */
					if( InputIndex < 100 ) {
						cInputString[ InputIndex ] = cRxedChar;
						InputIndex++;
					}
				}
			}
		}
		vTaskDelay(1);                																	// Mandatory Delay
	}
}
