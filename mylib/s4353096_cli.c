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
#include "s4353096_sysmon.h"
#include "s4353096_accelerometer.h"
#include "s4353096_hamming.h"
#include "s4353096_radio.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
struct PanTilt SendPosition;


extern BaseType_t prvRFChanSetCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	long lParam_len;
	const char *cCmd_string;

	/* Get parameters from command string */
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);


	if ((atoi(cCmd_string) != 0)
  /* Set the semaphore as available if the semaphore exists*/
	if (s4353096_SemaphoreLaser != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(s4353096_SemaphoreLaser);		/* Give PB Semaphore from ISR*/
	}
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
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
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
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
			debug_printf("Decoded Output: %c\n",decode_output);
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
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
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

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
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

	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}

extern BaseType_t prvBoxCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	/* Write command echo output string to write buffer. */
	sprintf((char *) pcWriteBuffer, "\n");
  /* Set the semaphore as available if the semaphore exists*/
  xSemaphoreGive(s4353096_SemaphoreBox);
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
extern BaseType_t prvTop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString) {

	sprintf((char *) pcWriteBuffer, "\n");
	GetTopList();
	/* Return pdFALSE, as there are no more strings to return */
	/* Only return pdTRUE, if more strings need to be printed */
	return pdFALSE;
}
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
