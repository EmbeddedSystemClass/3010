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

#include "s4353096_sysmon.h"
#include "s4353096_pantilt.h"
//#include "s4353096_cli.h"
/* Private typedef */
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_OC_InitTypeDef PWMConfig;
TIM_HandleTypeDef TIM_Init;
static uint16_t PrescalerValue = 0;
struct PanTilt *pantilt;
struct PanTilt servo_control;
/*Initialise pantilt GPIO, Timer, PWM */
extern void s4353096_pantilt_init(void) {
  /* Enable the PWM Pin Clocks */
  __PWM_PAN_GPIO_CLK();
  __PWM_TILT_GPIO_CLK();
  /* Configure the PWM Pan pin with Timer output */
  GPIO_InitStructure.Pin = PWM_PAN_PIN;				//Pin
  GPIO_InitStructure.Mode =GPIO_MODE_AF_PP; 		//Set mode to be output alternate
  GPIO_InitStructure.Pull = GPIO_NOPULL;			//Enable Pull up, down or no pull resister
  GPIO_InitStructure.Speed = GPIO_SPEED_FAST;			//Pin latency
  GPIO_InitStructure.Alternate = PWM_PAN_GPIO_AF_TIM;	//Set alternate function to be timer pan
  HAL_GPIO_Init(PWM_PAN_GPIO_PORT, &GPIO_InitStructure);	//Initialise Pin
  /* Configure the PWM Tilt pin with Timer output */
  GPIO_InitStructure.Pin = PWM_TILT_PIN;				//Pin
  GPIO_InitStructure.Alternate = PWM_TILT_GPIO_AF_TIM;	//Set alternate function to be timer tilt
  HAL_GPIO_Init(PWM_TILT_GPIO_PORT, &GPIO_InitStructure);
  /* Enable clock for pan and tilt timer's  */
  __PWM_PAN_TIMER_CLK();
  /* Compute the prescaler value. SystemCoreClock = 168000000 - set for 50Khz clock */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

  /* Configure Timer settings */
  TIM_Init.Instance = PWM_PAN_TIM;					//Enable Timer 2
  TIM_Init.Init.Period = (500000/1000)*20;			//Set for 200ms (5Hz) period
  TIM_Init.Init.Prescaler = PrescalerValue;	//Set presale value
  TIM_Init.Init.ClockDivision = 0;			//Set clock division
  TIM_Init.Init.RepetitionCounter = 0; 		// Set Reload Value
  TIM_Init.Init.CounterMode = TIM_COUNTERMODE_UP;	//Set timer to count up.

  /* PWM Mode configuration for Channel 2 - set pulse width*/
  PWMConfig.OCMode			 = TIM_OCMODE_PWM1;	//Set PWM MODE (1 or 2 - NOT CHANNEL)
  PWMConfig.Pulse				= (((2*500000)/10000) * 7.25); //Sets initial servo position to 0 degrees
  PWMConfig.OCPolarity	 = TIM_OCPOLARITY_HIGH;
  PWMConfig.OCNPolarity	= TIM_OCNPOLARITY_HIGH;
  PWMConfig.OCFastMode	 = TIM_OCFAST_DISABLE;
  PWMConfig.OCIdleState	= TIM_OCIDLESTATE_RESET;
  PWMConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  /* Enable PWM for PWM Pan Timer */
  HAL_TIM_PWM_Init(&TIM_Init);
  HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_PAN_TIM_CHANNEL);

  /* Start PWM for Pan*/
  HAL_TIM_PWM_Start(&TIM_Init, PWM_PAN_TIM_CHANNEL);

  /* Configure Timer setting for PWM Tilt */
  TIM_Init.Instance = PWM_TILT_TIM;
  /* Enable PWM for PWM Tilt Timer */
  HAL_TIM_PWM_Init(&TIM_Init);
  HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_TILT_TIM_CHANNEL);

  /* Start PWM for Tilt*/
  HAL_TIM_PWM_Start(&TIM_Init, PWM_TILT_TIM_CHANNEL);
  /*Create Queues*/
  s4353096_QueuePan = xQueueCreate(10, sizeof(servo_control));
  s4353096_QueueTilt = xQueueCreate(10, sizeof(servo_control));
  /*Create Task*/
  xTaskCreate( (void *) &s4353096_TaskPanTilt, (const signed char *) "s4353096_TaskPanTilt", mainTASKPANTILT_STACK_SIZE, NULL,  mainTASKPANTILT_PRIORITY, NULL );
}
/*Sets the angle of pan or tilt through pwm with type 1 = pan and type 0 = tilt*/
extern void s4353096_pantilt_angle_write(int type, int angle) {
  float pwm_pulse_period_percentage;
  float pwm_multiplier = 4.723 * (angle/85.000);
  /*If negative*/
  if ((angle < 0) && (angle > -77)) {
    pwm_pulse_period_percentage = (7.25 - (-1*pwm_multiplier));
    PWMConfig.Pulse				= (((2*500000)/10000) * pwm_pulse_period_percentage);
  } else if ((angle >= 0) && (angle < 77)) {
    pwm_pulse_period_percentage = ((7.25 + pwm_multiplier));
    PWMConfig.Pulse				= (((2*500000)/10000) * pwm_pulse_period_percentage);
  } else {

  }
  /*Type 1 == Pan */
  if (type == 1) {
    TIM_Init.Instance = PWM_PAN_TIM;
    /* Enable PWM for PWM Pan Timer */
    HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_PAN_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&TIM_Init, PWM_PAN_TIM_CHANNEL);
  } else if (type == 0) { /*If Type 0 == Tilt */
    TIM_Init.Instance = PWM_TILT_TIM;
    HAL_TIM_PWM_ConfigChannel(&TIM_Init, &PWMConfig, PWM_TILT_TIM_CHANNEL);
    HAL_TIM_PWM_Start(&TIM_Init, PWM_TILT_TIM_CHANNEL);
  } else {

  }
}
/*Checks the angle to see if the setting value has passed the allowed value*/
extern void s4353096_terminal_angle_check (void) {
  switch (pantilt->set_angle_pan) {
    case 77:
      pantilt->set_angle_pan = 76;
      break;
    case -77:
      pantilt->set_angle_pan = -76;
      break;
    default:
      break;
  }
  switch (pantilt->set_angle_tilt) {
    case 77:
      pantilt->set_angle_tilt = 76;
      break;
    case -77:
      pantilt->set_angle_tilt = -76;
      break;
    default:
      break;
  }
}
void s4353096_TaskPanTilt(void) {
  S4353096_LA_CHAN0_CLR();        //Clear LA Channel 0
	/*Set up delay until delay time*/
  //TickType_t xLastWakeTime1;
  //const TickType_t xFrequency1 = 1000 / portTICK_PERIOD_MS;; //1000 represents 1 second delay
  //xLastWakeTime1 = xTaskGetTickCount();
  //Initialise Timer Left Struct
  struct PanTilt Recieve_PT;
	servo_control.set_angle_pan = 0;
  servo_control.set_angle_tilt = 0;
  servo_control.laser_state = 0;
  char cRxedChar;
	char cInputString[100];
	int InputIndex = 0;
	char *pcOutputString;
	BaseType_t xReturned;
	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();
	for (;;) {
    S4353096_LA_CHAN0_SET();      //Set LA Channel 0
    /*Do Stuff Here, this is the loop*/
    /* Receive character */
    cRxedChar = debug_getc();

    /* Process if chacater if not Null */
    if (cRxedChar != '\0') {

      /* Put byte into USB buffer */
      debug_putc(cRxedChar);

      /* Process only if return is received. */
      if (cRxedChar == '\r') {

        //Put new line and transmit buffer
        debug_putc('\n');
        debug_flush();

        /* Put null character in command input string. */
        cInputString[InputIndex] = '\0';

        xReturned = pdTRUE;
        /* Process command input string. */
        while (xReturned != pdFALSE) {
          /* Returns pdFALSE, when all strings have been returned */
          /* CLIProcessCommand Checks which command was called and runs that function*/
          xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );
          /*Throw semaphores here*/
          if (s4353096_SemaphoreLaser != NULL) {	/* Check if semaphore exists */
      			/* See if we can obtain the PB semaphore. If the semaphore is not available
                 	wait 10 ticks to see if it becomes free. */

      			if( xSemaphoreTake( s4353096_SemaphoreLaser, 10 ) == pdTRUE ) {
              /* We were able to obtain the semaphore and can now access the shared resource. */
              /*Perform Laser command here*/
              /*Check which state was sent*/
              if (strcmp(pcOutputString,"on") == 0) {
                /*Turn on Laser*/
                HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, 1);
                //debug_printf("Y");
              } else if (strcmp(pcOutputString,"off") == 0) {
                /*Turn off Laser*/
                HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, 0);
                //debug_printf("Y");
              }
            }
      		}
          if (s4353096_SemaphorePanLeft != NULL) {	/* Check if semaphore exists */
      			/* See if we can obtain the PB semaphore. If the semaphore is not available
                 	wait 10 ticks to see if it becomes free. */

      			if( xSemaphoreTake( s4353096_SemaphorePanLeft, 10 ) == pdTRUE ) {
              /* We were able to obtain the semaphore and can now access the shared resource. */
              /*Increment the pan angle by -5degrees*/
              servo_control.set_angle_pan -= 5;
              s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);

            }
      		}
          if (s4353096_SemaphorePanRight != NULL) {	/* Check if semaphore exists */
      			/* See if we can obtain the PB semaphore. If the semaphore is not available
                 	wait 10 ticks to see if it becomes free. */

      			if( xSemaphoreTake( s4353096_SemaphorePanRight, 10 ) == pdTRUE ) {
              /* We were able to obtain the semaphore and can now access the shared resource. */
              /*Increment the pan angle by -5degrees*/
              servo_control.set_angle_pan += 5;
              s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);

            }
      		}
          if (s4353096_QueuePan != NULL) {	/* Check if queue exists */
            /* Check for item received - block atmost for 10 ticks */
            if (xQueueReceive(s4353096_QueuePan, &Recieve_PT, 10 )) {
              servo_control.set_angle_pan = Recieve_PT.set_angle_pan;
              s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
            }
          }
          /* Display CLI output string */
					debug_printf("%s\n\r",pcOutputString);
				    vTaskDelay(5);	//Must delay between debug_printfs.
        }

        memset(cInputString, 0, sizeof(cInputString));
        InputIndex = 0;

      } else {

        debug_flush();		//Transmit USB buffer

        if( cRxedChar == '\r' ) {

          /* Ignore the character. */
        } else if( cRxedChar == '\b' ) {

          /* Backspace was pressed.  Erase the last character in the
           string - if any.*/
          if( InputIndex > 0 ) {
            InputIndex--;
            cInputString[ InputIndex ] = '\0';
          }

        } else {

          /* A character was entered.  Add it to the string
             entered so far.  When a \n is entered the complete
             string will be passed to the command interpreter. */
          if( InputIndex < 20 ) {
            cInputString[ InputIndex ] = cRxedChar;
            InputIndex++;
          }
        }
      }
    }
    /*Normal Things Here*/
    //vTaskDelayUntil( &xLastWakeTime1, xFrequency1 );                //Extra Task Delay of 3ms
    S4353096_LA_CHAN0_CLR();
    vTaskDelay(1);                																	// Mandatory Delay
  }
}
