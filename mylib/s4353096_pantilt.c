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
#include <stdlib.h>
#include <math.h>
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"

#include "s4353096_sysmon.h"
#include "s4353096_pantilt.h"
#include "s4353096_rover.h"
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
  s4353096_QueueBox = xQueueCreate(10, sizeof(servo_control));
  /*Create Task*/
  xTaskCreate( (void *) &s4353096_TaskPanTilt, (const signed char *) "s4353096_TaskPanTilt", mainTASKPANTILT_STACK_SIZE, NULL,  mainTASKPANTILT_PRIORITY+2, &xHandlePanTilt );
  xTaskCreate( (void *) &s4353096_TaskBox, (const signed char *) "s4353096_TaskBox", mainTASKBOX_STACK_SIZE, NULL,  mainTASKBOX_PRIORITY+2, &xHandleBox );
}

/*Sets the angle of pan or tilt through pwm with type 1 = pan and type 0 = tilt*/
extern void s4353096_pantilt_angle_write(int type, float angle) {
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

/*Calculates the ratio for converting between ORB Co-ords and Display Pan/Tilt angles*/
extern void calculate_display_ratios(void) {
  int ratio_p;
  int ratio_tilt;
  servo_control.ratio_pan = fabsf(servo_control.display_c[0][1] - servo_control.display_c[0][0])/fabsf(servo_control.orb_c[0][1] - servo_control.orb_c[0][0]);
  servo_control.ratio_tilt = fabsf(servo_control.display_c[1][1] - servo_control.display_c[1][0])/fabsf(servo_control.orb_c[1][1] - servo_control.orb_c[1][0]);
  ratio_p = servo_control.ratio_pan;
  ratio_tilt = servo_control.ratio_tilt;
  debug_printf("\nPanR: %d, TiltR %d", servo_control.ratio_pan, servo_control.ratio_tilt );
}
/*Calculates the pan and tilt angles for the current rover position*/
extern void calculate_rover_display_pos(void) {
  servo_control.set_angle_pan = (-1*rover.rover_current_x*servo_control.ratio_pan) + servo_control.display_c[0][0];
  servo_control.set_angle_tilt = (rover.rover_current_y*servo_control.ratio_tilt) + servo_control.display_c[1][0];
  s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
  s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
}


extern void s4353096_TaskPanTilt(void) {
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

  /*Main loop for Task PanTilt*/
  for (;;) {
    if (s4353096_SemaphoreLaser != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
      wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake( s4353096_SemaphoreLaser, 1 ) == pdTRUE ) {
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

      if( xSemaphoreTake( s4353096_SemaphorePanLeft, 1 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Increment the pan angle by -5degrees*/
        servo_control.set_angle_pan += 5;
        s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);

      }
    }
    if (s4353096_SemaphorePanRight != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
      wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake( s4353096_SemaphorePanRight, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Increment the pan angle by -5degrees*/
        servo_control.set_angle_pan -= 5;
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
    if (s4353096_SemaphoreTiltDown != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
      wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake( s4353096_SemaphoreTiltDown, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Increment the pan angle by -5degrees*/
        servo_control.set_angle_tilt += 5;
        s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);

      }
    }
    if (s4353096_SemaphoreTiltUp != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
      wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake( s4353096_SemaphoreTiltUp, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Increment the pan angle by -5degrees*/
        servo_control.set_angle_tilt -= 5;
        s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);

      }
    }
    if (s4353096_QueueTilt != NULL) {	/* Check if queue exists */
      /* Check for item received - block atmost for 10 ticks */
      if (xQueueReceive(s4353096_QueueTilt, &Recieve_PT, 10 )) {
        servo_control.set_angle_tilt = Recieve_PT.set_angle_tilt;
        s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
      }
    }
    if (s4353096_QueueBox != NULL) {	/* Check if queue exists */
      /* Check for item received - block atmost for 10 ticks */

      if (xQueueReceive(s4353096_QueueBox, &Recieve_PT, 10 )) {
        servo_control.set_angle_tilt = Recieve_PT.set_angle_tilt;
        s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
        vTaskDelay(4000);
        servo_control.set_angle_pan = Recieve_PT.set_angle_pan;
        s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
        vTaskDelay(4000);

        if (xQueueReceive(s4353096_QueueBox, &Recieve_PT, 10 )) {
          servo_control.set_angle_tilt = Recieve_PT.set_angle_tilt;
          s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
          vTaskDelay(4000);
          servo_control.set_angle_pan = Recieve_PT.set_angle_pan;
          s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
          vTaskDelay(4000);

          if (xQueueReceive(s4353096_QueueBox, &Recieve_PT, 10 )) {
            servo_control.set_angle_tilt = Recieve_PT.set_angle_tilt;
            s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
            vTaskDelay(4000);
            servo_control.set_angle_pan = Recieve_PT.set_angle_pan;
            s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
            vTaskDelay(4000);

            if (xQueueReceive(s4353096_QueueBox, &Recieve_PT, 10 )) {
              servo_control.set_angle_tilt = Recieve_PT.set_angle_tilt;
              s4353096_pantilt_angle_write(0, servo_control.set_angle_tilt);
              vTaskDelay(4000);
              servo_control.set_angle_pan = Recieve_PT.set_angle_pan;
              s4353096_pantilt_angle_write(1, servo_control.set_angle_pan);
              vTaskDelay(4000);
            }
          }
        }
      }
    }

    /*Set current rover position here*/
    if (s4353096_SemaphoreCalibrate != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
      wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake(s4353096_SemaphoreCalibrate, 1 ) == pdTRUE ) {
        xSemaphoreGive(s4353096_SemaphoreCalibrate);
      } else {
        calculate_rover_display_pos();
      }
    }
    vTaskDelay(1);                																	// Mandatory Delay
  }
}

/*Task to draw a box*/
extern void s4353096_TaskBox(void) {
	S4353096_LA_CHAN1_CLR();
  struct PanTilt MakeBox;
  int current_angle_pan;
  int current_angle_tilt;
  TickType_t xLastWakeTime1;
  const TickType_t xFrequency1 = 50 / portTICK_PERIOD_MS;; //1000 represents 1 second delay
  xLastWakeTime1 = xTaskGetTickCount();

  /*Main loop for task box*/
	for (;;) {
		S4353096_LA_CHAN1_SET();      //Set LA Channel 0
    if (s4353096_SemaphoreBox != NULL) {	/* Check if semaphore exists */
      /* See if we can obtain the PB semaphore. If the semaphore is not available
            wait 10 ticks to see if it becomes free. */

      if( xSemaphoreTake( s4353096_SemaphoreBox, 10 ) == pdTRUE ) {
        /* We were able to obtain the semaphore and can now access the shared resource. */
        /*Increment the pan angle by -5degrees*/
        HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, 1);
        current_angle_pan = servo_control.set_angle_pan;
        current_angle_tilt = servo_control.set_angle_tilt;
        MakeBox.set_angle_tilt = current_angle_tilt + (10);
        MakeBox.set_angle_pan = current_angle_pan - (1.15);
        if (s4353096_QueueBox != NULL) {	/* Check if queue exists */
  				if( xQueueSendToBack(s4353096_QueueBox, ( void * ) &MakeBox, ( portTickType ) 10 ) != pdPASS ) {
  					debug_printf("Failed to post the message, after 10 ticks.\n\r");
  				}
  			}
        MakeBox.set_angle_tilt = current_angle_tilt - (10*0.725); /*Tilt Up*/
        MakeBox.set_angle_pan = current_angle_pan + (9*1.15);
        if (s4353096_QueueBox != NULL) {	/* Check if queue exists */
  				if( xQueueSendToBack(s4353096_QueueBox, ( void * ) &MakeBox, ( portTickType ) 10 ) != pdPASS ) {
  					debug_printf("Failed to post the message, after 10 ticks.\n\r");
  				}
  			}
        MakeBox.set_angle_tilt = current_angle_tilt + (10*0.725); /*Tilt Down*/
        MakeBox.set_angle_pan = current_angle_pan;
        if (s4353096_QueueBox != NULL) {	/* Check if queue exists */
  				if( xQueueSendToBack(s4353096_QueueBox, ( void * ) &MakeBox, ( portTickType ) 10 ) != pdPASS ) {
  					debug_printf("Failed to post the message, after 10 ticks.\n\r");
  				}
  			}
        MakeBox.set_angle_tilt = current_angle_tilt ; /*Tilt Back to center*/
        MakeBox.set_angle_pan = current_angle_pan;
        if (s4353096_QueueBox != NULL) {	/* Check if queue exists */
  				if( xQueueSendToBack(s4353096_QueueBox, ( void * ) &MakeBox, ( portTickType ) 10 ) != pdPASS ) {
  					debug_printf("Failed to post the message, after 10 ticks.\n\r");
  				}
  			}
      }
    }
	}
  vTaskDelayUntil( &xLastWakeTime1, xFrequency1 );                //Extra Task Delay of 3ms
  S4353096_LA_CHAN1_CLR();
  vTaskDelay(1);
}
