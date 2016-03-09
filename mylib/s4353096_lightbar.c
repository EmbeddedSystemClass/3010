/**
 ******************************************************************************
 * @file    mylib/s4353096_ledbar.c
 * @author  Steffen Mitchell - 43530960
 * @date    03032016
 * @brief   LED Light Bar peripheral driver
 *	     REFERENCE: LEDLightBar_datasheet.pdf
 *
 ******************************************************************************
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4353096_lightbar_init() – intialise LED Light BAR
 * s3242096_lightbar_write() – set LED Light BAR value
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_lightbar.h"

/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef  GPIO_InitStructure;
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/



void lightbar_seg_set(int segment, unsigned char segment_value) {

	/*
		Turn segment on (segment_value = 1) or off (segment_value = 0)

     */
		 if (segment_value == 1 || segment_value == 0) {
		 	switch (segment) {
			 	case 0:
					HAL_GPIO_WritePin(LED_0_GPIO_PORT, LED_0_PIN, segment_value);
					break;
				case 1:
					HAL_GPIO_WritePin(LED_1_GPIO_PORT, LED_1_PIN, segment_value);
					break;
				case 2:
					HAL_GPIO_WritePin(LED_2_GPIO_PORT, LED_2_PIN, segment_value);
					break;
				case 3:
					HAL_GPIO_WritePin(LED_3_GPIO_PORT, LED_0_PIN, segment_value);
					break;
				case 4:
					HAL_GPIO_WritePin(LED_4_GPIO_PORT, LED_4_PIN, segment_value);
					break;
				case 5:
					HAL_GPIO_WritePin(LED_5_GPIO_PORT, LED_5_PIN, segment_value);
					break;
				case 6:
					HAL_GPIO_WritePin(LED_6_GPIO_PORT, LED_6_PIN, segment_value);
					break;
				case 7:
					HAL_GPIO_WritePin(LED_7_GPIO_PORT, LED_7_PIN, segment_value);
					break;
				case 8:
					HAL_GPIO_WritePin(LED_8_GPIO_PORT, LED_8_PIN, segment_value);
					break;
				case 9:
					HAL_GPIO_WritePin(LED_9_GPIO_PORT, LED_9_PIN, segment_value);
					break;
			 default:
			 		break;
			}
		} else {

		}
}

/**
  * @brief  Initialise LEDBar.
  * @param  None
  * @retval None
  */
extern void s4353096_lightbar_init(void) {

	/* Configure the GPIO_D0 pin

	 	....

		Configure the GPIO_D9 pin
    */
		//Enable D0-D9 Clocks


		//int GPIO_Port[10];
		__LED_0_GPIO_CLK();
		__LED_1_GPIO_CLK();
		__LED_2_GPIO_CLK();
		__LED_3_GPIO_CLK();
		__LED_4_GPIO_CLK();
		__LED_5_GPIO_CLK();
		__LED_6_GPIO_CLK();
		__LED_7_GPIO_CLK();
		__LED_8_GPIO_CLK();
		__LED_9_GPIO_CLK();
		//Set up Pin behaviour
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP; //Output Mode
		GPIO_InitStructure.Pull = GPIO_PULLUP; //Pull down resistor
		GPIO_InitStructure.Speed = GPIO_SPEED_FAST; //Pun latency

		/*GPIO Pins D0-D9 are configured to the above specifications in the space
		bellow*/
		GPIO_InitStructure.Pin = LED_0_PIN;
		HAL_GPIO_Init(LED_0_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_1_PIN;
		HAL_GPIO_Init(LED_1_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_2_PIN;
		HAL_GPIO_Init(LED_2_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_3_PIN;
		HAL_GPIO_Init(LED_3_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_4_PIN;
		HAL_GPIO_Init(LED_4_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_5_PIN;
		HAL_GPIO_Init(LED_5_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_6_PIN;
		HAL_GPIO_Init(LED_6_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_7_PIN;
		HAL_GPIO_Init(LED_7_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_8_PIN;
		HAL_GPIO_Init(LED_8_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.Pin = LED_9_PIN;
		HAL_GPIO_Init(LED_9_GPIO_PORT, &GPIO_InitStructure);

}

/**
  * @brief  Set the LED Bar GPIO pins high or low, depending on the bit of ‘value’
  *         i.e. value bit 0 is 1 – LED Bar 0 on
  *          value bit 1 is 1 – LED BAR 1 on
  *
  * @param  value
  * @retval None
  */
extern void s4353096_lightbar_write(unsigned short value) {

	/* Use bit shifts (<< or >>) and bit masks (1 << bit_index) to determine if a bit is set

	   e.g. The following pseudo code checks if bit 0 of value is 1.
			if ((value & (1 << 0)) == (1 << 0))	{
				Turn on LED BAR Segment 0.
			}
		*/
	for (int i=0; i < 10; i++) {
		//debug_printf("i %d\n", i);
		if ((value & (1 << i)) == (1 << i)) {
			//Turn on LED BAR Segment i
			lightbar_seg_set(i, 1);
			//debug_printf("On Segment %d\n\n", i);
		} else if ((value & (1 << i)) == (0 << i)){
			//Turn off LED BAR Segment i
			//debug_printf("OK %d\n", i);
			lightbar_seg_set(i, 0);
		} else {

		}
	}

}
