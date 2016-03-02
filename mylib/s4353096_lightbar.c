/**   
 ******************************************************************************   
 * @file    mylib/sxxxxxx_ledbar.c    
 * @author  MyName – MyStudent ID   
 * @date    03032016   
 * @brief   LED Light Bar peripheral driver   
 *	     REFERENCE: LEDLightBar_datasheet.pdf   
 *
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * sxxxxxx_lightbar_init() – intialise LED Light BAR
 * sxxxxxx_lightbar_write() – set LED Light BAR value
 ******************************************************************************   
 */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_lightbar.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


void lightbar_seg_set(int segment, unsigned char segment_value) {

	/*
		Turn segment on (segment_value = 1) or off (segment_value = 0)
	
     */

}

/**
  * @brief  Initialise LEDBar.
  * @param  None
  * @retval None
  */
extern void sxxxxxx_lightbar_init(void) {

	/* Configure the GPIO_D0 pin
	
	 	.... 

		Configure the GPIO_D9 pin 
    */
	
}

/**
  * @brief  Set the LED Bar GPIO pins high or low, depending on the bit of ‘value’
  *         i.e. value bit 0 is 1 – LED Bar 0 on
  *          value bit 1 is 1 – LED BAR 1 on
  *
  * @param  value
  * @retval None
  */
extern void sxxxxxx_lightbar_write(unsigned short value) {

	/* Use bit shifts (<< or >>) and bit masks (1 << bit_index) to determine if a bit is set

	   e.g. The following pseudo code checks if bit 0 of value is 1.
			if ((value & (1 << 0)) == (1 << 0))	{
				Turn on LED BAR Segment 0.
			}
		*/


}

