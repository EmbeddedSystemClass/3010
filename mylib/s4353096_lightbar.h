/**   
 ******************************************************************************   
 * @file    mylib/sxxxxxx_ledbar.h  
 * @author  MyName – MyStudent ID   
 * @date    03032015   
 * @brief   LED Light Bar peripheral driver   
 *	     REFERENCE: LEDLightBar_datasheet.pdf   
 *
 *			NOTE: REPLACE sxxxxxx with your student login.
 ******************************************************************************   
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * sxxxxxx_ledbar_init() – intialise LED Light BAR
 * sxxxxxx_ledbar_set() – set LED Light BAR value
 ******************************************************************************   
 */

#ifndef SXXXXXX_LIGHTBAR_H
#define SXXXXXX_LIGHTBAR_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External function prototypes -----------------------------------------------*/

extern void sxxxxxx_lightbar_init(void);
extern void sxxxxxx_lightbar_write(unsigned short value);
#endif

