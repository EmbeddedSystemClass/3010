/**
 ******************************************************************************
 * @file    mylib/s4353096_ledbar.h
 * @author  Steffen Mitchell - 43530960
 * @date    03032015
 * @brief   LED Light Bar peripheral driver
 *	     REFERENCE: LEDLightBar_datasheet.pdf
 *
 *			NOTE: REPLACE sxxxxxx with your student login.
 ******************************************************************************
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4353096_ledbar_init() – intialise LED Light BAR
 * s4353096_ledbar_set() – set LED Light BAR value
 ******************************************************************************
 */

#ifndef S4353096_LIGHTBAR_H
#define S4353096_LIGHTBAR_H

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External function prototypes -----------------------------------------------*/

extern void s4353096_lightbar_init(void);
extern void s4353096_lightbar_write(unsigned short value);
#endif
