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
/* Task Priorities ------------------------------------------------------------*/
#define mainLIGHTBARTASK_PRIORITY					( tskIDLE_PRIORITY + 2 )
/* Task Stack Allocations -----------------------------------------------------*/
#define mainLIGHTBARTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

#define s4353096_QueueLightBar
/*The defines bellow define which pins the lightbar is plugged into
  The current configuration is D2-D11*/
#define LED_0_PIN BRD_D2_PIN
#define LED_0_GPIO_PORT BRD_D2_GPIO_PORT
#define __LED_0_GPIO_CLK() __BRD_D2_GPIO_CLK()
#define LED_0_EXTI_IRQ BRD_D2_EXTI_IRQ

#define LED_1_PIN BRD_D3_PIN
#define LED_1_GPIO_PORT BRD_D3_GPIO_PORT
#define __LED_1_GPIO_CLK() __BRD_D3_GPIO_CLK()
#define LED_1_EXTI_IRQ BRD_D3_EXTI_IRQ

#define LED_2_PIN BRD_D4_PIN
#define LED_2_GPIO_PORT BRD_D4_GPIO_PORT
#define __LED_2_GPIO_CLK() __BRD_D4_GPIO_CLK()
#define LED_2_EXTI_IRQ BRD_D4_EXTI_IRQ

#define LED_3_PIN BRD_D5_PIN
#define LED_3_GPIO_PORT BRD_D5_GPIO_PORT
#define __LED_3_GPIO_CLK() __BRD_D5_GPIO_CLK()
#define LED_3_EXTI_IRQ BRD_D5_EXTI_IRQ

#define LED_4_PIN BRD_D6_PIN
#define LED_4_GPIO_PORT BRD_D6_GPIO_PORT
#define __LED_4_GPIO_CLK() __BRD_D6_GPIO_CLK()
#define LED_4_EXTI_IRQ BRD_D6_EXTI_IRQ

#define LED_5_PIN BRD_D7_PIN
#define LED_5_GPIO_PORT BRD_D7_GPIO_PORT
#define __LED_5_GPIO_CLK() __BRD_D7_GPIO_CLK()
#define LED_5_EXTI_IRQ BRD_D7_EXTI_IRQ

#define LED_6_PIN BRD_D8_PIN
#define LED_6_GPIO_PORT BRD_D8_GPIO_PORT
#define __LED_6_GPIO_CLK() __BRD_D8_GPIO_CLK()
#define LED_6_EXTI_IRQ BRD_D8_EXTI_IRQ

#define LED_7_PIN BRD_D9_PIN
#define LED_7_GPIO_PORT BRD_D9_GPIO_PORT
#define __LED_7_GPIO_CLK() __BRD_D9_GPIO_CLK()
#define LED_7_EXTI_IRQ BRD_D9_EXTI_IRQ

#define LED_8_PIN BRD_D10_PIN
#define LED_8_GPIO_PORT BRD_D10_GPIO_PORT
#define __LED_8_GPIO_CLK() __BRD_D10_GPIO_CLK()
#define LED_8_EXTI_IRQ BRD_D10_EXTI_IRQ

#define LED_9_PIN BRD_D11_PIN
#define LED_9_GPIO_PORT BRD_D11_GPIO_PORT
#define __LED_9_GPIO_CLK() __BRD_D11_GPIO_CLK()
#define LED_9_EXTI_IRQ BRD_D11_EXTI_IRQ


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External function prototypes -----------------------------------------------*/

extern void s4353096_lightbar_init(void);
extern void s4353096_lightbar_write(unsigned short value);
#endif
