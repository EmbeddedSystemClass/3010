/**
 ******************************************************************************
 * @file    mylib/s4353096_pantilt.h
 * @author  Steffen Mitchell - 43530960
 * @date    16032016
 * @brief   Servo Pan and Tilt peripheral driver
 ******************************************************************************
 *     EXTERNAL FUNCTIONS
 ******************************************************************************
 * s4353096_pantilt_init() - Initialise servo (GPIO, PWM, Timer, etc)
 * s4353096_pantilt_angle(type, angle) - Write the pan or tilt servo to an angle
 ******************************************************************************
 */
// #ifndef S4353096_PANTILT_H
// #define S4353096_PANTILT_H

 #define PWM_PAN_TIM TIM4
 #define PWM_PAN_PIN BRD_D6_PIN
 #define PWM_PAN_GPIO_PORT BRD_D6_GPIO_PORT
 #define __PWM_PAN_GPIO_CLK() __BRD_D6_GPIO_CLK()
 #define __PWM_PAN_TIMER_CLK() __TIM4_CLK_ENABLE()
 #define PWM_PAN_GPIO_AF_TIM GPIO_AF2_TIM4
 #define PWM_PAN_TIM_CHANNEL TIM_CHANNEL_4

 #define PWM_TILT_TIM TIM4
 #define PWM_TILT_PIN BRD_D5_PIN
 #define PWM_TILT_GPIO_PORT BRD_D5_GPIO_PORT
 #define __PWM_TILT_GPIO_CLK() __BRD_D5_GPIO_CLK()
 #define __PWM_TILT_TIMER_CLK() __TIM4_CLK_ENABLE()
 #define PWM_TILT_GPIO_AF_TIM GPIO_AF2_TIM4
 #define PWM_TILT_TIM_CHANNEL TIM_CHANNEL_3

 #define PANTILT_IR_TIM TIM2
 #define __PANTILT_IR_TIMER_CLK() __TIM2_CLK_ENABLE()
 #define PANTILT_TIM_IRQn TIM2_IRQn

 extern void s4353096_pantilt_init(void);
 extern void s4353096_pantilt_angle_write(int type, int angle);
 void s4353096_pantilt_irqhandler(void);
