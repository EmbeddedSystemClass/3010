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

 #define PWM_PAN_TIM TIM2
 #define PWM_PAN_PIN BRD_D2_PIN
 #define PWM_PAN_GPIO_PORT BRD_D2_GPIO_PORT
 #define __PWM_PAN_GPIO_CLK() __BRD_D2_GPIO_CLK()
 #define __PWM_PAN_TIMER_CLK() __TIM2_CLK_ENABLE()
 #define PWM_PAN_GPIO_AF_TIM GPIO_AF1_TIM2
 #define PWM_PAN_TIM_CHANNEL TIM_CHANNEL_4

 #define PWM_TILT_TIM TIM3
 #define PWM_TILT_PIN BRD_D0_PIN
 #define PWM_TILT_GPIO_PORT BRD_D0_GPIO_PORT
 #define __PWM_TILT_GPIO_CLK() __BRD_D0_GPIO_CLK()
 #define __PWM_TILT_TIMER_CLK() __TIM3_CLK_ENABLE()
 #define PWM_TILT_GPIO_AF_TIM GPIO_AF2_TIM3
 #define PWM_TILT_TIM_CHANNEL TIM_CHANNEL_2

 extern void s4353096_pantilt_init(void);
 extern void s4353096_pantilt_angle_write(int type, int angle);
