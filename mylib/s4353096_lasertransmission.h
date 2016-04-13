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
#define SQR_WAVE_GEN_1_PIN BRD_D1_PIN
#define SQR_WAVE_GEN_1_GPIO_PORT BRD_D1_GPIO_PORT
#define __SQR_WAVE_GEN_1_GPIO_CLK() __BRD_D1_GPIO_CLK()
#define SQR_WAVE_GEN_1_EXTI_IRQ BRD_D1_EXTI_IRQ
