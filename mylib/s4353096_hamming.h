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

uint16_t hamming_byte_encoder(uint8_t input);
