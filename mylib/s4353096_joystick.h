/**
 ******************************************************************************
 * @file mylib/s4353096_joystick.c
 * @author Steffen Mitchell - 43530960
 * @date 09032015
 * @brief Joystick peripheral driver
 * REFERENCE:
 ******************************************************************************
 * EXTERNAL FUNCTIONS
 ******************************************************************************
 * sxxxxxx_ledbar_init() – intialise LED Light BAR
 * sxxxxxx_ledbar_set() – set LED Light BAR value
 ******************************************************************************
*/

/*Hash defines for different pins so that it is more convienient to change them
later*/
#define JOYSTICK_X_PIN BRD_A5_PIN
#define JOYSTICK_X_GPIO_PORT BRD_A5_GPIO_PORT
#define __JOYSTICK_X_GPIO_CLK() __BRD_A5_GPIO_CLK()

#define JOYSTICK_Y_PIN BRD_A4_PIN
#define JOYSTICK_Y_GPIO_PORT BRD_A4_GPIO_PORT
#define __JOYSTICK_Y_GPIO_CLK() __BRD_A4_GPIO_CLK()

#define JOYSTICK_Z_PIN BRD_A3_PIN
#define JOYSTICK_Z_GPIO_PORT BRD_A3_GPIO_PORT
#define __JOYSTICK_Z_GPIO_CLK() __BRD_A3_GPIO_CLK()