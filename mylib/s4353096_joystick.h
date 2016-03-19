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
 * s4353096_joystick_init() - Initialise ADC for Joystick
 * s4353096_joystick_x_read() - Read X Joystick value
 * s4353096_joystick_y_read() - Read Y Joystick value
 * s4353096_joystick_z_read() - Read Z Joystick   value
 ******************************************************************************
*/
#ifndef S4353096_JOYSTICK_H
#define S4353096_JOYSTICK_H
/*Hash defines for different pins so that it is more convienient to change them
later*/
#define JOYSTICK_X_PIN BRD_A0_PIN
#define JOYSTICK_X_GPIO_PORT BRD_A0_GPIO_PORT
#define __JOYSTICK_X_GPIO_CLK() __BRD_A0_GPIO_CLK()
#define JOYSTICK_X_ADC_CHAN BRD_A0_ADC_CHAN

#define JOYSTICK_Y_PIN BRD_A1_PIN
#define JOYSTICK_Y_GPIO_PORT BRD_A1_GPIO_PORT
#define __JOYSTICK_Y_GPIO_CLK() __BRD_A1_GPIO_CLK()
#define JOYSTICK_Y_ADC_CHAN BRD_A1_ADC_CHAN

#define JOYSTICK_Z_PIN BRD_A2_PIN
#define JOYSTICK_Z_GPIO_PORT BRD_A2_GPIO_PORT
#define __JOYSTICK_Z_GPIO_CLK() __BRD_A2_GPIO_CLK()
#define JOYSTICK_Z_EXTI_IRQ BRD_A2_EXTI_IRQ
#define JOYSTICK_Z_ADC_CHAN BRD_A2_ADC_CHAN

extern void s4353096_joystick_init(void);
extern unsigned int s4353096_joystick_x_read(void);
extern unsigned int s4353096_joystick_y_read(void);
void s4353096_joystick_z_read(void);
//void exti_joystick_z_interrupt_handler(void);
#endif
