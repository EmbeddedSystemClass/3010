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

/* Includes */
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_joystick.h"
/* Private typedef */
GPIO_InitTypeDef  GPIO_InitStructure;
ADC_HandleTypeDef AdcHandle;
ADC_ChannelConfTypeDef AdcChanConfig;
/*Initialise Joystick Pins*/
extern void s4353096_joystick_init(void) {
  /*Configure GPIO pins A5-A3 for joystick*/
  __JOYSTICK_X_GPIO_CLK();
  __JOYSTICK_Y_GPIO_CLK();
  __JOYSTICK_Z_GPIO_CLK();

  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  /*Set Joystick Pins as anaog pins*/
  GPIO_InitStructure.Pin = JOYSTICK_X_PIN;
  HAL_GPIO_Init(JOYSTICK_X_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = JOYSTICK_Y_PIN;
  HAL_GPIO_Init(JOYSTICK_Y_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.Pin = JOYSTICK_Z_PIN;
  HAL_GPIO_Init(JOYSTICK_Z_GPIO_PORT, &GPIO_InitStructure);
  /* Configure ADC1 clock */
  __ADC1_CLK_ENABLE();
  /* Configure ADC1 (Note ADC1 is the ADC not the pins)*/
  AdcHandle.Instance = (ADC_TypeDef *)(ADC1_BASE);						//Use ADC1
  AdcHandle.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
  AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
  AdcHandle.Init.ScanConvMode          = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion   = 0;
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
  AdcHandle.Init.NbrOfConversion       = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection          = DISABLE;

  HAL_ADC_Init(&AdcHandle);		//Initialise ADC
/* Configure ADC Channel for pin x */
  AdcChanConfig.Channel = BRD_A5_ADC_CHAN;							//Use AO pin
	AdcChanConfig.Rank         = 1;
  AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  AdcChanConfig.Offset       = 0;
  HAL_ADC_ConfigChannel(&AdcHandle, &AdcChanConfig);
}
/*Read X value*/
extern int s4353096_joystick_x_read(void) {
  int adc_value;
  HAL_ADC_Start(&AdcHandle); //Start ADC conversion
  /*Wait for ADC Conversion to complete*/
  while (HAL_ADCPollForConversion(&AdcHandle, 10) != HAL_OK);
  adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle));
  return adc_value;
}
/*Read Y value*/
extern int s4353096_joystick_y_read(void) {

}
/*Read Z value*/
extern int s4353096_joystick_z_read(void) {

}
