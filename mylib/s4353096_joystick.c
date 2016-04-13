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

/* Includes */
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_joystick.h"
/* Private typedef */
GPIO_InitTypeDef  GPIO_InitStructure;
ADC_HandleTypeDef AdcHandle1;
ADC_HandleTypeDef AdcHandle2;
ADC_ChannelConfTypeDef AdcChanConfig;
//int count = 0;
int interrupts = 0;
int state = 0;
int pressed_time = 0;
int last_button_state = 0;
int button_state;
int last_Debounce_Time = 0;
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
  /* Configure ADC1 & ADC2 clock */
  __ADC1_CLK_ENABLE();
  __ADC2_CLK_ENABLE();
  /* Configure ADC1 (Note ADC1 is the ADC not the pins)*/
  AdcHandle1.Instance = (ADC_TypeDef *)(ADC1_BASE);						//Use ADC1
  AdcHandle1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV2;	//Set clock prescaler
  AdcHandle1.Init.Resolution            = ADC_RESOLUTION12b;				//Set 12-bit data resolution
  AdcHandle1.Init.ScanConvMode          = DISABLE;
  AdcHandle1.Init.ContinuousConvMode    = DISABLE;
  AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle1.Init.NbrOfDiscConversion   = 0;
  AdcHandle1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;	//No Trigger
  AdcHandle1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_CC1;		//No Trigger
  AdcHandle1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;				//Right align data
  AdcHandle1.Init.NbrOfConversion       = 1;
  AdcHandle1.Init.DMAContinuousRequests = DISABLE;
  AdcHandle1.Init.EOCSelection          = DISABLE;

  HAL_ADC_Init(&AdcHandle1);		//Initialise ADC

  /* Configure ADC Channel */
	AdcChanConfig.Rank         = 1;
  AdcChanConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  AdcChanConfig.Offset       = 0;

  __JOYSTICK_Z_GPIO_CLK();
	//Initialise Interrupt, Priority set to 10
	/*HAL_NVIC_SetPriority(JOYSTICK_Z_EXTI_IRQ, 10, 0);
	GPIO_InitStructure.Pin = JOYSTICK_Z_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(JOYSTICK_Z_GPIO_PORT, &GPIO_InitStructure);
	//Enable external GPIO interrupt and interrupt vector for pin D0
	NVIC_SetVector(JOYSTICK_Z_EXTI_IRQ, (uint32_t)&s4353096_joystick_z_read);
	NVIC_EnableIRQ(JOYSTICK_Z_EXTI_IRQ);*/
}
/*Read X value*/
extern unsigned int s4353096_joystick_x_read(void) {
  unsigned int adc_value;
  AdcChanConfig.Channel = JOYSTICK_X_ADC_CHAN;
  HAL_ADC_ConfigChannel(&AdcHandle1, &AdcChanConfig);
  HAL_ADC_Start(&AdcHandle1); //Start ADC conversion
  /*Wait for ADC Conversion to complete*/
  while (HAL_ADC_PollForConversion(&AdcHandle1, 10) != HAL_OK);
  adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle1));
  //debug_printf("ADC Value: %u\n\r", adc_value);
  return adc_value;
}
/*Read Y value*/
extern unsigned int s4353096_joystick_y_read(void) {
  unsigned int adc_value;
  AdcChanConfig.Channel = JOYSTICK_Y_ADC_CHAN;
  HAL_ADC_ConfigChannel(&AdcHandle1, &AdcChanConfig);
  HAL_ADC_Start(&AdcHandle1); //Start ADC conversion
  /*Wait for ADC Conversion to complete*/
  while (HAL_ADC_PollForConversion(&AdcHandle1, 10) != HAL_OK);
  adc_value = (uint16_t)(HAL_ADC_GetValue(&AdcHandle1));
  //debug_printf("ADC Value y: %u\n\r", adc_value);
  return adc_value;
}
/*Read Z value*/
void s4353096_joystick_z_read(void) {
  //HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);

  /*int reading = HAL_GPIO_ReadPin(JOYSTICK_Z_GPIO_PORT, JOYSTICK_Z_PIN);
  if (reading != last_button_state) {
    last_Debounce_Time = HAL_GetTick()/1000;
  }
  if ((HAL_GetTick()/1000 - last_Debounce_Time) > 10) {
    if (reading != button_state) {
      button_state = reading;
      if (button_state == 1) {
        HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);
        debug_printf("Interrupts:\n");
      }
    }
  } else {
    HAL_GPIO_EXTI_IRQHandler(BRD_A2_PIN);
  }
  last_button_state = reading;*/
}

  //if ((HAL_GetTick()/1000 - last_Debounce_Time) >= 10) {
    //if (reading != button_state) {
      //button_state = reading;
      //interrupts++;
      //debug_printf("Interrupts: %d\n", interrupts);
      //}
    //}
    //state = 0;
  //}

  /*interrupts++;
  if (interrupts == 1) {
  int delay = 10; //Delay Time
  int delay_counter = HAL_GetTick()/1000;
  while (state == 0) {
    while (HAL_GetTick() <= (delay_counter + delay)) {

    }
    if (HAL_GPIO_ReadPin(JOYSTICK_Z_GPIO_PORT, JOYSTICK_Z_PIN) == 1) {
      state++;
      debug_printf("Interrupts: %d\n", interrupts);
    }
  }
  state = 0;
  interrupts = 0;
} else {

}*/
  //interrupts = 0;
  //} else {

  //}

  //debug_printf("Count %d\n",count);
  //if (count == 1) {
  //  joystick_position++;
    //debug_printf("Joystick %d\n",joystick_position);
  //} else if (count == 4) {
  //  count = 0;
  //}
