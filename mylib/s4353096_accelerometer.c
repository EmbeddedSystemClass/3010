/**
  ******************************************************************************
  * @file    mylib/s4353096_accelerometer.c
  * @author  Steffen Mitchell
  * @date    02052016
  * @brief   Accelerometer Task file
  ******************************************************************************
  * EXTERNAL FUNCTIONS
  ******************************************************************************
  * s4353096_TaskAccelerometer() - The function for the Accelerometer Task
  *
  * s4353096_read_acc_register(int reg) - Returns the read value of the given register
  *
  * s4353096_readPLBF() - Prints the current orientation of the accelerometer read from the PL_STATUS register
  *
  * s4353096_readXYZ() - Reads the raw values for X,Y,Z and assigns them to associated variables
  *
  * s4353096_write_acc_register(uint8_t reg, uint8_t value) - Write value to the given register
  *
  * s4353096_accelerometer_init() - Initialise I2C, GPIO PINS. Enable PL orientation and XYZ Raw values and place the
  *                                 Accelerometer into wake state
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "stm32f4xx_hal_conf.h"
#include "debug_printf.h"
#include "s4353096_accelerometer.h"
#include "s4353096_sysmon.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MMA8452Q_ADDRESS	0x1D << 1
#define X_OUT_START 0x01
#define Y_OUT_START 0x03
#define Z_OUT_START 0x05
#define PL_STATUS   0x10
#define SYS_MOD     0x0B
#define CTRL_REG_1  0x2A
#define PL_CFG      0x11
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef  I2CHandle;
struct Accelerometer Acc_vals;
/* Private function prototypes -----------------------------------------------*/


/*The main function for the Accelerometer Task*/
extern void s4353096_TaskAccelerometer(void) {
    /*Main loop for Accelerometer Task*/
  	for(;;) {


			if (s4353096_SemaphoreAccRaw != NULL) {	/* Check if semaphore exists */
				/* See if we can obtain the PB semaphore. If the semaphore is not available
							wait 10 ticks to see if it becomes free. */

			  if( xSemaphoreTake( s4353096_SemaphoreAccRaw, 10 ) == pdTRUE ) {
          /*Read Raw 12-bit values from the X,Y & Z Registers*/
					s4353096_readXYZ();
					debug_printf("X: %hd ,  Y: %hd ,  Z: %hd \n", Acc_vals.x_coord, Acc_vals.y_coord, Acc_vals.z_coord);

        }
			}
      if (s4353096_SemaphoreAccPl != NULL) {	/* Check if semaphore exists */
				/* See if we can obtain the PB semaphore. If the semaphore is not available
							wait 10 ticks to see if it becomes free. */

			  if( xSemaphoreTake( s4353096_SemaphoreAccPl, 10 ) == pdTRUE ) {
          /*Read and print orientation of accelerometer*/
					s4353096_readPLBF();
        }
			}
    	BRD_LEDToggle();	//Toggle LED on/off
			vTaskDelay(10);
    	vTaskDelay(1);		//Delay for 1s (1000ms)
	}
}
/*Returns the read value of the given register*/
extern uint8_t s4353096_read_acc_register(int reg) {
  uint8_t read_value;

  __HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/*Set read register*/
	I2CHandle.Instance->DR = reg;

	/* Wait until register Address byte is transmitted */
	while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));

	/* Generate the START condition, again */
	I2CHandle.Instance->CR1 |= I2C_CR1_START;

	/* Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Read Address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_READ(MMA8452Q_ADDRESS);

	/* Wait address is acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/* Wait to read X_Value_MSB */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_RXNE) == RESET);
	/* Read received MSB X value */
	read_value = (I2CHandle.Instance->DR);
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;
  return read_value;
}
/*Prints the current orientation of the accelerometer read from the PL_STATUS register*/
extern void s4353096_readPLBF (void) {
  Acc_vals.pl_status = s4353096_read_acc_register(PL_STATUS);
  Acc_vals.land_port = (Acc_vals.pl_status & 0x06) >> 1; //Extract the bits associated with portrait/landscape orientation
  Acc_vals.back_front = Acc_vals.pl_status & 0x01; //Extract the bits associated with front/back orientation

  /*Represents the value of land_port as a landscape/portrait orientation*/
  switch(Acc_vals.land_port) {
    case 0x00:
      debug_printf("|");
      break;
    case 0x01:
      debug_printf("_");
      break;
    case 0x02:
      debug_printf(">");
      break;
    case 0x03:
      debug_printf("<");
      break;
    default:
      break;
  }

  /*Represents the value of back_front as a front/back orientation*/
  switch (Acc_vals.back_front) {
    case 0x00:
      debug_printf(" +\n");
      break;
    case 0x01:
      debug_printf(" -\n");
      break;
    default:
      break;
  }
}
/*Reads the raw values for X,Y,Z and assigns them to associated variables*/
extern void s4353096_readXYZ (void) {
  /*Read X MSB & LSB Registers & combine bytes to form correct signed int value*/
  Acc_vals.x_coord_MSB = s4353096_read_acc_register(X_OUT_START);
  Acc_vals.x_coord_LSB = s4353096_read_acc_register(X_OUT_START + 1);
  Acc_vals.x_coord = (Acc_vals.x_coord_MSB) << 4 ^ (Acc_vals.x_coord_LSB >> 4);

  /*Read Y MSB & LSB Registers & combine bytes to form correct signed int value*/
  Acc_vals.y_coord_MSB = s4353096_read_acc_register(Y_OUT_START);
  Acc_vals.y_coord_LSB = s4353096_read_acc_register(Y_OUT_START + 1);
  Acc_vals.y_coord = (Acc_vals.y_coord_MSB) << 4 ^ (Acc_vals.y_coord_LSB >> 4);

  /*Read Z MSB & LSB Registers & combine bytes to form correct signed int value*/
  Acc_vals.z_coord_MSB = s4353096_read_acc_register(Z_OUT_START);
  Acc_vals.z_coord_LSB = s4353096_read_acc_register(Z_OUT_START + 1);
  Acc_vals.z_coord = (Acc_vals.z_coord_MSB) << 4 ^ (Acc_vals.z_coord_LSB >> 4);
}
/*Write value to the given register*/
extern void s4353096_write_acc_register(uint8_t reg, uint8_t value) {
  /*Place the Accelerometer into wake state*/
	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

  I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

  /*  Wait the START condition has been correctly sent */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

  /* Send Peripheral Device Write address */
  I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

  /* Wait for address to be acknowledged */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
  __HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

  /*Set First write register X_O*/
  I2CHandle.Instance->DR = reg;

  /* Wait until register Address byte is transmitted */
  while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));

  I2CHandle.Instance->DR = value;

  /* Wait to Write X_Value_MSB */
	while ((__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_TXE) == RESET) && (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_BTF) == RESET));
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

}
/*Initialise I2C, GPIO PINS. Enable PL orientation and XYZ Raw values and place the
Accelerometer into wake state*/
extern void s4353096_accelerometer_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable GPIO clocks */
	__BRD_SCL_GPIO_CLK();
	__BRD_SDA_GPIO_CLK();

	/* Enable I2C CLK */
	__BRD_I2C_CLK();

	/******************************************************/
	/* IMPORTANT NOTE: SCL Must be Initialised BEFORE SDA */
	/******************************************************/
	/* enable GPIO pins for I2C */
	GPIO_InitStructure.Pin = BRD_SCL_PIN;			//SCL
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SCL_AF;
	HAL_GPIO_Init(BRD_SCL_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = BRD_SDA_PIN;			//SDA
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FAST;
	GPIO_InitStructure.Pull = GPIO_PULLUP ;
	GPIO_InitStructure.Alternate = BRD_SDA_AF;
	HAL_GPIO_Init(BRD_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the I2C peripheral */
	I2CHandle.Instance = BRD_I2C;
	I2CHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;                               // 7bit addressing mode
	I2CHandle.Init.ClockSpeed      = 1000000;						// Transmission Frequency
	I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2CHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2CHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2CHandle.Init.OwnAddress1     = 0;
	I2CHandle.Init.OwnAddress2     = 0;

	/* Initialise and Start the I2C peripheral */
	HAL_I2C_Init(&I2CHandle);

	/* -> Wait for the end of the transfer */
	/* Before starting a new communication transfer, you need to check the current
	* state of the peripheral; if it’s busy you need to wait for the end of current
	* transfer before starting a new one.
	* For simplicity reasons, this example is just waiting till the end of the
	* transfer, but application may perform other tasks while transfer operation
	* is ongoing.
	*/
  /*Wait until I2C is ready*/
	while (HAL_I2C_GetState(&I2CHandle) != HAL_I2C_STATE_READY);

  /*Enable PLBF*/
  s4353096_write_acc_register(PL_CFG, (1 << 6));
  /*Place the Accelerometer into wake state*/
	s4353096_write_acc_register(SYS_MOD, 0x01);
  /*Place Full Scale Resolution into active state*/
  s4353096_write_acc_register(CTRL_REG_1, 0x01);
}
