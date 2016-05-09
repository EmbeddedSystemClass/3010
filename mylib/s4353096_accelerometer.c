/**
  ******************************************************************************
  * @file    ex8_i2c.c
  * @author  MDS
  * @date    02052016
  * @brief   I2C example with the MMA8462Q. Reads and displays the WHO_AM_I reg.
  *			 See the MMA8462 Datasheet (p15)
  ******************************************************************************
  *
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
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static I2C_HandleTypeDef  I2CHandle;
struct Accelerometer Acc_vals;
/* Private function prototypes -----------------------------------------------*/


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
extern void s4353096_TaskAccelerometer(void) {
		S4353096_LA_CHAN0_CLR();
  	for(;;) {
			S4353096_LA_CHAN0_SET();
      /*Read From the X,Y & Z Registers*/
      /*If all semaphores are available, run multi byte read*/
      s4353096_readXYZ();
      /*If not check each semaphore individually*/

			debug_printf("X: %d ,  Y: %d ,  Z: %d \n", Acc_vals.x_coord, Acc_vals.y_coord, Acc_vals.z_coord);
    	BRD_LEDToggle();	//Toggle LED on/off
			vTaskDelay(10);
			S4353096_LA_CHAN0_CLR();
    	vTaskDelay(1);		//Delay for 1s (1000ms)
	}
}
extern void s4353096_readXYZ (void) {
  __HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

  I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

  /*  Wait the START condition has been correctly sent */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

  /* Send Peripheral Device Write address */
  I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

  /* Wait for address to be acknowledged */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
  __HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

  /*Set First read register X_OUT*/
  I2CHandle.Instance->DR = X_OUT_START;

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
  Acc_vals.x_coord = (I2CHandle.Instance->DR) << 4;
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/*Set First read register X_OUT*/
	I2CHandle.Instance->DR = (X_OUT_START + 0x01);

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
	Acc_vals.x_coord = Acc_vals.x_coord ^ ((I2CHandle.Instance->DR) >> 4);
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;






	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/*Set First read register X_OUT*/
	I2CHandle.Instance->DR = Y_OUT_START;

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
	Acc_vals.y_coord = (I2CHandle.Instance->DR) << 4;
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/*Set First read register X_OUT*/
	I2CHandle.Instance->DR = (Y_OUT_START + 0x01);

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
	Acc_vals.y_coord = Acc_vals.y_coord ^ ((I2CHandle.Instance->DR) >> 4);
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;






	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

  I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

  /*  Wait the START condition has been correctly sent */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

  /* Send Peripheral Device Write address */
  I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

  /* Wait for address to be acknowledged */
  while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
  __HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

  /*Set First read register X_OUT*/
  I2CHandle.Instance->DR = Z_OUT_START;

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
  Acc_vals.z_coord = (I2CHandle.Instance->DR) << 4;
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;

	__HAL_I2C_CLEAR_FLAG(&I2CHandle, I2C_FLAG_AF);	//Clear Flags

	I2CHandle.Instance->CR1 |= I2C_CR1_START;	// Generate the START condition

	/*  Wait the START condition has been correctly sent */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_SB) == RESET);

	/* Send Peripheral Device Write address */
	I2CHandle.Instance->DR = __HAL_I2C_7BIT_ADD_WRITE(MMA8452Q_ADDRESS);

	/* Wait for address to be acknowledged */
	while (__HAL_I2C_GET_FLAG(&I2CHandle, I2C_FLAG_ADDR) == RESET);
	__HAL_I2C_CLEAR_ADDRFLAG(&I2CHandle);		//Clear ADDR Flag

	/*Set First read register X_OUT*/
	I2CHandle.Instance->DR = (Z_OUT_START + 0x01);

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
	Acc_vals.z_coord = Acc_vals.z_coord ^ ((I2CHandle.Instance->DR) >> 4);
	I2CHandle.Instance->CR1 &= ~I2C_CR1_ACK;
	I2CHandle.Instance->CR1 |= I2C_CR1_STOP;
	/* Generate NACK */

  /* Generate the STOP condition */

}

/**
  * @brief  Initialise Hardware
  * @param  None
  * @retval None
  */
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
	while (HAL_I2C_GetState(&I2CHandle) != HAL_I2C_STATE_READY);

}
