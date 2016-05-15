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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
struct Accelerometer {
  short x_coord;
  short y_coord;
  short z_coord;
  int coord_status;
};
SemaphoreHandle_t s4353096_SemaphoreAccRaw;		//Used to Specify Acc output
SemaphoreHandle_t s4353096_SemaphoreAccPl;		//Used to control laser
/* Task Priorities ------------------------------------------------------------*/
#define mainTASKACC_PRIORITY					( tskIDLE_PRIORITY + 2 )
/* Task Stack Allocations -----------------------------------------------------*/
#define mainTASKACC_STACK_SIZE		( configMINIMAL_STACK_SIZE * 4 )
extern void GetRunTimeStats(void);
extern void s4353096_readXYZ (void);
extern void s4353096_TaskAccelerometer(void);
extern void s4353096_accelerometer_init(void);
extern short twos_complement_proper (short number);
extern short s4353096_read_acc_register(int reg);
