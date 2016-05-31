/**
  ******************************************************************************
  * @file    mylib/s4353096_accelerometer.c
  * @author  Steffen Mitchell
  * @date    02052016
  * @brief   Accelerometer Task file
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
  int16_t x_coord;
  int8_t x_coord_MSB;
  uint8_t x_coord_LSB;
  int16_t y_coord;
  int8_t y_coord_MSB;
  uint8_t y_coord_LSB;
  int16_t z_coord;
  int8_t z_coord_MSB;
  uint8_t z_coord_LSB;
  uint8_t pl_status;
  uint8_t land_port;
  uint8_t back_front;
};
SemaphoreHandle_t s4353096_SemaphoreAccRaw;		//Used to Specify Acc output
SemaphoreHandle_t s4353096_SemaphoreAccPl;		//Used to control laser
SemaphoreHandle_t s4353096_SemaphoreAccControl; //Used to control Rover movements
/* Task Priorities ------------------------------------------------------------*/
#define mainTASKACC_PRIORITY					( tskIDLE_PRIORITY + 2 )
/* Task Stack Allocations -----------------------------------------------------*/
#define mainTASKACC_STACK_SIZE		( configMINIMAL_STACK_SIZE * 4 )
extern void GetRunTimeStats(void);
extern void s4353096_readXYZ (void);
extern void s4353096_readPLBF (void);
extern void s4353096_TaskAccelerometer(void);
extern void s4353096_accelerometer_init(void);
extern uint8_t s4353096_read_acc_register(int reg);
extern void s4353096_write_acc_register(uint8_t reg, uint8_t value);
void accelerometer_control(void);
