/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c
  * @author  MDS
  * @date    04022015
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED.
  ******************************************************************************
  *
  */

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Task Priorities ------------------------------------------------------------*/
#define mainLA_CHAN0TASK1_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainLA_CHAN1TASK2_PRIORITY					( tskIDLE_PRIORITY + 2 )
#define mainLA_CHAN2TASK3_PRIORITY					( tskIDLE_PRIORITY + 2 )
/* Task Stack Allocations -----------------------------------------------------*/
#define mainLA_CHAN0TASK1_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLA_CHAN1TASK2_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define mainLA_CHAN2TASK3_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )

/*Define associated BRD defines for GPIO ports to system monitor channels*/
#define LA_CHAN0_PIN BRD_A3_PIN
#define LA_CHAN0_GPIO_PORT BRD_A3_GPIO_PORT
#define __LA_CHAN0_GPIO_CLK() __BRD_A3_GPIO_CLK()
#define LA_CHAN0_EXTI_IRQ BRD_A3_EXTI_IRQ

#define LA_CHAN1_PIN BRD_A4_PIN
#define LA_CHAN1_GPIO_PORT BRD_A4_GPIO_PORT
#define __LA_CHAN1_GPIO_CLK() __BRD_A4_GPIO_CLK()
#define LA_CHAN1_EXTI_IRQ BRD_A4_EXTI_IRQ

#define LA_CHAN2_PIN BRD_A5_PIN
#define LA_CHAN2_GPIO_PORT BRD_A5_GPIO_PORT
#define __LA_CHAN2_GPIO_CLK() __BRD_A5_GPIO_CLK()
#define LA_CHAN2_EXTI_IRQ BRD_A5_EXTI_IRQ
/*Define CLR and SET for system monitor pins*/
#define S4353096_LA_CHAN0_CLR() {HAL_GPIO_WritePin(LA_CHAN0_GPIO_PORT, LA_CHAN0_PIN, 0);}
#define S4353096_LA_CHAN0_SET() {HAL_GPIO_WritePin(LA_CHAN0_GPIO_PORT, LA_CHAN0_PIN, 1);}
#define S4353096_LA_CHAN1_CLR() {HAL_GPIO_WritePin(LA_CHAN1_GPIO_PORT, LA_CHAN1_PIN, 0);}
#define S4353096_LA_CHAN1_SET() {HAL_GPIO_WritePin(LA_CHAN1_GPIO_PORT, LA_CHAN1_PIN, 1);}
#define S4353096_LA_CHAN2_CLR() {HAL_GPIO_WritePin(LA_CHAN2_GPIO_PORT, LA_CHAN2_PIN, 0);}
#define S4353096_LA_CHAN2_SET() {HAL_GPIO_WritePin(LA_CHAN2_GPIO_PORT, LA_CHAN2_PIN, 1);}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Task1_Task(void);
void Task2_Task(void);
void Task3_Task(void);
void s4353096_sysmon_init(void);
