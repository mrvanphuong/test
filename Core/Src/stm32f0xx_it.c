/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t CLR_Button_flag;
uint8_t Pump_Status_flag;
uint8_t Input_Water_flag;
uint8_t CLR_PP1_flag;
uint8_t CLR_UDF_flag;
uint8_t CLR_PP2_flag;
uint8_t CLR_Ro_flag;
uint8_t machine_out_flag;
uint8_t machine_out_timer1_flag;
uint8_t machine_out_timer2_flag;
uint8_t machine_out_timer_cnt;
uint8_t PP_out_timer_flag;
uint8_t UDF_out_timer_flag;
uint8_t PP2_out_timer_flag;
uint8_t Ro_out_timer_flag;
uint8_t Writte_data_fillter_flag;
uint8_t Led_tank_full_on_flag;
uint8_t Led_tank_full_off_flag;
uint8_t led_water_input_on_flag;
uint8_t led_water_input_off_flag;
uint8_t time_on_PP_flag;
uint8_t time_on_UDF_flag;
uint8_t time_on_PP2_flag;
uint8_t time_on_Ro_flag;
uint8_t time_on_Input_Water_flag;
uint8_t time_on_Machine_out_flag;
uint8_t led_fillter_flag;
uint8_t dislay_erro_lock_trigger;
uint8_t	dislay_erro_lock_flag;
uint8_t watch_Dog_Flag;

static int Clr_button_flag_cnt;
static int led_couter_cnt;
static int Pump_Status_cnt;
static int Writte_data_fillter_cnt;
static int Led_tank_on_off_cnt;
static int Machine_out_time_led_cnt;
static int Input_water_status_led_cnt;
static int time_on_PP_out_cnt;
static int time_on_UDF_out_cnt;
static int time_on_PP2_out_cnt;
static int time_on_Ro_out_cnt;
static int time_on_Input_Water_cnt;
static int time_on_Machine_out_cnt;
static int led_fillter_cnt;
static int dislay_erro_lock_cnt;
static int watch_Dog_cnt;

uint8_t Clr_Button_trigger;

uint8_t time_on_PP_out_trigger;
uint8_t time_on_UDF_out_trigger;
uint8_t time_on_PP2_out_trigger;
uint8_t time_on_Ro_out_trigger;
uint8_t watch_Dog_Trigger;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void timer(uint8_t,uint8_t*,int*,int);
void timer_togger(uint8_t,uint8_t*, uint8_t*,int*,int,int);
void counter(uint8_t, uint8_t*, uint8_t*, uint8_t*, uint8_t*,int* );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
/* USER CODE BEGIN EV */
void timer(uint8_t trigger,uint8_t* flag,int* cnt,int interval){
	  if (trigger) {
		  (*cnt)++;
	  } else {
		  *cnt=0;
	  }
	  if (*cnt==interval){
		  *flag=1;
		  *cnt=0;
	  }
}

void timer_togger(uint8_t trigger,uint8_t* flag_1, uint8_t* flag_2,int* cnt,int interval_1,int interval_2){
	  if (trigger) {
		  (*cnt)++;
	  } else {
		  *cnt=0;
		  *flag_1= 0;
		  *flag_2= 0;
	  }
	  if (*cnt ==interval_1) {
		  *flag_1= 1;
		  *flag_2=0;
	}
	  if (*cnt == interval_2) {
		  *flag_1= 0;
		  *flag_2=1;
		  *cnt=0;
	  }
}

void counter(uint8_t trigger, uint8_t* flag_1, uint8_t* flag_2, uint8_t* flag_3, uint8_t* flag_4, int* cnt){

	if(trigger){
		(*cnt)++;

	}
	if(*cnt==1){
		*flag_1=1;
		*flag_2 = *flag_3 = *flag_4 = 0;
	}
	if(*cnt==2){
		*flag_2=1;
		*flag_1 = *flag_3 = *flag_4 = 0;
	}
	if(*cnt==3){
		*flag_3=1;
		*flag_1 = *flag_2 = *flag_4 = 0;
	}
	if(*cnt==4){
		*flag_4=1;
		*flag_2 = *flag_3 = *flag_1 = 0;
	}
	if(*cnt>4){
		(*cnt)=0;
		*flag_4= *flag_2 = *flag_3 = *flag_1 = 0;

	}
}
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */
   Clr_Button_trigger = 1;
  	  counter(Clr_Button_trigger, &CLR_PP1_flag, &CLR_UDF_flag, &CLR_PP2_flag, &CLR_Ro_flag, &led_couter_cnt);
  	HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
  		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
  			HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
  			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
  			PP_out_timer_flag = UDF_out_timer_flag = PP2_out_timer_flag = Ro_out_timer_flag =0;
  			Input_Water_flag = Machine_out_time_led_cnt = Led_tank_full_on_flag = 0;


  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  	 timer_togger(machine_out_flag&(!Input_Water_flag),&machine_out_timer1_flag, &machine_out_timer2_flag, &Machine_out_time_led_cnt,  80, 160);
     timer_togger(Input_Water_flag, &led_water_input_on_flag, &led_water_input_off_flag, &Input_water_status_led_cnt, 80, 160);
     timer(HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin), &led_fillter_flag, & led_fillter_cnt, 200);
     timer(watch_Dog_Trigger, &watch_Dog_Flag, &watch_Dog_cnt, 2500);
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */
  	timer((HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin)), &Pump_Status_flag, &Pump_Status_cnt, 60);
   	timer((HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin)),&Writte_data_fillter_flag,&Writte_data_fillter_cnt,40);
   	timer((HAL_GPIO_ReadPin(Button_CLR_Port, Button_CLR_Pin)),&CLR_Button_flag,&Clr_button_flag_cnt,3);
   	timer_togger(!(HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin))&&!Clr_Button_trigger	,&Led_tank_full_on_flag	,&Led_tank_full_off_flag, &Led_tank_on_off_cnt, 5, 10);
   	timer(Input_Water_flag,&time_on_Input_Water_flag,&time_on_Input_Water_cnt,4);
   	timer(machine_out_flag,&time_on_Machine_out_flag,&time_on_Machine_out_cnt,4);
   	timer(time_on_PP_out_trigger,&time_on_PP_flag,&time_on_PP_out_cnt,4);
   	timer(time_on_UDF_out_trigger,&time_on_UDF_flag,&time_on_UDF_out_cnt,4);
   	timer(time_on_PP2_out_trigger, &time_on_PP2_flag, &time_on_PP2_out_cnt, 4);
   	timer(time_on_Ro_out_trigger, &time_on_Ro_flag, &time_on_Ro_out_cnt, 4);
   	timer(dislay_erro_lock_trigger,&dislay_erro_lock_flag, &dislay_erro_lock_cnt, 4);

  /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
