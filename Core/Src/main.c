/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "controller.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
extern uint8_t CLR_Button_flag;
extern uint8_t Pump_Status_flag;
extern uint8_t Input_Water_flag;
extern uint8_t CLR_PP1_flag;
extern uint8_t CLR_UDF_flag;
extern uint8_t CLR_PP2_flag;
extern uint8_t CLR_Ro_flag;
extern uint8_t machine_erro_flag;
extern uint8_t machine_out_timer1_flag;
extern uint8_t machine_out_timer2_flag;
extern uint8_t PP_out_timer_flag;
extern uint8_t UDF_out_timer_flag;
extern uint8_t PP2_out_timer_flag;
extern uint8_t Ro_out_timer_flag;
extern uint8_t machine_out_flag;
extern uint8_t Writte_data_fillter_flag;
extern uint8_t Led_tank_full_on_flag;
extern uint8_t Led_tank_full_off_flag;
extern uint8_t led_water_input_on_flag;
extern uint8_t led_water_input_off_flag;
extern uint8_t time_on_Input_Water_flag;
extern uint8_t time_on_Machine_out_flag;
extern uint8_t time_on_PP_flag;
extern uint8_t time_on_UDF_flag;
extern uint8_t time_on_PP2_flag;
extern uint8_t time_on_Ro_flag;
extern uint8_t led_fillter_flag;
uint8_t lock_led_erro;
uint8_t led_fan_action_cnt;

extern uint8_t Clr_Button_trigger;
extern uint8_t time_on_PP_out_trigger;
extern uint8_t time_on_UDF_out_trigger;
extern uint8_t time_on_PP2_out_trigger;
extern uint8_t time_on_Ro_out_trigger;
extern uint8_t dislay_erro_lock_trigger;
extern uint8_t	dislay_erro_lock_flag;
extern uint8_t watch_Dog_Trigger;
extern uint8_t watch_Dog_Flag;

uint32_t RO_time;
uint32_t PP2_time;
uint32_t PP1_time;
uint32_t UDF_time;
uint32_t Machine_time ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

void CLR_data(void);
void Dislay_erro(void);
void Writte_Flash(uint32_t, uint32_t);
uint32_t read_Flash(uint32_t );

void fillter_check(void);
void increase_time(uint8_t*);
void led_couter();
void led_fan_action(uint8_t*);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void increase_time(uint8_t* flag){
	if (*flag) {
		(RO_time)++;
		(PP2_time)++;
		(UDF_time)++;
		(PP1_time)++;
		(Machine_time)++;
		*flag=0;
	}
}

void Writte_flash(uint32_t start_Adress, uint32_t End_Adress){
	HAL_FLASH_Unlock();
			FLASH_EraseInitTypeDef EraseInitStruct;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInitStruct.PageAddress = start_Adress;
			EraseInitStruct.NbPages = (End_Adress - start_Adress) / FLASH_PAGE_SIZE;
			uint32_t PAGEError = 0;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress, PP1_time);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 4, UDF_time);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 8, PP2_time);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_Adress + 12, RO_time);
		HAL_FLASH_Lock();
}
uint32_t read_Flash(uint32_t addr){
	uint32_t Flash_data;
	Flash_data = *(uint32_t*) addr;
	return Flash_data;
}
void write_data_fillter(){
	if(Writte_data_fillter_flag){
		if((!HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin))){
			Writte_flash(0x08007000,0x08007C00);
			Writte_data_fillter_flag = 0;
		}
	}
}
void tank_full_led(){
	if(Led_tank_full_on_flag){
		if(!PP_out_timer_flag){
			HAL_GPIO_WritePin(Led_PP_Run_Port, Led_PP_Run_Pin, 0);
		}else{
			HAL_GPIO_WritePin(Led_PP_Run_Port, Led_PP_Run_Pin, 1);
		}
		if (!UDF_out_timer_flag) {
			HAL_GPIO_WritePin(Led_UDF_Run_Port, Led_UDF_Run_Pin, 0);
		}else {
			HAL_GPIO_WritePin(Led_UDF_Run_Port, Led_UDF_Run_Pin, 1);
			}
		if (!PP2_out_timer_flag) {
			HAL_GPIO_WritePin(Led_PP2_Run_Port, Led_PP2_Run_Pin, 0);
		}else {
			HAL_GPIO_WritePin(Led_PP2_Run_Port, Led_PP2_Run_Pin, 1);
			}
		if (!Ro_out_timer_flag) {
			HAL_GPIO_WritePin(Led_Ro_Run_Port, Led_Ro_Run_Pin, 0);
		}else {
			HAL_GPIO_WritePin(Led_Ro_Run_Port, Led_Ro_Run_Pin, 1);
			}
	}
	if(Led_tank_full_off_flag){
		HAL_GPIO_WritePin(Led_PP_Run_Port, Led_PP_Run_Pin, 1);
		HAL_GPIO_WritePin(Led_UDF_Run_Port, Led_UDF_Run_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Run_Port, Led_PP2_Run_Pin, 1);
		HAL_GPIO_WritePin(Led_Ro_Run_Port, Led_Ro_Run_Pin, 1);
	}

}
void CLR_data(){
	if (CLR_Button_flag&&CLR_PP1_flag) {
		// clear value
			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
			CLR_PP1_flag=0;
			CLR_Button_flag=0;
			PP_out_timer_flag=0;
			PP1_time = 0 ;
			Writte_flash(0x08007000,0x08007C00);
		}

	if (CLR_Button_flag&&CLR_UDF_flag){
			HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
			CLR_UDF_flag = 0;
			CLR_Button_flag=0;
			UDF_out_timer_flag=0;
			UDF_time = 0;
			Writte_flash(0x08007000,0x08007C00);
		}
	if (CLR_Button_flag&&CLR_PP2_flag){
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
			CLR_PP2_flag=0;
			CLR_Button_flag=0;
			PP2_out_timer_flag=0;
			PP2_time = 0;
			Writte_flash(0x08007000,0x08007C00);
		}
	if (CLR_Button_flag&&CLR_Ro_flag){
			HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
			CLR_Ro_flag=0;
			CLR_Button_flag=0;
			Ro_out_timer_flag=0;
			RO_time =0;
			Writte_flash(0x08007000,0x08007C00);
		}

}


void Dislay_erro(){
	if(Input_Water_flag){
		if(led_water_input_on_flag){
			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		}
		if(led_water_input_off_flag){
			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 0);
		}
		if(time_on_Input_Water_flag){
			Input_Water_flag = led_water_input_off_flag = led_water_input_on_flag = time_on_Input_Water_flag = 0;
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
		}
	}


	if (machine_out_flag&&(!Input_Water_flag)&&!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag)){
		if (machine_out_timer1_flag){
			HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
		}
		if (machine_out_timer2_flag){
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
			HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 0);
		}
		if (time_on_Machine_out_flag){
			machine_out_flag = machine_out_timer1_flag = machine_out_timer2_flag = time_on_Machine_out_flag =0;
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
			HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
		}
	}

	if (PP_out_timer_flag&&(!Input_Water_flag)&&(!machine_out_flag)&&(!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag))) {
		time_on_PP_out_trigger =1;
			HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_PP_Run_Port, Led_PP_Run_Pin, 1);
		if(time_on_PP_flag){

			PP_out_timer_flag  = 0;
			time_on_PP_flag =0;
			time_on_PP_out_trigger =0;
		}
	}

	if (UDF_out_timer_flag&&(!Input_Water_flag)&(!machine_out_flag)&&(!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag))){
		time_on_UDF_out_trigger = 1;
			HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_UDF_Run_Port, Led_UDF_Run_Pin, 1);
		if(time_on_UDF_flag){

			UDF_out_timer_flag  = 0;
			time_on_UDF_flag = 0;
			time_on_UDF_out_trigger = 0;

		}
	}

	if (PP2_out_timer_flag&&(!Input_Water_flag)&&(!machine_out_flag)&&(!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag))) {
		time_on_PP2_out_trigger =1;
			HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_PP2_Run_Port, Led_PP2_Run_Pin, 1);
		if(time_on_PP2_flag){

			 PP2_out_timer_flag = 0;
			 time_on_PP2_flag = 0;
			 time_on_PP2_out_trigger =0;

		}
	}

	if (Ro_out_timer_flag&&(!Input_Water_flag)&&(!machine_out_flag)&&(!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag))) {
			time_on_Ro_out_trigger =1;
			HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 0);
			HAL_GPIO_WritePin(Led_Ro_Run_Port, Led_Ro_Run_Pin, 1);
		if(time_on_Ro_flag){

			Ro_out_timer_flag = 0;
			time_on_Ro_flag = 0;
			time_on_Ro_out_trigger =0;

		}
	}
	if (!(Ro_out_timer_flag &PP2_out_timer_flag && UDF_out_timer_flag && PP_out_timer_flag&& machine_out_flag&& Input_Water_flag)&&(!(CLR_Ro_flag||CLR_PP2_flag||CLR_UDF_flag||CLR_PP1_flag))) {
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
		dislay_erro_lock_trigger =1 ;
		if (dislay_erro_lock_flag){
			dislay_erro_lock_trigger =0;
			dislay_erro_lock_flag= 0;
		}
	}

}

void fillter_check (){
	if (((!(HAL_GPIO_ReadPin(Input_water_Status_Port, Input_water_Pin))&(!lock_led_erro)))){
		Input_Water_flag = 1;
	}


	if ((Machine_time >= Machine_check_time)&(!lock_led_erro)){
			machine_out_flag = 1;
			if((!HAL_GPIO_ReadPin(Pump_Status_Port, Pump_Status_Pin))){
				Machine_time = 0;
				machine_out_flag = 0;
			}
		}

	if ((PP1_time >= PP_Check_Time)&(!lock_led_erro)) {
		PP_out_timer_flag =1;
	}
	if ((UDF_time >= UDF_Check_Time)&(!lock_led_erro)) {
		UDF_out_timer_flag = 1;
	}
	if ((PP2_time >= PP2_Check_Time) &(!lock_led_erro)){
		PP2_out_timer_flag = 1;
	}
	if ((RO_time >= RO_Check_Time)&(!lock_led_erro)){
		Ro_out_timer_flag = 1;
	}


	if(Input_Water_flag||machine_out_flag||PP_out_timer_flag||UDF_out_timer_flag||PP2_out_timer_flag||Ro_out_timer_flag||dislay_erro_lock_trigger){
		lock_led_erro = 1;
	}else{
		lock_led_erro = 0;

	}

}

void led_fan_action(uint8_t* led_fan_flag){
	if(*led_fan_flag){
	(led_fan_action_cnt++);
	*led_fan_flag =0;
	if(led_fan_action_cnt == 1){
		HAL_GPIO_TogglePin(Led_PP_Run_Port, Led_PP_Run_Pin);
	}
	if(led_fan_action_cnt == 2){
			HAL_GPIO_TogglePin(Led_UDF_Run_Port, Led_UDF_Run_Pin);
		}
	if(led_fan_action_cnt == 3){

				HAL_GPIO_TogglePin(Led_PP2_Run_Port, Led_PP2_Run_Pin);
			}
	if(led_fan_action_cnt == 4){

		HAL_GPIO_TogglePin(Led_Ro_Run_Port,Led_Ro_Run_Pin);

	}
	if(led_fan_action_cnt >=5 ){
		led_fan_action_cnt =0;
		}

	}
}

void led_couter(){

	if(CLR_PP1_flag&(!CLR_Button_flag)){
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 0);
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
	}
	if(CLR_UDF_flag&(!CLR_Button_flag)){
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 0);
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
	}
	if(CLR_PP2_flag&(!CLR_Button_flag)){
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 0);
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
	}
	if(CLR_Ro_flag&(!CLR_Button_flag)){
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 0);
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
	}
	if(Clr_Button_trigger&!(CLR_PP2_flag||CLR_Ro_flag||CLR_UDF_flag||CLR_PP1_flag)){
		HAL_GPIO_WritePin(Led_UDF_Erro_Port, Led_UDF_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP2_Erro_Port, Led_PP2_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_PP_Erro_Port, Led_PP_Erro_Pin, 1);
		HAL_GPIO_WritePin(Led_Ro_Erro_Port, Led_Ro_Erro_Pin, 1);
		Clr_Button_trigger = 0;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  	HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim14);
    PP1_time = read_Flash(0x08007000);
    UDF_time = read_Flash(0x08007000 + 4);
    PP2_time = read_Flash(0x08007000 + 8);
    RO_time = read_Flash(0x08007000 + 12);
    watch_Dog_Trigger = 1;
//	IWDG->KR = 0xAAAA; // Writing 0xAAAA in the Key register prevents watchdog reset
//  	 IWDG->KR = 0xCCCC; // Start the independent watchdog timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  increase_time(&Pump_Status_flag);
	  tank_full_led();
	  led_couter();
	  CLR_data();
	  fillter_check();
	  Dislay_erro();
	  led_fan_action(&led_fillter_flag);
	  write_data_fillter();
//	  if(watch_Dog_Flag){
//	 	  HAL_IWDG_Refresh(&hiwdg);
//	 	  watch_Dog_Flag =0;
//	 	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 800;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8000;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6 
                           PA7 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
