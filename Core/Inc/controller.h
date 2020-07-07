
/* Controller.h
	* Author: Phuong_Nguyen
	* Creat Date: 15/03/19
*/

#ifndef CONTROLLE_H_
#define CONTROLLE_H_

// configtion flash Addrees
#define Flash_page1 0x08000000;


//Configtion input

#define Button_CLR_Port 				GPIOA
#define Input_water_Status_Port			GPIOA
#define Pump_Status_Port				GPIOA
#define Button_CLR_Pin					GPIO_PIN_0
#define Input_water_Pin					GPIO_PIN_2
#define Pump_Status_Pin					GPIO_PIN_1

// Configtion output

#define Led_PP_Run_Port					GPIOA
#define Led_UDF_Run_Port				GPIOA
#define Led_PP2_Run_Port				GPIOB
#define Led_Ro_Run_Port					GPIOA
#define Led_PP_Erro_Port				GPIOA
#define Led_UDF_Erro_Port				GPIOA
#define Led_PP2_Erro_Port				GPIOA
#define Led_Ro_Erro_Port				GPIOA


#define Led_PP_Run_Pin					GPIO_PIN_4
#define Led_UDF_Run_Pin					GPIO_PIN_6
#define Led_PP2_Run_Pin					GPIO_PIN_1
#define Led_Ro_Run_Pin					GPIO_PIN_10
#define Led_PP_Erro_Pin					GPIO_PIN_3
#define Led_UDF_Erro_Pin				GPIO_PIN_5
#define Led_PP2_Erro_Pin				GPIO_PIN_7
#define Led_Ro_Erro_Pin					GPIO_PIN_9

#define PP_Check_Time					10800
#define UDF_Check_Time					21600
#define PP2_Check_Time					32400
#define RO_Check_Time					54000
#define	Machine_check_time				300 // phut


#endif // CONTROLLE_H_

