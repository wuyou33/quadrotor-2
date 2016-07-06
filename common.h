#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define Motor_Range_Min										1300
#define Motor_Range_Max										2300
#define Sub_Motor_Range_AND_RC_Range			200
#define DO_VOT_LO													100

#define RC_Throtte_Min										1100
#define RC_Throtte_Max										1900
#define RC_Medium													1500
#define RC_Effect_Min											1470
#define RC_Effect_Max											1530

//dung de khoi dong/tat quadrotor
#define RC_ON_OFF_MIN											1070
#define RC_ON_OFF_MAX											1130

//----------------------//
//				 ORANGE							//
//	Yellow 				Red					//
//				  BLUE	 			//
//----------------------//
#define LED_YELLOW							12
#define LED_ORANGE							13
#define LED_RED									14
#define LED_BLUE								15

//khai bao hang so ----------
#define LED_D_12_HIGH 					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_D_13_HIGH 					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_D_14_HIGH 					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_D_15_HIGH 					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)

#define LED_D_12_LOW 						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_D_13_LOW 						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_D_14_LOW 						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_D_15_LOW 						HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)

#define ABS(x)         					((x < 0) ? (-x) : x)
#define M_PI 										(float)3.1415926535
#define PI											(float)3.1415926535
#define RAD_TO_DEG 							(float)(180/PI)
	


#define ERROR_MPU6050_NOT_CONNECT									  11
#define ERROR_MPU6050_NOT_I_AM_VALUES							  12
#define ERROR_MPU6050_STATE_READY_NOT_OK						13

#define ERROR_TIM_1_INPUTCAPTURE										21
#define ERROR_TIM_2_INPUTCAPTURE										22
#define ERROR_TIM_4_INPUTCAPTURE										23
#define ERROR_TIM_5_INPUTCAPTURE										24



