#include <stdio.h>
#include <stdint.h>
#include <math.h>

//----------------------//
//				 ORANGE							//
//	Yellow 				Red					//
//				  BLUE	 			//
//----------------------//
#define DT 																(float)0.01 //10ms
	
#define LED_YELLOW												12
#define LED_ORANGE												13
#define LED_RED														14
#define LED_BLUE													15

#define LED_D_12_HIGH 										HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET)
#define LED_D_13_HIGH 										HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET)
#define LED_D_14_HIGH 										HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET)
#define LED_D_15_HIGH 										HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET)
#define LED_D_12_LOW 											HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET)
#define LED_D_13_LOW 											HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_D_14_LOW 											HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET)
#define LED_D_15_LOW 											HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET)





#define Motor_Range_Min										1100
#define Motor_Range_Max										1900
#define Motor_Range_Min_NOT_BALANCE_STATE	1150
#define DO_VOT_LO													70

#define RC_Throtte_Min										1100
#define RC_Throtte_Max										1900
#define RC_Effect_Min											1465 //1465 < 1500 < 1535
#define RC_Medium													1500
#define RC_Effect_Max											1535


#define CONFIG_PWM_MAX										2000
#define CONFIG_PWM_MIN										700
#define ENTER_BALANCE_STATE								1200


//dung de khoi dong/tat quadrotor
#define RC_ON_OFF_MIN											1060    // 1060 < 1100 < 1140
#define RC_ON_OFF_MAX											1140

#define ABS(x)         										((x < 0) ? (-x) : x)
#define M_PI 															(float)3.1415926535
#define PI																(float)3.1415926535
#define RAD_TO_DEG 												(float)(180/PI)
	
#define ZERO_									  				0
#define ONE_									  				1

#define STATE_FLY_ON									  1
#define STATE_FLY_OFF								  	0

#define MAIN_DELAY_TIME								  30

#define ERROR_MPU6050_NOT_CONNECT									  11
#define ERROR_MPU6050_NOT_I_AM_VALUES							  12
#define ERROR_MPU6050_STATE_READY_NOT_OK						13

#define ERROR_TIM_1_INPUTCAPTURE										21
#define ERROR_TIM_2_INPUTCAPTURE										22
#define ERROR_TIM_4_INPUTCAPTURE										23
#define ERROR_TIM_5_INPUTCAPTURE										24



