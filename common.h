#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define PWM_Throtte_Min					1100
#define PWM_Throtte_Max					1900
#define PWM_Avg									1500
#define PWM_Effect_Min					1470
#define PWM_Effect_Max					1530

//dung de khoi dong/tat quadrotor
#define PWM_ON_OFF_MIN					1075
#define PWM_ON_OFF_MAX					1125

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
#define MAX(a,b)								( (a > b) ? a : b )
#define MIN(a,b)								( (a < b) ? a : b )




