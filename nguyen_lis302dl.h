#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"


#define CS_HIGH HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define CS_LOW  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)



#define SENS_2G_RANGE_MG_PER_DIGIT		((float)0.06) /* Sensitivity for 2G range [mg/digit] */
#define LED_Threshold_MG											(1000)				/* 1000mg (1G) */ /* LED threshold value in mg */

#define LIS3DSH_OUT_X_L_ADDR					 0x28;
#define LIS3DSH_OUT_X_H_ADDR					 0x29;
#define LIS3DSH_OUT_Y_L_ADDR					 0x2A;
#define LIS3DSH_OUT_Y_H_ADDR					 0x2B;
#define LIS3DSH_OUT_Z_L_ADDR					 0x2C;
#define LIS3DSH_OUT_Z_H_ADDR					 0x2D;














/* ----------------------------------------- */
/* LIS302DL registers                        */
/* ----------------------------------------- */
#define LIS302DL_CTRL_REG1_ADDR							0x20
#define LIS302DL_CTRL_REG2_ADDR							0x21
#define LIS302DL_CTRL_REG3_ADDR							0x22
#define LIS302DL_OUT_X_ADDR								0x29
#define LIS302DL_OUT_Y_ADDR								0x2B
#define LIS302DL_OUT_Z_ADDR								0x2D

#define LIS302DL_SENSITIVITY_2_3G						18  /* 18 mg/digit*/
#define LIS302DL_SENSITIVITY_9_2G						72  /* 72 mg/digit*/

#define LIS302DL_DATARATE_100							((uint8_t)0x00)
#define LIS302DL_DATARATE_400							((uint8_t)0x80)

#define LIS302DL_LOWPOWERMODE_ACTIVE					((uint8_t)0x40)
#define LIS302DL_FULLSCALE_2_3							((uint8_t)0x00)
#define LIS302DL_FULLSCALE_9_2							((uint8_t)0x20)
#define LIS302DL_SELFTEST_NORMAL						((uint8_t)0x00)
#define LIS302DL_XYZ_ENABLE								((uint8_t)0x07)
#define LIS302DL_SERIALINTERFACE_4WIRE					((uint8_t)0x00)
#define LIS302DL_BOOT_NORMALMODE						((uint8_t)0x00)
#define LIS302DL_BOOT_REBOOTMEMORY						((uint8_t)0x40)
#define LIS302DL_FILTEREDDATASELECTION_OUTPUTREGISTER	((uint8_t)0x20)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_OFF			((uint8_t)0x00)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_1				((uint8_t)0x04)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_2				((uint8_t)0x08)
#define LIS302DL_HIGHPASSFILTERINTERRUPT_1_2			((uint8_t)0x0C)
#define LIS302DL_HIGHPASSFILTER_LEVEL_0					((uint8_t)0x00)
#define LIS302DL_HIGHPASSFILTER_LEVEL_1					((uint8_t)0x01)
#define LIS302DL_HIGHPASSFILTER_LEVEL_2					((uint8_t)0x02)
#define LIS302DL_HIGHPASSFILTER_LEVEL_3					((uint8_t)0x03)



