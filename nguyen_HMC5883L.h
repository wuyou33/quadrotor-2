#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define HMC5883L_Device_Address 						0x1E


#define HMC5883L_Register_A   							0x00
#define HMC5883L_Register_B   							0x01
#define HMC5883L_Measurement_Mode_Register 	0x02
#define HMC5883L_Data_Register_Begin 				0x03

#define Mode_Measurement_Continuous 				0x00
#define Mode_Measurement_SingleShot 				0x01
#define Mode_Measurement_Idle 							0x03


#define HMC5883L_COMPASS_XOUT_H							0x3B
#define HMC5883L_COMPASS_XOUT_L							0x3B

#define HMC5883L_COMPASS_YOUT_H							0x3B
#define HMC5883L_COMPASS_YOUT_L							0x3B

#define HMC5883L_COMPASS_ZOUT_H							0x3B
#define HMC5883L_COMPASS_ZOUT_L							0x3B


typedef struct _Compass_HMC5883L {
	float Compass_X; /* Tu truong truc X */
	float Compass_Y; /* Tu truong truc Y */
	float Compass_Z; /* Tu truong truc Z */
} Compass_HMC5883L;
