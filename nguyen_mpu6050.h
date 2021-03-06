#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define TM_MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define TM_MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define TM_MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define TM_MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define TM_MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define TM_MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define TM_MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define TM_MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */
/**
 * @brief  Parameters for accelerometer range
 */
#define TM_MPU6050_Accelerometer_2G   0x00 /*!< Range is +- 2G */
#define	TM_MPU6050_Accelerometer_4G   0x01 /*!< Range is +- 4G */
#define	TM_MPU6050_Accelerometer_8G   0x02 /*!< Range is +- 8G */
#define	TM_MPU6050_Accelerometer_16G  0x03 /*!< Range is +- 16G */

#define TM_MPU6050_Gyroscope_250s  0x00  /*!< Range is +- 250 degrees/s */
#define	TM_MPU6050_Gyroscope_500s  0x01  /*!< Range is +- 500 degrees/s */
#define	TM_MPU6050_Gyroscope_1000s  0x02 /*!< Range is +- 1000 degrees/s */
#define	TM_MPU6050_Gyroscope_2000s  0x03  /*!< Range is +- 2000 degrees/s */

/* Default I2C address */
#define MPU6050_I2C_ADDR							0xD0

/* Who I am register value */
#define MPU6050_I_AM_VALUES						0x68
#define MPU6050_WHO_AM_I_REGISTER			0x75

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			  0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				  0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			  0x6A

//Power management registers
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C

#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			  0x74


/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250				(float) 131.0
#define MPU6050_GYRO_SENS_500				(float) 65.5
#define MPU6050_GYRO_SENS_1000			(float) 32.8
#define MPU6050_GYRO_SENS_2000			(float) 16.4

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			(float) 16384.0
#define MPU6050_ACCE_SENS_4			(float) 8192.0
#define MPU6050_ACCE_SENS_8			(float) 4096.0
#define MPU6050_ACCE_SENS_16		(float) 2048.0
	

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
#define TM_MPU6050_Device_0  0x00 /*!< AD0 pin is set to low */
#define	TM_MPU6050_Device_1  0x02  /*!< AD0 pin is set to high */





typedef struct _TM_MPU6050_t {
	float 											Acc_X; /*!< Accelerometer value X axis */
	float 											Acc_Y; /*!< Accelerometer value Y axis */
	float 											Acc_Z; /*!< Accelerometer value Z axis */
	float 											Acc_Total_Vector;
	float 											Gyro_X;     /*!< Gyroscope value X axis */
	float 											Gyro_Y;     /*!< Gyroscope value Y axis */
	float 											Gyro_Z;     /*!< Gyroscope value Z axis */
	float 											Temperature;       /*!< Temperature in degrees */
} TM_MPU6050_t;


