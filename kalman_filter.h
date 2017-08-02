/*
http://icviet.vn/bai-hoc/thiet-ke-bo-loc-thong-thap-thong-cao-va-kalman-voi-stm32f4.html
http://codientu.org/threads/11413/

//Code kalman filter
http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
https://github.com/TKJElectronics/KalmanFilter
https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.cpp
https://github.com/TKJElectronics/KalmanFilter/blob/master/Kalman.h
https://github.com/TKJElectronics/KalmanFilter/blob/master/examples/MPU6050/MPU6050.ino
https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/blob/master/IMU/MPU6050/MPU6050.ino

*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

typedef struct _Kalman_Setting {
	float Q_angle ;
	float Q_gyro  ;
	float R_angle ;

	float x_bias ;
	
	float P_00 ;
	float P_01 ;
	float P_10 ;
	float P_11 ;
	float y; 
	float S;
	float K_0;
	float K_1;
	float angle;
} Kalman_Setting;

/*
#ifndef _Kalman_h
#define _Kalman_h

class Kalman 
{
private:
    // Kalman filter variables 
    double Q_angle; // Process noise variance for the accelerometer
    double Q_bias; // Process noise variance for the gyro bias
    double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise
    
    double angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
    double bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
    double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    
    double P[2][2]; // Error covariance matrix - This is a 2x2 matrix
    double K[2]; // Kalman gain - This is a 2x1 matrix   | 1|
    double y; // Angle difference - 1x1 matrix           | 2|
    double S; // Estimate error - 1x1 matrix
public:
    Kalman() 
    {
        // We will set the varibles like so, these can also be tuned by the user 
        Q_angle = 0.001;
        Q_bias = 0.003;
        R_measure = 0.03;
        
        bias = 0; // Reset bias
        P[0][0] = 0; // Since we assume tha the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix
        P[1][0] = 0; // is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        P[0][1] = 0;
        P[1][1] = 0;
    };
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    double getAngle(double newAngle, double newRate, double dt) 
    {
                       
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        // Step 1 
        rate = newRate - bias;
        angle += dt * rate;
        
        // Update estimation error covariance - Project the error covariance ahead
        // Step 2 
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        // Step 4 
        S = P[0][0] + R_measure;   //(Pk-) +R
        // Step 5 
        K[0] = P[0][0] / S;        // K[0] = Kk = (Pk-)/ [(Pk-) +R]
        K[1] = P[1][0] / S;
        
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        // Step 3 
        y = newAngle - angle;      // Zk - (Xk-)  = y
        // Step 6 
        angle += K[0] * y;         // Xk = (Xk-) + Kk * [Zk - (Xk-)]
        bias += K[1] * y;
        
        // Calculate estimation error covariance - Update the error covariance
        // Step 7 
        P[0][0] -= K[0] * P[0][0];   // Pk.new = (Pk-) - Kk*(Pk-)
        P[0][1] -= K[0] * P[0][1];
        P[1][0] -= K[1] * P[0][0];
        P[1][1] -= K[1] * P[0][1];
        
        return angle;
    };
    void setAngle(double newAngle) { angle = newAngle; }; // Used to set angle, this should be set as the starting angle
    double getRate() { return rate; }; // Return the unbiased rate
    
    // These are used to tune the Kalman filter 
    void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
    void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
    void setRmeasure(double newR_measure) { R_measure = newR_measure; };
};
#endif
*/




//
//
//-----------------------------------------------------------------------------------------------------------------------
//
//
//




// KasBot V1  -  Kalman filter module
/*
float Q_angle  =  0.01;    //0.001    //0.005
float Q_gyro   =  0.0003;  //0.003    //0.0003
float R_angle  =  0.01;    //0.03     //0.008

float x_bias = 0;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
float  y, S;
float K_0, K_1;




float kalmanCalculate(float newAngle, float newRate, int looptime)
{
	float dt = float(looptime)/1000;   
	x_angle += dt * (newRate - x_bias);
	P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
	P_01 +=  - dt * P_11;
	P_10 +=  - dt * P_11;
	P_11 +=  + Q_gyro * dt;
	
	y = newAngle - x_angle;
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;
	
	x_angle +=  K_0 * y;
	x_bias  +=  K_1 * y;
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;
	
	return x_angle;
}
*/


//
//
//-----------------------------------------------------------------------------------------------------------------------
//
//
//












//
//
//-----------------------------------------------------------------------------------------------------------------------
//
//
//



/*
//The complementary filter - BO LOC BU
//http://www.pieter-jan.com/node/11
#define ACCELEROMETER_SENSITIVITY 		8192.0
#define GYROSCOPE_SENSITIVITY 				65.536 
#define M_PI 													3.14159265359	    
#define dt 0.01												// 10 ms sample rate!    
 
void ComplementaryFilter(short accData[3], short gyrData[3], float *pitch, float *roll)
{
    float pitchAcc, rollAcc;               
 
    // Integrate the gyroscope data -> int(angularSpeed) = angle
    *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
    *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis
 
    // Compensate for drift with accelerometer data if !bullshit
    // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	
    int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
	
    if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
    {
				// Turning around the X axis results in a vector on the Y-axis
        pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
        *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
				// Turning around the Y axis results in a vector on the X-axis
        rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        *roll = *roll * 0.98 + rollAcc * 0.02;
    }
} 
*/


//
//
//-----------------------------------------------------------------------------------------------------------------------
//
//
//




/*
//3 bo loc: bo loc thong thap (tanso thap) LPF, bo loc thong cao (tan so cao)HPF, bo loc Kalman
float LPF(float x, float CUTOFF,float SAMPLE_RATE)
{
			float RC, dt, alpha, y;
			static float ylast=0;
			RC = 1.0/(CUTOFF*2*3.14);
			dt = 1.0/SAMPLE_RATE;
			alpha = dt/(RC+dt);
			y = ylast + alpha * ( x - ylast ); 
			ylast = y;
			return y;
}

float HPF(float x, float CUTOFF,float SAMPLE_RATE)
{
			float RC = 1.0/(CUTOFF*2*3.14);
			float dt = 1.0/SAMPLE_RATE;
			float alpha = RC/(RC+dt);
			float y;
			static float xlast=0, ylast=0;
			y = alpha * ( ylast + x - xlast); 
			ylast = y;
			xlast = x;
			return y;
}
float kalman_single(float z, float measure_noise, float process_noise)
{
			//z tin hieu bi nhieu~
			//measure_noise: nhieu he thong'
			//process_noise: nhieu do luong`
			const float R = measure_noise*measure_noise;
			const float Q = process_noise*process_noise; 
			static float x_hat,P;
			float P_,K;

			// noi suy kalman ***************
				P_ = P + Q;                     // P_ = A*P*A' + Q;
				K = P_/(P_ + R);                // K = P_*H'*inv(H*P_*H' + R);
				x_hat = x_hat + K*(z - x_hat);  // x_hat = x_hat + K*(z - H*x_hat);
				P = ( 1 - K)*P_ ;               // P = ( 1 - K*H)*P_ ;
				
			return x_hat;
}
*/
