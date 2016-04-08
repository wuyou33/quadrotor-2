//https://github.com/klaslofstedt/stm32f4_quadcopter/blob/master/inc/PID.h
//good PID https://www.youtube.com/watch?v=UBgYEstAfvY&nohtml5=False
//#define P=4.7 , I=0.048 , D= 36 
//				P=8.0 / I=0.020 / D=15
//				P=0.13 / I=0.00 / D=0.00014
//				P=0.0069 / I=0.000008 / D=0.015
//				P=0.12 / I=0.0000 / D=0.0001
//				P = 0.5 / I=0.00005 /D=0.01
//        P=0.0060/ I=0.0 / D=0.0018

//#define DT_milisecond 					10 //100ms = 0.1s
#define DT 											0.01 //10ms=0.01s hay 100ms=0.1s (10ms sample time)

#define ROLL_PID_KP  0.250							//0.250
#define ROLL_PID_KI  0.950 					//0.950
#define ROLL_PID_KD  0.011							//0.011
#define ROLL_PID_MIN  -400.0
#define ROLL_PID_MAX  400.0

#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950 				
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -400.0
#define PITCH_PID_MAX  400.0

/*#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  100.0
#define YAW_PID_MAX  100.0
*/

//Khai bao struct 
typedef struct 
{
		double setpoint; 		//gia tri dat(mong muon). vidu goc lech roll->setpoint = 0
    double kP; 					//proportional_gain; he so ti le kP
    double kI; 					//integral_gain;     he so ti le tich phan kI
    double kD; 					//derivative_gain;   he so ti le vi phan(dao ham) kD
		//double pre_pre_error;    
    double pre_error;    			//
    double integral_error;     	// error cua tich phan
		double diff_error;         	//do lech error giua 2 lan loop time(dt)
    double output;							//output cua PID controller - tin hieu dieu khien
		//double pre_output;
		
} PID;
void pid_setup_gain(PID* pid, double kP, double kI, double kD)
{
	pid->setpoint = 0;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}
void pid_setup_error(PID* pid) 
{
		//pid->pre_pre_error = 0;
    pid->pre_error = 0;
    pid->integral_error = 0;
		pid->diff_error = 0;
}
 
void pid_compute(PID* pid, double Input, double dt, int16_t pid_min, int16_t pid_max ) 
{
		//curr_error la do lech so voi gia tri dat, vi du goc roll lech 5 do (gia tri dat - mong muon = 0 do)
		double error 					= pid->setpoint - Input;
		
		pid->integral_error 	= pid->integral_error +  (pid->kI *  error * dt);
		pid->diff_error 			= 	(pid->kD * (error - pid->pre_error)) / dt;
  
		//Compute PID Output		//output += (Kp * err) + (Ki * int * dt) + (Kd * der /dt);
		//pid->output 					= pid->output +   (pid->kP*error) +    pid->integral_error +      pid->diff_error;
		pid->output 					= (pid->kP*error) +    pid->integral_error +      pid->diff_error;
		
		if(pid->output <= pid_min)
			pid->output = pid_min;
		if(pid->output >= pid_max)
			pid->output = pid_max;
		
		pid->pre_error = error;
}

/*
P_part = Kp*(Error - pre_Error);
I_part = (Error + pre_Error) * 0.5*Ki*T;
D_part = ( Error - 2*pre_Error+ pre_pre_Error) * Kd/T;
Out = pre_out + P_part + I_part + D_part ;
pre_pre_Error = pre_Error
pre_Error = Error
pre_Out = Out
*/


/*
void pidInit()
{
	pid.roll.Kp = 1.3;	//stable: 1.3
	pid.roll.Ki = 0.05;	//stable: 0.05
	pid.roll.Kd = 20;	//stable: 25
	pid.roll.max = 400;	//stable: 400
	
	pid.pitch.Kp = pid.roll.Kp;
	pid.pitch.Ki = pid.roll.Ki;
	pid.pitch.Kd = pid.roll.Kd;
	pid.pitch.max= pid.roll.max;
	
	pid.yaw.Kp = 4.0;  //stable: 4.0
	pid.yaw.Ki = 0.02; //stable: 0.02
	pid.yaw.Kd = 0.0;  //stable: 0
	pid.yaw.max= 400;  //stable: 400
	pidReset();
}

void calculate_pid()
{
	float pidError;
	//Roll
	pidError = gyroRate[ROL] - setPoint[ROL];
	pidState.roll.iTerm += pid.roll.Ki * pidError;
	pidState.roll.iTerm = limit(pidState.roll.iTerm,-pid.roll.max,pid.roll.max);
	pidOut[ROL] = pid.roll.Kp * pidError + pidState.roll.iTerm + pid.roll.Kd * (pidError - pidState.roll.lastDErr);
	pidOut[ROL] = limit(pidOut[ROL],-pid.roll.max,pid.roll.max);
	pidState.roll.lastDErr = pidError;
	
	//pitch
	pidError = gyroRate[PIT] - setPoint[PIT];
	pidState.pitch.iTerm += pid.pitch.Ki * pidError;
	pidState.pitch.iTerm = limit(pidState.pitch.iTerm, -pid.pitch.max, pid.pitch.max);
	pidOut[PIT] = pid.pitch.Kp * pidError + pidState.pitch.iTerm + pid.pitch.Kd * (pidError - pidState.pitch.lastDErr);
	pidOut[PIT] = limit(pidOut[PIT], -pid.pitch.max, pid.pitch.max);
	pidState.pitch.lastDErr = pidError;
	
	//Yaw
	pidError = gyroRate[YAW] - setPoint[YAW];
	pidState.yaw.iTerm += pid.yaw.Ki * pidError;
	pidState.yaw.iTerm = limit(pidState.yaw.iTerm, -pid.yaw.max, pid.yaw.max);
	pidOut[YAW] = pid.yaw.Kp * pidError + pidState.yaw.iTerm + pid.yaw.Kd * (pidError - pidState.yaw.lastDErr);
	pidOut[YAW] = limit(pidOut[YAW], -pid.yaw.max, pid.yaw.max);
	pidState.yaw.lastDErr = pidError;
}
void pidReset()
{
	pidState.roll.iTerm = 0;
	pidState.roll.lastDErr = 0;
	pidState.pitch.iTerm = 0;
	pidState.pitch.lastDErr = 0;
	pidState.yaw.iTerm = 0;
	pidState.yaw.lastDErr = 0;
	setPoint[ROL] = 0;
	setPoint[PIT] = 0;
	setPoint[YAW] = 0;
}
*/
