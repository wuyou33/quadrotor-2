//        P=4.7 			/ I=0.048 / D= 36 
//				P=8.0 			/ I=0.020 / D=15
//				P=0.13 			/ I=0.00 	/ D=0.00014
//				P=0.0069 		/ I=0.000008 / D=0.015
//				P=0.12 			/ I=0.0000 / D=0.0001
//				P = 0.5 		/ I=0.00005 /D=0.01
//        P=0.0060		/ I=0.0 / D=0.0018
//        P=0.250			/ I=0.950 / D=0.011
//        P= 1.3 			/ I=0.05 / D=20
	
	

//I think the 1000/50 is an error. Looks to me like the AC_PID library takes its dt in seconds. The main loop runs every 50 milliseconds, so the correct dt would be 50/1000 = 0.05 sec.
//#define DT_milisecond 					(float)0.01 //10ms


#define ROLL_PID_KP  (float)1.3							
#define ROLL_PID_KI  (float)0.05 					
#define ROLL_PID_KD  (float)20							
#define ROLL_PID_MIN  (float)-400.0
#define ROLL_PID_MAX  (float)400.0

#define PITCH_PID_KP  (float)1.3
#define PITCH_PID_KI  (float)0.05 				
#define PITCH_PID_KD  (float)20
#define PITCH_PID_MIN  (float)-400.0
#define PITCH_PID_MAX  (float)400.0

/*
#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  100.0
#define YAW_PID_MAX  100.0

pid.yaw.Kp = 4.0;  //stable: 4.0
pid.yaw.Ki = 0.02; //stable: 0.02
pid.yaw.Kd = 0.0;  //stable: 0
pid.yaw.max= 400;  //stable: 400
*/
typedef struct 
{
		double setpoint; 		//gia tri dat(mong muon). vidu goc lech roll->setpoint = 0
    double kP; 					//proportional_gain; he so ti le kP
    double kI; 					//integral_gain;     he so ti le tich phan kI
    double kD; 					//derivative_gain;   he so ti le vi phan(dao ham) kD 
    double pre_error;    			//
    double integral_error;     	// error cua tich phan
		double diff_error;         	//do lech error giua 2 lan loop time(dt)
    double output;							//output cua PID controller - tin hieu dieu khien		
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
		
		pid->integral_error 	+=   (pid->kI *  error)*dt; //   (pid->kI *  error)*dt
		pid->diff_error 			= 		pid->kD * (error - pid->pre_error)/dt; //   (error - pid->pre_error) / dt
  
		//Compute PID Output		//output += (Kp * err) + (Ki * int * dt) + (Kd * der /dt);
		pid->output 					= (pid->kP*error) +    pid->integral_error +      pid->diff_error;
		
		if(pid->output <= pid_min)
			pid->output = pid_min;
		if(pid->output >= pid_max)
			pid->output = pid_max;
		
		pid->pre_error = error;
}

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
