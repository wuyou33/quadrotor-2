//https://github.com/klaslofstedt/stm32f4_quadcopter/blob/master/inc/PID.h
//#define Kp 0.0060, #define Ki 0.0, #define Kd 0.0018

//good PID https://www.youtube.com/watch?v=UBgYEstAfvY&nohtml5=False
//#define P=4.7 , I=0.048 , D= 36 

#define Kp_Roll  0.13//0.0069
#define Ki_Roll  0.00//0.000008
#define Kd_Roll  0.00014//0.015

#define Kp_Pitch  0.12//0.16 
#define Ki_Pitch  0.0000//20//1//5//9//1//1
#define Kd_Pitch  0.0001//14//19 //0.012

//kp = 0.5,ki=0.00005 ,kd=0.01,  prerror, dt=100;

#define DT_milisecond 					10 //100ms = 0.1s
#define DT 											0.01 //10ms=0.01s hay 100ms=0.1s (10ms sample time)

//-------PID Config----------
#define ROLL_PID_KP  2							//0.250
#define ROLL_PID_KI  0 					//0.950
#define ROLL_PID_KD  0							//0.011
#define ROLL_PID_MIN  -400.0
#define ROLL_PID_MAX  400.0

#define PITCH_PID_KP  2
#define PITCH_PID_KI  0				
#define PITCH_PID_KD  0
#define PITCH_PID_MIN  -400.0
#define PITCH_PID_MAX  400.0

#define YAW_PID_KP  0.680
#define YAW_PID_KI  0.500
#define YAW_PID_KD  0.0001
#define YAW_PID_MIN  100.0
#define YAW_PID_MAX  100.0

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
  
		//Compute PID Output
		//output += (Kp * err) + (Ki * int * dt) + (Kd * der /dt);
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


