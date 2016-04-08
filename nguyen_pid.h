//https://github.com/klaslofstedt/stm32f4_quadcopter/blob/master/inc/PID.h
#define Kp 0.0060
#define Ki 0.0
#define Kd 0.0018

//good PID https://www.youtube.com/watch?v=UBgYEstAfvY&nohtml5=False
//#define P 4.7
//#define I 0.048
//#define D 36

//kp = .5,ki=0.00005 ,kd=.01,prerror, dt=100;

//-------PID Config----------
#define ROLL_PID_KP  0.250
#define ROLL_PID_KI  0.950
#define ROLL_PID_KD  0.011
#define ROLL_PID_MIN  -200.0
#define ROLL_PID_MAX  200.0

#define PITCH_PID_KP  0.250
#define PITCH_PID_KI  0.950
#define PITCH_PID_KD  0.011
#define PITCH_PID_MIN  -200.0
#define PITCH_PID_MAX  200.0

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
	
    double last_error;    			//
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
    pid->last_error = 0;
    pid->integral_error = 0;
		pid->diff_error = 0;
}
 
void pid_compute(PID* pid, double Input, double dt, int16_t pid_min, int16_t pid_max ) 
{
		//curr_error la do lech so voi gia tri dat, vi du goc roll lech 5 do (gia tri dat - mong muon = 0 do)
		double error = pid->setpoint - Input;
		
		pid->integral_error +=  error * dt;
		pid->diff_error = (error - pid->last_error)/dt;
  
		//Compute PID Output
		//cong thuc tinh output(control)
		//output = Kp * err + (Ki * int * dt) + (Kd * der /dt);
		pid->output = (pid->kP*error) + (pid->kI * pid->integral_error) + (pid->kD * pid->diff_error);
		
	if(pid->output <= pid_min)
			pid->output = pid_min;
		if(pid->output >= pid_max)
			pid->output = pid_max;
		
		pid->last_error = error;
}



