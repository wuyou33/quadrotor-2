
//				P=8.0 			
// 				I=0.020 
//				D=15

//        P=4.7 			
//				I=0.048 
// 				D= 36 

// 				Kp = 4.0;  
//				Ki = 0.02; 
//				Kd = 0.0;

// 				Kp =0.5;  
//				Ki = 0.00005; 
//				Kd = 0.01

// 				Kp =0.250;  
//				Ki = 0.950; 
//				Kd = 0.011

// 				Kp = 1.3 
//				Ki = 0.05 
//				Kd = =20


#define PID_KP  (float)1.3							
#define PID_KI  (float)0.05 					
#define PID_KD  (float)20							
#define PID_MIN_VALUE  (float)-200.0
#define PID_MAX_VALUE  (float)200.0

typedef struct 
{
		double setpoint; 		//gia tri dat(mong muon). vidu goc lech roll->setpoint = 0
    double kP; 					//proportional_gain; he so ti le kP
    double kI; 					//integral_gain;     he so ti le tich phan kI
    double kD; 					//derivative_gain;   he so ti le vi phan(dao ham) kD 
    double pre_error;    			//
    double integral_error;     	// error cua tich phan
		double derivative_error;         	//do lech error giua 2 lan loop time(dt)
    double output;							//output cua PID controller - tin hieu dieu khien	
		double output_mix;
		double output_max;	
} PID;

void pid_init(PID* pid, double kP, double kI, double kD, double pid_min, double pid_max)
{
	pid->setpoint = 0;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	
	pid->pre_error = 0;
	pid->integral_error = 0;
	pid->derivative_error = 0;
	
	pid->output_mix = pid_min;
	pid->output_max = pid_max;
}
 
void pid_compute(PID* pid, double Input, double dt ) 
{
		double error 					= pid->setpoint - Input;
		pid->integral_error 	= 		pid->integral_error + pid->kI*(error*dt); 
		pid->derivative_error = 													pid->kD*(error-pid->pre_error)/dt; 
		pid->output 					= pid->kP*error +  pid->integral_error + pid->derivative_error;
		
		if(pid->output < pid->output_mix){
			pid->output = pid->output_mix;
		}
		else if(pid->output > pid->output_max){
			pid->output = pid->output_max;
		}
		
		pid->pre_error = error;
}
