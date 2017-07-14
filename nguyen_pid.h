
//				P=8.0 			
// 				I=0.020 
//				D=15

//        P=4.7 			
//				I=0.048 
// 				D= 36 

// 				Kp = 4.0;  
//				Ki = 0.02; 
//				Kd = 0.0;


#define PID_KP  							(float)3.7		//tang KP den khi co giap dong, tang KD de giam vot lo, tang KI neu chua den vitri can bang		
#define PID_KI  							(float)0				
#define PID_KD  							(float)0			//12			
#define PID_KP_2  							(float)2		//tang KP den khi co giap dong, tang KD de giam vot lo, tang KI neu chua den vitri can bang		
#define PID_KI_2  							(float)0				
#define PID_KD_2  							(float)0			//12	
#define PID_MIN_VALUE  				(float)-250.0
#define PID_MAX_VALUE  				(float) 250.0
#define PID_LIMIT_MIN_KI_KD  	(float)-100.0
#define PID_LIMIT_MAX_KI_KD  	(float)100.0
#define DTime 																(float)0.005 	//10ms
#define SAMPLE_TIME 											(float)		0.1 			//10ms PID time loop
typedef struct 
{
		double setpoint; 						//gia tri dat(mong muon). vidu goc lech roll->setpoint = 0
    double kP; 									//proportional_gain; he so ti le kP
    double kI; 									//integral_gain;     he so ti le tich phan kI
    double kD; 									//derivative_gain;   he so ti le vi phan(dao ham) kD 
    double pre_error;    				//
    double integral_error;     	// error cua tich phan
		double derivative_error;    //do lech error giua 2 lan loop time(dt)
    double output;							//output cua PID controller - tin hieu dieu khien	
		double output_min;
		double output_max;	
} PID;
 
void pid_compute(PID* pid, double Input, double dt ) 
{
		double error 					= 	(pid->setpoint - Input);
		pid->integral_error 	= 		pid->integral_error + pid->kI * (error*dt); 
		pid->derivative_error = 													pid->kD * ((error - pid->pre_error)/dt); 
		
		if(pid->integral_error < PID_LIMIT_MIN_KI_KD)				{ pid->integral_error = PID_LIMIT_MIN_KI_KD; }
		else if(pid->integral_error > PID_LIMIT_MAX_KI_KD) 	{ pid->integral_error = PID_LIMIT_MAX_KI_KD; }
		
		if(pid->derivative_error < PID_LIMIT_MIN_KI_KD)					{			pid->derivative_error = PID_LIMIT_MIN_KI_KD;		}
		else if(pid->derivative_error > PID_LIMIT_MAX_KI_KD)		{			pid->derivative_error = PID_LIMIT_MAX_KI_KD;		}
	
		pid->output 	=(int)( pid->kP*error +  pid->integral_error - pid->derivative_error );
		
		if(pid->output < pid->output_min)					{			pid->output = pid->output_min;		}
		else if(pid->output > pid->output_max)		{			pid->output = pid->output_max;		}
		
		pid->pre_error = error;
}

void pid_init(PID* pid, double kP, double kI, double kD, double pid_min, double pid_max)
{
	pid->setpoint = 0.0;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	
	pid->pre_error = 0.0;
	pid->integral_error = 0.0;
	pid->derivative_error = 0.0;
	
	pid->output_min = pid_min;
	pid->output_max = pid_max;
}

void pid_reset_PID_value(PID* pid, double kP, double kI, double kD)
{
	//pid->setpoint = 0.0;
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	//pid->pre_error = 0.0;
	//pid->integral_error = 0.0;
	//pid->derivative_error = 0.0;
	//pid->output_min = pid_min;
	//pid->output_max = pid_max;
}
