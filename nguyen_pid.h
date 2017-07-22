#define PID_KP  								(float)1.3		//tang KP den khi co giap dong, tang KD de giam vot lo, tang KI neu chua den vitri can bang		
#define PID_KI  								(float)0.04				
#define PID_KD  								(float)18.0		
	
#define PID_MAX_VALUE_ROLL  				(float) 400.0
#define PID_MAX_VALUE_PITCH  				(float) 400.0
#define PID_MAX_VALUE_YAW  					(float) 400.0

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
		double output_max;	
} PID;
 
void pid_compute(PID* pid, double Input, double dt ) 
{
		double error 					= 	(Input - pid->setpoint );
		pid->integral_error 	= 		pid->integral_error + pid->kI * (error*dt); 
		pid->derivative_error = 													pid->kD * ((error - pid->pre_error)/dt); 
	
		pid->output 	= ( pid->kP*error +  pid->integral_error - pid->derivative_error );
		
		if(pid->output < pid->output_max*-1)					
		{			
			pid->output = pid->output_max*-1;		
		}
		else if(pid->output > pid->output_max)		
		{			
			pid->output = pid->output_max;		
		}	
		pid->pre_error = error;
}

