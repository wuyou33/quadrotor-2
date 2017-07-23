

#define PID_KP  								    (float)1.3		
#define PID_KI  								    (float)0.04				
#define PID_KD  								    (float)18.0		
	
#define PID_KP_YAW  								(float)4.0		
#define PID_KI_YAW  								(float)0.02
#define PID_KD_YAW 								  (float)0.0	
	
#define PID_MAX_VALUE_ROLL  				(int) 400
#define PID_MAX_VALUE_PITCH  				(int) 400
#define PID_MAX_VALUE_YAW  					(int) 400

typedef struct 
{
		float setpoint; 						//gia tri dat(mong muon). vidu goc lech roll->setpoint = 0
    float kP; 									//proportional_gain; he so ti le kP
    float kI; 									//integral_gain;     he so ti le tich phan kI
    float kD; 									//derivative_gain;   he so ti le vi phan(dao ham) kD 
    float pre_error;    				//
    float integral_error;     	// error cua tich phan
		float derivative_error;    //do lech error giua 2 lan loop time(dt)
    float output;							//output cua PID controller - tin hieu dieu khien 
		float output_max;	
} PID;
 


