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

//-------------------------
// https://github.com/benripley/Arduino-Quadcopter/tree/master/Quadcopter
// http://yameb.blogspot.com/2013/05/my-first-quadrotor-control-overview-and.html
/*working variables
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;
void Compute()
{
   //How long since we last calculated
   unsigned long now = millis();
   double timeChange = (double)(now - lastTime);
  
   //Compute all the working error variables
   double error = Setpoint - Input;
   errSum += (error * timeChange);
   double dErr = (error - lastErr) / timeChange;
  
   //Compute PID Output
   Output = kp * error + ki * errSum + kd * dErr;
  
   //Remember some variables for next time
   lastErr = error;
   lastTime = now;
}
  
void SetTunings(double Kp, double Ki, double Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}


void Compute(void)
{
   SampleTime = 100;		
   unsigned long now = millis();
   unsigned long timeChange = (millis() - lastTime);
   if(timeChange>=SampleTime)
   {
      //Compute all the working error variables
	  double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      //Compute PID Output
      double output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      //Remember some variables for next time
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}

void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0 || Kd<0) return;
 
   dispKp = Kp; 
	dispKi = Ki; 
	dispKd = Kd;
   
   double SampleTimeInSec = ((double)SampleTime)/1000;  
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
	 {
			kp = (0 - kp);
			ki = (0 - ki);
			kd = (0 - kd);
	 }
}

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
   }
}
double update(double x,double dx)
{  
     double command = 0.0;  
			xError = (xDesired - x);   //xDesired gia tri dat -  x: gia tri do dc
     xErrorIntegral+=xError;  //tich phan cua error theo t
     if(feed)  
      command = kP*xError + kI*xErrorIntegral +kD*dx;  
     else  
      command = kP*xError + kI*xErrorIntegral +kD*(xError-xErrorPrev);  
     xErrorPrev = xError;  
     return command;  
   }  

*/



