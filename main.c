/*
TONG HOP CAC PIN SU DUNG

PORT A: PA0 																	=> Button User  
				PA3																		=> TIM5_CH4 InputCaptrue
				PA5																		=> TIM2_CH1 InputCaptrue
				
PORT B: PB6, PB7 															=> I2C1 cam bien 10 truc mpu6050 (PB6->I2C1_SCL,	PB7->I2C1_SDA) 
				PB8																		=> TIM4_CH3 Inputcapture
																									
PORT C: PC6,  																=> TIM3_CH1 Output Campare PWM
				PC7, PC8, PC9(timer 3 chua su dung)
				
PORT D: PD12, PD13, PD14, PD15  							=> LEDSang (PD12 GREEN, PD13 ORANGE, PD14 RED, PD15 BLUE)

PORT E: PE9 																	=> TIM1_CH1 InputCaptrue

--------------------------------------------------------------------------
InputCaptrue PIN
PE9 ->TIM1_CH1  //Throttle (can ga) tang giam toc do quay				keo len +(1900), keo xuong -(1100)
PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw						keo qua trai +(1900), keo qua phai -(1100)
PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch. 						keo len la +, keo xuong la -
PA3 ->TIM5_CH4  //Aileron_TraiPhai (trai - phai) - goc Roll     keo qua trai +, keo qua phai -
*/
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "main.h"
#include "common.h"
#include "nguyen_mpu6050.h"
#include "nguyen_pid.h"
#include "kalman_filter.h"
#include "nguyen_fuzzy_logic.h"

//----------Khai bao BIEN-------------------------------------------------------------------------
I2C_HandleTypeDef 				I2C_Handle_10truc;  //I2C handle, dung de doc value cua cam bien MPU6050
TIM_HandleTypeDef 				Tim3_Handle_PWM;		//timer 3 dung de output PWM ra 4 channel

TIM_HandleTypeDef 				htim1; //PE9 ->TIM1_CH1  //Throttle (can ga)
TIM_HandleTypeDef 				htim2; //PA5 ->TIM2_CH1  //Rudder
TIM_HandleTypeDef 				htim4; //PB8 ->TIM4_CH3  //Elevator
TIM_HandleTypeDef 				htim5; //PA3 - TIM5_CH4  //Aileron

				//---------RF Module, PWM Capture
int16_t 									IC_Throttle1, IC_Throttle2, 								IC_Throttle_pusle_width; //Throttle (can ga) tang giam toc do quay
int16_t 									IC_Rudder_Xoay1, IC_Rudder_Xoay2, 					IC_Rudder_Xoay_pusle_width; //Rudder (xoay theo truc z) - goc Yaw
int16_t 									IC_Elevator_TienLui1, IC_Elevator_TienLui2, IC_Elevator_TienLui_pusle_width; //Elevator (tien - lui) - goc Pitch
int16_t 									IC_Aileron_TraiPhai1, IC_Aileron_TraiPhai2, IC_Aileron_TraiPhai_pusle_width;//Aileron_TraiPhai (trai - phai) - goc Roll

				//---------PWM 4 motor
int16_t 									pwm_motor_1, pwm_motor_2, pwm_motor_3, pwm_motor_4;
int16_t 									FlyState; // 0: May bay ngung hoat dong, 1:may bay dang bay

				//---------sensor
uint8_t 									who_i_am_reg_value_MPU6050;
int32_t 									timer;
int 											i;
float 										accX_angle, accY_angle, accZ_angle;
float 										gyroX_angle, gyroY_angle, gyroZ_angle;
float 										Kalman_angelX, Kalman_angelY, Kalman_angelZ;
float 										gyroXrate, gyroYrate, gyroZrate;
Kalman_Setting 						kalmanX,	kalmanY,   kalmanZ;
TM_MPU6050_t 							mpu6050;

				//--------PID controller
PID 											pid_roll, pid_pitch, pid_yaw; //PID controller

				//--------Fuzzy System--------------
FuzzyController						rollFuzzyControl, pitchFuzzyControl, yawFuzzyControl;
//--END Khai bao BIEN-------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------------------
//..........................code default of ARM
#ifdef _RTE_
	#include "RTE_Components.h"             
#endif
#ifdef RTE_CMSIS_RTOS                   
	#include "cmsis_os.h"                   
#endif
#ifdef RTE_CMSIS_RTOS_RTX
	extern uint32_t os_time;
	uint32_t HAL_GetTick(void) { return os_time; }
#endif

																
//-----------Khai bao HAM-------------------------------------------------------
				//Kalman filter
float kalmanCalculate(Kalman_Setting *kalman, float newAngle, float newRate, float DT_);															
																
				//ham handle error						
static void 			SystemClock_Config(void);
static void 			Error_Handler(void); 
void 							Error_Handler_Custom(int type);
															
				//ham Led Sang
void 							SANG_1_LED(int8_t pin);
void 							SANG_2_LED(int8_t type);
void 							SANG_4_LED(void);
void 							SANG_4_LED_FOREVER(void);
void 							SANG_4_LED_OFF(void);
void 							Check_EveryThing_OK(void);
void 							Turn_On_Quadrotor(void);
void 							Turn_Off_Quadrotor(void);
															
				//Khoi Tao LED, BUTTON USER
void 							Init_LEDSANG_PORTD_12_13_14_15(void);
void 							Init_BUTTON_USER_PORT_A_0(void);
															
				//Khoi tao TIMER3 output PWM											
void 							Init_TIM3_OUTPUT_COMPARE_4_Channel(void);
															
				//timer 1 & 2 inputcapture for RF module
void 							Init_Receiver_TIM_PWM_Capture_TIM1(void); //PE9 ->TIM1_CH1  //Throttle (can ga)
void 							Init_Receiver_TIM_PWM_Capture_TIM2(void); //PA5 ->TIM2_CH1  //Rudder  (xoay)
void 							Init_Receiver_TIM_PWM_Capture_TIM4(void); //PB8 ->TIM4_CH3  //Elevator (tien - lui)
void 							Init_Receiver_TIM_PWM_Capture_TIM5(void); //PA3 - TIM5_CH4  //Ailenron (trai - phai)

				//Dieu chinh huong bay qua receiver
void 							SetInitDataQuadrotor(void);
void 							SetPWM_4_Motor(int16_t value);
void 							UpdatePWM_4_Motor_By_TIM_CCRx(void);
void 							SetPWM_1_Motor(int16_t numberMotor, int16_t newValue);
void 							SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue);
void 							SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue);
void 							Direct_Quadrotor_By_Receiver(void);


				//i2c chip mpu6050 10truc
void 							Init_I2C_GPIO_PortB(void);
void 							Init_I2C_Handle_10truc(void);
void 							TM_I2C_IS_DEVICE_CONNECTED(void);
uint8_t 					TM_I2C_WHO_I_AM( uint8_t device_address, uint8_t register_address);
void 							TM_MPU6050_SetDataRate(uint8_t device_address, uint8_t register_address, uint8_t rate);
void 							TM_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value);
void 							TM_MPU6050_SetGyroscope(uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value); 
void 							TM_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data);
void 							TM_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void 							TM_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data);
void 							TM_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output );
void 							Calculate_Accel_X_Angles(TM_MPU6050_t* output, float* angel_x);
void 							Calculate_Accel_Y_Angles(TM_MPU6050_t* output, float* angel_y);
void 							Calculate_Accel_Z_Angles(TM_MPU6050_t* output, float* angel_z);
void 							Sang_Led_By_MPU6050_Values(float angel_x, float angel_y, float angel_z );

				//ham delay default
volatile 					uint32_t g_iSysTicks = 0;
void 							SysTick_Handler(){	g_iSysTicks++;}
void 							delay_ms(uint32_t piMillis){	uint32_t iStartTime = g_iSysTicks;	while( (g_iSysTicks - iStartTime ) < piMillis)	{}}			

				//Fuzzy system
void 							initFuzzySystem(void);
void 							Fuzzification_All_MF(float x, FuzzyController * fuzzyController);
void 							Apply_All_Rule( FuzzyController * fuzzyController );
void 							Defuzzification( FuzzyController * fuzzyController );
//-----------END Khai bao HAM-------------------------------------------------------

int main(void)
{
		/*..................code default cua ARM co san						*/
		#ifdef RTE_CMSIS_RTOS                   
			osKernelInitialize();                 
		#endif		
	
		HAL_Init();
		SystemClock_Config();	
	
		initFuzzySystem(); 					//init fuzzy system
		SetInitDataQuadrotor(); 		// set kalman filter, input cature
	
						//---cap xung clock------------------------------------
		__GPIOA_CLK_ENABLE();		
		__GPIOB_CLK_ENABLE();			
		__GPIOC_CLK_ENABLE();			
		__GPIOD_CLK_ENABLE();			
		__GPIOE_CLK_ENABLE();		
		__TIM1_CLK_ENABLE(); 		
		__TIM2_CLK_ENABLE(); 		 
		__TIM3_CLK_ENABLE(); 			
		__TIM4_CLK_ENABLE(); 			
		__TIM5_CLK_ENABLE();    
		__TIM9_CLK_ENABLE(); 	 
		__I2C1_CLK_ENABLE();
						//---GPIO init cho 4 led sang--------gpio init cho button user-----------------------------------
		Init_LEDSANG_PORTD_12_13_14_15(); 	Init_BUTTON_USER_PORT_A_0();
		
					//---Setting cho 4 PIN of PWM		//Khoi tao timer 3//cau hinh timer 3 voi mode output PWM
		Init_TIM3_OUTPUT_COMPARE_4_Channel();		


		
					//---Code config cho motor, luc dau tao clock PWM max 2000ms trong vong 2s, sau do giam xuong 700ms
		TIM3->CCR1 = 2000; 		
		TIM3->CCR2 = 2000;		 
		TIM3->CCR3 = 2000;		 
		TIM3->CCR4 = 2000;		
		SANG_4_LED(); delay_ms(2000); 		
		TIM3->CCR1 =  700; 		
		TIM3->CCR2 =  700;			
		TIM3->CCR3 =  700;			
		TIM3->CCR4 =  700;		
		SANG_4_LED_OFF();		
		
					//---Config DEVO 7 RF module - INPUT CAPTURE MODE---------------------------------------------
		Init_Receiver_TIM_PWM_Capture_TIM1(); //PE9 ->TIM1_CH1  //Throttle (can ga)
		Init_Receiver_TIM_PWM_Capture_TIM2(); //PA5 ->TIM2_CH1  //Rudder  (xoay)
		Init_Receiver_TIM_PWM_Capture_TIM4(); //PB8 ->TIM4_CH3  //Elevator (tien - lui)
		Init_Receiver_TIM_PWM_Capture_TIM5(); //PA3 - TIM5_CH4  //Ailenron (trai - phai)	
		
					//---MPU6050 cau hinh PB6, PB7 doc cam bien mpu6050---------------------------------------------------			
		Init_I2C_GPIO_PortB();		
		Init_I2C_Handle_10truc();			
		TM_I2C_IS_DEVICE_CONNECTED();//khong connect dc => LED VANG sang lien tuc
				
					//---Doc gia tri cua WHO I AM register, if error => LED RED(14) sang nhap nhay
		who_i_am_reg_value_MPU6050 = TM_I2C_WHO_I_AM( MPU6050_I2C_ADDR, MPU6050_WHO_AM_I_REGISTER);
		if( who_i_am_reg_value_MPU6050 != MPU6050_I_AM_VALUES )	{ Error_Handler_Custom(	ERROR_MPU6050_NOT_I_AM_VALUES ); }	
		
					//---Setting/config cho MPU6050-----------------------------------------------------------
		TM_I2C_WRITE( MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x00); 
		TM_MPU6050_SetDataRate( MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, TM_MPU6050_DataRate_1KHz); 
		TM_MPU6050_SetAccelerometer( MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, TM_MPU6050_Accelerometer_2G); 
		TM_MPU6050_SetGyroscope( MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, TM_MPU6050_Gyroscope_250s); 
		TM_MPU6050_ReadAll( MPU6050_I2C_ADDR, &mpu6050);
				
					//-----------Caculate Roll/Pitch/Yaw Angel------------------------------------------------------------------------		
		accX_angle  = ( atan2(-mpu6050.Acc_Y, mpu6050.Acc_Z)) * RAD_TO_DEG; 
		accY_angle =  ( atan2(mpu6050.Acc_X, sqrt(mpu6050.Acc_Y*mpu6050.Acc_Y + mpu6050.Acc_Z*mpu6050.Acc_Z) ) )* RAD_TO_DEG;
		
		//.........code dafault cua ARM		
		#ifdef RTE_CMSIS_RTOS 
			osKernelStart();      // when using CMSIS RTOS	// start thread execution 
		#endif
		Check_EveryThing_OK();		
		while(1)
		{		
				//---Khoi dong may bay	------------------------------------------------------------------------------
				while(FlyState == 0)
				{					
					Turn_On_Quadrotor();
				}		
				//---END Khoi dong may bay	-------------------------------------------------------------------------

				//---Read value from MPU6050
				TM_MPU6050_ReadAll( MPU6050_I2C_ADDR, &mpu6050); 
			
				//-----------Caculate Roll/Pitch/Yaw Angel------------------------------------------------------------------------		
				accX_angle  = ( atan2(-mpu6050.Acc_Y, mpu6050.Acc_Z)) * RAD_TO_DEG; //roll equation provides [-180, 180] range
				accY_angle =  ( atan2(mpu6050.Acc_X, sqrt(mpu6050.Acc_Y*mpu6050.Acc_Y + mpu6050.Acc_Z*mpu6050.Acc_Z) ) )* RAD_TO_DEG; //[-90, 90] range, which is exactly what is expected for the pitch angle
				
				gyroXrate = ((float)mpu6050.Gyro_X)/131;
				gyroYrate = ((float)mpu6050.Gyro_Y)/131;				
				//gyroX_angle += gyroXrate * DT; // Calculate gyro angle without any filter
				//gyroY_angle += gyroYrate * DT; // Calculate gyro angle without any filter
								
				Kalman_angelX = kalmanCalculate(&kalmanX, accX_angle, gyroXrate, DT);
				Kalman_angelY = kalmanCalculate(&kalmanY, accY_angle, gyroYrate, DT);
				//compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * accX_angle; // Calculate the angle using a Complimentary filter
				//-----------------------------------------------------------------------------------
				Sang_Led_By_MPU6050_Values(Kalman_angelX, Kalman_angelY, Kalman_angelZ);	
				
				
							
				
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)
				{		
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_SET){} 
					/*
					if(pwm_motor_1 < 1100)
						pwm_motor_1 = 1100;
					if(pwm_motor_1 > 2000)
						pwm_motor_1 = 1100;				
					pwm_motor_1 = pwm_motor_1 + 50;
					SetPWM_1_Motor(1, pwm_motor_1);
					*/
				}//khi nhan buttun USER ma chua tha ra -> khong lam gi
			
			
				//---Quadrotor Fly------------------------------------------------------------------
				if(FlyState == 1)
				{
					//khi ga nho nhat, keo can gat 5s thi tat may bay
					if( (IC_Throttle_pusle_width         >= PWM_ON_OFF_MIN && IC_Throttle_pusle_width         <= PWM_ON_OFF_MAX) && 
							(IC_Aileron_TraiPhai_pusle_width >= PWM_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= PWM_ON_OFF_MAX) && 
							(IC_Elevator_TienLui_pusle_width >= PWM_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= PWM_ON_OFF_MAX) &&
							(IC_Rudder_Xoay_pusle_width      >= PWM_ON_OFF_MIN && IC_Rudder_Xoay_pusle_width      <= PWM_ON_OFF_MAX)
					)
					{
								Turn_Off_Quadrotor();
					}
					//trang thai Can Bang, k co tac dong cua receiver
					else if( (IC_Aileron_TraiPhai_pusle_width >= PWM_Effect_Min && IC_Aileron_TraiPhai_pusle_width <= PWM_Effect_Max ) &&
									 (IC_Elevator_TienLui_pusle_width >= PWM_Effect_Min && IC_Elevator_TienLui_pusle_width <= PWM_Effect_Max) &&
									 (IC_Rudder_Xoay_pusle_width      >= PWM_Effect_Min && IC_Rudder_Xoay_pusle_width      <= PWM_Effect_Max)
						)
					{			
								//vao trang thai can bang, k co tac dong tu receiver, PID controller dieu chinh can bang	
								//fuzzy dieu khien can bang						
								//	  (1)\   /(2)
								//        \ /				        y
								//         X				        |
								//        / \          x____|
								//    (4)/   \(3)				
								
								//-----FUZZY SET Membership Function-------------------------------------------------------------------------
								Fuzzification_All_MF( (float) Kalman_angelX, &rollFuzzyControl);
								Fuzzification_All_MF( (float) Kalman_angelY , &pitchFuzzyControl);						
								Apply_All_Rule( 				  &rollFuzzyControl );
								Apply_All_Rule( 				  &pitchFuzzyControl );								
								Defuzzification( 				  &rollFuzzyControl );				
								Defuzzification( 				  &pitchFuzzyControl );
								
								//Set PWM 4 rotor
								pwm_motor_1 = IC_Throttle_pusle_width + rollFuzzyControl.output + pitchFuzzyControl.output;
								pwm_motor_2 = IC_Throttle_pusle_width + rollFuzzyControl.output - pitchFuzzyControl.output;				
								pwm_motor_3 = IC_Throttle_pusle_width - rollFuzzyControl.output + pitchFuzzyControl.output;
								pwm_motor_4 = IC_Throttle_pusle_width - rollFuzzyControl.output - pitchFuzzyControl.output;
								
								SetPWM_1_Motor(1, pwm_motor_1);
								SetPWM_1_Motor(2, pwm_motor_2);
								SetPWM_1_Motor(3, pwm_motor_3);
								SetPWM_1_Motor(4, pwm_motor_4);
								delay_ms(50);
					}
					else
					{		//co tac dong tu receiver, k phai trang thai CAN BANG
							Direct_Quadrotor_By_Receiver();
					}					
				}
				//---END Quadrotor Fly------------------------------------------------------------------
		}		
		//End while(1)
}

void SetInitDataQuadrotor(void)
{
		who_i_am_reg_value_MPU6050 = 0;
		
		//set kalman filter	
		kalmanX.Q_angle  =  0.001;  //0.001    //0.005
		kalmanX.Q_gyro   =  0.003;  //0.003    //0.0003
		kalmanX.R_angle  =  0.03;  //0.03     //0.008
		kalmanX.bias     =  0;
		kalmanX.P_00     =  0;
		kalmanX.P_01     =  0;
		kalmanX.P_10     =  0; 
		kalmanX.P_11     =  0;
	
		kalmanY.Q_angle  =  0.001;  //0.001    //0.005
		kalmanY.Q_gyro   =  0.003;  //0.003    //0.0003
		kalmanY.R_angle  =  0.03;  //0.03     //0.008
		kalmanY.bias     =  0;
		kalmanY.P_00     =  0;
		kalmanY.P_01     =  0;
		kalmanY.P_10     =  0; 
		kalmanY.P_11     =  0;
		
		kalmanZ.Q_angle  =  0.001;  //0.001    //0.005
		kalmanZ.Q_gyro   =  0.003;  //0.003    //0.0003
		kalmanZ.R_angle  =  0.03;  //0.03     //0.008
		kalmanZ.bias     =  0;
		kalmanZ.P_00     =  0;
		kalmanZ.P_01     =  0;
		kalmanZ.P_10     =  0; 
		kalmanZ.P_11     =  0;
		
		//set input capture
		IC_Throttle1 = 0;
		IC_Throttle2 = 0; 
		IC_Throttle_pusle_width = 0;
	
		IC_Elevator_TienLui1 = 0;
		IC_Elevator_TienLui2 = 0; 
		IC_Elevator_TienLui_pusle_width = 0;
	
		IC_Aileron_TraiPhai1 = 0;
		IC_Aileron_TraiPhai2 = 0;
		IC_Aileron_TraiPhai_pusle_width = 0;
	
		IC_Rudder_Xoay1 = 0;
		IC_Rudder_Xoay2 = 0;
		IC_Rudder_Xoay_pusle_width = 0;
		
		FlyState = 0;
		
		//SetPWM_4_Motor(0);
		//set pid controller
		//pid_setup_gain(&pid_roll,   ROLL_PID_KP,  ROLL_PID_KI,  ROLL_PID_KD);
		//pid_setup_gain(&pid_pitch,  PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD);
		//pid_setup_error(&pid_roll);
		//pid_setup_error(&pid_pitch);
}
void Turn_On_Quadrotor(void)
{
		int i=0;
		int timedelay = 200;
		if( (IC_Throttle_pusle_width             >= PWM_ON_OFF_MIN && IC_Throttle_pusle_width         <= PWM_ON_OFF_MAX) && 
						(IC_Aileron_TraiPhai_pusle_width >= PWM_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= PWM_ON_OFF_MAX) && 
						(IC_Elevator_TienLui_pusle_width >= PWM_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= PWM_ON_OFF_MAX) 
				)
				{
						while(i<3)
						{
							SANG_1_LED(12); delay_ms(timedelay);
							SANG_1_LED(13); delay_ms(timedelay);
							SANG_1_LED(14); delay_ms(timedelay);
							SANG_1_LED(15); delay_ms(timedelay);
							i++;
						}
						SANG_4_LED(); 	
						delay_ms(1000);  
						SANG_4_LED_OFF();						
						if( (IC_Throttle_pusle_width         >= PWM_ON_OFF_MIN && IC_Throttle_pusle_width         <= PWM_ON_OFF_MAX) && 
								(IC_Aileron_TraiPhai_pusle_width >= PWM_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= PWM_ON_OFF_MAX) && 
								(IC_Elevator_TienLui_pusle_width >= PWM_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= PWM_ON_OFF_MAX) 
						)
						{
								//Khoi dong quadrotor
								SANG_4_LED_OFF();
								FlyState = 1;
								SetPWM_4_Motor(1200);
							
								SANG_4_LED(); 	
								delay_ms(2000);  
								SANG_4_LED_OFF();
						}
				}
		else
		{
			SANG_4_LED();
			delay_ms(300);
			SANG_4_LED_OFF();
			delay_ms(300);
		}
}

void Turn_Off_Quadrotor(void) //tat quadrotor
{
	int i=0;
	int timedelay = 500;
						while(i<3)
						{
							SANG_1_LED(15); delay_ms(timedelay);
							SANG_1_LED(14); delay_ms(timedelay);
							SANG_1_LED(13); delay_ms(timedelay);
							SANG_1_LED(12); delay_ms(timedelay);
							i++;
						}
						SANG_4_LED(); 	delay_ms(1000);  SANG_4_LED_OFF();
						if( (IC_Throttle_pusle_width         >= PWM_ON_OFF_MIN && IC_Throttle_pusle_width         <= PWM_ON_OFF_MAX) && 
								(IC_Aileron_TraiPhai_pusle_width >= PWM_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= PWM_ON_OFF_MAX) && 
								(IC_Elevator_TienLui_pusle_width >= PWM_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= PWM_ON_OFF_MAX) &&
								(IC_Rudder_Xoay_pusle_width      >= PWM_ON_OFF_MIN && IC_Rudder_Xoay_pusle_width      <= PWM_ON_OFF_MAX)
						)
						{				
								SANG_4_LED_OFF();
								FlyState = 0;
								SetPWM_4_Motor(0); //stop all motor
						}
}
//
//
//Timer 1 PWM input capture
//
void Init_Receiver_TIM_PWM_Capture_TIM1(void)
{
	GPIO_InitTypeDef 						GPIO_PWM_InputCapture;	
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
	
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	
	GPIO_PWM_InputCapture.Pin 			= GPIO_PIN_9;
	GPIO_PWM_InputCapture.Mode 			= GPIO_MODE_AF_PP; 
	GPIO_PWM_InputCapture.Pull 			= GPIO_NOPULL;
	GPIO_PWM_InputCapture.Speed 		= GPIO_SPEED_FAST;
	GPIO_PWM_InputCapture.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOE, &GPIO_PWM_InputCapture);

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim1);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1);
	//__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	if(HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)		
	{
		Error_Handler_Custom(ERROR_TIM_1_INPUTCAPTURE);
		//Error_Handler();			
	}	
}	

void Init_Receiver_TIM_PWM_Capture_TIM2(void)
{
	GPIO_InitTypeDef 						GPIO_PWM_InputCapture;	
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	//PA5 ->TIM2_CH1
	GPIO_PWM_InputCapture.Pin 			= GPIO_PIN_5;
	GPIO_PWM_InputCapture.Mode 			= GPIO_MODE_AF_PP; 
	GPIO_PWM_InputCapture.Pull 			= GPIO_NOPULL;
	GPIO_PWM_InputCapture.Speed 		= GPIO_SPEED_FAST;
	GPIO_PWM_InputCapture.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(GPIOA, &GPIO_PWM_InputCapture);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim2);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);
	if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)		{							Error_Handler();			}
}	

void Init_Receiver_TIM_PWM_Capture_TIM4(void)
{
	GPIO_InitTypeDef 						GPIO_PWM_InputCapture;	
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
	
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	//PB8 -> TIM4_CH3
	GPIO_PWM_InputCapture.Pin 			= GPIO_PIN_8;
	GPIO_PWM_InputCapture.Mode 			= GPIO_MODE_AF_PP; 
	GPIO_PWM_InputCapture.Pull 			= GPIO_NOPULL;
	GPIO_PWM_InputCapture.Speed 		= GPIO_SPEED_FAST;
	GPIO_PWM_InputCapture.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(GPIOB, &GPIO_PWM_InputCapture);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim4);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
	//PB8 -> TIM4_CH3
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
	if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK)		{							Error_Handler();			}
}

void Init_Receiver_TIM_PWM_Capture_TIM5(void) //PA1 - TIM5_CH2
{
	GPIO_InitTypeDef 						GPIO_PWM_InputCapture;	
	TIM_ClockConfigTypeDef 			sClockSourceConfig;
  TIM_SlaveConfigTypeDef 			sSlaveConfig;
  TIM_MasterConfigTypeDef 		sMasterConfig;
  TIM_IC_InitTypeDef 					sConfigIC;
	
	HAL_NVIC_SetPriority( TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ( TIM5_IRQn);
	
	
	GPIO_PWM_InputCapture.Pin 			= GPIO_PIN_3;
	GPIO_PWM_InputCapture.Mode 			= GPIO_MODE_AF_PP; 
	GPIO_PWM_InputCapture.Pull 			= GPIO_NOPULL;
	GPIO_PWM_InputCapture.Speed 		= GPIO_SPEED_FAST;
	GPIO_PWM_InputCapture.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(GPIOA, &GPIO_PWM_InputCapture);

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim5);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim5);

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET; // TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;// TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchronization(&htim5, &sSlaveConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
	
  HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4);
	if(HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4) != HAL_OK)		{							Error_Handler();			}
}	


void TIM1_CC_IRQHandler(void) //PE9 ->TIM1_CH1
{
					//khi co interrup la TIM_IT_CC1, co su kien capture canh	
						if(__HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_CC1) == SET)
						{		
									__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
									__HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1);	
									if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)== SET)
									{ //ngat canh len
										IC_Throttle1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
									}else if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)== RESET)
									{//ngat canh xuong
										IC_Throttle2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
										IC_Throttle_pusle_width		= (IC_Throttle2 - IC_Throttle1);
										TIM1->CNT = 0;
									}					
						}
}

void TIM2_IRQHandler(void) //PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw
{
	if(__HAL_TIM_GET_ITSTATUS(&htim2, TIM_IT_CC1) == SET)
			{		
						__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
						__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);		
									if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)== SET)
									{ //ngat canh len
										IC_Rudder_Xoay1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
									}else if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)== RESET)
									{//ngat canh xuong
										IC_Rudder_Xoay2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
										IC_Rudder_Xoay_pusle_width		= (IC_Rudder_Xoay2 - IC_Rudder_Xoay1);
										TIM2->CNT = 0;
									}						
			}
}
void TIM4_IRQHandler(void) //PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch
{ 	
			if(__HAL_TIM_GET_ITSTATUS(&htim4, TIM_IT_CC3) == SET)
			{		
						__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
						__HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_CC3);		
						if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== SET)
							{ //ngat canh len
								IC_Elevator_TienLui1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
							}else if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)== RESET)
							{//ngat canh xuong
								IC_Elevator_TienLui2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
								IC_Elevator_TienLui_pusle_width = (IC_Elevator_TienLui2 - IC_Elevator_TienLui1);
								TIM4->CNT = 0;
							}					
			}
}
void TIM5_IRQHandler(void) //PA3 - TIM5_CH4 ////Aileron_TraiPhai (trai - phai) - goc Roll
{
			if(__HAL_TIM_GET_ITSTATUS(&htim5, TIM_IT_CC4) == SET)
			{		
						__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC4);
						__HAL_TIM_CLEAR_FLAG(&htim5, TIM_IT_CC4);	
						if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== SET)
						{ //ngat canh len
							IC_Aileron_TraiPhai1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
						}else if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== RESET)
						{//ngat canh xuong
							IC_Aileron_TraiPhai2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
							IC_Aileron_TraiPhai_pusle_width		= IC_Aileron_TraiPhai2 - IC_Aileron_TraiPhai1;
							TIM5->CNT = 0;
						}
			}
}
//
//End PWM input capture
//

//
//
//
//
void Check_EveryThing_OK(void)
{
		int i = 0;
		int time = 50;
		while(i < 5)
		{
			SANG_1_LED(12); delay_ms(time);
			SANG_1_LED(13); delay_ms(time);			
			SANG_1_LED(14); delay_ms(time);	
			SANG_1_LED(15); delay_ms(time);
			SANG_4_LED_OFF();			
			delay_ms(100);
			i++;
		}
		SANG_4_LED();
		delay_ms(1000);	
		SANG_4_LED_OFF();
}

void Init_LEDSANG_PORTD_12_13_14_15(void)
{
  GPIO_InitTypeDef PIN_LED_SANG_PORTD;
	PIN_LED_SANG_PORTD.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	PIN_LED_SANG_PORTD.Mode = GPIO_MODE_OUTPUT_PP;
	PIN_LED_SANG_PORTD.Pull = GPIO_NOPULL;
	PIN_LED_SANG_PORTD.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &PIN_LED_SANG_PORTD);
}

void Init_BUTTON_USER_PORT_A_0(void)
{
  GPIO_InitTypeDef BUTTON_USER_PORTA_0;	
	BUTTON_USER_PORTA_0.Pin = GPIO_PIN_0;
	BUTTON_USER_PORTA_0.Mode = GPIO_MODE_INPUT;
	BUTTON_USER_PORTA_0.Pull = GPIO_NOPULL;
	BUTTON_USER_PORTA_0.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &BUTTON_USER_PORTA_0);
}
//
//
//
//Timer 3 output PWM ra 4 channel
//

void Init_TIM3_OUTPUT_COMPARE_4_Channel()
{
		GPIO_InitTypeDef GPIO_PWM_PORTC_6789;			
		TIM_OC_InitTypeDef  TIM_Output_compare;
	
		//Init GPIOC cho 4 channel		
		GPIO_PWM_PORTC_6789.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_PWM_PORTC_6789.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORTC_6789.Pull = GPIO_NOPULL;
		GPIO_PWM_PORTC_6789.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_PWM_PORTC_6789.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOC, &GPIO_PWM_PORTC_6789);
		//-----------------------------------------
	
		//config tim3 handle	
		Tim3_Handle_PWM.Instance = TIM3;
		Tim3_Handle_PWM.Init.Prescaler = 84-1; 									//vi timer 3 co max clock la 84MHz, -1 la vi dem(count) tu 0
		Tim3_Handle_PWM.Init.CounterMode = TIM_COUNTERMODE_UP; 	//dem len
		Tim3_Handle_PWM.Init.Period = 20000-1;									//Period(chu ki) = 20 mili s
		Tim3_Handle_PWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// =0
		HAL_TIM_PWM_Init(&Tim3_Handle_PWM);										//Init PWM voi TIM HANDLE
		//----------------------------------------------------
	
		
		TIM_Output_compare.OCMode 				= TIM_OCMODE_PWM1;
		//TIM_Output_compare.OCIdleState 		= TIM_OCIDLESTATE_SET;
		TIM_Output_compare.Pulse 					= 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity 		= TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode 		= TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_1); //config PWM cho channel 1 (PORTC.6)
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_2); //config PWM cho channel 2 (PORTC.7)
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_3); //config PWM cho channel 3 (PORTC.8)
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_4); //config PWM cho channel 4 (PORTC.9)
	
		/*
		//TIM_OC_InitTypeDef  TIM_Output_compare;
		TIM_Output_compare.OCMode 				= TIM_OCMODE_PWM1;
		TIM_Output_compare.Pulse 					= 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity 		= TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode 		= TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_2); //config PWM cho channel 1 (PORTC.7)

		
				
		//TIM_OC_InitTypeDef  TIM_Output_compare;
		TIM_Output_compare.OCMode 				= TIM_OCMODE_PWM1;
		TIM_Output_compare.Pulse 					= 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity 		= TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode 		= TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_3); //config PWM cho channel 1 (PORTC.8)
				
		
		
		//TIM_OC_InitTypeDef  TIM_Output_compare;
		TIM_Output_compare.OCMode 				= TIM_OCMODE_PWM1;
		TIM_Output_compare.Pulse 					= 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity 		= TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode 		= TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_4); //config PWM cho channel 1 (PORTC.9)
		*/
		
		//----------------------------------
		HAL_TIM_PWM_Init(&Tim3_Handle_PWM);
		//		HAL_TIM_PWM_MspInit(&Tim3_Handle_PWM);
		
		//---Khoi dong PWM motor---------------------------------------
		HAL_TIM_Base_Start(&Tim3_Handle_PWM);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_1);			//HAL_TIMEx_PWMN_Start(&Tim3_Handle_PWM,TIM_CHANNEL_1);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_2); 
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_3);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_4);
}
//
//
//
//
//
//Accelerametor 10truc mpu6050
//
void Init_I2C_GPIO_PortB(void)
{
		//I2C1 GPIO Configuration    
    //PB6     ------> I2C1_SCL
    //PB7     ------> I2C1_SDA     
		GPIO_InitTypeDef 	GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);		
}
void Init_I2C_Handle_10truc()
{
	I2C_Handle_10truc.Instance = I2C1;
  I2C_Handle_10truc.Init.ClockSpeed = 100000;
  I2C_Handle_10truc.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_Handle_10truc.Init.OwnAddress1 = 0;
  I2C_Handle_10truc.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_Handle_10truc.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_Handle_10truc.Init.OwnAddress2 = 0;
  I2C_Handle_10truc.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_Handle_10truc.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&I2C_Handle_10truc);	
	__HAL_I2C_ENABLE(&I2C_Handle_10truc);
}
void TM_I2C_IS_DEVICE_CONNECTED()
{
	if(HAL_I2C_IsDeviceReady(&I2C_Handle_10truc, MPU6050_I2C_ADDR, 2, 5) != HAL_OK) 
	{
			Error_Handler_Custom(ERROR_MPU6050_NOT_CONNECT);
	}
	while( HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY )
	{
		Error_Handler_Custom(ERROR_MPU6050_STATE_READY_NOT_OK);
		
	}
}

uint8_t TM_I2C_WHO_I_AM(uint8_t device_address, uint8_t register_address)
{	
	uint8_t data;
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}		
	
	if (HAL_I2C_Master_Receive(&I2C_Handle_10truc, device_address, &data, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
	return data;
}


void TM_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data) 
{
	uint8_t d[2];		
	d[0] = register_address;
	d[1] = data;	
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}	
}
	
	
void TM_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data)
{
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(&I2C_Handle_10truc, device_address, data, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
}


void TM_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)  
{
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(&I2C_Handle_10truc, device_address, data, count, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}	
}


void TM_MPU6050_SetDataRate(uint8_t device_address, uint8_t register_address, uint8_t rate) 
{
	TM_I2C_WRITE( device_address, register_address, rate);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value) 
{
	uint8_t temp;		
	TM_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Acc_8G_Value << 3;
	TM_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_SetGyroscope( uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value) 
{
	uint8_t temp;		
	TM_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Gyro_250s_Value << 3;
	TM_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_ReadAccelerometer( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	TM_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
		
	output->Acc_X = (int16_t)(data[0] << 8 | data[1]);
	output->Acc_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Acc_Z = (int16_t)(data[4] << 8 | data[5]);
}

void TM_MPU6050_ReadGyroscope( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	TM_I2C_READ_MULTI( device_address, MPU6050_GYRO_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
		
	output->Gyro_X = (int16_t)(data[0] << 8 | data[1]);
	output->Gyro_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Gyro_Z = (int16_t)(data[4] << 8 | data[5]);
}	

void TM_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[14];
	int16_t temp;	
	
	TM_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 14); /* Read full raw data, 14bytes */
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}	
	
	output->Acc_X = (int16_t)(data[0] << 8 | data[1]);	/* Format accelerometer data */
	output->Acc_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Acc_Z = (int16_t)(data[4] << 8 | data[5]);
	
	temp = (data[6] << 8 | data[7]); /* Format temperature */
	output->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	output->Gyro_X = (int16_t)(data[8] << 8 | data[9]); /* Format gyroscope data */
	output->Gyro_Y = (int16_t)(data[10] << 8 | data[11]);
	output->Gyro_Z = (int16_t)(data[12] << 8 | data[13]);
}



void Sang_Led_By_MPU6050_Values(float kalman_angel_x, float kalman_angel_y, float kalman_angel_z )
{
	SANG_4_LED_OFF();
	if(kalman_angel_x > 10)
	{
		LED_D_15_HIGH;		 //SANG_1_LED(LED_YELLOW);
	}else if(kalman_angel_x < -10)
	{
		LED_D_13_HIGH;			//SANG_1_LED(LED_RED);
	}
	
	if(kalman_angel_y > 10)
	{
		LED_D_14_HIGH;	//SANG_1_LED(LED_BLUE);  
	}else if(kalman_angel_y < -10)
	{
		LED_D_12_HIGH;	//SANG_1_LED(LED_ORANGE);  
	}
	delay_ms(10);
}
/*
void Calculate_Accel_X_Angles(TM_MPU6050_t* output, float* angel_x)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);	
		float ACCEL_XOUT = (float)(output->Acc_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Acc_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Acc_Z/MPU6050_ACCE_SENS_2);
	
		_sqrt = (float)sqrt( ACCEL_ZOUT*ACCEL_ZOUT + ACCEL_XOUT*ACCEL_XOUT );	
		*angel_x = (float)(mot_180_do_chia_pi * (float)atan(ACCEL_YOUT/_sqrt));
}

void Calculate_Accel_Y_Angles(TM_MPU6050_t* output, float* angel_y)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);	
		float ACCEL_XOUT = (float)(output->Acc_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Acc_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Acc_Z/MPU6050_ACCE_SENS_2);
		_sqrt = (float)sqrt( ACCEL_ZOUT*ACCEL_ZOUT + ACCEL_YOUT*ACCEL_YOUT );
		*angel_y = (float)(mot_180_do_chia_pi * (float)-atan(ACCEL_XOUT /_sqrt));
}

	
void Calculate_Accel_Z_Angles(TM_MPU6050_t* output, float* angel_z)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);
		float ACCEL_XOUT = (float)(output->Acc_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Acc_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Acc_Z/MPU6050_ACCE_SENS_2);
		_sqrt= (float)sqrt( ACCEL_YOUT*ACCEL_YOUT + ACCEL_XOUT*ACCEL_XOUT );
		*angel_z = (float)(mot_180_do_chia_pi * atan(_sqrt/ACCEL_ZOUT));
}
*/
//end Accelerametor 10truc
//
//
//


//
//Lay gia tri cua Receiver de dieu khien 4 motor
//
//
void UpdatePWM_4_Motor_By_TIM_CCRx(void) //update lai speed
{
		TIM3->CCR1 = pwm_motor_1;
		TIM3->CCR2 = pwm_motor_2;
		TIM3->CCR3 = pwm_motor_3;
		TIM3->CCR4 = pwm_motor_4;
}

void SetPWM_4_Motor(int16_t value) //set 4 motor cung 1 speed
{
	if(value==0)
	{
					pwm_motor_1 = 0;	pwm_motor_2 = 0;		pwm_motor_3 = 0;		pwm_motor_4 = 0;		
					UpdatePWM_4_Motor_By_TIM_CCRx();
	}
	else{
			if(value <= PWM_Throtte_Min)
			{
					pwm_motor_1 = PWM_Throtte_Min;					pwm_motor_2 = PWM_Throtte_Min;					
					pwm_motor_3 = PWM_Throtte_Min;					pwm_motor_4 = PWM_Throtte_Min;
			}else if(value >= PWM_Throtte_Max)
			{
					pwm_motor_1 = PWM_Throtte_Max;					pwm_motor_2 = PWM_Throtte_Max;
					pwm_motor_3 = PWM_Throtte_Max;					pwm_motor_4 = PWM_Throtte_Max;
			}else{
					pwm_motor_1 = value;										pwm_motor_2 = value;
					pwm_motor_3 = value;										pwm_motor_4 = value;
			}	
			UpdatePWM_4_Motor_By_TIM_CCRx();
	}
}

		
void SetPWM_1_Motor(int16_t numberMotor, int16_t newValue) //set speed cho moi motor rieng le
{
	switch(numberMotor) 
	{
		 case 1  :
				if(newValue <= PWM_Throtte_Min)							{	pwm_motor_1 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{ pwm_motor_1 = PWM_Throtte_Max;}
				else 																				{ pwm_motor_1 = newValue;					}
				//TIM3->CCR1 = pwm_motor_1;
				break; 
		
		 case 2  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_2 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_2 = PWM_Throtte_Max;}
				else 																				{pwm_motor_2 = newValue;	}
				//TIM3->CCR2 = pwm_motor_2;
				break; 
		 
		 case 3  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_3 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_3 = PWM_Throtte_Max;}
				else 																				{pwm_motor_3 = newValue;	}
				//TIM3->CCR3 = pwm_motor_3;
				break; 
		
		 case 4  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_4 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_4 = PWM_Throtte_Max;}
				else 																				{pwm_motor_4 = newValue;	}
				//TIM3->CCR4 = pwm_motor_4;
				break; 
	}
	UpdatePWM_4_Motor_By_TIM_CCRx();
}



void SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue)
{
	if(changeValue <= 0 ) return;
	switch(numberMotor) 
	{
		 case 1  :
				SetPWM_1_Motor(1, (IC_Throttle_pusle_width + changeValue) );
				break; 
		
		 case 2  :
				SetPWM_1_Motor(2, (IC_Throttle_pusle_width + changeValue) );
				break; 
		 
		 case 3  :
				SetPWM_1_Motor(3, (IC_Throttle_pusle_width + changeValue) );
				break; 
		
		 case 4  :
				SetPWM_1_Motor(4, (IC_Throttle_pusle_width + changeValue) );
				break; 
	}
	
}

void SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue)
{
	if(changeValue <= 0 ) return;
	switch(numberMotor) 
	{
		 case 1  :
				SetPWM_1_Motor(1, (IC_Throttle_pusle_width - changeValue) );
				break; 
		
		 case 2  :
				SetPWM_1_Motor(2, (IC_Throttle_pusle_width - changeValue) );
				break; 
		 
		 case 3  :
				SetPWM_1_Motor(3, (IC_Throttle_pusle_width - changeValue) );
				break; 
		
		 case 4  :
				SetPWM_1_Motor(4, (IC_Throttle_pusle_width - changeValue) );
				break; 
	}
}

void Direct_Quadrotor_By_Receiver(void)
{
	int16_t chenhLechGiaTri = 0;
	//Ga, tang-giam Ga
	if(IC_Throttle_pusle_width >= 1000 && IC_Throttle_pusle_width <= 2000)
	{
		SetPWM_4_Motor(IC_Throttle_pusle_width);
		delay_ms(50);
	}
	
	//Tien - Lui
	if(IC_Elevator_TienLui_pusle_width >= 1000 && IC_Elevator_TienLui_pusle_width <= 2000)
	{
		//truong hop can dieu khien bi lech, thi phai xu ly trong long vap While
		//lay tin hieu cua recevier thay doi PWM 
		//cho den khi can dieu khien o vi tri can bang
		while(IC_Elevator_TienLui_pusle_width <= PWM_Effect_Min )
		{
					//Tien ve truoc, giam dong co 1+2 va tang dong co 3+4			
					chenhLechGiaTri = PWM_Avg - IC_Elevator_TienLui_pusle_width;
					SetPWM_Motor_Giam(1,chenhLechGiaTri);
					SetPWM_Motor_Giam(2,chenhLechGiaTri);
					SetPWM_Motor_Tang(3,chenhLechGiaTri);
					SetPWM_Motor_Tang(4,chenhLechGiaTri);
					delay_ms(50);
		}
			
		while(IC_Elevator_TienLui_pusle_width >= PWM_Effect_Max )
		{
					//Lui ve phia sau, giam dong co 3+4 va tang dong co 1+2
					chenhLechGiaTri = IC_Elevator_TienLui_pusle_width - PWM_Avg;
					SetPWM_Motor_Giam(3,chenhLechGiaTri);
					SetPWM_Motor_Giam(4,chenhLechGiaTri);
					SetPWM_Motor_Tang(1,chenhLechGiaTri);
					SetPWM_Motor_Tang(2,chenhLechGiaTri);			
					delay_ms(50);
		}			
		
	}
	
	//Trai - Phai
	if(IC_Aileron_TraiPhai_pusle_width >= 1000 && IC_Aileron_TraiPhai_pusle_width <= 2000)
	{
		while(IC_Aileron_TraiPhai_pusle_width <= PWM_Effect_Min )
		{
					//qua Trai, giam dong co 1+4 va tang dong co 2+3			
					chenhLechGiaTri = PWM_Avg - IC_Aileron_TraiPhai_pusle_width;
					SetPWM_Motor_Giam(1,chenhLechGiaTri);
					SetPWM_Motor_Giam(4,chenhLechGiaTri);
					SetPWM_Motor_Tang(2,chenhLechGiaTri);
					SetPWM_Motor_Tang(3,chenhLechGiaTri);
					delay_ms(50);
		}

		while(IC_Aileron_TraiPhai_pusle_width >= PWM_Effect_Max )
		{
					//qua Phai, giam dong co 2+3 va tang dong co 1+4
					chenhLechGiaTri = IC_Aileron_TraiPhai_pusle_width - PWM_Avg;
					SetPWM_Motor_Giam(2,chenhLechGiaTri);
					SetPWM_Motor_Giam(3,chenhLechGiaTri);
					SetPWM_Motor_Tang(1,chenhLechGiaTri);
					SetPWM_Motor_Tang(4,chenhLechGiaTri);		
					delay_ms(50);
		}
		//trong khoang 1470 - 1530 khong lam gi		
				
	}
	
	//Xoay
	if(IC_Rudder_Xoay_pusle_width >= 1000 && IC_Rudder_Xoay_pusle_width <= 2000)
	{		
		while(IC_Rudder_Xoay_pusle_width <= PWM_Effect_Min )
		{
			//Xoay cung chieu kim dong ho, giam dong co 1+3 va tang dong co 2+4			
			chenhLechGiaTri = PWM_Avg - IC_Rudder_Xoay_pusle_width;
			SetPWM_Motor_Giam(1,chenhLechGiaTri);
			SetPWM_Motor_Giam(3,chenhLechGiaTri);
			SetPWM_Motor_Tang(2,chenhLechGiaTri);
			SetPWM_Motor_Tang(4,chenhLechGiaTri);
			delay_ms(50);
		}
		
		while(IC_Rudder_Xoay_pusle_width >= PWM_Effect_Max )
		{
			//Xoay nguoc chieu kim dong ho, giam dong co 2+4 va tang dong co 1+3
			chenhLechGiaTri = IC_Rudder_Xoay_pusle_width - PWM_Avg;
			SetPWM_Motor_Giam(2,chenhLechGiaTri);
			SetPWM_Motor_Giam(4,chenhLechGiaTri);
			SetPWM_Motor_Tang(1,chenhLechGiaTri);
			SetPWM_Motor_Tang(3,chenhLechGiaTri);		
			delay_ms(50);
		}
		//trong khoang 1470 - 1530 khong lam gi				
	}
}



//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
//__________________________________________________________________________________
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}

static void Error_Handler(void)
{
  while(1)
  {			
		SANG_4_LED();
		delay_ms(200);
		SANG_4_LED_OFF();
		delay_ms(200);
	}
}



void Error_Handler_Custom(int type)
{

				if(type == ERROR_MPU6050_NOT_CONNECT )
				{
						while(1)
						{	//neu k connect dc MPU6050 => LED VANG sang lien tuc
									SANG_1_LED(12); 			delay_ms(50);			SANG_4_LED_OFF();			delay_ms(50);
						}
				}
				else
				if(type == ERROR_MPU6050_NOT_I_AM_VALUES )
				{	
						while(1)
						{ /*ERROR LED RED nhay lien tuc*/  
							SANG_1_LED(14); 	 delay_ms(50); SANG_4_LED_OFF();  delay_ms(50); 											
						}
				}
				else
				if(type == ERROR_MPU6050_STATE_READY_NOT_OK )
				{
					while(1)
						{
							//neu k connect dc MPU6050 => LED CAM sang lien tuc
							SANG_1_LED(13); 			delay_ms(50);			SANG_4_LED_OFF();			delay_ms(50);
						}
				}
				else 
				if(type == ERROR_TIM_1_INPUTCAPTURE ||
					 type == ERROR_TIM_2_INPUTCAPTURE || 
					 type == ERROR_TIM_4_INPUTCAPTURE || 
					 type == ERROR_TIM_5_INPUTCAPTURE 
				)
				{
					while(1)
					{ 
						/*ERROR LED RED nhay lien tuc*/  
						SANG_1_LED(14); 	 delay_ms(50); 						SANG_4_LED_OFF();  delay_ms(50); 					
					}
				}
				else
				{
						while(1)
						{ 
							/*ERROR LED RED nhay lien tuc*/  
							SANG_1_LED(14); 	 delay_ms(50); 							SANG_4_LED_OFF();  delay_ms(50); 					
						}
				}
		

}

void SANG_1_LED(int8_t PIN)
{
		LED_D_12_LOW;		LED_D_13_LOW;		LED_D_14_LOW;		LED_D_15_LOW;
		if(PIN==12) {			LED_D_12_HIGH;		}
		else if(PIN==13)		{			LED_D_13_HIGH;		}
		else if(PIN==14)		{			LED_D_14_HIGH;		}
		else if(PIN==15)		{			LED_D_15_HIGH;		}
}

void SANG_2_LED(int8_t type)
{
	LED_D_12_LOW;	LED_D_13_LOW;	LED_D_14_LOW;	LED_D_15_LOW;
	if(type==1)	{			LED_D_12_HIGH;		LED_D_14_HIGH;	}
	else if(type==2)	{		LED_D_13_HIGH;		LED_D_15_HIGH;	}
}

void SANG_4_LED()
{
		LED_D_12_HIGH;		LED_D_13_HIGH;		LED_D_14_HIGH;		LED_D_15_HIGH;
}
void SANG_4_LED_FOREVER()
{
		LED_D_12_HIGH;		LED_D_13_HIGH;		LED_D_14_HIGH;		LED_D_15_HIGH;		while(1){}
}
void SANG_4_LED_OFF()
{
		LED_D_12_LOW;		LED_D_13_LOW;		LED_D_14_LOW;		LED_D_15_LOW;
}



float kalmanCalculate(Kalman_Setting *kalman, float newAngle, float newRate, float DT_)
{
			float dt = (float)DT_;	
			kalman->angle += dt * (newRate - kalman->bias);

		//!    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
			kalman->P_00 += dt * (dt*kalman->P_11 - kalman->P_01 - kalman->P_10 + kalman->Q_angle);
		//!    P_01 +=  - dt * P_11;
			kalman->P_01 -= dt * kalman->P_11;    
		//!    P_10 +=  - dt * P_11;
			kalman->P_10 -= dt * kalman->P_11;
		//!    P_11 +=  + Q_gyro * dt;
			kalman->P_11 += kalman->Q_gyro * dt;

			kalman->S = kalman->P_00 + kalman->R_angle;
			
			kalman->K_0 = kalman->P_00 / kalman->S;
			kalman->K_1 = kalman->P_10 / kalman->S;
			
			kalman->y = newAngle - kalman->angle;
			
			kalman->angle +=  kalman->K_0 * kalman->y;
			kalman->bias  +=  kalman->K_1 * kalman->y;

			kalman->P_00 -= kalman->K_0 * kalman->P_00;
			kalman->P_01 -= kalman->K_0 * kalman->P_01;
			kalman->P_10 -= kalman->K_1 * kalman->P_00;
			kalman->P_11 -= kalman->K_1 * kalman->P_01;
			
			return kalman->angle;
}


















//------------------------------------------------------------------------------------------------------------------------------
//FUZZY SYSTEM //FUZZY SYSTEM//FUZZY SYSTEM//FUZZY SYSTEM
//FUZZY SYSTEM
void initFuzzySystem(void)
{
	rollFuzzyControl.pre_GocLech = 0;
	pitchFuzzyControl.pre_GocLech = 0;
	rollFuzzyControl.output = 0;
	pitchFuzzyControl.output = 0;
	
	//Buoc 1: Khai bao MF:
	/*GocLech*/
	setABCD_MF(&rollFuzzyControl.inGocLech[0],   -30,   -30,   -18,  -12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VerySmall
	setABCD_MF(&rollFuzzyControl.inGocLech[1],   -18,   -12,   -12,   -6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleSmall
	setABCD_MF(&rollFuzzyControl.inGocLech[2],   -12,    -6,    -6,    0, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Small
	setABCD_MF(&rollFuzzyControl.inGocLech[3],    -6,     0,     0,    6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.inGocLech[4],     0,     6,     6,   12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Big
	setABCD_MF(&rollFuzzyControl.inGocLech[5],     6,    12,    12,   18, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleBig
	setABCD_MF(&rollFuzzyControl.inGocLech[6],    12,    18,    30,   30, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VeryBig
	
	
	setABCD_MF(&pitchFuzzyControl.inGocLech[0],  -30,   -30,   -18,  -12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VerySmall
	setABCD_MF(&pitchFuzzyControl.inGocLech[1],  -18,   -12,   -12,   -6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleSmall
	setABCD_MF(&pitchFuzzyControl.inGocLech[2],  -12,    -6,    -6,    0, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Small
	setABCD_MF(&pitchFuzzyControl.inGocLech[3],   -6,     0,     0,    6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.inGocLech[4],    0,     6,     6,   12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Big
	setABCD_MF(&pitchFuzzyControl.inGocLech[5],    6,    12,    12,   18, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleBig
	setABCD_MF(&pitchFuzzyControl.inGocLech[6],   12,    18,    30,   30, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VeryBig
	

	/*GocLech_dot*/
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[0] ,  -10,   -10,   -6,   -4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryNagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[1] ,   -6,    -4,   -4,   -2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittleNagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[2] ,   -4,    -2,   -2,    0, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Nagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[3] ,   -2,     0,    0,    2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[4] ,    0,     2,    2,    4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Positive
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[5] ,    2,     4,    4,    6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittlePositive
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[6] ,    4,     6,   10,   10, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryPositive

	
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[0],  -10,   -10,   -6,   -4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryNagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[1],   -6,    -4,   -4,   -2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittleNagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[2],   -4,    -2,   -2,    0, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Nagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[3],   -2,     0,    0,    2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[4],    0,     2,    2,    4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Positive
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[5],    2,     4,    4,    6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittlePositive
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[6],    4,     6,   10,   10, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryPositive
	
	
	/*ValuePWMControl*/
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[0] , -200,  -200,  -120,   -80, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[1] , -120,   -80,   -80,   -40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[2] ,  -80,   -40,   -40,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[3] ,  -40,     0,     0,    40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[4] ,    0,    40,    40,    80, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[5] ,   40,    80,    80,   120, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[6] ,   80,   120,   200,   200, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast

	
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[0], -200,  -200,  -120,   -80, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[1], -120,   -80,   -80,   -40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[2],  -80,   -40,   -40,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[3],  -40,     0,     0,    40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[4],    0,    40,    40,    80, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[5],   40,    80,    80,   120, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[6],   80,   120,   200,   200, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast
	
	
	//Buoc 2: RULE FuzzySystem  --------------------------------------------	
	/*
		GocLech =     {VerySmall,    LittleSmall,    Small,    Zero, Big,      LittleBig,      VeryBig}
		GocLech_dot = {VeryNagative, LittleNagative, Nagative, Zero, Positive, LittlePositive, VeryPositive}
		ValuePWM =    {VerySlow,     LittleSlow,     Slow,     Zero, Fast,     LittleFast,     VeryFast}
	*/
	//ROLL
	setOneRule(&rollFuzzyControl.fuzzy_rules[0], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[6], 1);//rule 1
	setOneRule(&rollFuzzyControl.fuzzy_rules[1], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[6], 2);//rule 2
	setOneRule(&rollFuzzyControl.fuzzy_rules[2], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[5], 3);//rule 3
	setOneRule(&rollFuzzyControl.fuzzy_rules[3], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[5], 4);//rule 4
	setOneRule(&rollFuzzyControl.fuzzy_rules[4], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[4], 5);//rule 5
	setOneRule(&rollFuzzyControl.fuzzy_rules[5], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[4], 6);//rule 6
	setOneRule(&rollFuzzyControl.fuzzy_rules[6], &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[3], 7);//rule 7
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[7],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[6], 8);//rule 8
	setOneRule(&rollFuzzyControl.fuzzy_rules[8],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[5], 9);//rule 9
	setOneRule(&rollFuzzyControl.fuzzy_rules[9],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[5], 10);//rule 10
	setOneRule(&rollFuzzyControl.fuzzy_rules[10], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[4], 11);//rule 11
	setOneRule(&rollFuzzyControl.fuzzy_rules[11], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[4], 12);//rule 12
	setOneRule(&rollFuzzyControl.fuzzy_rules[12], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[3], 13);//rule 13
	setOneRule(&rollFuzzyControl.fuzzy_rules[13], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[2], 14);//rule 14
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[14], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[5], 15);//rule 15
	setOneRule(&rollFuzzyControl.fuzzy_rules[15], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[5], 16);//rule 16
	setOneRule(&rollFuzzyControl.fuzzy_rules[16], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[4], 17);//rule 17
	setOneRule(&rollFuzzyControl.fuzzy_rules[17], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[4], 18);//rule 18
	setOneRule(&rollFuzzyControl.fuzzy_rules[18], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[3], 19);//rule 19
	setOneRule(&rollFuzzyControl.fuzzy_rules[19], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[2], 20);//rule 20
	setOneRule(&rollFuzzyControl.fuzzy_rules[20], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[2], 21);//rule 21
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[21], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[5], 22);//rule 22
	setOneRule(&rollFuzzyControl.fuzzy_rules[22], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[4], 23);//rule 23
	setOneRule(&rollFuzzyControl.fuzzy_rules[23], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[4], 24);//rule 24
	setOneRule(&rollFuzzyControl.fuzzy_rules[24], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[3], 25);//rule 25
	setOneRule(&rollFuzzyControl.fuzzy_rules[25], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[2], 26);//rule 26
	setOneRule(&rollFuzzyControl.fuzzy_rules[26], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[2], 27);//rule 27
	setOneRule(&rollFuzzyControl.fuzzy_rules[27], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[1], 28);//rule 28
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[28], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[4], 29);//rule 29
	setOneRule(&rollFuzzyControl.fuzzy_rules[29], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[4], 30);//rule 30
	setOneRule(&rollFuzzyControl.fuzzy_rules[30], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[3], 31);//rule 31
	setOneRule(&rollFuzzyControl.fuzzy_rules[31], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[2], 32);//rule 32
	setOneRule(&rollFuzzyControl.fuzzy_rules[32], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[2], 33);//rule 33
	setOneRule(&rollFuzzyControl.fuzzy_rules[33], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[1], 34);//rule 34
	setOneRule(&rollFuzzyControl.fuzzy_rules[34], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[1], 35);//rule 35
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[35], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[4], 36);//rule 36
	setOneRule(&rollFuzzyControl.fuzzy_rules[36], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[3], 37);//rule 37
	setOneRule(&rollFuzzyControl.fuzzy_rules[37], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[2], 38);//rule 38
	setOneRule(&rollFuzzyControl.fuzzy_rules[38], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[2], 39);//rule 39
	setOneRule(&rollFuzzyControl.fuzzy_rules[39], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[1], 40);//rule 40
	setOneRule(&rollFuzzyControl.fuzzy_rules[40], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[1], 41);//rule 41
	setOneRule(&rollFuzzyControl.fuzzy_rules[41], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[0], 42);//rule 42

	setOneRule(&rollFuzzyControl.fuzzy_rules[42], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[3], 43);//rule 43
	setOneRule(&rollFuzzyControl.fuzzy_rules[43], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[2], 44);//rule 44
	setOneRule(&rollFuzzyControl.fuzzy_rules[44], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[2], 45);//rule 45
	setOneRule(&rollFuzzyControl.fuzzy_rules[45], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[1], 46);//rule 46
	setOneRule(&rollFuzzyControl.fuzzy_rules[46], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[1], 47);//rule 47
	setOneRule(&rollFuzzyControl.fuzzy_rules[47], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[0], 48);//rule 48
	setOneRule(&rollFuzzyControl.fuzzy_rules[48], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[0], 49);//rule 49
	
	
	
	//PITCH
	setOneRule(&pitchFuzzyControl.fuzzy_rules[0], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[6], 1);//rule 1
	setOneRule(&pitchFuzzyControl.fuzzy_rules[1], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[6], 2);//rule 2
	setOneRule(&pitchFuzzyControl.fuzzy_rules[2], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[5], 3);//rule 3
	setOneRule(&pitchFuzzyControl.fuzzy_rules[3], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[5], 4);//rule 4
	setOneRule(&pitchFuzzyControl.fuzzy_rules[4], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[4], 5);//rule 5
	setOneRule(&pitchFuzzyControl.fuzzy_rules[5], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[4], 6);//rule 6
	setOneRule(&pitchFuzzyControl.fuzzy_rules[6], &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[3], 7);//rule 7
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[7],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[6], 8);//rule 8
	setOneRule(&pitchFuzzyControl.fuzzy_rules[8],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[5], 9);//rule 9
	setOneRule(&pitchFuzzyControl.fuzzy_rules[9],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[5], 10);//rule 10
	setOneRule(&pitchFuzzyControl.fuzzy_rules[10], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[4], 11);//rule 11
	setOneRule(&pitchFuzzyControl.fuzzy_rules[11], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[4], 12);//rule 12
	setOneRule(&pitchFuzzyControl.fuzzy_rules[12], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[3], 13);//rule 13
	setOneRule(&pitchFuzzyControl.fuzzy_rules[13], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[2], 14);//rule 14
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[14], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[5], 15);//rule 15
	setOneRule(&pitchFuzzyControl.fuzzy_rules[15], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[5], 16);//rule 16
	setOneRule(&pitchFuzzyControl.fuzzy_rules[16], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[4], 17);//rule 17
	setOneRule(&pitchFuzzyControl.fuzzy_rules[17], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[4], 18);//rule 18
	setOneRule(&pitchFuzzyControl.fuzzy_rules[18], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[3], 19);//rule 19
	setOneRule(&pitchFuzzyControl.fuzzy_rules[19], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[2], 20);//rule 20
	setOneRule(&pitchFuzzyControl.fuzzy_rules[20], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[2], 21);//rule 21
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[21], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[5], 22);//rule 22
	setOneRule(&pitchFuzzyControl.fuzzy_rules[22], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[4], 23);//rule 23
	setOneRule(&pitchFuzzyControl.fuzzy_rules[23], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[4], 24);//rule 24
	setOneRule(&pitchFuzzyControl.fuzzy_rules[24], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[3], 25);//rule 25
	setOneRule(&pitchFuzzyControl.fuzzy_rules[25], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[2], 26);//rule 26
	setOneRule(&pitchFuzzyControl.fuzzy_rules[26], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[2], 27);//rule 27
	setOneRule(&pitchFuzzyControl.fuzzy_rules[27], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[1], 28);//rule 28
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[28], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[4], 29);//rule 29
	setOneRule(&pitchFuzzyControl.fuzzy_rules[29], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[4], 30);//rule 30
	setOneRule(&pitchFuzzyControl.fuzzy_rules[30], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[3], 31);//rule 31
	setOneRule(&pitchFuzzyControl.fuzzy_rules[31], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[2], 32);//rule 32
	setOneRule(&pitchFuzzyControl.fuzzy_rules[32], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[2], 33);//rule 33
	setOneRule(&pitchFuzzyControl.fuzzy_rules[33], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[1], 34);//rule 34
	setOneRule(&pitchFuzzyControl.fuzzy_rules[34], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[1], 35);//rule 35
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[35], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[4], 36);//rule 36
	setOneRule(&pitchFuzzyControl.fuzzy_rules[36], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[3], 37);//rule 37
	setOneRule(&pitchFuzzyControl.fuzzy_rules[37], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[2], 38);//rule 38
	setOneRule(&pitchFuzzyControl.fuzzy_rules[38], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[2], 39);//rule 39
	setOneRule(&pitchFuzzyControl.fuzzy_rules[39], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[1], 40);//rule 40
	setOneRule(&pitchFuzzyControl.fuzzy_rules[40], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[1], 41);//rule 41
	setOneRule(&pitchFuzzyControl.fuzzy_rules[41], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[0], 42);//rule 42

	setOneRule(&pitchFuzzyControl.fuzzy_rules[42], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[3], 43);//rule 43
	setOneRule(&pitchFuzzyControl.fuzzy_rules[43], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[2], 44);//rule 44
	setOneRule(&pitchFuzzyControl.fuzzy_rules[44], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[2], 45);//rule 45
	setOneRule(&pitchFuzzyControl.fuzzy_rules[45], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[1], 46);//rule 46
	setOneRule(&pitchFuzzyControl.fuzzy_rules[46], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[1], 47);//rule 47
	setOneRule(&pitchFuzzyControl.fuzzy_rules[47], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[0], 48);//rule 48
	setOneRule(&pitchFuzzyControl.fuzzy_rules[48], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[0], 49);//rule 49
	
}

void Fuzzification_All_MF(float x, FuzzyController * fuzzyController)
{
	//Buoc 3: mo hoa ngo~ vao. tinh degree cho tat ca MF
	int i,j;
	float xx;
	//Mo hoa input GocLech
	for (i=0; i < 7; i++) 
	{ 
		fuzzification( (float)x, &fuzzyController->inGocLech[i] );
		//fuzzification( (float)TEST_GOCLECH, &fuzzyController->inGocLech[i] );
	}
	
	//Mo hoa input GocLech_dot
	xx = (float)( x - fuzzyController->pre_GocLech );
	for (j=0; j < 7; j++) 
	{ 
		fuzzification( (float)xx,  &fuzzyController->inGocLech_dot[j] );
		//fuzzification( (float)TEST_GOCLECH_DOT,  &fuzzyController->inGocLech_dot[j] );
	}
	fuzzyController->pre_GocLech = x;
}




void Apply_All_Rule( FuzzyController * fuzzyController )
{ 
	//Buoc 4: apply rule: loop all rule and caculate output degree of Every Rule
	int i;
	for(i=0; i < NUMBER_RULE; i++)
	{
		//if(!fuzzyController->fuzzy_rules[i]) continue;
		calcule_H_and_Y_PerRule(&fuzzyController->fuzzy_rules[i]);
	}
}



void Defuzzification( FuzzyController * fuzzyController )
{
	//buoc 5
	//--------GIAI MO` De-fuzzify-------------------------------
	//De-fuzzify the fuzzy output functions to get "crisp" output values.
	float output = 0;
	int i;
	float sum_h = 0; 					//Sum degree of all MF
	float total_y = 0; 				//Sum y=(a+d)/2 of all MF
	for (i=0; i < NUMBER_RULE; i++) 
	{ 
		//if(!fuzzyController->fuzzy_rules[i]) continue;
		sum_h = 	(float)sum_h + fuzzyController->fuzzy_rules[i].h;
		total_y = (float)total_y + (float)fuzzyController->fuzzy_rules[i].y * fuzzyController->fuzzy_rules[i].h;
	}
	if(sum_h != 0)
	{
			output = (float)(total_y/sum_h);
	}else{
		output = 0;
	}
	fuzzyController->output = output;//PWM + hoac - mot gia tri output
}














//--------------------------------------------------------------------------------------------------------------------------------------------
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
