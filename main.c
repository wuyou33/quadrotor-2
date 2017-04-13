/*
	  (1)\   /(2)  ^^Head
        \ /			 ||       y
         X			 ||       |
        / \          x____|
    (4)/   \(3)				

TONG HOP CAC PIN SU DUNG
PORT A: 
			PA0 																	=> Button User
			PA3 																	=> TIM5_CH4  //Devo 7 Aileron_TraiPhai 	
			PA5																		=> TIM2_CH1  //Devo 7  Rudder Xoay
PORT B: 
			PB6, PB7 															=> I2C1 cam bien 10 truc mpu6050 (PB6->I2C1_SCL,	PB7->I2C1_SDA)
			PB8 																	=> TIM4_CH3  //Devo 7 Elevator
			PB4 pwm 1															=> PWM TIMER 3: dong co 1
			PB5 pwm 2															=> PWM TIMER 3: dong co 2
			PB0 pwm 3															=> PWM TIMER 3: dong co 3
			PB1 pwm 4															=> PWM TIMER 3: dong co 4		
PORT D: 
			PD12 
			PD13 
			PD14 
			PD15  																=> LEDSang (PD12 GREEN, PD13 ORANGE, PD14 RED, PD15 BLUE)
PORT E:
			PE9																		=>PE9 ->TIM1_CH1  //Devo 7 Throttle
--------------------------------------------------------------------------
InputCaptrue PIN:
PE9 ->TIM1_CH1  //Throttle (can ga) tang giam toc do quay				keo len +(1900), keo xuong +(1100)
PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw						keo qua trai +(1900), keo qua phai +(1100)
PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch. 						keo len +(1900), keo xuong +(1100)
PA3 ->TIM5_CH4  //Aileron_TraiPhai (trai - phai) - goc Roll     keo qua trai +(1900), keo qua phai +(1100)
--------------------------------------------------------------------------
Output PWM TIMER 3:
PB4 pwm 1
PB5 pwm 2
PB0 pwm 3
PB1 pwm 4
--------------------------------------------------------------------------
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
#include "nguyen_HMC5883L.h"

//----------Khai bao BIEN-------------------------------------------------------------------------
I2C_HandleTypeDef 				I2C_Handle_10truc;  //I2C handle, dung de doc value cua cam bien MPU6050
TIM_HandleTypeDef 				Tim3_Handle_PWM;		//timer 3 dung de output PWM ra 4 channel
TIM_HandleTypeDef 				htim1 , htim2 , htim4 , htim5; //  4 timer de capture Devo 7

				//---------RF Module, PWM Capture
int16_t 									IC_Throttle1, IC_Throttle2, 								IC_Throttle_pusle_width; //Throttle (can ga) tang giam toc do quay
int16_t 									IC_Rudder_Xoay1, IC_Rudder_Xoay2, 					IC_Rudder_Xoay_pusle_width; //Rudder (xoay theo truc z) - goc Yaw
int16_t 									IC_Elevator_TienLui1, IC_Elevator_TienLui2, IC_Elevator_TienLui_pusle_width; //Elevator (tien - lui) - goc Pitch
int16_t 									IC_Aileron_TraiPhai1, IC_Aileron_TraiPhai2, IC_Aileron_TraiPhai_pusle_width;//Aileron_TraiPhai (trai - phai) - goc Roll

				//---------PWM 4 motor
int16_t 									pwm_motor_1, pwm_motor_2, pwm_motor_3, pwm_motor_4;
int16_t										pwm_test, is_already_config_pwm;
int16_t 									FlyState; // 0: May bay ngung hoat dong, 1:may bay dang bay

				//---------sensor
uint8_t 									who_i_am_reg_value_MPU6050, i;
int32_t 									timer;
float 										gyro_x_zero_offset, gyro_y_zero_offset, gyro_z_zero_offset;
float 										accX_angle, accY_angle, accZ_angle;
float 										gyroX_rate, gyroY_rate, gyroZ_rate;
float 										gyroX_angle, gyroY_angle, gyroZ_angle;
float 										Kalman_roll, Kalman_pitch, Kalman_yaw;
float 										smooth_accX, smooth_accY, smooth_accZ, smooth_gyroX, smooth_gyroY, smooth_gyroZ;

Kalman_Setting 						kalmanX,	kalmanY,   kalmanZ;

TM_MPU6050_t 							mpu6050;
Compass_HMC5883L					compassHMC5883L;


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
float 						kalmanCalculate(Kalman_Setting *kalman, float newAngle, float newRate, float DT_);
void 							lowPassFilterCalculate(TM_MPU6050_t* output);	
																
				//ham handle error						
static void 			SystemClock_Config(void);
static void 			Error_Handler(void); 
void 							Error_Handler_Custom(int type);
int16_t 					limitOutputPWMFuzzy(int16_t value);
															
				//ham Led Sang
void 							SANG_1_LED(int8_t pin);
void 							SANG_2_LED(int8_t type);
void 							SANG_4_LED(void);
void 							SANG_4_LED_OFF(void);
void 							SANG_4_LED_LAN_LUOT(int16_t n, int16_t delaytime);
void 							SANG_4_LED_LOOP(int16_t n, int16_t delaytime);
void 							Check_EveryThing_OK(void);
void 							Turn_On_Quadrotor(void);
void 							Turn_Off_Quadrotor(void);
															
				//Khoi Tao LED, BUTTON USER
void 							Init_LEDSANG_AND_BUTTON_USER_PORT_A0(void);
															
				//Khoi tao TIMER3 output PWM											
void 							Init_TIM3_OUTPUT_PWM(void);
															
				//timer 1 & 2 & 4 & 5 Inputcapture for RF module
void 							PWM_Input_Capture_TIM1(void); 
void 							PWM_Input_Capture_TIM2(void); 
void 							PWM_Input_Capture_TIM4(void);
void 							PWM_Input_Capture_TIM5(void);

				//Dieu chinh huong bay qua receiver
void 							SetInitDataQuadrotor(void);
void 							SetPWM_4_Motor(int16_t value);
void 							setPWM_4_Motor_Cung_Value(int16_t value);
void 							UpdatePWM_4_Motor_By_TIM_CCRx(void);
void 							SetPWM_1_Motor(int16_t numberMotor, int16_t newValue);
void 							SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue);
void 							SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue);
void 							Direct_Quadrotor_By_Receiver(void);

				//i2c chip mpu6050 10truc
void 							GY86_I2C_Handle_GY86(void);
void 							GY86_I2C_IS_DEVICE_CONNECTED(void);
uint8_t 					GY86_I2C_WHO_I_AM( uint8_t device_address, uint8_t register_address);
void 							GY86_MPU6050_SetDataRate(uint8_t device_address, uint8_t register_address, uint8_t rate);
void 							GY86_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value);
void 							GY86_MPU6050_SetGyroscope(uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value); 
void 							GY86_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data);
void 							GY86_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void 							GY86_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data);
void 							GY86_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output );
void 							MPU6050_gyro_zero_offset(void);
void 							HMC5883L_set_config(void);
void 							HMC5883L_read_compass_data(Compass_HMC5883L* compass);
void 							Sang_Led_By_MPU6050_Values(float roll, float pitch, float yaw );
				//ham delay default
volatile 					uint32_t g_iSysTicks = 0;
void 							SysTick_Handler(){	g_iSysTicks++;}
void 							delay_ms(uint32_t piMillis){	uint32_t iStartTime = g_iSysTicks;	while( (g_iSysTicks - iStartTime ) < piMillis)	{}}			
void 							Delay_milisecond(__IO uint32_t nCount){  while(nCount--)  {  } }
				//Fuzzy system
void 							initFuzzySystem(void);
void 							Fuzzification_All_MF(float x, FuzzyController * fuzzyController);
void 							Apply_All_Rule( FuzzyController * fuzzyController );
void 							Defuzzification( FuzzyController * fuzzyController );
//-----------END Khai bao HAM-------------------------------------------------------

int main(void)
{		
																	#ifdef RTE_CMSIS_RTOS                   
																		osKernelInitialize(); /*..................code default cua ARM co san						*/                 
																	#endif			
																	HAL_Init();		
																	SystemClock_Config();	
	
		initFuzzySystem(); 					//init fuzzy system
		SetInitDataQuadrotor(); 		// set kalman filter, input cature	
	
							__GPIOA_CLK_ENABLE();	__GPIOB_CLK_ENABLE();	__GPIOC_CLK_ENABLE(); __GPIOD_CLK_ENABLE();	__GPIOE_CLK_ENABLE();		
							__TIM1_CLK_ENABLE();  __TIM2_CLK_ENABLE(); 	__TIM3_CLK_ENABLE(); 	__TIM4_CLK_ENABLE(); 	__TIM5_CLK_ENABLE();  	 
							__I2C1_CLK_ENABLE();						
		Init_LEDSANG_AND_BUTTON_USER_PORT_A0(); //---GPIO 4 led & GPIO button user---------------						
		
		//---Config DEVO 7 RF module - INPUT CAPTURE MODE------------
		PWM_Input_Capture_TIM1(); 
		PWM_Input_Capture_TIM2(); 
		PWM_Input_Capture_TIM4(); 
		PWM_Input_Capture_TIM5();	
		
		//-----------------------------------------------------------------------	
		GY86_I2C_Handle_GY86();			//---MPU6050 cau hinh PB6, PB7 doc cam bien
		GY86_I2C_WRITE( MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x00); //---Setting/config cho MPU6050-----------------------------------------------------------
		GY86_MPU6050_SetDataRate( MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, TM_MPU6050_DataRate_1KHz); 
		GY86_MPU6050_SetAccelerometer( MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, TM_MPU6050_Accelerometer_2G); 
		GY86_MPU6050_SetGyroscope( MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, TM_MPU6050_Gyroscope_250s); 
		HMC5883L_set_config();
		
	
		//-----------------------------------------------------------------------
		SANG_4_LED_LAN_LUOT(10,50);
		SANG_4_LED(); 
		delay_ms(1000); 
		
		Init_TIM3_OUTPUT_PWM();//---Timer 3 4 channel PWM
		SANG_4_LED_OFF(); 
		delay_ms(1000); 
		is_already_config_pwm = 0;
		setPWM_4_Motor_Cung_Value(CONFIG_PWM_MAX);
		SANG_4_LED(); 
		delay_ms(2000); 
		setPWM_4_Motor_Cung_Value(CONFIG_PWM_MIN);
		SANG_4_LED_OFF();
		delay_ms(1000);
		SANG_4_LED_LAN_LUOT(5,50);
		SANG_4_LED_OFF();
		setPWM_4_Motor_Cung_Value(ZERO_);		
		//-----------------------------------------------------------------------
		
		GY86_I2C_IS_DEVICE_CONNECTED();	//ERROR khong connect dc GY-86 => LED VANG sang lien tuc	
		who_i_am_reg_value_MPU6050 = GY86_I2C_WHO_I_AM( MPU6050_I2C_ADDR, MPU6050_WHO_AM_I_REGISTER);
		if( who_i_am_reg_value_MPU6050 != MPU6050_I_AM_VALUES )	
		{ 
			//---Doc gia tri cua WHO I AM register, if error => LED RED(14) sang nhap nhay
			Error_Handler_Custom(	ERROR_MPU6050_NOT_I_AM_VALUES ); 
		}
													#ifdef RTE_CMSIS_RTOS 
														osKernelStart(); //.........code dafault cua ARM		// when using CMSIS RTOS	// start thread execution 
													#endif
		//tinh gyro offset 3 truc 
		MPU6050_gyro_zero_offset();
		Check_EveryThing_OK();			//ERROR => 4 led sang tat lien tuc 20ms
		while(1)
		{		
				if(FlyState == STATE_FLY_OFF && is_already_config_pwm == 0)
				{
						SANG_4_LED_LAN_LUOT(5,50);
						SANG_4_LED(); 
						delay_ms(2000); 
						SANG_4_LED_OFF(); 
						delay_ms(1000); 
						setPWM_4_Motor_Cung_Value(1000);
						is_already_config_pwm = 1;
				}
				
				//---------Quadrotor state OFF, //check sign to start
				while(FlyState == STATE_FLY_OFF) {	 Turn_On_Quadrotor();  }
				
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET) //TEST 
				{		//khi nhan buttun USER ma chua tha ra -> khong lam gi
					while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_SET){}
				}				
	
				//---Quadrotor Fly------------------------------------------------------------------
				if(FlyState == STATE_FLY_ON) //quadrotor State Fly
				{ 
						//Check sign to TURN OFF quadrotor
						if( (IC_Throttle_pusle_width         >= RC_ON_OFF_MIN && IC_Throttle_pusle_width         <= RC_ON_OFF_MAX) && 
								(IC_Aileron_TraiPhai_pusle_width >= RC_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= RC_ON_OFF_MAX) && 
								(IC_Elevator_TienLui_pusle_width >= RC_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= RC_ON_OFF_MAX) &&
								(IC_Rudder_Xoay_pusle_width      >= RC_ON_OFF_MIN && IC_Rudder_Xoay_pusle_width      <= RC_ON_OFF_MAX) 
						) 
								{  Turn_Off_Quadrotor(); }
						else
						
						//BALANCE MODE-------------TRANG THAI CAN BANG,  k co tac dong cua receiver( can` dieu khien o giua)
						if((IC_Aileron_TraiPhai_pusle_width >= RC_Effect_Min && IC_Aileron_TraiPhai_pusle_width <= RC_Effect_Max ) &&
							 (IC_Elevator_TienLui_pusle_width >= RC_Effect_Min && IC_Elevator_TienLui_pusle_width <= RC_Effect_Max) &&
							 (IC_Rudder_Xoay_pusle_width      >= RC_Effect_Min && IC_Rudder_Xoay_pusle_width      <= RC_Effect_Max)
						)
						{				
								if(IC_Throttle_pusle_width < ENTER_BALANCE_STATE)
								{
										//Quadrotor can not FLY ----- khi can` dieu khien Throttle value < 1200
										SetPWM_4_Motor(Motor_Range_Min_NOT_BALANCE_STATE);									
								}else
								
								if(IC_Throttle_pusle_width >= ENTER_BALANCE_STATE)
								{	
										//BALANCE MODE-------------TRANG THAI CAN BANG
										//Quadrotor can FLY ----- khi can` dieu khien Throttle value >= 1200	
										GY86_MPU6050_ReadAll( MPU6050_I2C_ADDR, &mpu6050);  //---Read value from MPU6050
										//lowPassFilterCalculate(&mpu6050); /* Apply low pass filter to smooth*/
										HMC5883L_read_compass_data(&compassHMC5883L);

										gyroX_rate = ((float)((float)mpu6050.Gyro_X - (float)gyro_x_zero_offset))/(float)131.0;
										gyroY_rate = ((float)((float)mpu6050.Gyro_Y - (float)gyro_y_zero_offset))/(float)131.0;	
										gyroZ_rate = ((float)((float)mpu6050.Gyro_Z - (float)gyro_z_zero_offset))/(float)131.0;	
									
										accX_angle =  	atan2(mpu6050.Acc_Y, mpu6050.Acc_Z)*RAD_TO_DEG; //roll equation provides [-180, 180] range
										accY_angle =   	atan2(-mpu6050.Acc_X, sqrt(mpu6050.Acc_Y*mpu6050.Acc_Y + mpu6050.Acc_Z*mpu6050.Acc_Z) )*RAD_TO_DEG; //[-90, 90] range, which is exactly what is expected for the pitch angle										
										accZ_angle = 		atan2((float)compassHMC5883L.Compass_Y, (float)compassHMC5883L.Compass_X)*RAD_TO_DEG;
										if (( atan2((float)compassHMC5883L.Compass_Y, (float)compassHMC5883L.Compass_X) )>=0) 
									  {
												 accZ_angle = atan2((float)compassHMC5883L.Compass_Y, (float)compassHMC5883L.Compass_X)*RAD_TO_DEG;
										}
									  if (( atan2((float)compassHMC5883L.Compass_Y, (float)compassHMC5883L.Compass_X) ) < 0) 
									  {
												accZ_angle = 360-(atan2((float)compassHMC5883L.Compass_Y, (float)compassHMC5883L.Compass_X)*RAD_TO_DEG);
									  }
										/*heading = atan2(com_y,com_x);
											heading-=declination_angle;
											//heading+=declination_angle; 
											if(heading < 0.0) heading += (2.0 * 3.141592654); 
											if(heading > (2.0 * 3.141592654)) heading -= (2.0 * 3.141592654); 
											zDegrees=heading*rad_to_degree;
											if(zDegrees >= 1 && zDegrees < 240) com_z_angle = (zDegrees*179/239);
											else if(zDegrees >= 240) com_z_angle = (zDegrees*180/120);
										*/
										//gyroX_angle = gyroX_angle + gyroXrate * DT; // Calculate gyro angle without any filter	
									
										Kalman_roll  = kalmanCalculate(&kalmanX, accX_angle, gyroX_rate, DT);
										Kalman_pitch = kalmanCalculate(&kalmanY, accY_angle, gyroY_rate, DT);
										Kalman_yaw   = kalmanCalculate(&kalmanZ, accZ_angle, gyroZ_rate, DT);
										
										Sang_Led_By_MPU6050_Values(Kalman_roll, Kalman_pitch, Kalman_yaw);			
										//-----------------------------------------------------------------------------------
															
										//FUZZY SYSTEM------------------------------------------------------------------------
										Fuzzification_All_MF( (float) Kalman_roll, 	&rollFuzzyControl);
										Fuzzification_All_MF( (float) Kalman_pitch, &pitchFuzzyControl);			
										Fuzzification_All_MF( (float) Kalman_yaw, 	&yawFuzzyControl);
										
										Apply_All_Rule( 				  &rollFuzzyControl  );
										Apply_All_Rule( 				  &pitchFuzzyControl );			
										Apply_All_Rule( 				  &yawFuzzyControl );	
										
										Defuzzification( 				  &rollFuzzyControl  );				
										Defuzzification( 				  &pitchFuzzyControl );	
										Defuzzification( 				  &yawFuzzyControl );
										
										//pwm_motor_1 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( rollFuzzyControl.output   + pitchFuzzyControl.output + yawFuzzyControl.output);
										//pwm_motor_2 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( - rollFuzzyControl.output + pitchFuzzyControl.output - yawFuzzyControl.output);				
										//pwm_motor_3 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( rollFuzzyControl.output   - pitchFuzzyControl.output + yawFuzzyControl.output);
										//pwm_motor_4 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( - rollFuzzyControl.output - pitchFuzzyControl.output - yawFuzzyControl.output);
										
										pwm_motor_1 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( rollFuzzyControl.output   - pitchFuzzyControl.output );
										pwm_motor_2 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( - rollFuzzyControl.output - pitchFuzzyControl.output );				
										pwm_motor_3 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( - rollFuzzyControl.output + pitchFuzzyControl.output );
										pwm_motor_4 = IC_Throttle_pusle_width + limitOutputPWMFuzzy( rollFuzzyControl.output   + pitchFuzzyControl.output );
										
										SetPWM_1_Motor(1,pwm_motor_1);
										SetPWM_1_Motor(2,pwm_motor_2);
										SetPWM_1_Motor(3,pwm_motor_3);
										SetPWM_1_Motor(4,pwm_motor_4); 										
								}								
					}
					else //K phai TRANG THAI CAN BANG
					{		
							if(IC_Throttle_pusle_width < ENTER_BALANCE_STATE)
							{
									SetPWM_4_Motor(Motor_Range_Min_NOT_BALANCE_STATE);
							}else	
							if(IC_Throttle_pusle_width >= ENTER_BALANCE_STATE)
							{	
									Direct_Quadrotor_By_Receiver();
							}
					}
					//delay_ms(MAIN_DELAY_TIME);					
				}
				//end while(FlyState == 1) ---Quadrotor Fly
		}		
		//End while(1)
}

//-----------------------------------------------------------------------------------
void SetInitDataQuadrotor(void)
{
		who_i_am_reg_value_MPU6050 = ZERO_;		
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
		
		gyro_x_zero_offset = 0;
		gyro_y_zero_offset = 0;
		gyro_z_zero_offset = 0;
		FlyState = STATE_FLY_OFF;
}
void Turn_On_Quadrotor(void)
{
	//khoi dong quadrotor bang Receiver, ta gat 2 tick den vi tri thap nhat
		if( (IC_Throttle_pusle_width             >= RC_ON_OFF_MIN && IC_Throttle_pusle_width         <= RC_ON_OFF_MAX) && 
						(IC_Aileron_TraiPhai_pusle_width >= RC_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= RC_ON_OFF_MAX) && 
						(IC_Elevator_TienLui_pusle_width >= RC_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= RC_ON_OFF_MAX) && 
						(FlyState == 0	))
				{	
						SANG_4_LED_LAN_LUOT(5,50);						
						SANG_4_LED(); 							
						delay_ms(2000);  						
						SANG_4_LED_OFF();						
						if( (IC_Throttle_pusle_width         >= RC_ON_OFF_MIN && IC_Throttle_pusle_width         <= RC_ON_OFF_MAX) && 
								(IC_Aileron_TraiPhai_pusle_width >= RC_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= RC_ON_OFF_MAX) && 
								(IC_Elevator_TienLui_pusle_width >= RC_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= RC_ON_OFF_MAX) &&
								(FlyState == 0	)		)
						{							
								FlyState = STATE_FLY_ON;						
								SANG_4_LED_OFF(); 
								delay_ms(1000);
								SANG_4_LED();
								SetPWM_4_Motor(Motor_Range_Min_NOT_BALANCE_STATE); 
								delay_ms(1000);
								SANG_4_LED_OFF();
						}
				}
		else
		{
				SANG_2_LED(1);			
				delay_ms(300);			
				SANG_4_LED_OFF();	
				SANG_2_LED(2);	
				delay_ms(300);
				SANG_4_LED_OFF();	
		}
}

void Turn_Off_Quadrotor(void) //tat quadrotor
{
		int i=0;
		int timedelay = 50;
		while(i<5)
		{
			SANG_1_LED(12); delay_ms(timedelay);
			SANG_1_LED(13); delay_ms(timedelay);
			SANG_1_LED(14); delay_ms(timedelay);
			SANG_1_LED(15); delay_ms(timedelay);
			i++;
		}
		SANG_4_LED(); 	
		delay_ms(500);
		if( (IC_Throttle_pusle_width         >= RC_ON_OFF_MIN && IC_Throttle_pusle_width         <= RC_ON_OFF_MAX) && 
				(IC_Aileron_TraiPhai_pusle_width >= RC_ON_OFF_MIN && IC_Aileron_TraiPhai_pusle_width <= RC_ON_OFF_MAX) && 
				(IC_Elevator_TienLui_pusle_width >= RC_ON_OFF_MIN && IC_Elevator_TienLui_pusle_width <= RC_ON_OFF_MAX) &&
				(IC_Rudder_Xoay_pusle_width      >= RC_ON_OFF_MIN && IC_Rudder_Xoay_pusle_width      <= RC_ON_OFF_MAX) &&
				(FlyState == 1)
		)
		{				
				FlyState = STATE_FLY_OFF;
				SANG_4_LED_OFF();
				//SetPWM_4_Motor(0);
				TIM3->CCR1 = 1000;
				TIM3->CCR2 = 1000;
				TIM3->CCR3 = 1000;
				TIM3->CCR4 = 1000;
						//TIM3->CCR1 =  1100;		
						//TIM3->CCR2 =  1100;		
						//TIM3->CCR3 =  1100;		
						//TIM3->CCR4 = 	1100; 
				delay_ms(2000);
		}
}
//-----------------------------------------------------------------------------------
//
//
//Timer 1 PWM input capture
//
void PWM_Input_Capture_TIM1(void)
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

void PWM_Input_Capture_TIM2(void) //timer 2 input capture
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

void PWM_Input_Capture_TIM4(void) //timer 4 input capture
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

void PWM_Input_Capture_TIM5(void) //PA1 - TIM5_CH2 timer 5 input capture
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
//-----------------------------------------------------------------------------------
void Check_EveryThing_OK(void)
{
		//SANG_4_LED_LAN_LUOT(5,100); //sang 4 led lan luot. loop 5 lan, delay 50ms
		//SANG_4_LED();			
		//delay_ms(1000);		
		SANG_4_LED_OFF();
		delay_ms(1000);		
		if(IC_Throttle_pusle_width == 0 || IC_Aileron_TraiPhai_pusle_width == 0 || IC_Elevator_TienLui_pusle_width == 0 || IC_Rudder_Xoay_pusle_width == 0)
		{
			//Error_Handler();
					while(1)
					{			
						SANG_4_LED();
						delay_ms(50);
						SANG_4_LED_OFF();
						delay_ms(50);
					}
		}
		if(IC_Throttle_pusle_width > 2000 || IC_Aileron_TraiPhai_pusle_width > 2000 || IC_Elevator_TienLui_pusle_width > 2000 || IC_Rudder_Xoay_pusle_width > 2000)
		{
					//Error_Handler();
					while(1)
					{			
						SANG_4_LED();
						delay_ms(50);
						SANG_4_LED_OFF();
						delay_ms(50);
					}
		}		
}

void Init_LEDSANG_AND_BUTTON_USER_PORT_A0(void)
{
		GPIO_InitTypeDef PIN_LED_SANG_PD;
		GPIO_InitTypeDef BUTTON_USER_PA_0;	
	
		PIN_LED_SANG_PD.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
		PIN_LED_SANG_PD.Mode = GPIO_MODE_OUTPUT_PP;
		PIN_LED_SANG_PD.Pull = GPIO_NOPULL;
		PIN_LED_SANG_PD.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOD, &PIN_LED_SANG_PD);	
		
		BUTTON_USER_PA_0.Pin = GPIO_PIN_0;
		BUTTON_USER_PA_0.Mode = GPIO_MODE_INPUT;
		BUTTON_USER_PA_0.Pull = GPIO_NOPULL;
		BUTTON_USER_PA_0.Speed = GPIO_SPEED_HIGH;
		HAL_GPIO_Init(GPIOA, &BUTTON_USER_PA_0);
}
//
//
//
//Timer 3 output PWM ra 4 channel
//
void Init_TIM3_OUTPUT_PWM(void)
{
		GPIO_InitTypeDef 		GPIO_PWM_PORTB_4_5;	
		GPIO_InitTypeDef 		GPIO_PWM_PORTB_0_1;		
		TIM_OC_InitTypeDef  PWMConfig;
	
	
		GPIO_PWM_PORTB_4_5.Pin =  GPIO_PIN_4 | GPIO_PIN_5;
		GPIO_PWM_PORTB_4_5.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORTB_4_5.Pull = GPIO_NOPULL;
		GPIO_PWM_PORTB_4_5.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_PWM_PORTB_4_5.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_PWM_PORTB_4_5);
	
		GPIO_PWM_PORTB_0_1.Pin =  GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_PWM_PORTB_0_1.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORTB_0_1.Pull = GPIO_NOPULL;
		GPIO_PWM_PORTB_0_1.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_PWM_PORTB_0_1.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &GPIO_PWM_PORTB_0_1);
		//-----------------------------------------
	
		//config tim3 handle	
		Tim3_Handle_PWM.Instance = TIM3;
		Tim3_Handle_PWM.Init.Prescaler = 84-1; 									//vi timer 3 co max clock la 84MHz, -1 la vi dem(count) tu 0
		Tim3_Handle_PWM.Init.CounterMode = TIM_COUNTERMODE_UP; 	//dem len
		Tim3_Handle_PWM.Init.Period = 20000-1;									//Period(chu ki) = 20 mili s
		Tim3_Handle_PWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// =0
		HAL_TIM_PWM_Init(&Tim3_Handle_PWM);
		
		//----------------------------------------------------		
		// PWM mode 2 = Clear on compare match 
    // PWM mode 1 = Set on compare match 
		PWMConfig.OCMode 					= TIM_OCMODE_PWM1;
		PWMConfig.OCPolarity 			= TIM_OCPOLARITY_HIGH;
		PWMConfig.OCNPolarity 		= TIM_OCNPOLARITY_HIGH;
		PWMConfig.OCIdleState 		= TIM_OCIDLESTATE_SET;
		PWMConfig.OCNIdleState		= TIM_OCNIDLESTATE_RESET;
		PWMConfig.OCFastMode 			= TIM_OCFAST_DISABLE;
		PWMConfig.Pulse 					= 0;
		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &PWMConfig, TIM_CHANNEL_1); //config PWM cho channel 1 (PORTC.6)
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &PWMConfig, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &PWMConfig, TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &PWMConfig, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM,TIM_CHANNEL_1);		
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM,TIM_CHANNEL_2);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM,TIM_CHANNEL_3);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM,TIM_CHANNEL_4);
}
//
//
//
//
//
//Accelerametor 10truc mpu6050
//

void GY86_I2C_Handle_GY86()
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
void GY86_I2C_IS_DEVICE_CONNECTED()
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

uint8_t GY86_I2C_WHO_I_AM(uint8_t device_address, uint8_t register_address)
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


void GY86_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data) 
{
	uint8_t d[2];		
	d[0] = register_address;
	d[1] = data;	
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}	
}
	
	
void GY86_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data)
{
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(&I2C_Handle_10truc, device_address, data, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
}


void GY86_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)  
{
	if (HAL_I2C_Master_Transmit(&I2C_Handle_10truc, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(&I2C_Handle_10truc, device_address, data, count, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY) {}	
}


void GY86_MPU6050_SetDataRate(uint8_t device_address, uint8_t register_address, uint8_t rate) 
{
	GY86_I2C_WRITE( device_address, register_address, rate);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void GY86_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value) 
{
	uint8_t temp;		
	GY86_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Acc_8G_Value << 3;
	GY86_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void GY86_MPU6050_SetGyroscope( uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value) 
{
	uint8_t temp;		
	GY86_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Gyro_250s_Value << 3;
	GY86_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
}

void GY86_MPU6050_ReadAccelerometer( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	GY86_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
		
	output->Acc_X = (int16_t)(data[0] << 8 | data[1]);
	output->Acc_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Acc_Z = (int16_t)(data[4] << 8 | data[5]);
}

void GY86_MPU6050_ReadGyroscope( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	GY86_I2C_READ_MULTI( device_address, MPU6050_GYRO_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
		
	output->Gyro_X = (int16_t)(data[0] << 8 | data[1]);
	output->Gyro_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Gyro_Z = (int16_t)(data[4] << 8 | data[5]);
}	

void GY86_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[14];
	int16_t temp;	
	
	GY86_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 14); /* Read full raw data, 14bytes */
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

void MPU6050_gyro_zero_offset(void)//tinh gyro offset 3 truc
{
		int8_t i;
		for(i=0; i<100;i++)
		{
			GY86_MPU6050_ReadAll( MPU6050_I2C_ADDR, &mpu6050);  //---Read value from MPU6050
			gyro_x_zero_offset = gyro_x_zero_offset + mpu6050.Gyro_X;
			gyro_y_zero_offset = gyro_y_zero_offset + mpu6050.Gyro_Y;
			gyro_z_zero_offset = gyro_z_zero_offset + mpu6050.Gyro_Z;
		}
		gyro_x_zero_offset = gyro_x_zero_offset/100;
		gyro_y_zero_offset = gyro_y_zero_offset/100;
		gyro_z_zero_offset = gyro_z_zero_offset/100;
}


void HMC5883L_set_config(void) 
{
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Register_A, 0x18);//set rate = 75Hz
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Register_B, 0x60);//full scale = +/- 2.5 Gauss
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Measurement_Mode_Register, Mode_Measurement_Continuous);
		if(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY)
		{
			while(1){SANG_4_LED();}
		}
}

void HMC5883L_read_compass_data(Compass_HMC5883L* compass)
{
		uint8_t data[6];
		GY86_I2C_READ_MULTI( HMC5883L_Device_Address, HMC5883L_Data_Register_Begin, data, 6); //select register 3
		while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
			
		compass->Compass_X = (int16_t)(data[0] << 8 | data[1]);
		compass->Compass_Y = (int16_t)(data[2] << 8 | data[3]);
		compass->Compass_Z = (int16_t)(data[4] << 8 | data[5]);
}	




void Sang_Led_By_MPU6050_Values(float kalman_angel_x, float kalman_angel_y, float kalman_angel_z )
{
		if(kalman_angel_x > 10)
		{
			LED_D_14_HIGH;		 //SANG_1_LED(LED_YELLOW);
		}else if(kalman_angel_x < -10)
		{
			LED_D_12_HIGH;			//SANG_1_LED(LED_RED);
		}
		
		if(kalman_angel_y > 10)
		{
			LED_D_13_HIGH;	//LED_ORANGE 
		}else if(kalman_angel_y < -10)
		{
			LED_D_15_HIGH;	//XANH
		}
		delay_ms(10);
		SANG_4_LED_OFF();
		delay_ms(10);
}
//end Accelerametor 10truc
//
//
//

//-----------------------------------------------------------------
//
//Lay gia tri cua Receiver de dieu khien 4 motor
//
//
void setPWM_4_Motor_Cung_Value(int16_t value)
{
	TIM3->CCR1 = 	value;
	TIM3->CCR2 = 	value;
	TIM3->CCR3 = 	value;
	TIM3->CCR4 = 	value;
}
void UpdatePWM_4_Motor_By_TIM_CCRx(void) //update lai speed
{	
	if( pwm_motor_1 <= Motor_Range_Min)
		TIM3->CCR1 = 	Motor_Range_Min;
	else
		TIM3->CCR1 = pwm_motor_1;
	
	if( pwm_motor_2 <= Motor_Range_Min )
		TIM3->CCR2 = Motor_Range_Min;
	else
		TIM3->CCR2 = pwm_motor_2;
	
	if( pwm_motor_3 <= Motor_Range_Min )
		TIM3->CCR3 = Motor_Range_Min;
	else
		TIM3->CCR3 = pwm_motor_3;
	
	if( pwm_motor_4 <= Motor_Range_Min )
		TIM3->CCR4 = Motor_Range_Min;
	else
		TIM3->CCR4 = pwm_motor_4;
}

void SetPWM_4_Motor(int16_t value) //set 4 motor cung 1 speed
{
	if(value==0)
	{
					pwm_motor_1 = 0;	pwm_motor_2 = 0;		pwm_motor_3 = 0;		pwm_motor_4 = 0;						
	}
	else
	{
			if(value <= Motor_Range_Min)
			{
					pwm_motor_1 = Motor_Range_Min;					
					pwm_motor_2 = Motor_Range_Min;					
					pwm_motor_3 = Motor_Range_Min;					
					pwm_motor_4 = Motor_Range_Min;
			}else 
			if(value >= Motor_Range_Max)
			{
					pwm_motor_1 = Motor_Range_Max;					
					pwm_motor_2 = Motor_Range_Max;
					pwm_motor_3 = Motor_Range_Max;					
					pwm_motor_4 = Motor_Range_Max;
			}else{
					pwm_motor_1 = value;										
					pwm_motor_2 = value;
					pwm_motor_3 = value;										
					pwm_motor_4 = value;
			}	
	}
	UpdatePWM_4_Motor_By_TIM_CCRx();
}

		
void SetPWM_1_Motor(int16_t numberMotor, int16_t newValue) //set speed cho moi motor rieng le
{
	switch(numberMotor) 
	{
		 case 1  :
				if(newValue <= RC_Throtte_Min)							{	pwm_motor_1 = RC_Throtte_Min;}
				else if (newValue >= RC_Throtte_Max)				{ pwm_motor_1 = RC_Throtte_Max;}
				else 																				{ pwm_motor_1 = newValue;					}
				
				break; 
		
		 case 2  :
				if(newValue <= RC_Throtte_Min)							{pwm_motor_2 = RC_Throtte_Min;}
				else if (newValue >= RC_Throtte_Max)				{pwm_motor_2 = RC_Throtte_Max;}
				else 																				{pwm_motor_2 = newValue;	}
				
				break; 
		 
		 case 3  :
				if(newValue <= RC_Throtte_Min)							{pwm_motor_3 = RC_Throtte_Min;}
				else if (newValue >= RC_Throtte_Max)				{pwm_motor_3 = RC_Throtte_Max;}
				else 																				{pwm_motor_3 = newValue;	}
				
				break; 
		
		 case 4  :
				if(newValue <= RC_Throtte_Min)							{pwm_motor_4 = RC_Throtte_Min;}
				else if (newValue >= RC_Throtte_Max)				{pwm_motor_4 = RC_Throtte_Max;}
				else 																				{pwm_motor_4 = newValue;	}
				
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

int16_t limitOutputPWMFuzzy(int16_t value)
{
	if(value >= 100)	return 100;
	if(value <= -100) return -100;
	return value;
}


int16_t Giam_Do_Vot_Lo(int16_t value)
{
	if(value >= DO_VOT_LO)		value = DO_VOT_LO;
	if(value <= 0)		value = 0;
	return value;
}

void Direct_Quadrotor_By_Receiver(void)
{
			int16_t chenhLechGiaTri = 0;
			//int16_t timedelay = 50;
			//Tien - Lui
			if( IC_Elevator_TienLui_pusle_width >= 1000 && IC_Elevator_TienLui_pusle_width <= 2000  )
			{
				//LUI` ra sau => giam 3,4 ; tang 1,2
				while(IC_Elevator_TienLui_pusle_width <= RC_Effect_Min ) 
				{			
							chenhLechGiaTri = RC_Medium - IC_Elevator_TienLui_pusle_width;
							chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
							SetPWM_Motor_Tang(1,chenhLechGiaTri);
							SetPWM_Motor_Tang(2,chenhLechGiaTri);
							SetPWM_Motor_Giam(3,chenhLechGiaTri);
							SetPWM_Motor_Giam(4,chenhLechGiaTri);
							//delay_ms(timedelay);
				}
				//TIEN' ve truoc => giam 1,2 ; tang 3,4
				while(IC_Elevator_TienLui_pusle_width >= RC_Effect_Max )
				{
							//Lui ve phia sau, giam dong co 3+4 va tang dong co 1+2
							chenhLechGiaTri = IC_Elevator_TienLui_pusle_width - RC_Medium;
							chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
							SetPWM_Motor_Tang(3,chenhLechGiaTri);
							SetPWM_Motor_Tang(4,chenhLechGiaTri);
							SetPWM_Motor_Giam(1,chenhLechGiaTri);
							SetPWM_Motor_Giam(2,chenhLechGiaTri);			
							//delay_ms(timedelay);
				}					
			}
	
			//Trai - Phai
			if(IC_Aileron_TraiPhai_pusle_width >= 1000 && IC_Aileron_TraiPhai_pusle_width <= 2000  )
			{
				//PHAI => giam 2,3; tang 1,4
				while(IC_Aileron_TraiPhai_pusle_width <= RC_Effect_Min )
				{		
							chenhLechGiaTri = RC_Medium - IC_Aileron_TraiPhai_pusle_width;
							chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
							SetPWM_Motor_Tang(1,chenhLechGiaTri);
							SetPWM_Motor_Tang(4,chenhLechGiaTri);
							SetPWM_Motor_Giam(2,chenhLechGiaTri);
							SetPWM_Motor_Giam(3,chenhLechGiaTri);
							//delay_ms(timedelay);
				}
				//TRAI => giam 1,4; tang 2,3
				while(IC_Aileron_TraiPhai_pusle_width >= RC_Effect_Max )
				{
							chenhLechGiaTri = IC_Aileron_TraiPhai_pusle_width - RC_Medium;
							chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
							SetPWM_Motor_Tang(2,chenhLechGiaTri);
							SetPWM_Motor_Tang(3,chenhLechGiaTri);
							SetPWM_Motor_Giam(1,chenhLechGiaTri);
							SetPWM_Motor_Giam(4,chenhLechGiaTri);		
							//delay_ms(timedelay);
				}
				//trong khoang 1470 - 1530 khong lam gi		
			}
	
			//Xoay
			if(IC_Rudder_Xoay_pusle_width >= 1000 && IC_Rudder_Xoay_pusle_width <= 2000  )
			{		
				while(IC_Rudder_Xoay_pusle_width <= RC_Effect_Min )
				{
					//Xoay cung chieu kim dong ho, giam dong co 1+3 va tang dong co 2+4			
					chenhLechGiaTri = RC_Medium - IC_Rudder_Xoay_pusle_width;
					chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
					SetPWM_Motor_Giam(1,chenhLechGiaTri);
					SetPWM_Motor_Giam(3,chenhLechGiaTri);
					SetPWM_Motor_Tang(2,chenhLechGiaTri);
					SetPWM_Motor_Tang(4,chenhLechGiaTri);
					//delay_ms(timedelay);
				}
				
				while(IC_Rudder_Xoay_pusle_width >= RC_Effect_Max )
				{
					//Xoay nguoc chieu kim dong ho, giam dong co 2+4 va tang dong co 1+3
					chenhLechGiaTri = IC_Rudder_Xoay_pusle_width - RC_Medium;
					chenhLechGiaTri = Giam_Do_Vot_Lo(chenhLechGiaTri);
					SetPWM_Motor_Tang(1,chenhLechGiaTri);
					SetPWM_Motor_Tang(3,chenhLechGiaTri);	
					SetPWM_Motor_Giam(2,chenhLechGiaTri);
					SetPWM_Motor_Giam(4,chenhLechGiaTri);				
					//delay_ms(timedelay);
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
		delay_ms(50);
		SANG_4_LED_OFF();
		delay_ms(50);
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
	if(type==1)	
	{			
		LED_D_12_HIGH;		
		LED_D_14_HIGH;	
	}
	else if(type==2)	
	{		
		LED_D_13_HIGH;		
		LED_D_15_HIGH;	
	}
}

void SANG_4_LED()
{
		LED_D_12_HIGH;		LED_D_13_HIGH;		LED_D_14_HIGH;		LED_D_15_HIGH;
}

void SANG_4_LED_OFF()
{
		LED_D_12_LOW;		LED_D_13_LOW;		LED_D_14_LOW;		LED_D_15_LOW;
}

void SANG_4_LED_LOOP(int16_t n, int16_t delaytime)
{
	int16_t i = 0;
	while(i < n)
	{
		SANG_4_LED();			
		delay_ms(delaytime);			
		SANG_4_LED_OFF();			
		delay_ms(delaytime);
		i++;
	}	
}

void SANG_4_LED_LAN_LUOT(int16_t n, int16_t delaytime)
{
	int16_t i = 0;
	while(i < n)
	{
		SANG_1_LED(12); 	delay_ms(delaytime);
		SANG_1_LED(13); 	delay_ms(delaytime);			
		SANG_1_LED(14); 	delay_ms(delaytime);	
		SANG_1_LED(15); 	delay_ms(delaytime);
		SANG_4_LED_OFF();	delay_ms(delaytime);					
		i++;
	}	
}


float kalmanCalculate(Kalman_Setting *kalman, float newAngle, float newRate, float DT_)
{
			float dt = (float)DT_;	
			kalman->angle += dt * (newRate - kalman->bias);

		//!    P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
			kalman->P_00 += dt * (dt*kalman->P_11 - kalman->P_01 - kalman->P_10 + kalman->Q_angle);
		
			kalman->P_01 += - dt * kalman->P_11;  //!     P_01 +=  - dt * P_11;  
			kalman->P_10 += - dt * kalman->P_11; //!      P_10 +=  - dt * P_11;
			kalman->P_11 += + kalman->Q_gyro * dt; //!    P_11 +=  + Q_gyro * dt;

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
/*
http://nvtienanh.com/bai-viet/code-ccs-c-loc-kalman-cho-cam-bien-mpu6050-01-05-2014/
float32 dt = 0.01; // T Sampling
float32 Q_angle = 0.005;
float32 Q_bias = 0.003;
float32 R_measure = 0.03;
float32 bias = 0; // Reset bias
float32 rate;
float32 angle;
float32 S; // estimate error
float32 y; // different angle
float32 P_00 = 0 , P_01 = 0 , P_10 =0 ,P_11 =0;
float32 K_0 =0,K_1=0; // Kalman gain
 
float32 Kalman(float32 newAngle, float32 newRate){      
 
        // Discrete Kalman filter time update equations - Time Update ("Predict")
        // Update xhat - Project the state ahead
        // Step 1 
        //angle = X_Raw_Gyro_Angle;
        rate = newRate - bias;
        angle += dt * rate;
 
        // Update estimation error covariance - Project the error covariance ahead
        // Step 2 
        P_00 += dt * ( dt*P_11 - P_10 - P_01 + Q_angle);
        P_01 -= dt * P_11;
        P_10 -= dt * P_11;
        P_11 += Q_bias * dt;
 
        // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        // Calculate Kalman gain - Compute the Kalman gain
        // Step 4 
        S = P_00 + R_measure;
        // Step 5 
        K_0 = P_00 / S;
        K_1 = P_10 / S;
 
        // Calculate angle and bias - Update estimate with measurement zk (newAngle)
        // Step 3 
        y = newAngle - angle;
        // Step 6 
        angle += K_0 * y;
        bias += K_1 * y;
 
        // Calculate estimation error covariance - Update the error covariance
        // Step 7
        P_00 -= K_0 * P_00;
        P_01 -= K_0 * P_01;
        P_10 -= K_1 * P_00;
        P_11 -= K_1 * P_01;
 
        return angle;
    }
*/


void lowPassFilterCalculate(TM_MPU6050_t* mpu6050)
{
	 float alpha = 0.05;
	 float oneMinusAlpha = 0.95; //(1-alpha);
	 /* Apply low pass filter to smooth accelerometer values. */
		smooth_accX = (float)( oneMinusAlpha * smooth_accX + alpha * mpu6050->Acc_X );
		smooth_accY = (float)( oneMinusAlpha * smooth_accY + alpha * mpu6050->Acc_Y );
		smooth_accZ = (float)( oneMinusAlpha * smooth_accZ + alpha * mpu6050->Acc_Z );
	
		//smooth_gyroX = (float)( oneMinusAlpha * smooth_gyroX + alpha * mpu6050->Gyro_X );
		//smooth_gyroY = (float)( oneMinusAlpha * smooth_gyroY + alpha * mpu6050->Gyro_Y );
		//smooth_gyroZ = (float)( oneMinusAlpha * smooth_gyroZ + alpha * mpu6050->Gyro_Z );
	
		mpu6050->Acc_X = smooth_accX;
		mpu6050->Acc_Y = smooth_accY;
		mpu6050->Acc_Z = smooth_accZ;
		//mpu6050->Gyro_X = smooth_gyroX;
		//mpu6050->Gyro_Y = smooth_gyroY;
		//mpu6050->Gyro_Z = smooth_gyroZ;
}


















//------------------------------------------------------------------------------------------------------------------------------
//
//
//
//FUZZY SYSTEM --------------------------------------------------------------
//
void initFuzzySystem(void)
{
	rollFuzzyControl.pre_GocLech = 0;
	pitchFuzzyControl.pre_GocLech = 0;
	yawFuzzyControl.pre_GocLech = 0;
	
	rollFuzzyControl.output = 0;
	pitchFuzzyControl.output = 0;
	yawFuzzyControl.output = 0;
	
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
	
	setABCD_MF(&yawFuzzyControl.inGocLech[0],  -30,   -30,   -18,  -12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VerySmall
	setABCD_MF(&yawFuzzyControl.inGocLech[1],  -18,   -12,   -12,   -6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleSmall
	setABCD_MF(&yawFuzzyControl.inGocLech[2],  -12,    -6,    -6,    0, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Small
	setABCD_MF(&yawFuzzyControl.inGocLech[3],   -6,     0,     0,    6, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Zero
	setABCD_MF(&yawFuzzyControl.inGocLech[4],    0,     6,     6,   12, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Big
	setABCD_MF(&yawFuzzyControl.inGocLech[5],    6,    12,    12,   18, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleBig
	setABCD_MF(&yawFuzzyControl.inGocLech[6],   12,    18,    30,   30, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VeryBig
	

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
	
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[0],  -10,   -10,   -6,   -4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryNagative
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[1],   -6,    -4,   -4,   -2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittleNagative
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[2],   -4,    -2,   -2,    0, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Nagative
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[3],   -2,     0,    0,    2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Zero
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[4],    0,     2,    2,    4, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Positive
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[5],    2,     4,    4,    6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittlePositive
	setABCD_MF(&yawFuzzyControl.inGocLech_dot[6],    4,     6,   10,   10, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryPositive
	
	
	/*ValuePWMControl*/
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[0] , -100,  -100,   -60,   -40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[1] ,  -60,   -40,   -40,   -20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[2] ,  -40,   -20,   -20,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[3] ,  -20,     0,     0,    20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[4] ,    0,    20,    20,    40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[5] ,   20,    40,    40,    60, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[6] ,   40,    60,   100,   100, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast

	
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[0], -100,  -100,   -60,   -40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[1],  -60,   -40,   -40,   -20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[2],  -40,   -20,   -20,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[3],  -20,     0,     0,    20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[4],    0,    20,    20,    40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[5],   20,    40,    40,    60, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[6],   40,    60,   100,   100, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast
	
	
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[0],  -100,  -100,   -60,   -40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[1],   -60,   -40,   -40,   -20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[2],   -40,   -20,   -20,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[3],   -20,     0,     0,    20, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[4],     0,    20,    20,    40, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[5],    20,    40,    40,    60, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&yawFuzzyControl.outValuePWMControl[6],    40,    60,   100,   100, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast
	
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
	
	
	//YAW
	setOneRule(&yawFuzzyControl.fuzzy_rules[0], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[6], 1);//rule 1
	setOneRule(&yawFuzzyControl.fuzzy_rules[1], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[6], 2);//rule 2
	setOneRule(&yawFuzzyControl.fuzzy_rules[2], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[5], 3);//rule 3
	setOneRule(&yawFuzzyControl.fuzzy_rules[3], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[5], 4);//rule 4
	setOneRule(&yawFuzzyControl.fuzzy_rules[4], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[4], 5);//rule 5
	setOneRule(&yawFuzzyControl.fuzzy_rules[5], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[4], 6);//rule 6
	setOneRule(&yawFuzzyControl.fuzzy_rules[6], &yawFuzzyControl.inGocLech[0], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[3], 7);//rule 7
	
	setOneRule(&yawFuzzyControl.fuzzy_rules[7],  &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[6], 8);//rule 8
	setOneRule(&yawFuzzyControl.fuzzy_rules[8],  &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[5], 9);//rule 9
	setOneRule(&yawFuzzyControl.fuzzy_rules[9],  &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[5], 10);//rule 10
	setOneRule(&yawFuzzyControl.fuzzy_rules[10], &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[4], 11);//rule 11
	setOneRule(&yawFuzzyControl.fuzzy_rules[11], &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[4], 12);//rule 12
	setOneRule(&yawFuzzyControl.fuzzy_rules[12], &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[3], 13);//rule 13
	setOneRule(&yawFuzzyControl.fuzzy_rules[13], &yawFuzzyControl.inGocLech[1], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[2], 14);//rule 14
	
	setOneRule(&yawFuzzyControl.fuzzy_rules[14], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[5], 15);//rule 15
	setOneRule(&yawFuzzyControl.fuzzy_rules[15], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[5], 16);//rule 16
	setOneRule(&yawFuzzyControl.fuzzy_rules[16], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[4], 17);//rule 17
	setOneRule(&yawFuzzyControl.fuzzy_rules[17], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[4], 18);//rule 18
	setOneRule(&yawFuzzyControl.fuzzy_rules[18], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[3], 19);//rule 19
	setOneRule(&yawFuzzyControl.fuzzy_rules[19], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[2], 20);//rule 20
	setOneRule(&yawFuzzyControl.fuzzy_rules[20], &yawFuzzyControl.inGocLech[2], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[2], 21);//rule 21
	
	setOneRule(&yawFuzzyControl.fuzzy_rules[21], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[5], 22);//rule 22
	setOneRule(&yawFuzzyControl.fuzzy_rules[22], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[4], 23);//rule 23
	setOneRule(&yawFuzzyControl.fuzzy_rules[23], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[4], 24);//rule 24
	setOneRule(&yawFuzzyControl.fuzzy_rules[24], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[3], 25);//rule 25
	setOneRule(&yawFuzzyControl.fuzzy_rules[25], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[2], 26);//rule 26
	setOneRule(&yawFuzzyControl.fuzzy_rules[26], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[2], 27);//rule 27
	setOneRule(&yawFuzzyControl.fuzzy_rules[27], &yawFuzzyControl.inGocLech[3], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[1], 28);//rule 28
	
	setOneRule(&yawFuzzyControl.fuzzy_rules[28], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[4], 29);//rule 29
	setOneRule(&yawFuzzyControl.fuzzy_rules[29], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[4], 30);//rule 30
	setOneRule(&yawFuzzyControl.fuzzy_rules[30], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[3], 31);//rule 31
	setOneRule(&yawFuzzyControl.fuzzy_rules[31], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[2], 32);//rule 32
	setOneRule(&yawFuzzyControl.fuzzy_rules[32], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[2], 33);//rule 33
	setOneRule(&yawFuzzyControl.fuzzy_rules[33], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[1], 34);//rule 34
	setOneRule(&yawFuzzyControl.fuzzy_rules[34], &yawFuzzyControl.inGocLech[4], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[1], 35);//rule 35
	
	setOneRule(&yawFuzzyControl.fuzzy_rules[35], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[4], 36);//rule 36
	setOneRule(&yawFuzzyControl.fuzzy_rules[36], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[3], 37);//rule 37
	setOneRule(&yawFuzzyControl.fuzzy_rules[37], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[2], 38);//rule 38
	setOneRule(&yawFuzzyControl.fuzzy_rules[38], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[2], 39);//rule 39
	setOneRule(&yawFuzzyControl.fuzzy_rules[39], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[1], 40);//rule 40
	setOneRule(&yawFuzzyControl.fuzzy_rules[40], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[1], 41);//rule 41
	setOneRule(&yawFuzzyControl.fuzzy_rules[41], &yawFuzzyControl.inGocLech[5], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[0], 42);//rule 42

	setOneRule(&yawFuzzyControl.fuzzy_rules[42], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[0], &yawFuzzyControl.outValuePWMControl[3], 43);//rule 43
	setOneRule(&yawFuzzyControl.fuzzy_rules[43], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[1], &yawFuzzyControl.outValuePWMControl[2], 44);//rule 44
	setOneRule(&yawFuzzyControl.fuzzy_rules[44], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[2], &yawFuzzyControl.outValuePWMControl[2], 45);//rule 45
	setOneRule(&yawFuzzyControl.fuzzy_rules[45], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[3], &yawFuzzyControl.outValuePWMControl[1], 46);//rule 46
	setOneRule(&yawFuzzyControl.fuzzy_rules[46], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[4], &yawFuzzyControl.outValuePWMControl[1], 47);//rule 47
	setOneRule(&yawFuzzyControl.fuzzy_rules[47], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[5], &yawFuzzyControl.outValuePWMControl[0], 48);//rule 48
	setOneRule(&yawFuzzyControl.fuzzy_rules[48], &yawFuzzyControl.inGocLech[6], &yawFuzzyControl.inGocLech_dot[6], &yawFuzzyControl.outValuePWMControl[0], 49);//rule 49
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
	}
	
	//Mo hoa input GocLech_dot
	xx = (float)( x - fuzzyController->pre_GocLech );
	for (j=0; j < 7; j++) 
	{ 
		fuzzification( (float)xx,  &fuzzyController->inGocLech_dot[j] );
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
