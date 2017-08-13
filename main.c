/*
	  (4)\   /(1)  ^^Head
        \ /			 ||       x
         X			 ||       |
        / \          y____|O (mpu6050)
    (3)/   \(2)				

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
			PD15  																=> LEDSang (PD12 vang, PD13 cam, PD14 do, PD15 xanh)
			
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

//ERROR
		//VANG sang, Doc gia tri cua WHO I AM register, 
		//CAM sang, kiem tra da cal gyro 1000 lan chua
		//DO sang, kiem tra da set gia tri angle roll, pitch; set_gyro_angles_first_time == 0
		//VANG sang -> DO sang; battery_voltage_first == 0
		//VANG + DO cung luc; battery_voltage_first < 1000 && battery_voltage_first > 600
		//XANH sang, kiem tra doc gia tri RX ok chua
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
I2C_HandleTypeDef 				I2C_Handle_GY86_10truc;  //I2C handle, dung de doc value cua cam bien MPU6050
TIM_HandleTypeDef 				Tim3_Handle_PWM;		//timer 3 dung de output PWM ra 4 channel
TIM_HandleTypeDef 				htim1 , htim2 , htim4 , htim5; //  4 timer de capture Devo 7
ADC_HandleTypeDef 				g_Adc_HandleTypeDef; //read analog battery vol
int 											battery_voltage;
int 											battery_voltage_first;

				//---------RF Module, PWM Capture
int32_t 									IC_Throttle1,          IC_Throttle2, 								IC_Throttle_pusle_width; //Throttle (can ga) tang giam toc do quay
int32_t 									IC_Rudder_Xoay1,       IC_Rudder_Xoay2, 					  IC_Rudder_Xoay_pusle_width; //Rudder (xoay theo truc z) - goc Yaw
int32_t 									IC_Elevator_TienLui1,  IC_Elevator_TienLui2,        IC_Elevator_TienLui_pusle_width; //Elevator (tien - lui) - goc Pitch
int32_t 									IC_Aileron_TraiPhai1,  IC_Aileron_TraiPhai2,        IC_Aileron_TraiPhai_pusle_width;//Aileron_TraiPhai (trai - phai) - goc Roll
int32_t                   IC_Throttle_last_change, IC_Rudder_Xoay_last_change, IC_Elevator_TienLui_last_change, IC_Aileron_TraiPhai_last_change;
			
			//---------PWM 4 motor
int16_t 									pwm_motor_1, pwm_motor_2, pwm_motor_3, pwm_motor_4;
uint8_t 									FlyState; // 0: May bay ngung hoat dong, 1:may bay dang bay

				//---------sensor
unsigned long 						loop_timer;
int16_t 									who_i_am_reg_value_MPU6050, loop_time_cal_gyro, set_gyro_angles_first_time=0;  
int16_t										acc_x, 						acc_y, 							acc_z; 
int16_t										gyro_x, 					gyro_y, 						gyro_z; 
int16_t										gyro_x_cal, 			gyro_y_cal, 				gyro_z_cal; 
float 										angle_roll_acc, 	angle_pitch_acc, 		angle_yaw_acc;
float 										angle_roll, 			angle_pitch, 				angle_yaw;
float 										roll_level_adjust, pitch_level_adjust 	  ;
float											gyro_roll_input_pid, gyro_pitch_input_pid, gyro_yaw_input_pid; 

TM_MPU6050_t 							mpu6050_Object;
PID												PID_roll, PID_pitch, PID_yaw;
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

void 							Init_Config_ADC_read_Vol_battery(void);
															
				//Khoi tao TIMER3 output PWM											
void 							Init_TIM3_OUTPUT_PWM(void);
															
				//timer 1 & 2 & 4 & 5 Inputcapture for RF module
void 							PWM_Input_Capture_TIM1(void); 
void 							PWM_Input_Capture_TIM2(void); 
void 							PWM_Input_Capture_TIM4(void);
void 							PWM_Input_Capture_TIM5(void);
void							Reset_IC_Devo7(void);

				//Dieu chinh huong bay qua receiver
void 							SetInitDataQuadrotor(void);
void 							SetPWM_4_Motor(int16_t value);
void 							setPWM_4_Motor_Cung_Value(int16_t value); 
void 							SetPWM_1_Motor(int16_t numberMotor, int16_t newValue);
void 							SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue);
void 							SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue);
void 							Update_PWM_MOTOR_1_4_CheckMinMax(void);
void 							Apply_PWM_MOTOR_1_4_TO_ESC(void); 

				//i2c chip mpu6050 10truc
void 							GY86_I2C_Handle_GY86(void);
void 							GY86_I2C_IS_DEVICE_CONNECTED(void);
void 					    GY86_I2C_WHO_I_AM( uint8_t device_address, uint8_t register_address);
void 							GY86_MPU6050_Set_Sample_Rate(uint8_t device_address, uint8_t register_address, uint8_t rate);
void 							GY86_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value);
void 							GY86_MPU6050_SetGyroscope(uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value); 
void 							GY86_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data);
void 							GY86_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void 							GY86_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data);
void 							GY86_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output );
void 							GY86_Cal_Angle_By_Gyro_And_Acc(TM_MPU6050_t* output);
void 							GY86_Cal_Gyro_Offset(void);
void 							HMC5883L_set_config(void);
void 							HMC5883L_read_compass_data(Compass_HMC5883L* compass);
void 							Sang_Led_By_MPU6050_Values(float roll, float pitch, float yaw );
				//ham delay default
volatile 					uint32_t g_iSysTicks = 0;
void 							SysTick_Handler(){	g_iSysTicks++; }
uint32_t 					get_current_time_us(){ return g_iSysTicks; }
uint32_t 					get_current_time_ms(){ return g_iSysTicks/1000; }
void 							delay_us(uint32_t usParam){   uint32_t iStartTime = g_iSysTicks;  while( (g_iSysTicks - iStartTime) < usParam) {} }
void 							delay_ms(uint32_t piMillis){	uint32_t iStartTime = g_iSysTicks;	while( (g_iSysTicks - iStartTime ) < piMillis*1000 )	{}}			

				//Fuzzy system
void 							initFuzzySystem(void);
void 							Fuzzification_All_MF(float x, FuzzyController * fuzzyController);
void 							Apply_All_Rule( FuzzyController * fuzzyController );
void 							Defuzzification( FuzzyController * fuzzyController );
				//pid
void 							calculate_pid(void);
void 							reset_PID(void);
//-----------END Khai bao HAM-------------------------------------------------------

int main(void)
{		 				 		
		HAL_Init();		
		SystemClock_Config();	
		SysTick_Config(SystemCoreClock/1000000);
		//initFuzzySystem(); 					
		SetInitDataQuadrotor();       	
		__GPIOA_CLK_ENABLE();	__GPIOB_CLK_ENABLE();	__GPIOC_CLK_ENABLE(); __GPIOD_CLK_ENABLE();	__GPIOE_CLK_ENABLE();		
		__TIM1_CLK_ENABLE();  __TIM2_CLK_ENABLE(); 	__TIM3_CLK_ENABLE(); 	__TIM4_CLK_ENABLE(); 	__TIM5_CLK_ENABLE();  	 
		__I2C1_CLK_ENABLE(); 	        
		__ADC1_CLK_ENABLE();				 
			
		Init_LEDSANG_AND_BUTTON_USER_PORT_A0();  delay_ms(10);	//---GPIO 4 led & GPIO button user---------------	 
		Init_Config_ADC_read_Vol_battery();   	 delay_ms(10);//config ADC read vol battery 
		Init_TIM3_OUTPUT_PWM();		   delay_ms(10);	 //---Timer 3 4 channel PWM 
		PWM_Input_Capture_TIM1();    delay_ms(10);	   //Capture xung PWM of devo 7
		PWM_Input_Capture_TIM2();    delay_ms(10);	  
		PWM_Input_Capture_TIM4();	   delay_ms(10); 
		PWM_Input_Capture_TIM5();    delay_ms(10); 
		SANG_4_LED();	
		delay_ms(2000);
		SANG_4_LED_OFF();
			//---MPU6050 cau hinh PB6 SCL, PB7 SDA 
		GY86_I2C_Handle_GY86();	 																		delay_ms(50); 
		GY86_I2C_WRITE( 								0xD0, 0x6B, 0x00);					delay_ms(50);  // 0x6B PWR_MGMT_1 register; set to zero (wakes up the MPU-6050)  
		GY86_MPU6050_Set_Sample_Rate(		0xD0, 0x1A, 0x00);					delay_ms(50);  //set sample rate 8kHz
		GY86_MPU6050_SetAccelerometer( 	0xD0, 0x1C, 0x02);					delay_ms(50);  //Range is +- 8G
		GY86_MPU6050_SetGyroscope( 			0xD0, 0x1B, 0x01);					delay_ms(50);  //Gyroscope Range is +- 500 degrees/s
																								 																					 	
		//-----------------------------------------------------------------------
		GY86_I2C_IS_DEVICE_CONNECTED();	 															delay_ms(10);
		GY86_I2C_WHO_I_AM( 0xD0, 0x75); 															delay_ms(10);
		
		//cal gyro 1000 time		
		GY86_Cal_Gyro_Offset(); 																			delay_ms(10);   
		GY86_MPU6050_ReadAll( 0xD0, &mpu6050_Object); 								delay_ms(10); 
		GY86_Cal_Angle_By_Gyro_And_Acc(&mpu6050_Object); 							delay_ms(10);  
		if(set_gyro_angles_first_time == 0 && loop_time_cal_gyro == 1000)
		{
			angle_roll = angle_roll_acc;  angle_pitch = angle_pitch_acc;  set_gyro_angles_first_time = 1;	 delay_ms(10); 
		} 
		Check_EveryThing_OK();																				delay_ms(10);
		Reset_IC_Devo7();																							delay_ms(10);
		//-----------------------------------------------------------------------	  
		TIM3->CCR1 = 	2000; TIM3->CCR2 = 	2000; TIM3->CCR3 = 	2000; TIM3->CCR4 = 	2000; 
		SANG_4_LED();  delay_ms(2000); 
		TIM3->CCR1 = 	700; TIM3->CCR2 = 	700; TIM3->CCR3 = 	700; TIM3->CCR4 = 	700;
		SANG_4_LED_OFF(); 									
		SANG_4_LED_LAN_LUOT(5,40);		
		SANG_4_LED(); 		
		loop_timer = get_current_time_us();
		//-----------------------------------------------------------------------
		while(1) //main loop
		{		  
				while(FlyState == STATE_FLY_OFF)
				{
						SANG_4_LED_OFF();
						//TIM3->CCR1 = 	1000; TIM3->CCR2 = 	1000; TIM3->CCR3  = 1000; TIM3->CCR4  = 1000;
						loop_timer = get_current_time_us(); //truoc khi vao while, ta set lai loop_timer de do thoigian xu ly < 4000us
						while(FlyState == STATE_FLY_OFF) {  //Check sign to TURN ON quadrotor 
								Turn_On_Quadrotor();  	
						}
				}
				
				//---Quadrotor Fly------------------------------------------------------------------
				reset_PID();
				loop_timer = get_current_time_us(); // truoc khi vao while, ta set lai loop_timer de check < 4000us
				while(FlyState == STATE_FLY_ON) //quadrotor State Fly
				{ 
							GY86_MPU6050_ReadAll( 0xD0, &mpu6050_Object);  
							GY86_Cal_Angle_By_Gyro_And_Acc(&mpu6050_Object);  
							/*if( HAL_ADC_PollForConversion(&g_Adc_HandleTypeDef, 500) == HAL_OK )
							{ 
								battery_voltage =  battery_voltage * 0.96 + (HAL_ADC_GetValue(&g_Adc_HandleTypeDef)+65) * 1.2317 * 0.04;
							}*/
							if(IC_Throttle_pusle_width > 1150 && IC_Throttle_pusle_width <=  2000)
							{
									 	//-----------------------------------------------------
										//ROLL PID
										PID_roll.setpoint = 0;
										if(IC_Aileron_TraiPhai_pusle_width > 1510 && IC_Aileron_TraiPhai_pusle_width <= 2000)
										  PID_roll.setpoint = (IC_Aileron_TraiPhai_pusle_width - 1510); 
										else if(IC_Aileron_TraiPhai_pusle_width < 1490 && IC_Aileron_TraiPhai_pusle_width > 1000) 
											PID_roll.setpoint = (IC_Aileron_TraiPhai_pusle_width - 1490); 
										PID_roll.setpoint -= roll_level_adjust;
										PID_roll.setpoint /= (float)3.0;  
										
										//PITCH PID
										PID_pitch.setpoint = 0;
										if(IC_Elevator_TienLui_pusle_width > 1510 && IC_Elevator_TienLui_pusle_width <= 2000)
											PID_pitch.setpoint = (IC_Elevator_TienLui_pusle_width - 1510);
										else if(IC_Elevator_TienLui_pusle_width < 1490 && IC_Elevator_TienLui_pusle_width > 1000)
											PID_pitch.setpoint = (IC_Elevator_TienLui_pusle_width - 1490); 
										PID_pitch.setpoint -= pitch_level_adjust;
										PID_pitch.setpoint /= (float)3.0;  
										
										//YAW PID
										PID_yaw.setpoint = 0;
										if(IC_Throttle_pusle_width > 1150 )
										{
											if(IC_Rudder_Xoay_pusle_width > 1510 && IC_Rudder_Xoay_pusle_width <= 2000)
												PID_yaw.setpoint = (IC_Rudder_Xoay_pusle_width - 1510)/(float)3.0;
											else if(IC_Rudder_Xoay_pusle_width < 1490 && IC_Rudder_Xoay_pusle_width > 1000 )
												PID_yaw.setpoint = (IC_Rudder_Xoay_pusle_width - 1490)/(float)3.0; 
										}

										calculate_pid(); 
										
										pwm_motor_1 = (int)IC_Throttle_pusle_width + (int)PID_roll.output - (int)PID_pitch.output  - (int)PID_yaw.output;
										pwm_motor_2 = (int)IC_Throttle_pusle_width + (int)PID_roll.output + (int)PID_pitch.output  + (int)PID_yaw.output;
										pwm_motor_3 = (int)IC_Throttle_pusle_width - (int)PID_roll.output + (int)PID_pitch.output  - (int)PID_yaw.output;
										pwm_motor_4 = (int)IC_Throttle_pusle_width - (int)PID_roll.output - (int)PID_pitch.output  + (int)PID_yaw.output;
										/*if(battery_voltage < 1240 && battery_voltage > 800)
										{
											pwm_motor_1 += pwm_motor_1*( (1240-battery_voltage)/(float)3500 );
											pwm_motor_2 += pwm_motor_2*( (1240-battery_voltage)/(float)3500 );
											pwm_motor_3 += pwm_motor_3*( (1240-battery_voltage)/(float)3500 );
											pwm_motor_4 += pwm_motor_4*( (1240-battery_voltage)/(float)3500 );
										}*/
										Update_PWM_MOTOR_1_4_CheckMinMax(); 
										Apply_PWM_MOTOR_1_4_TO_ESC();		
										//-----------------------------------------------------
										Sang_Led_By_MPU6050_Values(angle_roll, angle_pitch, angle_yaw);
										if( (get_current_time_us() - loop_timer) > 4000)  SANG_4_LED_LOOP(3,30);
										while( (get_current_time_us() - loop_timer) < 4000){}; //4000 us = 4ms = 1/250Hz
										loop_timer = get_current_time_us(); 
							}
							else
							{ 
								reset_PID();
								pwm_motor_1 = 1000;  pwm_motor_2 = 1000;  pwm_motor_3 = 1000;  pwm_motor_4 = 1000;
								TIM3->CCR1 = 	1000;  TIM3->CCR2  = 1000;  TIM3->CCR3  = 1000;  TIM3->CCR4  = 1000; 
								SANG_4_LED_OFF(); LED_D_12_HIGH; LED_D_14_HIGH; delay_ms(200);
								SANG_4_LED_OFF(); LED_D_13_HIGH; LED_D_15_HIGH; delay_ms(200);
								Turn_Off_Quadrotor(); 
								loop_timer = get_current_time_us(); 
							}	   
				}
				//end while(FlyState == 1)
		}//End while(1)
}


void Turn_On_Quadrotor(void)
{
		//khoi dong quadrotor bang Receiver, ta gat 2 tick den vi tri thap nhat
		if( (IC_Throttle_pusle_width       >= 1070 && IC_Throttle_pusle_width         <= 1130) && 
			(IC_Aileron_TraiPhai_pusle_width >= 1070 && IC_Aileron_TraiPhai_pusle_width <= 1130) && 
			(IC_Elevator_TienLui_pusle_width >= 1070 && IC_Elevator_TienLui_pusle_width <= 1130) && 
			(FlyState == 0	)) 
		{	
						SANG_4_LED_LAN_LUOT(7,40);	 SANG_4_LED(); 	 delay_ms(1000); 	
			
						if( (IC_Throttle_pusle_width         >= 1070 && IC_Throttle_pusle_width         <= 1130) && 
								(IC_Aileron_TraiPhai_pusle_width >= 1070 && IC_Aileron_TraiPhai_pusle_width <= 1130) && 
								(IC_Elevator_TienLui_pusle_width >= 1070 && IC_Elevator_TienLui_pusle_width <= 1130) &&
								(FlyState == 0	)		)
						{			
								SANG_4_LED_OFF();  
								FlyState = STATE_FLY_ON; 
								pwm_motor_1 = 1060; pwm_motor_2 = 1060; pwm_motor_3 = 1060; pwm_motor_4 = 1060;
								TIM3->CCR1 = 	1060; TIM3->CCR2 = 	1060; TIM3->CCR3  = 1060; TIM3->CCR4 = 1060;
								SANG_4_LED_LOOP(7,40);  SANG_4_LED_OFF(); delay_ms(1000);   
								reset_PID();
								loop_timer = get_current_time_us(); 
						}
		} else 
		{ 
				//Turn_On_Quadrotor
				GY86_MPU6050_ReadAll( 0xD0, &mpu6050_Object);  
				GY86_Cal_Angle_By_Gyro_And_Acc(&mpu6050_Object);
				/*if( HAL_ADC_PollForConversion(&g_Adc_HandleTypeDef, 100) == HAL_OK )
				{
					battery_voltage =  battery_voltage * 0.96 + (HAL_ADC_GetValue(&g_Adc_HandleTypeDef)+65) * 1.2317 * 0.04;
					if( (battery_voltage_first < 1000 && battery_voltage_first > 600) ) 
					{
						LED_D_12_HIGH; LED_D_14_HIGH;  delay_ms(50); SANG_4_LED_OFF(); delay_ms(50); 
					} 
				}*/
					
				//-----------------------------------------------------
				//ROLL PID
				PID_roll.setpoint = 0;
				if(IC_Aileron_TraiPhai_pusle_width > 1510 && IC_Aileron_TraiPhai_pusle_width <= 2000)
					PID_roll.setpoint = (IC_Aileron_TraiPhai_pusle_width - 1510); 
				else if(IC_Aileron_TraiPhai_pusle_width < 1490 && IC_Aileron_TraiPhai_pusle_width > 1000) 
					PID_roll.setpoint = (IC_Aileron_TraiPhai_pusle_width - 1490); 
				PID_roll.setpoint -= roll_level_adjust;
				PID_roll.setpoint /= (float)3.0;  
				
				//PITCH PID
				PID_pitch.setpoint = 0;
				if(IC_Elevator_TienLui_pusle_width > 1510 && IC_Elevator_TienLui_pusle_width <= 2000)
					PID_pitch.setpoint = (IC_Elevator_TienLui_pusle_width - 1510);
				else if(IC_Elevator_TienLui_pusle_width < 1490 && IC_Elevator_TienLui_pusle_width > 1000)
					PID_pitch.setpoint = (IC_Elevator_TienLui_pusle_width - 1490); 
				PID_pitch.setpoint -= pitch_level_adjust;
				PID_pitch.setpoint /= (float)3.0;  
				
				//YAW PID
				PID_yaw.setpoint = 0;
				if(IC_Throttle_pusle_width > 1200 )
				{
					if(IC_Rudder_Xoay_pusle_width > 1510 && IC_Rudder_Xoay_pusle_width <= 2000)
						PID_yaw.setpoint = (IC_Rudder_Xoay_pusle_width - 1510)/(float)3.0;
					else if(IC_Rudder_Xoay_pusle_width < 1490 && IC_Rudder_Xoay_pusle_width > 1000 )
						PID_yaw.setpoint = (IC_Rudder_Xoay_pusle_width - 1490)/(float)3.0; 
				}
										
				calculate_pid(); 
				// hack debug; throttle IC_Throttle_pusle_width = 1500
				pwm_motor_1 = (int)1500 + (int)PID_roll.output - (int)PID_pitch.output  - (int)PID_yaw.output;
				pwm_motor_2 = (int)1500 + (int)PID_roll.output + (int)PID_pitch.output  + (int)PID_yaw.output;
				pwm_motor_3 = (int)1500 - (int)PID_roll.output + (int)PID_pitch.output  - (int)PID_yaw.output;
				pwm_motor_4 = (int)1500 - (int)PID_roll.output - (int)PID_pitch.output  + (int)PID_yaw.output;
				/*if(battery_voltage < 1240 && battery_voltage > 800)
				{
					pwm_motor_1 += pwm_motor_1*( (1240-battery_voltage)/(float)3500 );
					pwm_motor_2 += pwm_motor_2*( (1240-battery_voltage)/(float)3500 );
					pwm_motor_3 += pwm_motor_3*( (1240-battery_voltage)/(float)3500 );
					pwm_motor_4 += pwm_motor_4*( (1240-battery_voltage)/(float)3500 );
				}*/
				Update_PWM_MOTOR_1_4_CheckMinMax(); 
				TIM3->CCR1 = 	1000; TIM3->CCR2 = 	1000; TIM3->CCR3 = 	1000; TIM3->CCR4 = 	1000;
				Sang_Led_By_MPU6050_Values(angle_roll, angle_pitch, angle_yaw);
				//----------------------------------------------------- 
				if((get_current_time_us() - loop_timer) > 4000)  SANG_4_LED_LOOP(3,30); 
				while( (get_current_time_us() - loop_timer) < 4000){}; //4000 us = 4ms = 1/250Hz
				loop_timer = get_current_time_us(); 
		}
}

//-----------------------------------------------------------------------------------
void Reset_IC_Devo7(void)
{
		IC_Throttle1 = 0;		
		IC_Throttle2 = 0; 		
		IC_Throttle_pusle_width = 0;	
		IC_Throttle_last_change = 0;
		
		IC_Elevator_TienLui1 = 0;		
		IC_Elevator_TienLui2 = 0; 		
		IC_Elevator_TienLui_pusle_width = 0;
		IC_Elevator_TienLui_last_change = 0;	
		
		IC_Aileron_TraiPhai1 = 0;		
		IC_Aileron_TraiPhai2 = 0;		
		IC_Aileron_TraiPhai_pusle_width = 0;	
		IC_Aileron_TraiPhai_last_change = 0;
		
		IC_Rudder_Xoay1 = 0;		
		IC_Rudder_Xoay2 = 0;			
		IC_Rudder_Xoay_pusle_width = 0;	
		IC_Rudder_Xoay_last_change = 0; 	
}

void SetInitDataQuadrotor(void)
{
		who_i_am_reg_value_MPU6050 = ZERO_;
		battery_voltage = 0;
		FlyState = STATE_FLY_OFF; 
		//-------------------------------------------------
		IC_Throttle1 = 0;		
		IC_Throttle2 = 0; 		
		IC_Throttle_pusle_width = 0;	
		IC_Throttle_last_change = 0;
		
		IC_Elevator_TienLui1 = 0;		
		IC_Elevator_TienLui2 = 0; 		
		IC_Elevator_TienLui_pusle_width = 0;
		IC_Elevator_TienLui_last_change = 0;	
		
		IC_Aileron_TraiPhai1 = 0;		
		IC_Aileron_TraiPhai2 = 0;		
		IC_Aileron_TraiPhai_pusle_width = 0;	
		IC_Aileron_TraiPhai_last_change = 0;
		
		IC_Rudder_Xoay1 = 0;		
		IC_Rudder_Xoay2 = 0;			
		IC_Rudder_Xoay_pusle_width = 0;	
		IC_Rudder_Xoay_last_change = 0; 
		//-------------------------------------------------
	
		PID_roll.derivative_error = 0;
		PID_roll.integral_error = 0;
		PID_roll.pre_error = 0;
		PID_roll.setpoint = 0;
		PID_roll.output = 0;
		PID_roll.kP = PID_KP;
		PID_roll.kI = PID_KI;
		PID_roll.kD = PID_KD;
		PID_roll.output_max = PID_MAX_VALUE_ROLL;
		
		PID_pitch.derivative_error = 0;
		PID_pitch.integral_error = 0;
		PID_pitch.pre_error = 0;
		PID_pitch.setpoint = 0;
		PID_pitch.output = 0;
		PID_pitch.kP = PID_KP;
		PID_pitch.kI = PID_KI;
		PID_pitch.kD = PID_KD;
		PID_pitch.output_max = PID_MAX_VALUE_PITCH;
		
		
		PID_yaw.derivative_error = 0;
		PID_yaw.integral_error = 0;
		PID_yaw.pre_error = 0;
		PID_yaw.setpoint = 0;
		PID_yaw.output = 0;
		PID_yaw.kP = PID_KP_YAW;
		PID_yaw.kI = PID_KI_YAW;
		PID_yaw.kD = PID_KD_YAW;
		PID_yaw.output_max = PID_MAX_VALUE_YAW;
		//------------------------------------------------- 
}

void reset_PID(void)
{
		PID_roll.derivative_error = 0;
		PID_roll.integral_error = 0;
		PID_roll.pre_error = 0;
		PID_roll.setpoint = 0;
		PID_roll.output = 0;
		// PID_roll.kP = PID_KP;
		// PID_roll.kI = PID_KI;
		// PID_roll.kD = PID_KD;
		// PID_roll.output_max = PID_MAX_VALUE_ROLL;
		
		PID_pitch.derivative_error = 0;
		PID_pitch.integral_error = 0;
		PID_pitch.pre_error = 0;
		PID_pitch.setpoint = 0;
		PID_pitch.output = 0;
		// PID_pitch.kP = PID_KP;
		// PID_pitch.kI = PID_KI;
		// PID_pitch.kD = PID_KD;
		// PID_pitch.output_max = PID_MAX_VALUE_PITCH;
		
		
		PID_yaw.derivative_error = 0;
		PID_yaw.integral_error = 0;
		PID_yaw.pre_error = 0;
		PID_yaw.setpoint = 0;
		PID_yaw.output = 0;
		// PID_yaw.kP = PID_KP_YAW;
		// PID_yaw.kI = PID_KI_YAW;
		// PID_yaw.kD = PID_KD_YAW;
		// PID_yaw.output_max = PID_MAX_VALUE_YAW;
}



void calculate_pid(void) 
{
		float pid_error_temp;

		//Roll calculation
		pid_error_temp = gyro_roll_input_pid - PID_roll.setpoint; 
		PID_roll.integral_error +=  PID_roll.kI * pid_error_temp;
		PID_roll.derivative_error = PID_roll.kD * (pid_error_temp - PID_roll.pre_error) ;
	
		if(PID_roll.integral_error > MAX_SUM_I_VALUE_PID) 
			{ PID_roll.integral_error = MAX_SUM_I_VALUE_PID; }
		else if(PID_roll.integral_error <  MAX_SUM_I_VALUE_PID * -1) 
			{ PID_roll.integral_error = MAX_SUM_I_VALUE_PID * -1; }
		
		PID_roll.output = (PID_roll.kP * pid_error_temp) +  PID_roll.integral_error +  PID_roll.derivative_error ;
		
		if(PID_roll.output > PID_roll.output_max) { PID_roll.output = PID_roll.output_max; }
		else if(PID_roll.output < PID_roll.output_max * -1) { PID_roll.output = PID_roll.output_max * -1; }
			
		PID_roll.pre_error = pid_error_temp;
		

		
		//PITCH calculation 
		pid_error_temp = gyro_pitch_input_pid - PID_pitch.setpoint; 
		PID_pitch.integral_error +=  PID_pitch.kI * pid_error_temp;
		PID_pitch.derivative_error = PID_pitch.kD * (pid_error_temp - PID_pitch.pre_error) ;
		
		if(PID_pitch.integral_error > MAX_SUM_I_VALUE_PID) 
			{ PID_pitch.integral_error = MAX_SUM_I_VALUE_PID; }
		else if(PID_pitch.integral_error <  MAX_SUM_I_VALUE_PID * -1) 
			{ PID_pitch.integral_error = MAX_SUM_I_VALUE_PID * -1; }
		
		PID_pitch.output = (PID_pitch.kP * pid_error_temp) +  PID_pitch.integral_error +  PID_pitch.derivative_error ;
		
		if(PID_pitch.output > PID_pitch.output_max) { PID_pitch.output = PID_pitch.output_max; }
		else if(PID_pitch.output < PID_pitch.output_max * -1) { PID_pitch.output = PID_pitch.output_max * -1; }
			
		PID_pitch.pre_error = pid_error_temp;
		

		
		//YAW calculation
		pid_error_temp = gyro_yaw_input_pid - PID_yaw.setpoint; 
		PID_yaw.integral_error +=  PID_yaw.kI * pid_error_temp;
		PID_yaw.derivative_error = PID_yaw.kD * (pid_error_temp - PID_yaw.pre_error) ;
	
		if(PID_yaw.integral_error > MAX_SUM_I_VALUE_PID) 
			{ PID_yaw.integral_error = MAX_SUM_I_VALUE_PID; }
		else if(PID_yaw.integral_error <  MAX_SUM_I_VALUE_PID * -1) 
			{ PID_yaw.integral_error = MAX_SUM_I_VALUE_PID * -1; }
		
		PID_yaw.output = (PID_yaw.kP * pid_error_temp) +  PID_yaw.integral_error +  PID_yaw.derivative_error ;
		
		if(PID_yaw.output > PID_yaw.output_max) { PID_yaw.output = PID_yaw.output_max; }
		else if(PID_yaw.output < PID_yaw.output_max * -1) { PID_yaw.output = PID_yaw.output_max * -1; }
			
		PID_yaw.pre_error = pid_error_temp; 
}



void Turn_Off_Quadrotor(void) //tat quadrotor
{
	if( (IC_Throttle_pusle_width           >= 1070 && IC_Throttle_pusle_width         <= 1130) && 
			(IC_Aileron_TraiPhai_pusle_width   >= 1070 && IC_Aileron_TraiPhai_pusle_width <= 1130) && 
			(IC_Elevator_TienLui_pusle_width   >= 1070 && IC_Elevator_TienLui_pusle_width <= 1130) &&
			(IC_Rudder_Xoay_pusle_width        >= 1070 && IC_Rudder_Xoay_pusle_width      <= 1130) )  
	{ 
			SANG_4_LED_LAN_LUOT(7,40);
			SANG_4_LED(); delay_ms(1000);
			if( (IC_Throttle_pusle_width         >= 1070 && IC_Throttle_pusle_width         <= 1130) && 
					(IC_Aileron_TraiPhai_pusle_width >= 1070 && IC_Aileron_TraiPhai_pusle_width <= 1130) && 
					(IC_Elevator_TienLui_pusle_width >= 1070 && IC_Elevator_TienLui_pusle_width <= 1130) &&
					(IC_Rudder_Xoay_pusle_width      >= 1070 && IC_Rudder_Xoay_pusle_width      <= 1130) &&
					(FlyState == 1)
			)
			{				
					FlyState = STATE_FLY_OFF; 
					pwm_motor_1 = 1000;
					pwm_motor_2 = 1000;
					pwm_motor_3 = 1000;
					pwm_motor_4 = 1000;
					TIM3->CCR1 = 1000;
					TIM3->CCR2 = 1000;
					TIM3->CCR3 = 1000;
					TIM3->CCR4 = 1000;
					SANG_4_LED_OFF();
					delay_ms(2500);
					reset_PID();
					loop_timer = get_current_time_us();
			}
	}	
		
}
//-----------------------------------------------------------------------------------
//
//
//
//
void PWM_Input_Capture_TIM1(void) //Timer 1  TIM_CHANNEL_1 PWM input capture
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

void PWM_Input_Capture_TIM2(void) //timer 2  TIM_CHANNEL_1 input capture
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
		if(HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1) != HAL_OK)		
		{							Error_Handler();			}
}	

void PWM_Input_Capture_TIM4(void) //timer 4 TIM_CHANNEL_3 input capture
{
		GPIO_InitTypeDef 						GPIO_PWM_InputCapture;	
		TIM_ClockConfigTypeDef 			sClockSourceConfig;
		TIM_SlaveConfigTypeDef 			sSlaveConfig;
		TIM_MasterConfigTypeDef 		sMasterConfig;
		TIM_IC_InitTypeDef 					sConfigIC;
		
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
		if(HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3) != HAL_OK)		
		{							Error_Handler();			}
}

void PWM_Input_Capture_TIM5(void) //PA1 - TIM5_TIM_CHANNEL_4 timer 5 input capture
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
		if(HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4) != HAL_OK)		
		{							Error_Handler();			}
}	



void TIM1_CC_IRQHandler(void) //PE9 ->TIM1_CH1  IC_Throttle_pusle_width
{
	if (__HAL_TIM_GET_FLAG(&htim1, TIM_IT_CC1) == SET && __HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_CC1) == SET ) 
	{
						if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9) == SET)  //ngat canh len
						{
							if(IC_Throttle_last_change == 0)
							{
								IC_Throttle_last_change=1; 
								IC_Throttle1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
							}
						}
						else if(IC_Throttle_last_change == 1) //ngat canh xuong
						{	
							IC_Throttle_last_change = 0; 
							IC_Throttle2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); 
							if(IC_Throttle2 > IC_Throttle1 && (IC_Throttle2 - IC_Throttle1) > 1000 && (IC_Throttle2 - IC_Throttle1) <= 2000)
							{
								IC_Throttle_pusle_width		= (IC_Throttle2 - IC_Throttle1); 
							} 	 
							__HAL_TIM_SetCounter (&htim1, 0);  TIM1->CNT = 0;  
						}
						
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1); __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
    }
}


void TIM2_IRQHandler(void) //PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw
{
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_IT_CC1) == SET && __HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_CC1) == SET )       
	{
					if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)== SET) //ngat canh len
					{ 
						if(IC_Rudder_Xoay_last_change == 0)
						{
							IC_Rudder_Xoay_last_change = 1; 
							IC_Rudder_Xoay1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); 
						} 
					}else if( IC_Rudder_Xoay_last_change == 1) //ngat canh xuong
					{
						IC_Rudder_Xoay_last_change = 0; 
						IC_Rudder_Xoay2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); 
						if(IC_Rudder_Xoay2>IC_Rudder_Xoay1 && (IC_Rudder_Xoay2 - IC_Rudder_Xoay1) > 1000 && (IC_Rudder_Xoay2 - IC_Rudder_Xoay1) <= 2000)
						{
							IC_Rudder_Xoay_pusle_width		= (IC_Rudder_Xoay2 - IC_Rudder_Xoay1); 
						} 
						__HAL_TIM_SetCounter (&htim2, 0);  
						TIM2->CNT = 0; 
					}
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1); __HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);
	}
}


void TIM4_IRQHandler(void) //PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch
{ 	
	
	if (__HAL_TIM_GET_FLAG(&htim4, TIM_IT_CC3) == SET && __HAL_TIM_GET_ITSTATUS(&htim4, TIM_IT_CC3) == SET) 
  { 
				
			if( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == SET) //ngat canh len
			{ 
							if(IC_Elevator_TienLui_last_change == 0)
							{
								IC_Elevator_TienLui_last_change = 1;
								IC_Elevator_TienLui1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);   
							} 
			}else if( IC_Elevator_TienLui_last_change == 1)//ngat canh xuong
			{ 
							IC_Elevator_TienLui_last_change = 0; 
							IC_Elevator_TienLui2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);  
							if(IC_Elevator_TienLui2 >  IC_Elevator_TienLui1 && (IC_Elevator_TienLui2 - IC_Elevator_TienLui1) > 1000 && (IC_Elevator_TienLui2 - IC_Elevator_TienLui1) <= 2000 )
							{
								IC_Elevator_TienLui_pusle_width = (IC_Elevator_TienLui2 - IC_Elevator_TienLui1); 
							}  
							__HAL_TIM_SetCounter (&htim4, 0); 
							TIM4->CNT = 0;  
			}

			__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3); __HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_CC3);
	}
}
void TIM5_IRQHandler(void) //PA3 - TIM5_CH4 ////Aileron_TraiPhai (trai - phai) - goc Roll
{
	
		if (__HAL_TIM_GET_FLAG(&htim5, TIM_IT_CC4) == SET && __HAL_TIM_GET_ITSTATUS(&htim5, TIM_IT_CC4) == SET) 
			{ 
							if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)== SET) //ngat canh len
							{ 
								if(IC_Aileron_TraiPhai_last_change == 0)
								{
									IC_Aileron_TraiPhai_last_change = 1; 
									IC_Aileron_TraiPhai1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);  
								} 
							}else if( IC_Aileron_TraiPhai_last_change == 1) //ngat canh xuong
							{
								IC_Aileron_TraiPhai_last_change = 0; 
								IC_Aileron_TraiPhai2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);  
								if(IC_Aileron_TraiPhai2>IC_Aileron_TraiPhai1 && (IC_Aileron_TraiPhai2 - IC_Aileron_TraiPhai1) > 1000 && (IC_Aileron_TraiPhai2 - IC_Aileron_TraiPhai1) <= 2000)
								{
									IC_Aileron_TraiPhai_pusle_width		= (IC_Aileron_TraiPhai2 - IC_Aileron_TraiPhai1); 
								} 
								__HAL_TIM_SetCounter (&htim5, 0); 
								TIM5->CNT = 0;  
							} 
							
					__HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC4); __HAL_TIM_CLEAR_FLAG(&htim5, TIM_IT_CC4);
		}
}
//
//End PWM input capture
//
//-----------------------------------------------------------------------------------
void Check_EveryThing_OK(void)
{	   
	  //        15 XANH             ^^
		//14 DO           12 VANG     ||
	  //        13 CAM              || 
		SANG_4_LED_OFF();
		if(who_i_am_reg_value_MPU6050 != MPU6050_I_AM_VALUES)  //---VANG sang, Doc gia tri cua WHO I AM register, 
		{
				while(1) {			 LED_D_12_HIGH; delay_ms(500); SANG_4_LED_OFF(); delay_ms(500); }
		}	
		if(loop_time_cal_gyro != 1000) //CAM sang, kiem tra da cal gyro 1000 lan chua
		{
				while(1) {			 LED_D_13_HIGH; delay_ms(500); SANG_4_LED_OFF(); delay_ms(500); }
		}	
		
		if(set_gyro_angles_first_time == 0 ) //DO sang, kiem tra da set gia tri angle roll, pitch lan dau chua
		{
				while(1) {			 LED_D_14_HIGH; delay_ms(500); SANG_4_LED_OFF(); delay_ms(500); }
		}    
		
		/* 
		int i = 0;
		while(i < 1000) // kiem tra battery volt
		{
			if( HAL_ADC_PollForConversion(&g_Adc_HandleTypeDef, 500) == HAL_OK )
			{
				battery_voltage_first = (HAL_ADC_GetValue(&g_Adc_HandleTypeDef)+65) * 1.2317;
				battery_voltage =       (HAL_ADC_GetValue(&g_Adc_HandleTypeDef)+65) * 1.2317;
			} 
			i++;
		} 
		if(battery_voltage_first == 0) // VANG sang -> DO sang
		{
			while(1) {			 LED_D_12_HIGH;  delay_ms(250); SANG_4_LED_OFF(); delay_ms(250);  
											 LED_D_14_HIGH;  delay_ms(250); SANG_4_LED_OFF(); delay_ms(250);  }
		} 
		if( battery_voltage_first != 0 && battery_voltage_first < 1000 && battery_voltage_first > 600 ) //VANG + DO cung luc
		{
			while(1) {	 LED_D_12_HIGH; LED_D_14_HIGH;  delay_ms(500); SANG_4_LED_OFF(); delay_ms(500);  }
		}*/ 	
		 
		//XANH sang, kiem tra doc gia tri RX ok chua
		if(IC_Throttle_pusle_width <= 0 || IC_Aileron_TraiPhai_pusle_width <= 0 || IC_Elevator_TienLui_pusle_width <= 0 || IC_Rudder_Xoay_pusle_width <= 0 || 
			IC_Throttle_pusle_width > 2000 || IC_Aileron_TraiPhai_pusle_width > 2000 || IC_Elevator_TienLui_pusle_width > 2000 || IC_Rudder_Xoay_pusle_width > 2000)
		{
				while(1) {			 LED_D_15_HIGH; delay_ms(500); SANG_4_LED_OFF(); delay_ms(500); }
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
//Init ADC ADC1 -  channel 11 - pin PC1
//

void 	Init_Config_ADC_read_Vol_battery(void)
{
		GPIO_InitTypeDef gpioInit;
		ADC_ChannelConfTypeDef adcChannel;
	
		gpioInit.Pin = GPIO_PIN_1;
		gpioInit.Mode = GPIO_MODE_ANALOG;
		gpioInit.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &gpioInit);

		HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC_IRQn);
	
		g_Adc_HandleTypeDef.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2; 
		g_Adc_HandleTypeDef.Instance = ADC1;
		g_Adc_HandleTypeDef.Init.ScanConvMode = DISABLE;
		g_Adc_HandleTypeDef.Init.Resolution = ADC_RESOLUTION_10B; //2^10 = 1024 (0->1023)
		g_Adc_HandleTypeDef.Init.ContinuousConvMode = ENABLE; //important Set hadc.Init.ContinuousConvMode to ENABLE
		g_Adc_HandleTypeDef.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
		g_Adc_HandleTypeDef.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		g_Adc_HandleTypeDef.Init.DataAlign = ADC_DATAALIGN_RIGHT;
		g_Adc_HandleTypeDef.Init.NbrOfConversion = 1;  
		g_Adc_HandleTypeDef.Init.DiscontinuousConvMode = DISABLE;
		g_Adc_HandleTypeDef.Init.NbrOfDiscConversion = 0;
		g_Adc_HandleTypeDef.Init.DMAContinuousRequests = DISABLE;
		g_Adc_HandleTypeDef.Init.EOCSelection = DISABLE; //important hadc.Init.EOCSelection to DISABLE for continuous sampling. 
		HAL_ADC_Init(&g_Adc_HandleTypeDef);
		
		adcChannel.Channel = ADC_CHANNEL_11;
		adcChannel.Rank = 1;
		adcChannel.SamplingTime = ADC_SAMPLETIME_3CYCLES;
		adcChannel.Offset = 0;

		while (HAL_ADC_ConfigChannel(&g_Adc_HandleTypeDef, &adcChannel) != HAL_OK) {  }
		
		while (HAL_ADC_Start(&g_Adc_HandleTypeDef) ){}
		
		//__HAL_ADC_ENABLE_IT(&g_Adc_HandleTypeDef, ADC_IT_EOC);
		//while(HAL_ADC_Start_IT(&g_Adc_HandleTypeDef) ){} 
}

/*
void ADC_IRQHandler() //interrupt handle cua ADC
{
    HAL_ADC_IRQHandler(&g_Adc_HandleTypeDef);  
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) // xu ly interrupt handle
{
    if(hadc->Instance == ADC1)
    {
    	if(__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_EOC) )
    	{ 
				battery_voltage =  battery_voltage * 0.96 + (HAL_ADC_GetValue(&g_Adc_HandleTypeDef)+65) * 1.2317 * 0.04;
    	}
    } 
}
*/


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

void GY86_I2C_Handle_GY86(void)
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
	
		I2C_Handle_GY86_10truc.Instance = I2C1;
		I2C_Handle_GY86_10truc.Init.ClockSpeed = 400000; // 400kHz
		I2C_Handle_GY86_10truc.Init.DutyCycle = I2C_DUTYCYCLE_2;
		I2C_Handle_GY86_10truc.Init.OwnAddress1 = 0;
		I2C_Handle_GY86_10truc.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		I2C_Handle_GY86_10truc.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		I2C_Handle_GY86_10truc.Init.OwnAddress2 = 0;
		I2C_Handle_GY86_10truc.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
		I2C_Handle_GY86_10truc.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
		HAL_I2C_Init(&I2C_Handle_GY86_10truc) ; 	
		//while (__HAL_I2C_ENABLE(&I2C_Handle_GY86_10truc) ){}; 
}



void GY86_I2C_IS_DEVICE_CONNECTED(void)
{
	while(HAL_I2C_IsDeviceReady(&I2C_Handle_GY86_10truc, 0xD0, 10, 100) != HAL_OK) 
	{
		//k ket noi dc mpu6050; LED VANG sang 
		LED_D_12_HIGH;		delay_ms(50);			SANG_4_LED_OFF();			delay_ms(50);
	}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
}

void GY86_I2C_WHO_I_AM(uint8_t device_address, uint8_t register_address)
{	
	uint8_t data;
	while (HAL_I2C_Master_Transmit(&I2C_Handle_GY86_10truc, (uint16_t)device_address, &register_address, 1, 100) != HAL_OK) 
	{}	
	
	while (HAL_I2C_Master_Receive(&I2C_Handle_GY86_10truc, device_address, &data, 1, 100) != HAL_OK) 
	{}
	who_i_am_reg_value_MPU6050 = data;
}


void GY86_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data) 
{
	uint8_t d[2];		
	d[0] = register_address;
	d[1] = data;	
	while(HAL_I2C_Master_Transmit(&I2C_Handle_GY86_10truc, (uint16_t)device_address, (uint8_t *)d, 2, 100) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY)
	{}
}
	
	
void GY86_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data)
{
	while (HAL_I2C_Master_Transmit(&I2C_Handle_GY86_10truc, (uint16_t)device_address, &register_address, 1, 100) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
		
	while (HAL_I2C_Master_Receive(&I2C_Handle_GY86_10truc, device_address, data, 1, 100) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
}


void GY86_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)  
{
	while (HAL_I2C_Master_Transmit(&I2C_Handle_GY86_10truc, (uint16_t)device_address, &register_address, 1, 100) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
		
	while (HAL_I2C_Master_Receive(&I2C_Handle_GY86_10truc, device_address, data, count, 100) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
}


void GY86_MPU6050_Set_Sample_Rate(uint8_t device_address, uint8_t register_address, uint8_t rate) 
{
	GY86_I2C_WRITE( device_address, register_address, rate);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
}

void GY86_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value) 
{
	uint8_t temp;		
	GY86_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
		
	temp = (temp & 0xE7) | (uint8_t)Acc_8G_Value << 3;
	GY86_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
}

void GY86_MPU6050_SetGyroscope( uint8_t device_address, uint8_t register_address, uint8_t Gyro_Config_Value) 
{
	uint8_t temp;		
	GY86_I2C_READ( device_address, register_address, &temp);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
	temp = (temp & 0xE7) | (uint8_t)Gyro_Config_Value << 3;
	GY86_I2C_WRITE( device_address, register_address, temp);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
}

void GY86_MPU6050_ReadAccelerometer( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	GY86_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
		
	output->Acc_X = (int16_t)(data[0] << 8 | data[1]);
	output->Acc_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Acc_Z = (int16_t)(data[4] << 8 | data[5]);
}

void GY86_MPU6050_ReadGyroscope( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	GY86_I2C_READ_MULTI( device_address, MPU6050_GYRO_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY) {}
		
	output->Gyro_X = (int16_t)(data[0] << 8 | data[1]);
	output->Gyro_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Gyro_Z = (int16_t)(data[4] << 8 | data[5]);
}	



void GY86_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[14];
	int16_t temp;	
	
	GY86_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 14); /* Read full raw data, 14bytes */	
	while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY)
	{}
	output->Acc_X = (int16_t)(data[0] << 8 | data[1]);	/* Format accelerometer data */
	output->Acc_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Acc_Z = (int16_t)(data[4] << 8 | data[5]);
 
	temp = (data[6] << 8 | data[7]); /* Format temperature */
	output->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	output->Gyro_X = (int16_t)(data[8] << 8 | data[9]); /* Format gyroscope data */
	output->Gyro_Y = (int16_t)(data[10] << 8 | data[11]);
	output->Gyro_Z = (int16_t)(data[12] << 8 | data[13]);
		
	acc_x = output->Acc_X;
	acc_y = output->Acc_Y;
	acc_z = output->Acc_Z;
	gyro_x = output->Gyro_X;
	gyro_y = output->Gyro_Y;
	gyro_z = output->Gyro_Z;
}

void GY86_Cal_Gyro_Offset(void)//tinh gyro offset 3 truc
{
		float total_x, total_y, total_z;
		total_x=0.0;  total_y=0.0;  total_z=0.0;
		for(loop_time_cal_gyro=0; loop_time_cal_gyro < 1000; loop_time_cal_gyro++)
		{
			GY86_MPU6050_ReadAll( 0xD0, &mpu6050_Object); 
			delay_ms(1);
			if(loop_time_cal_gyro % 100 == 0) { SANG_4_LED(); delay_ms(30); SANG_4_LED_OFF(); } 
			total_x +=  mpu6050_Object.Gyro_X;
			total_y +=  mpu6050_Object.Gyro_Y;
			total_z +=  mpu6050_Object.Gyro_Z; 
		}
		if(loop_time_cal_gyro==1000 && total_x != 0 && total_y != 0 && total_z != 0)
		{
			gyro_x_cal = (int)total_x/1000;
			gyro_y_cal = (int)total_y/1000;
			gyro_z_cal = (int)total_z/1000;
		}
}

void GY86_Cal_Angle_By_Gyro_And_Acc(TM_MPU6050_t* mpu6050_Object)
{
	int sign; float miu = 0.001; 
	gyro_x = gyro_x - gyro_x_cal;
	gyro_y = (gyro_y - gyro_y_cal) * -1; // vi theo chieu + 
	gyro_z = (gyro_z - gyro_z_cal) * -1; // vi theo chieu +  
	
	//Calculate angle by gyro data
	angle_roll  += gyro_x * (float)0.0000611; 
	angle_pitch += gyro_y * (float)0.0000611;  //0.0000611 = 1/ (250Hz /65.5)
	
	 //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin( gyro_z * (float)0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin( gyro_z * (float)0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.
	
	//Calculate angle by accelerate data
	if(acc_z > 0) sign =1;
	else if(acc_z <= 0) sign = -1;
	angle_roll_acc =  atan2(acc_y, sign*sqrt(acc_z*acc_z + miu*acc_x*acc_x))  * (float)57.296;
	angle_pitch_acc = atan2(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))           * (float)57.296 *-1;  // *-1 vi theo chieu +
	
	//Calculate angle by gyro & accelerate data
	angle_roll =  angle_roll  * (float)0.9996 + angle_roll_acc  * (float)0.0004;
	angle_pitch = angle_pitch * (float)0.9996 + angle_pitch_acc * (float)0.0004;  
	
	//Calculate setpoint PID
	roll_level_adjust =  angle_roll  * 15;
	pitch_level_adjust = angle_pitch * 15;
	
	//Calculate input (error) PID
	gyro_roll_input_pid  =   gyro_roll_input_pid *(float)0.8 + (gyro_x/(float)65.5)*(float)0.2; //gyro pid input is deg/second
	gyro_pitch_input_pid =  gyro_pitch_input_pid *(float)0.8 + (gyro_y/(float)65.5)*(float)0.2; 
	gyro_yaw_input_pid  =   gyro_yaw_input_pid   *(float)0.8 + (gyro_z/(float)65.5)*(float)0.2; 
}	







void HMC5883L_set_config(void) 
{
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Register_A, 0x18);//set rate = 75Hz
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Register_B, 0x60);//full scale = +/- 2.5 Gauss
		GY86_I2C_WRITE( HMC5883L_Device_Address, HMC5883L_Measurement_Mode_Register, Mode_Measurement_Continuous);
		while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY)
		{
			//while(1){SANG_4_LED();}
		}
}

void HMC5883L_read_compass_data(Compass_HMC5883L* compass)
{
		uint8_t data[6];
		GY86_I2C_READ_MULTI( HMC5883L_Device_Address, HMC5883L_Data_Register_Begin, data, 6); //select register 3
		while(HAL_I2C_GetState(&I2C_Handle_GY86_10truc)!=HAL_I2C_STATE_READY){}
			
		compass->Compass_X = (int16_t)(data[0] << 8 | data[1]);
		compass->Compass_Y = (int16_t)(data[2] << 8 | data[3]);
		compass->Compass_Z = (int16_t)(data[4] << 8 | data[5]);
}	




void Sang_Led_By_MPU6050_Values(float roll, float pitch, float yaw )
{
	//   xanh
	//do      vang
	//   cam
	//(PD12 vang, PD13 cam, PD14 do, PD15 xanh) 
	if(roll > VALUE_SANG_LED_BY_MPU6050 && pitch > VALUE_SANG_LED_BY_MPU6050)
	{
			LED_D_14_HIGH;
			LED_D_15_HIGH;
					LED_D_12_LOW;
					LED_D_13_LOW;
	}else if(roll > VALUE_SANG_LED_BY_MPU6050 && pitch < VALUE_SANG_LED_BY_MPU6050*-1 )
	{
			LED_D_14_HIGH;
			LED_D_13_HIGH;
					LED_D_12_LOW;
					LED_D_15_LOW;
	}else if(roll < (VALUE_SANG_LED_BY_MPU6050*-1) && pitch > VALUE_SANG_LED_BY_MPU6050)
	{
			LED_D_12_HIGH;
			LED_D_15_HIGH;
					LED_D_13_LOW;
					LED_D_14_LOW;
	}else if(roll < (VALUE_SANG_LED_BY_MPU6050*-1) && pitch <  VALUE_SANG_LED_BY_MPU6050*-1 )
	{
		LED_D_12_HIGH;
		LED_D_13_HIGH;
					LED_D_14_LOW;
					LED_D_15_LOW;
	}else{
		if(roll > VALUE_SANG_LED_BY_MPU6050)
		{
			LED_D_14_HIGH;
		}else if(roll < -1* VALUE_SANG_LED_BY_MPU6050)
		{
			LED_D_12_HIGH;
		}else{
			LED_D_14_LOW;
			LED_D_12_LOW;
		}
		
		if(pitch > VALUE_SANG_LED_BY_MPU6050)
		{
			LED_D_15_HIGH;
		}else if(pitch < -1* VALUE_SANG_LED_BY_MPU6050)
		{
			LED_D_13_HIGH;
		}else{ 
				LED_D_13_LOW; 
				LED_D_15_LOW;
		} 
	}  
		
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


void Apply_PWM_MOTOR_1_4_TO_ESC(void)
{
	TIM3->CCR1 = 	pwm_motor_1;
	TIM3->CCR2 = 	pwm_motor_2;
	TIM3->CCR3 = 	pwm_motor_3;
	TIM3->CCR4 = 	pwm_motor_4;
}

void Update_PWM_MOTOR_1_4_CheckMinMax(void)
{
	if(IC_Throttle_pusle_width > 1000 && IC_Throttle_pusle_width < 2000)
	{

			if(pwm_motor_1 > (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_1 = (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_2 > (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_2 = (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_3 > (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_3 = (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_4 > (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_4 = (IC_Throttle_pusle_width + MAX_VALUE_MOTOR_THROTTLE);
			}



			if(pwm_motor_1 < (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_1 = (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_2 < (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_2 = (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_3 < (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_3 = (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE);
			}

			if(pwm_motor_4 < (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE) )
			{
				pwm_motor_4 = (IC_Throttle_pusle_width - MAX_VALUE_MOTOR_THROTTLE);
			}
	}

	if( pwm_motor_1 < 1100) pwm_motor_1 = 	1100;
	else if( pwm_motor_1 > 1900) pwm_motor_1 = 1900;
	
	if( pwm_motor_2 < 1100) pwm_motor_2 = 	1100;
	else if( pwm_motor_2 > 1900) pwm_motor_2 = 1900;
	
	if( pwm_motor_3 < 1100) pwm_motor_3 = 	1100;
	else if( pwm_motor_3 > 1900) pwm_motor_3 = 1900;
	
	if( pwm_motor_4 < 1100) pwm_motor_4 = 	1100;
	else if( pwm_motor_4 > 1900) pwm_motor_4 = 1900;

}
	
void setPWM_4_Motor_Cung_Value(int16_t value)
{
	TIM3->CCR1 = 	value;
	TIM3->CCR2 = 	value;
	TIM3->CCR3 = 	value;
	TIM3->CCR4 = 	value;
}


void SetPWM_4_Motor(int16_t value) //set 4 motor cung 1 speed
{
	if(value==0)
	{
		pwm_motor_1 = 0;	
		pwm_motor_2 = 0;		
		pwm_motor_3 = 0;		
		pwm_motor_4 = 0;						
	}
	else
	{
			if(value < 1100)
			{
					pwm_motor_1 = 1100;					
					pwm_motor_2 = 1100;					
					pwm_motor_3 = 1100;					
					pwm_motor_4 = 1100;
			}else 
			if(value > 1900)
			{
					pwm_motor_1 = 1900;					
					pwm_motor_2 = 1900;
					pwm_motor_3 = 1900;					
					pwm_motor_4 = 1900;
			}else{
					pwm_motor_1 = value;										
					pwm_motor_2 = value;
					pwm_motor_3 = value;										
					pwm_motor_4 = value;
			}	
	}
	TIM3->CCR1 = 	pwm_motor_1;
	TIM3->CCR2 = 	pwm_motor_2;
	TIM3->CCR3 = 	pwm_motor_3;
	TIM3->CCR4 = 	pwm_motor_4;
}

		
void SetPWM_1_Motor(int16_t numberMotor, int16_t newValue) //set speed cho moi motor rieng le
{
		switch(numberMotor) 
		{
		 case 1  :
				if(newValue <= 1100)		 {	pwm_motor_1 = 1100;}
				else if (newValue >= 1900)	 { pwm_motor_1 = 1900;}
				else 	 { pwm_motor_1 = newValue;					}
				break; 
		
		 case 2  :
				if(newValue <= 1100)							{pwm_motor_2 = 1100;}
				else if (newValue >= 1900)				{pwm_motor_2 = 1900;}
				else 																				{pwm_motor_2 = newValue;	}
				break; 
		 
		 case 3  :
				if(newValue <= 1100)							{pwm_motor_3 = 1100;}
				else if (newValue >= 1900)				{pwm_motor_3 = 1900;}
				else 																				{pwm_motor_3 = newValue;	}
				break; 
		
		 case 4  :
				if(newValue <= 1100)							{pwm_motor_4 = 1100;}
				else if (newValue >= 1900)				{pwm_motor_4 = 1900;}
				else 																				{pwm_motor_4 = newValue;	}
				break; 
		}
		TIM3->CCR1 = 	pwm_motor_1;
		TIM3->CCR2 = 	pwm_motor_2;
		TIM3->CCR3 = 	pwm_motor_3;
		TIM3->CCR4 = 	pwm_motor_4;
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
	if(value >= LIMIT_OUTPUT_FUZZY)	return LIMIT_OUTPUT_FUZZY;
	if(value <= -LIMIT_OUTPUT_FUZZY) return -LIMIT_OUTPUT_FUZZY;
	return value;
}


int16_t Giam_Do_Vot_Lo(int16_t value)
{
	if(value >= DO_VOT_LO)		
		value = DO_VOT_LO;
	if(value <= 0)		value = 0;
	return value;
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



//------------------------------------------------------------------------------------------------------------------------------
//
//
//
//FUZZY SYSTEM --------------------------------------------------------------
//
void initFuzzySystem(void)
{
	//rollFuzzyControl.pre_GocLech = 0;
	//pitchFuzzyControl.pre_GocLech = 0;
	//yawFuzzyControl.pre_GocLech = 0;
	
	//rollFuzzyControl.output = 0;
	//pitchFuzzyControl.output = 0;
	//yawFuzzyControl.output = 0;
	
	//Buoc 1: Khai bao MF:
	/*GocLech*/
	setABCD_MF(&rollFuzzyControl.inGocLech[0],   -45,      -45,      -21,     -14,    TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VerySmall
	setABCD_MF(&rollFuzzyControl.inGocLech[1],   -21,      -14,     -14,      -7,    TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleSmall
	setABCD_MF(&rollFuzzyControl.inGocLech[2],   -14,     -7,        -7,       0,     TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Small
	setABCD_MF(&rollFuzzyControl.inGocLech[3],    -7,      0,          0,      7,      TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.inGocLech[4],     0,     7,         7,       14,      TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Big
	setABCD_MF(&rollFuzzyControl.inGocLech[5],     7,      14,         14,     21,     TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleBig
	setABCD_MF(&rollFuzzyControl.inGocLech[6],    14,     21,         45,      45,   TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VeryBig


	setABCD_MF(&pitchFuzzyControl.inGocLech[0], -45,      -45,      -21,     -14, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VerySmall
	setABCD_MF(&pitchFuzzyControl.inGocLech[1],  -21,      -14,     -14,      -7, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleSmall
	setABCD_MF(&pitchFuzzyControl.inGocLech[2],  -14,     -7,        -7,       0,   TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Small
	setABCD_MF(&pitchFuzzyControl.inGocLech[3],   -7,      0,          0,      7,   TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.inGocLech[4],    0,     7,         7,       14, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//Big
	setABCD_MF(&pitchFuzzyControl.inGocLech[5],   7,      14,         14,     21,  TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//LittleBig
	setABCD_MF(&pitchFuzzyControl.inGocLech[6],  14,     21,         45,      45, TYPE_INPUT_GOCLECH, GocLech_minXD, GocLech_maxXD);//VeryBig
		
	

	/*GocLech_dot*/
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[0] ,  -20,      -20,    -9.6,  -7, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryNagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[1] ,   -9.6,  -5.5,   -5.5,  -3, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittleNagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[2] ,   -6,     -2,  -2,    0, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Nagative
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[3] ,   -2,     0,        0,     2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[4] ,    0,      2,     2,    6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Positive
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[5] ,    3,    5.5,     5.5,   9.6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittlePositive
	setABCD_MF(&rollFuzzyControl.inGocLech_dot[6] ,    7,     9.6,     20,    20, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryPositive

	
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[0],  -20,      -20,    -9.6,  -7, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryNagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[1],   -9.6,  -5.5,   -5.5,  -3, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittleNagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[2],  -6,     -2,  -2,    0, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Nagative
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[3],   -2,     0,      0,   2, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[4],    0,      2,     2,    6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//Positive
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[5],    3,    5.5,     5.5,   9.6, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//LittlePositive
	setABCD_MF(&pitchFuzzyControl.inGocLech_dot[6],    7,     9.6,     20,    20, TYPE_INPUT_GOCLECH_DOT, GocLech_dot_minXD, GocLech_dot_maxXD);//VeryPositive
	
	
	
	/*ValuePWMControl*/
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[0] , -120,  -120,   -75,    -50, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[1] ,  -75,   -50,   -50,   -25, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[2] ,  -50,   -25,   -25,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[3] ,  -25,     0,     0,    25, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[4] ,    0,    25,    25,    50, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[5] ,   25,    50,    50,    75, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&rollFuzzyControl.outValuePWMControl[6] ,   50,    75,   120,   120, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast

	
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[0], -120,  -120,   -75,    -50, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VerySlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[1],  -75,   -50,   -50,    -25, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleSlow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[2],   -50,   -25,   -25,     0, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Slow
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[3],   -25,     0,     0,    25,  TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Zero
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[4],    0,    25,    25,     50, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//Fast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[5],    25,    50,    50,    75, TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//LittleFast
	setABCD_MF(&pitchFuzzyControl.outValuePWMControl[6],   50,    75,   120,    120,  TYPE_OUTPUT_ValuePWMControl, ValuePWMControl_minXD, ValuePWMControl_maxXD);//VeryFast
	
	
	
	//Buoc 2: RULE FuzzySystem  --------------------------------------------	
	/*
		GocLech =     {VerySmall,    LittleSmall,    Small,    Zero, Big,      LittleBig,      VeryBig}
		GocLech_dot = {VeryNagative, LittleNagative, Nagative, Zero, Positive, LittlePositive, VeryPositive}
		ValuePWM =    {VerySlow,     LittleSlow,     Slow,     Zero, Fast,     LittleFast,     VeryFast}
	*/
	/*
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
	*/
	
	
	//ROLL
	setOneRule(&rollFuzzyControl.fuzzy_rules[0],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[6], 1);//rule 1
	setOneRule(&rollFuzzyControl.fuzzy_rules[1],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[6], 2);//rule 2
	setOneRule(&rollFuzzyControl.fuzzy_rules[2],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[6], 3);//rule 3
	setOneRule(&rollFuzzyControl.fuzzy_rules[3],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[6], 4);//rule 4
	setOneRule(&rollFuzzyControl.fuzzy_rules[4],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[6], 5);//rule 5
	setOneRule(&rollFuzzyControl.fuzzy_rules[5],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[6], 6);//rule 6
	setOneRule(&rollFuzzyControl.fuzzy_rules[6],  &rollFuzzyControl.inGocLech[0], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[6], 7);//rule 7
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[7],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[5], 8);//rule 8
	setOneRule(&rollFuzzyControl.fuzzy_rules[8],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[5], 9);//rule 9
	setOneRule(&rollFuzzyControl.fuzzy_rules[9],  &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[5], 10);//rule 10
	setOneRule(&rollFuzzyControl.fuzzy_rules[10], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[5], 11);//rule 11
	setOneRule(&rollFuzzyControl.fuzzy_rules[11], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[5], 12);//rule 12
	setOneRule(&rollFuzzyControl.fuzzy_rules[12], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[5], 13);//rule 13
	setOneRule(&rollFuzzyControl.fuzzy_rules[13], &rollFuzzyControl.inGocLech[1], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[5], 14);//rule 14
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[14], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[4], 15);//rule 15
	setOneRule(&rollFuzzyControl.fuzzy_rules[15], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[4], 16);//rule 16
	setOneRule(&rollFuzzyControl.fuzzy_rules[16], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[4], 17);//rule 17
	setOneRule(&rollFuzzyControl.fuzzy_rules[17], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[4], 18);//rule 18
	setOneRule(&rollFuzzyControl.fuzzy_rules[18], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[4], 19);//rule 19
	setOneRule(&rollFuzzyControl.fuzzy_rules[19], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[4], 20);//rule 20
	setOneRule(&rollFuzzyControl.fuzzy_rules[20], &rollFuzzyControl.inGocLech[2], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[4], 21);//rule 21
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[21], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[3], 22);//rule 22
	setOneRule(&rollFuzzyControl.fuzzy_rules[22], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[3], 23);//rule 23
	setOneRule(&rollFuzzyControl.fuzzy_rules[23], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[3], 24);//rule 24
	setOneRule(&rollFuzzyControl.fuzzy_rules[24], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[3], 25);//rule 25
	setOneRule(&rollFuzzyControl.fuzzy_rules[25], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[3], 26);//rule 26
	setOneRule(&rollFuzzyControl.fuzzy_rules[26], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[3], 27);//rule 27
	setOneRule(&rollFuzzyControl.fuzzy_rules[27], &rollFuzzyControl.inGocLech[3], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[3], 28);//rule 28
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[28], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[2], 29);//rule 29
	setOneRule(&rollFuzzyControl.fuzzy_rules[29], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[2], 30);//rule 30
	setOneRule(&rollFuzzyControl.fuzzy_rules[30], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[2], 31);//rule 31
	setOneRule(&rollFuzzyControl.fuzzy_rules[31], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[2], 32);//rule 32
	setOneRule(&rollFuzzyControl.fuzzy_rules[32], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[2], 33);//rule 33
	setOneRule(&rollFuzzyControl.fuzzy_rules[33], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[2], 34);//rule 34
	setOneRule(&rollFuzzyControl.fuzzy_rules[34], &rollFuzzyControl.inGocLech[4], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[2], 35);//rule 35
	
	setOneRule(&rollFuzzyControl.fuzzy_rules[35], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[1], 36);//rule 36
	setOneRule(&rollFuzzyControl.fuzzy_rules[36], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[1], 37);//rule 37
	setOneRule(&rollFuzzyControl.fuzzy_rules[37], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[1], 38);//rule 38
	setOneRule(&rollFuzzyControl.fuzzy_rules[38], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[1], 39);//rule 39
	setOneRule(&rollFuzzyControl.fuzzy_rules[39], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[1], 40);//rule 40
	setOneRule(&rollFuzzyControl.fuzzy_rules[40], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[1], 41);//rule 41
	setOneRule(&rollFuzzyControl.fuzzy_rules[41], &rollFuzzyControl.inGocLech[5], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[1], 42);//rule 42

	setOneRule(&rollFuzzyControl.fuzzy_rules[42], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[0], &rollFuzzyControl.outValuePWMControl[0], 43);//rule 43
	setOneRule(&rollFuzzyControl.fuzzy_rules[43], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[1], &rollFuzzyControl.outValuePWMControl[0], 44);//rule 44
	setOneRule(&rollFuzzyControl.fuzzy_rules[44], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[2], &rollFuzzyControl.outValuePWMControl[0], 45);//rule 45
	setOneRule(&rollFuzzyControl.fuzzy_rules[45], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[3], &rollFuzzyControl.outValuePWMControl[0], 46);//rule 46
	setOneRule(&rollFuzzyControl.fuzzy_rules[46], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[4], &rollFuzzyControl.outValuePWMControl[0], 47);//rule 47
	setOneRule(&rollFuzzyControl.fuzzy_rules[47], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[5], &rollFuzzyControl.outValuePWMControl[0], 48);//rule 48
	setOneRule(&rollFuzzyControl.fuzzy_rules[48], &rollFuzzyControl.inGocLech[6], &rollFuzzyControl.inGocLech_dot[6], &rollFuzzyControl.outValuePWMControl[0], 49);//rule 49
	
	
	
	//PITCH
	setOneRule(&pitchFuzzyControl.fuzzy_rules[0],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[6], 1);//rule 1
	setOneRule(&pitchFuzzyControl.fuzzy_rules[1],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[6], 2);//rule 2
	setOneRule(&pitchFuzzyControl.fuzzy_rules[2],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[6], 3);//rule 3
	setOneRule(&pitchFuzzyControl.fuzzy_rules[3],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[6], 4);//rule 4
	setOneRule(&pitchFuzzyControl.fuzzy_rules[4],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[6], 5);//rule 5
	setOneRule(&pitchFuzzyControl.fuzzy_rules[5],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[6], 6);//rule 6
	setOneRule(&pitchFuzzyControl.fuzzy_rules[6],  &pitchFuzzyControl.inGocLech[0], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[6], 7);//rule 7
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[7],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[5], 8);//rule 8
	setOneRule(&pitchFuzzyControl.fuzzy_rules[8],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[5], 9);//rule 9
	setOneRule(&pitchFuzzyControl.fuzzy_rules[9],  &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[5], 10);//rule 10
	setOneRule(&pitchFuzzyControl.fuzzy_rules[10], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[5], 11);//rule 11
	setOneRule(&pitchFuzzyControl.fuzzy_rules[11], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[5], 12);//rule 12
	setOneRule(&pitchFuzzyControl.fuzzy_rules[12], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[5], 13);//rule 13
	setOneRule(&pitchFuzzyControl.fuzzy_rules[13], &pitchFuzzyControl.inGocLech[1], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[5], 14);//rule 14
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[14], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[4], 15);//rule 15
	setOneRule(&pitchFuzzyControl.fuzzy_rules[15], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[4], 16);//rule 16
	setOneRule(&pitchFuzzyControl.fuzzy_rules[16], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[4], 17);//rule 17
	setOneRule(&pitchFuzzyControl.fuzzy_rules[17], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[4], 18);//rule 18
	setOneRule(&pitchFuzzyControl.fuzzy_rules[18], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[4], 19);//rule 19
	setOneRule(&pitchFuzzyControl.fuzzy_rules[19], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[4], 20);//rule 20
	setOneRule(&pitchFuzzyControl.fuzzy_rules[20], &pitchFuzzyControl.inGocLech[2], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[4], 21);//rule 21
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[21], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[3], 22);//rule 22
	setOneRule(&pitchFuzzyControl.fuzzy_rules[22], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[3], 23);//rule 23
	setOneRule(&pitchFuzzyControl.fuzzy_rules[23], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[3], 24);//rule 24
	setOneRule(&pitchFuzzyControl.fuzzy_rules[24], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[3], 25);//rule 25
	setOneRule(&pitchFuzzyControl.fuzzy_rules[25], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[3], 26);//rule 26
	setOneRule(&pitchFuzzyControl.fuzzy_rules[26], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[3], 27);//rule 27
	setOneRule(&pitchFuzzyControl.fuzzy_rules[27], &pitchFuzzyControl.inGocLech[3], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[3], 28);//rule 28
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[28], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[2], 29);//rule 29
	setOneRule(&pitchFuzzyControl.fuzzy_rules[29], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[2], 30);//rule 30
	setOneRule(&pitchFuzzyControl.fuzzy_rules[30], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[2], 31);//rule 31
	setOneRule(&pitchFuzzyControl.fuzzy_rules[31], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[2], 32);//rule 32
	setOneRule(&pitchFuzzyControl.fuzzy_rules[32], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[2], 33);//rule 33
	setOneRule(&pitchFuzzyControl.fuzzy_rules[33], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[2], 34);//rule 34
	setOneRule(&pitchFuzzyControl.fuzzy_rules[34], &pitchFuzzyControl.inGocLech[4], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[2], 35);//rule 35
	
	setOneRule(&pitchFuzzyControl.fuzzy_rules[35], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[1], 36);//rule 36
	setOneRule(&pitchFuzzyControl.fuzzy_rules[36], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[1], 37);//rule 37
	setOneRule(&pitchFuzzyControl.fuzzy_rules[37], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[1], 38);//rule 38
	setOneRule(&pitchFuzzyControl.fuzzy_rules[38], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[1], 39);//rule 39
	setOneRule(&pitchFuzzyControl.fuzzy_rules[39], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[1], 40);//rule 40
	setOneRule(&pitchFuzzyControl.fuzzy_rules[40], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[1], 41);//rule 41
	setOneRule(&pitchFuzzyControl.fuzzy_rules[41], &pitchFuzzyControl.inGocLech[5], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[1], 42);//rule 42

	setOneRule(&pitchFuzzyControl.fuzzy_rules[42], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[0], &pitchFuzzyControl.outValuePWMControl[0], 43);//rule 43
	setOneRule(&pitchFuzzyControl.fuzzy_rules[43], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[1], &pitchFuzzyControl.outValuePWMControl[0], 44);//rule 44
	setOneRule(&pitchFuzzyControl.fuzzy_rules[44], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[2], &pitchFuzzyControl.outValuePWMControl[0], 45);//rule 45
	setOneRule(&pitchFuzzyControl.fuzzy_rules[45], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[3], &pitchFuzzyControl.outValuePWMControl[0], 46);//rule 46
	setOneRule(&pitchFuzzyControl.fuzzy_rules[46], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[4], &pitchFuzzyControl.outValuePWMControl[0], 47);//rule 47
	setOneRule(&pitchFuzzyControl.fuzzy_rules[47], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[5], &pitchFuzzyControl.outValuePWMControl[0], 48);//rule 48
	setOneRule(&pitchFuzzyControl.fuzzy_rules[48], &pitchFuzzyControl.inGocLech[6], &pitchFuzzyControl.inGocLech_dot[6], &pitchFuzzyControl.outValuePWMControl[0], 49);//rule 49
	
	
	//YAW
	/*
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
	*/
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
		calcule_H_and_Y_PerRule(fuzzyController->pre_GocLech, &fuzzyController->fuzzy_rules[i]);
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
