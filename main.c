/*
Clock of timer
http://www.farrellf.com/projects/hardware/2012-08-11_STM32F4_Basics:_Timers_(Part_1)/
*/
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
#include "main.h"
#include "common.h"
#include "nguyen_mpu6050.h"
#include "kalman_filter.h"

//I2C handle, dung de doc value cua cam bien MPU6050
I2C_HandleTypeDef I2C_Handle_10truc;

//timer 3 dung de output PWM ra 4 channel
TIM_HandleTypeDef Tim3_Handle_PWM;		

//timer 1 dung de capture PWM cua RF module
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5; //PA3 - TIM5_CH4

//Throttle (can ga) tang giam toc do quay
int16_t IC_Throttle1, IC_Throttle2;
int16_t IC_Throttle_flag_capture_number;
int16_t IC_Throttle_pusle_width;
int16_t IC_Throttle_cycle_time;
int16_t IC_Throttle_frequency;
int32_t IC_counter_register_timer;
int32_t IC_interrupt_numbers_timer;

//Rudder (xoay theo truc z) - goc Yaw
int16_t IC_Rudder_Xoay1, IC_Rudder_Xoay2;
int16_t IC_Rudder_Xoay_flag_capture_number;
int16_t IC_Rudder_Xoay_pusle_width;
int16_t IC_Rudder_Xoay_cycle_time;
int16_t IC_Rudder_Xoay_frequency;


//Elevator (tien - lui) - goc Pitch
int16_t IC_Elevator_TienLui1, IC_Elevator_TienLui2;
int16_t IC_Elevator_TienLui_flag_capture_number;
int16_t IC_Elevator_TienLui_pusle_width;
int16_t IC_Elevator_TienLui_cycle_time;
int16_t IC_Elevator_TienLui_frequency;

//Aileron_TraiPhai (trai - phai) - goc Roll
int16_t IC_Aileron_TraiPhai1, IC_Aileron_TraiPhai2;
int16_t IC_Aileron_TraiPhai_flag_capture_number;
int16_t IC_Aileron_TraiPhai_pusle_width;
int16_t IC_Aileron_TraiPhai_cycle_time;
int16_t IC_Aileron_TraiPhai_frequency;


//PWM 4 motor
int16_t pwm_motor_1; //  Motor1           Motor2
int16_t pwm_motor_2; //            ^^  Head(dau quadrotor)
int16_t pwm_motor_3; //            ||
int16_t pwm_motor_4; //            ||
										 //  Motor4           Motor3
int16_t FlyState; // 0: May bay ngung hoat dong, 1:may bay dang bay

//cam bien MPU6050
uint8_t who_i_am_reg_value_MPU6050;
float rotation_x, rotation_y, rotation_z;


//------------------------------
															//...code default of ARM
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

//------------------------------------------------------------------
//ham handle error						
static void SystemClock_Config(void);
static void Error_Handler(void); 
void Error_Handler1(void); 
void Error_Handler2(void);
void Error_Handler3(void); 
void Error_Handler4(void);
															
//ham Led Sang
void SANG_1_LED(int8_t pin);
void SANG_2_LED(int8_t type);
void SANG_4_LED(void);
void SANG_4_LED_FOREVER(void);
void SANG_4_LED_OFF(void);
void KiemTraCodeOK(void);
															
//Khoi Tao LED, BUTTON USER
void Init_LEDSANG_PORTD_12_13_14_15(void);
void Init_BUTTON_USER_PORT_A_0(void);
															
//Khoi tao TIMER3 output PWM											
void Init_PWM_GPIO_PORT_C_4Channel(void);
void Init_PWM_TIM3_Handle(void);
void Init_TIM3_OUTPUT_COMPARE(void);
															
//timer 1 & 2 inputcapture for RF module
void Init_Receiver_TIM_PWM_Capture_TIM1(void);
void Init_Receiver_TIM_PWM_Capture_TIM2(void);
void Init_Receiver_TIM_PWM_Capture_TIM4(void);
void Init_Receiver_TIM_PWM_Capture_TIM5(void); //PA3 - TIM5_CH4


//Dieu chinh huong bay qua receiver
void SetInitDataQuadrotor(void);
void SetPWM_4_Motor(int16_t value);
void SetPWM_1_Motor(int16_t numberMotor, int16_t newValue);
void SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue);
void SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue);
void DieuChinhHuongBay_Qua_Receiver(void);


//i2c chip mpu6050 10truc
void Init_I2C_GPIO_PortB(void);
void Init_I2C_Handle_10truc(void);
void TM_I2C_IS_DEVICE_CONNECTED(void);
uint8_t TM_I2C_WHO_I_AM( uint8_t device_address, uint8_t register_address);
void TM_MPU6050_SetDataRate(uint8_t device_address, uint8_t register_address, uint8_t rate);
void TM_MPU6050_SetAccelerometer(uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value);
void TM_MPU6050_SetGyroscope(uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value); 
void TM_I2C_WRITE( uint8_t device_address, uint8_t register_address, uint8_t data);
void TM_I2C_READ_MULTI( uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void TM_I2C_READ(  uint8_t device_address, uint8_t register_address, uint8_t* data);
void TM_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output );
void Calculate_Accel_X_Angles(TM_MPU6050_t* output, float* angel_x);
void Calculate_Accel_Y_Angles(TM_MPU6050_t* output, float* angel_y);
void Calculate_Accel_Z_Angles(TM_MPU6050_t* output, float* angel_z);
void Sang_Led_By_MPU6050_Values(float angel_x, float angel_y, float angel_z );



//ham delay default
volatile uint32_t g_iSysTicks = 0;
void SysTick_Handler(){	g_iSysTicks++;}
void delay_ms(uint32_t piMillis){	uint32_t iStartTime = g_iSysTicks;	while( (g_iSysTicks - iStartTime ) < piMillis)	{}}			

	
int main(void)
{	
	
		TM_MPU6050_t output;			
		SetInitDataQuadrotor();
	
																			/* code default cua ARM co san						*/
																			#ifdef RTE_CMSIS_RTOS                   
																				osKernelInitialize();                 
																			#endif		
																			HAL_Init();
																			SystemClock_Config();	
		//cap xung clock
		__GPIOA_CLK_ENABLE();			__GPIOB_CLK_ENABLE();			__GPIOC_CLK_ENABLE();	
		__GPIOD_CLK_ENABLE();			__GPIOE_CLK_ENABLE();		
		__TIM1_CLK_ENABLE(); 		__TIM2_CLK_ENABLE(); 		__TIM3_CLK_ENABLE(); 		
		__TIM4_CLK_ENABLE(); 		__TIM5_CLK_ENABLE();    __TIM9_CLK_ENABLE(); 
	 
		__I2C1_CLK_ENABLE();						
		
		//----------------------------------------------------
		Init_LEDSANG_PORTD_12_13_14_15(); //GPIO init cho 4 led sang
		Init_BUTTON_USER_PORT_A_0();			//gpio init cho button user

		//--------------------------------------------------
		Init_PWM_GPIO_PORT_C_4Channel();					//setting cho 4 PIN of PWM		
		Init_PWM_TIM3_Handle();		//Khoi tao timer 3
		Init_TIM3_OUTPUT_COMPARE();//cau hinh timer 3 voi mode output PWM
		
		
		//Output compare PWM. Khoi dong PWM motor
		HAL_TIM_Base_Start_IT(&Tim3_Handle_PWM); 
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_1);		
		HAL_TIMEx_PWMN_Start(&Tim3_Handle_PWM,TIM_CHANNEL_1);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_2);		
		HAL_TIMEx_PWMN_Start(&Tim3_Handle_PWM,TIM_CHANNEL_2);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_3);		
		HAL_TIMEx_PWMN_Start(&Tim3_Handle_PWM,TIM_CHANNEL_3);	
		HAL_TIM_PWM_Start(&Tim3_Handle_PWM, TIM_CHANNEL_4);		
		HAL_TIMEx_PWMN_Start(&Tim3_Handle_PWM,TIM_CHANNEL_4);	
			
		
		SANG_4_LED();	
		delay_ms(2000); 
		__HAL_TIM_SetCompare(&Tim3_Handle_PWM, TIM_CHANNEL_1, 700);
		TIM3->CCR1 = 700;
		
		SANG_4_LED_OFF();	
		delay_ms(500); 
		__HAL_TIM_SetCompare(&Tim3_Handle_PWM, TIM_CHANNEL_1, 800);
		//TIM3->CCR1 = 800;	
		
		
		
		//MPU6050---------------------------------------------------			
		Init_I2C_GPIO_PortB();											//cau hinh PB6, PB7 doc cam bien mpu6050
		
		Init_I2C_Handle_10truc();	//khoi tao I2C handle
		//HAL_I2C_Init(&I2C_Handle_10truc);
		//__HAL_I2C_ENABLE(&I2C_Handle_10truc);				//start I2C handle	
		TM_I2C_IS_DEVICE_CONNECTED();	
				
		//doc gia tri cua WHO I AM register, neu co loi se Sang 4 LED mãi
		who_i_am_reg_value_MPU6050 = TM_I2C_WHO_I_AM( MPU6050_I2C_ADDR, MPU6050_WHO_AM_I_REGISTER);
		if( who_i_am_reg_value_MPU6050 != MPU6050_I_AM_VALUES )
		{
			SANG_4_LED_FOREVER();
		}
		//setting/config cho cam bien mpu6050
		TM_I2C_WRITE( MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x00); 
		TM_MPU6050_SetDataRate( MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, TM_MPU6050_DataRate_1KHz); 
		TM_MPU6050_SetAccelerometer( MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, TM_MPU6050_Accelerometer_2G); 
		TM_MPU6050_SetGyroscope( MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, TM_MPU6050_Gyroscope_250s); 
		TM_MPU6050_ReadAll( MPU6050_I2C_ADDR, &output);
		
		
		//---------------------------------------------------
		Init_Receiver_TIM_PWM_Capture_TIM1();			
		Init_Receiver_TIM_PWM_Capture_TIM2();
		Init_Receiver_TIM_PWM_Capture_TIM4();
		Init_Receiver_TIM_PWM_Capture_TIM5();//PA3 - TIM5_CH4
		//Init_Receiver_TIM_PWM_Capture_TIM9(); //PE5 - TIM9_CH1
		
		
			
		
			
																//.... code dafault cua ARM		// when using CMSIS RTOS	// start thread execution 
																#ifdef RTE_CMSIS_RTOS 
																	osKernelStart();     
																#endif			
		
		KiemTraCodeOK();		
		while(1)
		{				
			//MPU6050-------------
			TM_MPU6050_ReadAll( MPU6050_I2C_ADDR, &output);
			Calculate_Accel_X_Angles(&output, &rotation_x);
			Calculate_Accel_Y_Angles(&output, &rotation_y);
			Calculate_Accel_Z_Angles(&output, &rotation_z);
			Sang_Led_By_MPU6050_Values(rotation_x, rotation_y, rotation_z);
			//END MPU6050----------
			
			//
			
			if(FlyState == 0)
			{
				//Khoi dong may bay	
				if( (IC_Throttle_pusle_width >= PWM_START_MIN && IC_Throttle_pusle_width <= PWM_START_MAX) && (IC_Aileron_TraiPhai_pusle_width >= PWM_START_MIN && IC_Aileron_TraiPhai_pusle_width <=PWM_START_MAX) && (IC_Elevator_TienLui_pusle_width >= PWM_START_MIN && IC_Elevator_TienLui_pusle_width <= PWM_START_MAX) )
				{
					SANG_4_LED();
						delay_ms(5000);
						if( (IC_Throttle_pusle_width >= PWM_START_MIN && IC_Throttle_pusle_width <= PWM_START_MAX) && (IC_Aileron_TraiPhai_pusle_width >= PWM_START_MIN && IC_Aileron_TraiPhai_pusle_width <=PWM_START_MAX) && (IC_Elevator_TienLui_pusle_width >= PWM_START_MIN && IC_Elevator_TienLui_pusle_width <= PWM_START_MAX) )
						{
							SANG_4_LED_OFF();
							FlyState = 1;
							SetPWM_4_Motor(800);
						}
				}
			}
			else if(FlyState == 1)
			{
				//khi ga nho nhat, keo can gat 5s thi tat may bay
				//Khoi dong may bay	
				if( (IC_Throttle_pusle_width >= PWM_START_MIN && IC_Throttle_pusle_width <= PWM_START_MAX) && (IC_Aileron_TraiPhai_pusle_width >= PWM_START_MIN && IC_Aileron_TraiPhai_pusle_width <=PWM_START_MAX) && (IC_Elevator_TienLui_pusle_width >= PWM_START_MIN && IC_Elevator_TienLui_pusle_width <= PWM_START_MAX) )
				{
						SANG_4_LED();
						delay_ms(5000);
						if( (IC_Throttle_pusle_width >= PWM_START_MIN && IC_Throttle_pusle_width <= PWM_START_MAX) && (IC_Aileron_TraiPhai_pusle_width >= PWM_START_MIN && IC_Aileron_TraiPhai_pusle_width <=PWM_START_MAX) && (IC_Elevator_TienLui_pusle_width >= PWM_START_MIN && IC_Elevator_TienLui_pusle_width <= PWM_START_MAX) )
						{
							SANG_4_LED_OFF();
							FlyState = 0;
							SetPWM_4_Motor(0);
							KiemTraCodeOK();
						}
				}
				DieuChinhHuongBay_Qua_Receiver();
			}
			
			
			//------------PWM- Motor-------------------------------------------------
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_SET)
				{	//khi nhan buttun USER ma chua tha ra -> khong lam gi
				}				
			}
			//TIM3->CCR1 = pwm_speed;	//__HAL_TIM_SetCompare(&Tim3_Handle_PWM,TIM_CHANNEL_1, pwm_speed);
			//END------------PWM--------------------------------------------------
		}		
		//End while(1)
}

void SetInitDataQuadrotor(void)
{
		IC_Throttle1 = 0;
		IC_Throttle2 = 0; 
		IC_Throttle_pusle_width = 0;
		IC_Throttle_cycle_time = 0;
		IC_Throttle_frequency = 0;
		IC_Throttle_flag_capture_number=0;
	
		IC_Elevator_TienLui1 = 0;
		IC_Elevator_TienLui2 = 0; 
		IC_Elevator_TienLui_pusle_width = 0;
		IC_Elevator_TienLui_cycle_time = 0;
		IC_Elevator_TienLui_frequency = 0;
		IC_Elevator_TienLui_flag_capture_number=0;
	
		IC_Aileron_TraiPhai1 = 0;
		IC_Aileron_TraiPhai2 = 0;
		IC_Aileron_TraiPhai_pusle_width = 0;
		IC_Aileron_TraiPhai_cycle_time = 0;
		IC_Aileron_TraiPhai_frequency = 0;
		IC_Aileron_TraiPhai_flag_capture_number = 0;
	
		IC_Rudder_Xoay1 = 0;
		IC_Rudder_Xoay2 = 0;
		IC_Rudder_Xoay_pusle_width = 0;
		IC_Rudder_Xoay_cycle_time = 0;
		IC_Rudder_Xoay_frequency = 0;
		IC_Rudder_Xoay_flag_capture_number = 0;
		
		IC_counter_register_timer = 0;
		IC_interrupt_numbers_timer = 0;
		who_i_am_reg_value_MPU6050 = 0;
		SetPWM_4_Motor(0);
		FlyState = 0;
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
	if(HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1) != HAL_OK)		{							Error_Handler();			}	
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


void TIM1_CC_IRQHandler(void)
{
					//khi co interrup la TIM_IT_CC1, co su kien capture canh	
						if(__HAL_TIM_GET_ITSTATUS(&htim1, TIM_IT_CC1) == SET)
						{		
									__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
									__HAL_TIM_CLEAR_FLAG(&htim1, TIM_IT_CC1);		
									if(IC_Throttle_flag_capture_number==0)
									{					
												IC_Throttle1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
												IC_Throttle_flag_capture_number = 1;
													
									}else if(IC_Throttle_flag_capture_number==1)
									{
												IC_Throttle2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
												IC_Throttle_pusle_width		= (IC_Throttle2 - IC_Throttle1);
												IC_Throttle_flag_capture_number = 2;	
														
									}
									else if(IC_Throttle_flag_capture_number==2)
									{
												IC_Throttle_cycle_time = (HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1) - IC_Throttle1);
												IC_Throttle_flag_capture_number = 3;
									}else if(IC_Throttle_flag_capture_number==3)
									{
												IC_Throttle_flag_capture_number=0;
												TIM1->CNT = 0;
									}						
						}
}

void TIM2_IRQHandler(void)
{
	//PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw
		if(__HAL_TIM_GET_ITSTATUS(&htim2, TIM_IT_CC1) == SET)
			{		
						__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
						__HAL_TIM_CLEAR_FLAG(&htim2, TIM_IT_CC1);		
						if(IC_Rudder_Xoay_flag_capture_number==0)
						{					
									IC_Rudder_Xoay1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
									IC_Rudder_Xoay_flag_capture_number = 1;
										
						}else if(IC_Rudder_Xoay_flag_capture_number==1)
						{
									IC_Rudder_Xoay2 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
									IC_Rudder_Xoay_pusle_width		= (IC_Rudder_Xoay2 - IC_Rudder_Xoay1);
									IC_Rudder_Xoay_flag_capture_number = 2;	
											
						}
						else if(IC_Rudder_Xoay_flag_capture_number==2)
						{
									IC_Rudder_Xoay_cycle_time = (HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1) - IC_Rudder_Xoay1);
									IC_Rudder_Xoay_flag_capture_number = 3;
						}else if(IC_Rudder_Xoay_flag_capture_number==3)
						{
									IC_Rudder_Xoay_flag_capture_number=0;
									TIM2->CNT = 0;
						}						
			}
}
void TIM4_IRQHandler(void)
{ 
	//PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch
			if(__HAL_TIM_GET_ITSTATUS(&htim4, TIM_IT_CC3) == SET)
			{		
						__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC3);
						__HAL_TIM_CLEAR_FLAG(&htim4, TIM_IT_CC3);		
						if(IC_Elevator_TienLui_flag_capture_number ==0)
						{					
									IC_Elevator_TienLui1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
									IC_Elevator_TienLui_flag_capture_number = 1;
										
						}else if(IC_Elevator_TienLui_flag_capture_number==1)
						{
									IC_Elevator_TienLui2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
									IC_Elevator_TienLui_pusle_width = (IC_Elevator_TienLui2 - IC_Elevator_TienLui1);
									IC_Elevator_TienLui_flag_capture_number = 2;	
											
						}
						else if(IC_Elevator_TienLui_flag_capture_number==2)
						{
									IC_Elevator_TienLui_cycle_time = (HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3) - IC_Elevator_TienLui1);
									IC_Elevator_TienLui_flag_capture_number = 3;
						}else if(IC_Elevator_TienLui_flag_capture_number==3)
						{
									IC_Elevator_TienLui_flag_capture_number=0;
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
						if(IC_Aileron_TraiPhai_flag_capture_number ==0)
						{					
									IC_Aileron_TraiPhai1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
									IC_Aileron_TraiPhai_flag_capture_number = 1;
										
						}else if(IC_Aileron_TraiPhai_flag_capture_number==1)
						{
									IC_Aileron_TraiPhai2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4);
									IC_Aileron_TraiPhai_pusle_width		= IC_Aileron_TraiPhai2 - IC_Aileron_TraiPhai1;
									IC_Aileron_TraiPhai_flag_capture_number = 2;	
											
						}
						else if(IC_Aileron_TraiPhai_flag_capture_number==2)
						{
									IC_Aileron_TraiPhai_cycle_time = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_4) - IC_Aileron_TraiPhai1;
									IC_Aileron_TraiPhai_flag_capture_number = 3;
						}else if(IC_Aileron_TraiPhai_flag_capture_number==3)
						{
									IC_Aileron_TraiPhai_flag_capture_number=0;
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
void KiemTraCodeOK(void)
{
		int i = 0;
		while(i < 5)
		{
			SANG_2_LED(1);				
			delay_ms(100);				
			SANG_2_LED(2);				
			delay_ms(100);
			i++;
		}
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
void Init_PWM_GPIO_PORT_C_4Channel(void)
{
	//Init PWM GPIOC cho 4 channel
		GPIO_InitTypeDef GPIO_PWM_PORTC_6789;			
		GPIO_PWM_PORTC_6789.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_PWM_PORTC_6789.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORTC_6789.Pull = GPIO_NOPULL;
		GPIO_PWM_PORTC_6789.Speed = GPIO_SPEED_FAST;
		GPIO_PWM_PORTC_6789.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOC, &GPIO_PWM_PORTC_6789);
}

void Init_PWM_TIM3_Handle()
{
		Tim3_Handle_PWM.Instance = TIM3;
		Tim3_Handle_PWM.Init.Prescaler = 84-1; 									//vi timer 3 co max clock la 84MHz, -1 la vi dem(count) tu 0
		Tim3_Handle_PWM.Init.CounterMode = TIM_COUNTERMODE_UP; 	//dem len
		Tim3_Handle_PWM.Init.Period = 20000-1;									//Period(chu ki) = 20 mili s
		Tim3_Handle_PWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// =0
		HAL_TIM_PWM_Init(&Tim3_Handle_PWM);										//Init PWM voi TIM HANDLE
}

void Init_TIM3_OUTPUT_COMPARE()
{
		TIM_OC_InitTypeDef  TIM_Output_compare;
		TIM_Output_compare.OCMode = TIM_OCMODE_PWM1;
		TIM_Output_compare.OCIdleState = TIM_OCIDLESTATE_SET;
		TIM_Output_compare.Pulse = 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode = TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_1); //config PWM cho channel 1 (PORTC.6)
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&Tim3_Handle_PWM, &TIM_Output_compare, TIM_CHANNEL_4);
		HAL_TIM_PWM_Init(&Tim3_Handle_PWM);
		HAL_TIM_PWM_MspInit(&Tim3_Handle_PWM);
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
		while(1)
		{
			Error_Handler();
		}
	}
	while( HAL_I2C_GetState(&I2C_Handle_10truc) != HAL_I2C_STATE_READY )
	{
		while(1)
		{
			Error_Handler();
		}
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
		
	output->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	output->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
}

void TM_MPU6050_ReadGyroscope( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	TM_I2C_READ_MULTI( device_address, MPU6050_GYRO_XOUT_H, data, 6);
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}
		
	output->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	output->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);
}	

void TM_MPU6050_ReadAll( uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[14];
	int16_t temp;	
	
	TM_I2C_READ_MULTI( device_address, MPU6050_ACCEL_XOUT_H, data, 14); /* Read full raw data, 14bytes */
	while(HAL_I2C_GetState(&I2C_Handle_10truc)!=HAL_I2C_STATE_READY){}	
	
	output->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	/* Format accelerometer data */
	output->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
	temp = (data[6] << 8 | data[7]); /* Format temperature */
	output->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	output->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]); /* Format gyroscope data */
	output->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	output->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
}



void Sang_Led_By_MPU6050_Values(float angel_x, float angel_y, float angel_z )
{
	SANG_4_LED_OFF();
	if(angel_x > 20)
	{
		SANG_1_LED(LED_YELLOW); 
	}else if(angel_x < -20)
	{
		SANG_1_LED(LED_RED); 	
	}
	
	if(angel_y > 20)
	{
		SANG_1_LED(LED_BLUE);  
	}else if(angel_y < -20)
	{
		SANG_1_LED(LED_ORANGE);  
	}
	delay_ms(50);
}

void Calculate_Accel_X_Angles(TM_MPU6050_t* output, float* angel_x)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);	
		float ACCEL_XOUT = (float)(output->Accelerometer_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Accelerometer_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Accelerometer_Z/MPU6050_ACCE_SENS_2);
	
		_sqrt = (float)sqrt( ACCEL_ZOUT*ACCEL_ZOUT + ACCEL_XOUT*ACCEL_XOUT );	
		*angel_x = (float)(mot_180_do_chia_pi * (float)atan(ACCEL_YOUT/_sqrt));
}

void Calculate_Accel_Y_Angles(TM_MPU6050_t* output, float* angel_y)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);	
		float ACCEL_XOUT = (float)(output->Accelerometer_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Accelerometer_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Accelerometer_Z/MPU6050_ACCE_SENS_2);
		_sqrt = (float)sqrt( ACCEL_ZOUT*ACCEL_ZOUT + ACCEL_YOUT*ACCEL_YOUT );
		*angel_y = (float)(mot_180_do_chia_pi * (float)-atan(ACCEL_XOUT /_sqrt));
}

	
void Calculate_Accel_Z_Angles(TM_MPU6050_t* output, float* angel_z)
{
		float _sqrt;
		float mot_180_do_chia_pi = (float)((float)180.0/PI);
		float ACCEL_XOUT = (float)(output->Accelerometer_X/MPU6050_ACCE_SENS_2);
		float ACCEL_YOUT = (float)(output->Accelerometer_Y/MPU6050_ACCE_SENS_2);
		float ACCEL_ZOUT = (float)(output->Accelerometer_Z/MPU6050_ACCE_SENS_2);
		_sqrt= (float)sqrt( ACCEL_YOUT*ACCEL_YOUT + ACCEL_XOUT*ACCEL_XOUT );
		*angel_z = (float)(mot_180_do_chia_pi * atan(_sqrt/ACCEL_ZOUT));
}
//end Accelerametor 10truc
//
//
//


//
//Lay gia tri cua Receiver de dieu khien 4 motor
//
//
void SetPWM_4_Motor(int16_t value)
{
	if(value==0)
	{
					pwm_motor_1 = 0;					
					pwm_motor_2 = 0;					
					pwm_motor_3 = 0;					
					pwm_motor_4 = 0;
		
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
	}
	TIM3->CCR1 = pwm_motor_1;
	TIM3->CCR2 = pwm_motor_2;
	TIM3->CCR3 = pwm_motor_3;
	TIM3->CCR4 = pwm_motor_4;
}

		
void SetPWM_1_Motor(int16_t numberMotor, int16_t newValue)
{
	switch(numberMotor) 
	{
		 case 1  :
				if(newValue <= PWM_Throtte_Min)							{	pwm_motor_1 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{ pwm_motor_1 = PWM_Throtte_Max;}
				else 																				{pwm_motor_1 = newValue;					}
				TIM3->CCR1 = pwm_motor_1;
				break; 
		
		 case 2  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_2 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_2 = PWM_Throtte_Max;}
				else 																				{pwm_motor_2 = newValue;	}
				TIM3->CCR2 = pwm_motor_2;
				break; 
		 
		 case 3  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_3 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_3 = PWM_Throtte_Max;}
				else 																				{pwm_motor_3 = newValue;	}
				TIM3->CCR3 = pwm_motor_3;
				break; 
		
		 case 4  :
				if(newValue <= PWM_Throtte_Min)							{pwm_motor_4 = PWM_Throtte_Min;}
				else if (newValue >= PWM_Throtte_Max)				{pwm_motor_4 = PWM_Throtte_Max;}
				else 																				{pwm_motor_4 = newValue;	}
				TIM3->CCR4 = pwm_motor_4;
				break; 
	}
}



void SetPWM_Motor_Tang(int16_t numberMotor, int16_t changeValue)
{
	if(changeValue > 0)
	{
			switch(numberMotor) 
			{
				 case 1  :
						SetPWM_1_Motor(1, (pwm_motor_1 + changeValue) );
						break; 
				
				 case 2  :
						SetPWM_1_Motor(2, (pwm_motor_2 + changeValue) );
						break; 
				 
				 case 3  :
						SetPWM_1_Motor(3, (pwm_motor_3 + changeValue) );
						break; 
				
				 case 4  :
						SetPWM_1_Motor(4, (pwm_motor_4 + changeValue) );
						break; 
			}
	}
}

void SetPWM_Motor_Giam(int16_t numberMotor, int16_t changeValue)
{
	switch(numberMotor) 
	{
		 case 1  :
				SetPWM_1_Motor(1, (pwm_motor_1 - changeValue) );
				break; 
		
		 case 2  :
				SetPWM_1_Motor(2, (pwm_motor_2 - changeValue) );
				break; 
		 
		 case 3  :
				SetPWM_1_Motor(3, (pwm_motor_3 - changeValue) );
				break; 
		
		 case 4  :
				SetPWM_1_Motor(4, (pwm_motor_4 - changeValue) );
				break; 
	}
}

void DieuChinhHuongBay_Qua_Receiver(void)
{
	int16_t chenhLechGiaTri = 0;
	//Ga, tang-giam Ga
	if(IC_Throttle_pusle_width >= 1000 && IC_Throttle_pusle_width <= 2000)
	{
		SetPWM_4_Motor(IC_Throttle_pusle_width);
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
		}
			
		while(IC_Elevator_TienLui_pusle_width >= PWM_Effect_Max )
		{
					//Lui ve phia sau, giam dong co 3+4 va tang dong co 1+2
					chenhLechGiaTri = IC_Elevator_TienLui_pusle_width - PWM_Avg;
					SetPWM_Motor_Giam(3,chenhLechGiaTri);
					SetPWM_Motor_Giam(4,chenhLechGiaTri);
					SetPWM_Motor_Tang(1,chenhLechGiaTri);
					SetPWM_Motor_Tang(2,chenhLechGiaTri);			
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
		}

		while(IC_Aileron_TraiPhai_pusle_width >= PWM_Effect_Max )
		{
					//qua Phai, giam dong co 2+3 va tang dong co 1+4
					chenhLechGiaTri = IC_Aileron_TraiPhai_pusle_width - PWM_Avg;
					SetPWM_Motor_Giam(2,chenhLechGiaTri);
					SetPWM_Motor_Giam(3,chenhLechGiaTri);
					SetPWM_Motor_Tang(1,chenhLechGiaTri);
					SetPWM_Motor_Tang(4,chenhLechGiaTri);			
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
			
		}
		
		while(IC_Rudder_Xoay_pusle_width >= PWM_Effect_Max )
		{
			//Xoay nguoc chieu kim dong ho, giam dong co 2+4 va tang dong co 1+3
			chenhLechGiaTri = IC_Rudder_Xoay_pusle_width - PWM_Avg;
			SetPWM_Motor_Giam(2,chenhLechGiaTri);
			SetPWM_Motor_Giam(4,chenhLechGiaTri);
			SetPWM_Motor_Tang(1,chenhLechGiaTri);
			SetPWM_Motor_Tang(3,chenhLechGiaTri);			
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

void Error_Handler1(void)
{
  while(1)
  {			
		SANG_2_LED(1);
		delay_ms(500);
		SANG_2_LED(2);
		delay_ms(500);
	}
}
void Error_Handler2(void)
{
  while(1)
  {			
		SANG_2_LED(1);
		delay_ms(500);
		SANG_4_LED_OFF();
		delay_ms(500);
	}
}
void Error_Handler3(void)
{
  while(1)
  {			
		SANG_2_LED(2);
		delay_ms(500);
		SANG_4_LED_OFF();
		delay_ms(500);
	}
}
void Error_Handler4(void)
{
  while(1)
  {			
		SANG_1_LED(12); delay_ms(500);
		SANG_1_LED(13); delay_ms(500);
		SANG_1_LED(14); delay_ms(500);
		SANG_1_LED(15); delay_ms(500);
	}
}

void SANG_1_LED(int8_t PIN)
{
		LED_D_12_LOW;
		LED_D_13_LOW;
		LED_D_14_LOW;
		LED_D_15_LOW;
		if(PIN==12)
		{
			LED_D_12_HIGH;
		}
		else if(PIN==13)
		{
			LED_D_13_HIGH;
		}
		else if(PIN==14)
		{
			LED_D_14_HIGH;
		}
		else if(PIN==15)
		{
			LED_D_15_HIGH;
		}
}

void SANG_2_LED(int8_t type)
{
	LED_D_12_LOW;
	LED_D_13_LOW;
	LED_D_14_LOW;
	LED_D_15_LOW;
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
		LED_D_12_HIGH;
		LED_D_13_HIGH;
		LED_D_14_HIGH;
		LED_D_15_HIGH;
}
void SANG_4_LED_FOREVER()
{
	
		LED_D_12_HIGH;
		LED_D_13_HIGH;
		LED_D_14_HIGH;
		LED_D_15_HIGH;
		while(1){}
}
void SANG_4_LED_OFF()
{
		LED_D_12_LOW;
		LED_D_13_LOW;
		LED_D_14_LOW;
		LED_D_15_LOW;
}
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
