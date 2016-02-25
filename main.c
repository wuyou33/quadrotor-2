/*
PORT A: PA0 																	=>button User  
				PA9, PA10															=>USART1
PORT B: PB6, PB7 															=>I2C1 chip 10 truc mpu6050
PORT C: PC6, PC7, PC8, PC9 										=>timer3
PORT D: PD12, PD13, PD14, PD15  							=>LEDSang
PORT E: PE3, PE0 															=>SPI1
				PE9, PE11, PE13, PE14									=>timer 1
*/
#include "math.h"
#include "main.h"
#include "common.h"
#include "nguyen_mpu6050.h"

//khai bao bien
I2C_HandleTypeDef I2C_InitStruct_10truc;
TIM_HandleTypeDef handle_timer_3;					//khai bao Timer Handle
TIM_OC_InitTypeDef TIM_Output_compare;   //Khai bao Output Compare	
TIM_HandleTypeDef htimer1;
long input_capture;
int16_t pwm_speed, pwm_left, pwm_right, pwm_front, pwm_back;	
uint8_t who_i_am_reg_value_MPU6050;
float rotation_x;
float rotation_y;
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
															uint32_t HAL_GetTick(void) 
															{ 
																return os_time; 
															}
															#endif

// khai bao nguyen mau Ham ------------------------------------------------------------------
static void SystemClock_Config(void);
static void Error_Handler(void); static void Error_Handler1(void); static void Error_Handler2(void);
static void Error_Handler3(void); static void Error_Handler4(void);

void KiemTraCodeOK(void);
void Acc_Led_Sang(int16_t Acc_X, int16_t Acc_Y, int16_t Acc_Z);
void SANG_1_LED(int8_t pin);
void SANG_2_LED(int8_t type);
void SANG_4_LED(void);
void SANG_4_LED_OFF(void);

//Init function
void Init_LEDSANG_PORTD_12_13_14_15(void);
void Init_BUTTON_USER_PORT_A_0(void);
															
//timer 3															
void Init_PWM_GPIO_PORT_C_4Channel(void);
void Init_PWM_TIM3_Handle(TIM_HandleTypeDef handle_timer_3);
void Init_TIM3_OUTPUT_COMPARE(TIM_HandleTypeDef handle_timer_3, TIM_OC_InitTypeDef  TIM_Output_compare);
															
//timer 1
void Init_Receiver_TIM1_PWM_Capture_PortE(TIM_HandleTypeDef handle_timer_1);


//i2c chip mpu6050 10truc
void Init_I2C_GPIO_PortB(void);
void Init_I2C_Handle_10truc(void);
void TM_I2C_IS_DEVICE_CONNECTED( I2C_HandleTypeDef* Handle);
uint8_t TM_I2C_WHO_I_AM(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address);
void TM_MPU6050_SetDataRate(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t rate);
void TM_MPU6050_SetAccelerometer(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value);
void TM_MPU6050_SetGyroscope(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value); 
void TM_I2C_WRITE(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t data);
void TM_I2C_READ_MULTI(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count);
void TM_I2C_READ( I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t* data);
void TM_MPU6050_ReadAll(I2C_HandleTypeDef* Handle, uint8_t device_address, TM_MPU6050_t* output );
void Sang_Led_By_MPU6050_Values(float rotation_x, float rotation_y);


//function delay, viet lai
volatile uint32_t g_iSysTicks = 0;
void SysTick_Handler()
{
	g_iSysTicks++;
}
void delay_ms(uint32_t piMillis)
{
	uint32_t iStartTime = g_iSysTicks;
	while( (g_iSysTicks - iStartTime ) < piMillis)
	{}
}			


int main(void)
{	
		//---khoi tao gia tri
		TM_MPU6050_t output;
		pwm_speed=800;			
		who_i_am_reg_value_MPU6050 = 0;
	
																			/* code default cua ARM co san						*/
																			#ifdef RTE_CMSIS_RTOS                   
																				osKernelInitialize();                 
																			#endif		
																			HAL_Init();
																			SystemClock_Config();	
		
		//cap xung clock
		__GPIOA_CLK_ENABLE();	__GPIOB_CLK_ENABLE();	__GPIOC_CLK_ENABLE();	__GPIOD_CLK_ENABLE();	__GPIOE_CLK_ENABLE();
		__TIM1_CLK_ENABLE(); 	__TIM3_CLK_ENABLE();
		__I2C1_CLK_ENABLE();	
	
					
		//GPIO init cho 4 led sang
		Init_LEDSANG_PORTD_12_13_14_15();
		
		//gpio init cho button user
		Init_BUTTON_USER_PORT_A_0();
		
		//Init PWM motor
		Init_PWM_GPIO_PORT_C_4Channel();		
		
		//init timer 3
		Init_PWM_TIM3_Handle(handle_timer_3);
		
		//init timer 3 output pwm
		Init_TIM3_OUTPUT_COMPARE(handle_timer_3, TIM_Output_compare);	
		
		
		//Init Accelerometer 10 truc MPU6050		
		Init_I2C_GPIO_PortB();
		Init_I2C_Handle_10truc();	
		__HAL_I2C_ENABLE(&I2C_InitStruct_10truc);
		TM_I2C_IS_DEVICE_CONNECTED(&I2C_InitStruct_10truc);	
		
		//test register who_i_am value of mpu6050
		who_i_am_reg_value_MPU6050 = TM_I2C_WHO_I_AM(&I2C_InitStruct_10truc, MPU6050_I2C_ADDR, MPU6050_WHO_AM_I_REGISTER);
		if( who_i_am_reg_value_MPU6050 != MPU6050_I_AM_VALUES )
		{
			while(1){Error_Handler();}
		}
		TM_I2C_WRITE(&I2C_InitStruct_10truc, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, 0x00); //# Now wake the 6050 up as it starts in sleep mode
		TM_MPU6050_SetDataRate(&I2C_InitStruct_10truc, MPU6050_I2C_ADDR, MPU6050_SMPLRT_DIV, TM_MPU6050_DataRate_1KHz); 
		TM_MPU6050_SetAccelerometer(&I2C_InitStruct_10truc, MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, TM_MPU6050_Accelerometer_2G); 
		TM_MPU6050_SetGyroscope(&I2C_InitStruct_10truc, MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG, TM_MPU6050_Gyroscope_250s); 
		TM_MPU6050_ReadAll(&I2C_InitStruct_10truc,MPU6050_I2C_ADDR, &output);
		delay_ms(1);		
		
		Init_Receiver_TIM1_PWM_Capture_PortE(htimer1);
		
		//Khoi dong PWM motor
		HAL_TIM_PWM_Start(&handle_timer_3, TIM_CHANNEL_1);		//HAL_TIM_PWM_Start(&handle_timer_3, TIM_CHANNEL_2);
		HAL_TIM_Base_Start_IT(&handle_timer_3); 
		HAL_TIM_PWM_Start(&handle_timer_3,TIM_CHANNEL_1); 
		HAL_TIMEx_PWMN_Start(&handle_timer_3,TIM_CHANNEL_1);	
		
		SANG_4_LED();	delay_ms(2000); 
		__HAL_TIM_SetCompare(&handle_timer_3, TIM_CHANNEL_1, 700);
		TIM3->CCR1 = 700;
		
		SANG_4_LED_OFF();	delay_ms(500); 
		__HAL_TIM_SetCompare(&handle_timer_3, TIM_CHANNEL_1, 800);
		TIM3->CCR1 = 800;
		//END --------------		
		
																//.... code dafault cua ARM
																// when using CMSIS RTOS
																// start thread execution 
																#ifdef RTE_CMSIS_RTOS 
																	osKernelStart();     
																#endif			
		
		KiemTraCodeOK();
		while(1)
		{				
			TM_MPU6050_ReadAll(&I2C_InitStruct_10truc,MPU6050_I2C_ADDR, &output);
			delay_ms(1);		
			rotation_x = Calculate_Accel_X_Angles(output.Accelerometer_X, output.Accelerometer_Y,output.Accelerometer_Z);
			rotation_y = Calculate_Accel_Y_Angles(output.Accelerometer_X, output.Accelerometer_Y,output.Accelerometer_Z);
			
			Sang_Led_By_MPU6050_Values(rotation_x, rotation_y);
			
			//------------PWM- Motor-------------------------------------------------
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_SET)
			{
				while(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)== GPIO_PIN_SET)
				{	//khi nhan buttun USER ma chua tha ra -> khong lam gi
				}
				HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
				pwm_speed = pwm_speed + 50;
				
				if(pwm_speed >= 2000)
				{
					pwm_speed = 800;
				}
			}
			TIM3->CCR1 = pwm_speed;
			__HAL_TIM_SetCompare(&handle_timer_3,TIM_CHANNEL_1, pwm_speed);
			//END------------PWM--------------------------------------------------			
		}
}

void KiemTraCodeOK(void)
{
		int i = 0;
		while(i < 5)
		{
			SANG_2_LED(1);				delay_ms(200);				SANG_2_LED(2);				delay_ms(200);
			i++;
		}
		SANG_4_LED_OFF();
}

void Init_LEDSANG_PORTD_12_13_14_15(void)
{
  GPIO_InitTypeDef PIN_LED_SANG_PORTD;

//Init PIN cho led sang
	PIN_LED_SANG_PORTD.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	PIN_LED_SANG_PORTD.Mode = GPIO_MODE_OUTPUT_PP;
	PIN_LED_SANG_PORTD.Pull = GPIO_NOPULL;
	PIN_LED_SANG_PORTD.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOD, &PIN_LED_SANG_PORTD);
}

void Init_BUTTON_USER_PORT_A_0(void)
{
  GPIO_InitTypeDef BUTTON_USER_PORTA_0;	
	//Init button User
	BUTTON_USER_PORTA_0.Pin = GPIO_PIN_0;
	BUTTON_USER_PORTA_0.Mode = GPIO_MODE_INPUT;
	BUTTON_USER_PORTA_0.Pull = GPIO_NOPULL;
	BUTTON_USER_PORTA_0.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &BUTTON_USER_PORTA_0);
}

void Init_PWM_GPIO_PORT_C_4Channel(void)
{
		GPIO_InitTypeDef GPIO_PWM_PORTC_6789;	
		//Init PWM GPIOC cho 4 channel
		GPIO_PWM_PORTC_6789.Pin = GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6;
		GPIO_PWM_PORTC_6789.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORTC_6789.Pull = GPIO_NOPULL;
		GPIO_PWM_PORTC_6789.Speed = GPIO_SPEED_FAST;
		GPIO_PWM_PORTC_6789.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOC, &GPIO_PWM_PORTC_6789);
}

void Init_PWM_TIM3_Handle(TIM_HandleTypeDef handle_timer_3)
{
		handle_timer_3.Instance = TIM3;
		handle_timer_3.Init.Prescaler = 84-1; 									//vi timer 3 co max clock la 84MHz, -1 la vi dem(count) tu 0
		handle_timer_3.Init.CounterMode = TIM_COUNTERMODE_UP; 	//dem len
		handle_timer_3.Init.Period = 20000-1;									//Period(chu ki) = 20 mili s
		handle_timer_3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// =0
		HAL_TIM_PWM_Init(&handle_timer_3);										//Init PWM voi TIM HANDLE
}

void Init_TIM3_OUTPUT_COMPARE(TIM_HandleTypeDef handle_timer_3, TIM_OC_InitTypeDef  TIM_Output_compare)
{
		TIM_Output_compare.OCMode = TIM_OCMODE_PWM1;
		TIM_Output_compare.OCIdleState = TIM_OCIDLESTATE_SET;
		TIM_Output_compare.Pulse = 2000;											//set dutty cycle = Pulse*100/Period = 2000*100 / 20000 = 10%
		TIM_Output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
		TIM_Output_compare.OCFastMode = TIM_OCFAST_ENABLE;		
		HAL_TIM_PWM_ConfigChannel(&handle_timer_3, &TIM_Output_compare, TIM_CHANNEL_1); //config PWM cho channel 1 (PORTC.6)
		HAL_TIM_PWM_Init(&handle_timer_3);
		HAL_TIM_PWM_MspInit(&handle_timer_3);
}









/////Accelerametor 10truc///////////
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
void Init_I2C_Handle_10truc(void)
{
	I2C_InitStruct_10truc.Instance = I2C1;
  I2C_InitStruct_10truc.Init.ClockSpeed = 100000;
  I2C_InitStruct_10truc.Init.DutyCycle = I2C_DUTYCYCLE_2;
  I2C_InitStruct_10truc.Init.OwnAddress1 = 0;
  I2C_InitStruct_10truc.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2C_InitStruct_10truc.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  I2C_InitStruct_10truc.Init.OwnAddress2 = 0;
  I2C_InitStruct_10truc.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  I2C_InitStruct_10truc.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&I2C_InitStruct_10truc);	
}


void TM_I2C_IS_DEVICE_CONNECTED(I2C_HandleTypeDef* Handle)
{
	if(HAL_I2C_IsDeviceReady(Handle, MPU6050_I2C_ADDR, 2, 5) != HAL_OK) 
	{
		while(1)
		{
			Error_Handler();
		}
	}
	while( HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY ){}
}

uint8_t TM_I2C_WHO_I_AM(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address)
{	
	uint8_t data;
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}		
	
	if (HAL_I2C_Master_Receive(Handle, device_address, &data, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}
	return data;
}


void TM_I2C_WRITE(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t data) 
{
	uint8_t d[2];		
	d[0] = register_address;
	d[1] = data;	
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, (uint8_t *)d, 2, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}	
}
	
	
void TM_I2C_READ( I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t* data)
{
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(Handle, device_address, data, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}
}


void TM_I2C_READ_MULTI(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t* data, uint16_t count)  
{
	if (HAL_I2C_Master_Transmit(Handle, (uint16_t)device_address, &register_address, 1, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}
	
	if (HAL_I2C_Master_Receive(Handle, device_address, data, count, 1000) != HAL_OK) 
	{}
	while(HAL_I2C_GetState(Handle) != HAL_I2C_STATE_READY) {}	
}


void TM_MPU6050_SetDataRate(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t rate) 
{
	TM_I2C_WRITE(Handle, device_address, register_address, rate);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_SetAccelerometer(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t Acc_8G_Value) 
{
	uint8_t temp;		
	TM_I2C_READ(Handle, device_address, register_address, &temp);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Acc_8G_Value << 3;
	TM_I2C_WRITE(Handle, device_address, register_address, temp);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_SetGyroscope(I2C_HandleTypeDef* Handle, uint8_t device_address, uint8_t register_address, uint8_t Gyro_250s_Value) 
{
	uint8_t temp;		
	TM_I2C_READ(Handle, device_address, register_address, &temp);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
	temp = (temp & 0xE7) | (uint8_t)Gyro_250s_Value << 3;
	TM_I2C_WRITE(Handle, device_address, register_address, temp);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
}

void TM_MPU6050_ReadAccelerometer(I2C_HandleTypeDef* Handle, uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	TM_I2C_READ_MULTI(Handle, device_address, MPU6050_ACCEL_XOUT_H, data, 6);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
		
	output->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	output->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
}

void TM_MPU6050_ReadGyroscope(I2C_HandleTypeDef* Handle, uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[6];
	TM_I2C_READ_MULTI(Handle, device_address, MPU6050_GYRO_XOUT_H, data, 6);
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}
		
	output->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	output->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);
}	

void TM_MPU6050_ReadAll(I2C_HandleTypeDef* Handle, uint8_t device_address, TM_MPU6050_t* output )
{
	uint8_t data[14];
	int16_t temp;	
	
	TM_I2C_READ_MULTI(Handle, device_address, MPU6050_ACCEL_XOUT_H, data, 14); /* Read full raw data, 14bytes */
	while(HAL_I2C_GetState(Handle)!=HAL_I2C_STATE_READY){}	
	
	output->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);	/* Format accelerometer data */
	output->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	output->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);
	
	temp = (data[6] << 8 | data[7]); /* Format temperature */
	output->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	output->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]); /* Format gyroscope data */
	output->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	output->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);
}



void Sang_Led_By_MPU6050_Values(float rotation_x,  float rotation_y)
{
	if(rotation_x > 10)
	{
		SANG_1_LED(12); 		delay_ms(10);	
	}else if(rotation_x < -10)
	{
		SANG_1_LED(13); 		delay_ms(10);	
	}
	if(rotation_y > 10)
	{
		SANG_1_LED(14);   delay_ms(10);	
	}else if(rotation_y < -10)
	{
		SANG_1_LED(15);    delay_ms(10);	
	}
	SANG_4_LED_OFF();
}
////end /Accelerametor 10truc///////////





//
//Timer 1 PWM input capture
//
void Init_Receiver_TIM1_PWM_Capture_PortE(TIM_HandleTypeDef timer1)
{
		GPIO_InitTypeDef GPIO_PWM_PORT_E;	
		TIM_ClockConfigTypeDef sClockSourceConfig;
		TIM_MasterConfigTypeDef TIM_sMasterConfig;
		TIM_IC_InitTypeDef TIM_Input_Capture;
		
		HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	
		//GPIO 
		GPIO_PWM_PORT_E.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_13 | GPIO_PIN_14;
		GPIO_PWM_PORT_E.Mode = GPIO_MODE_AF_PP;
		GPIO_PWM_PORT_E.Pull = GPIO_NOPULL;
		GPIO_PWM_PORT_E.Speed = GPIO_SPEED_FAST;
		GPIO_PWM_PORT_E.Alternate = GPIO_AF1_TIM1;
		HAL_GPIO_Init(GPIOE, &GPIO_PWM_PORT_E);
	
		//timer 1 handle
		timer1.Instance = TIM1;
		timer1.Init.Prescaler = 84-1; 									//vi timer 3 co max clock la 84MHz, -1 la vi dem(count) tu 0
		timer1.Init.CounterMode = TIM_COUNTERMODE_UP; 	//dem len
		timer1.Init.Period = 20000-1;									//Period(chu ki) = 20 mili s
		timer1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// =0
		if(HAL_TIM_Base_Init(&timer1)!=HAL_OK) 	{			Error_Handler();		}
		
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		HAL_TIM_ConfigClockSource(&timer1, &sClockSourceConfig);
		if(HAL_TIM_IC_Init(&timer1)!=HAL_OK)	{			Error_Handler();		}
		
		TIM_sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		TIM_sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		HAL_TIMEx_MasterConfigSynchronization(&timer1, &TIM_sMasterConfig);
	

		//timer 1 Input capture
		TIM_Input_Capture.ICPolarity = TIM_ICPOLARITY_RISING;
		TIM_Input_Capture.ICSelection = TIM_ICSELECTION_DIRECTTI;
		TIM_Input_Capture.ICPrescaler = TIM_ICPSC_DIV1;
		TIM_Input_Capture.ICFilter = 0;
		
		//start Timer 1 with Input Capture
		if(HAL_TIM_IC_ConfigChannel(&timer1, &TIM_Input_Capture, TIM_CHANNEL_1) != HAL_OK){			Error_Handler();		}	
		__HAL_TIM_ENABLE_IT(&timer1, TIM_IT_UPDATE);		
		if(HAL_TIM_IC_Start_IT(&timer1, TIM_CHANNEL_1) != HAL_OK){		Error_Handler();	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance==TIM2)
		{
				input_capture = __HAL_TIM_GetCompare(&htimer1, TIM_CHANNEL_1);	//read TIM1 channel 1 capture value
				__HAL_TIM_SetCounter(&htimer1, 0);															//reset counter after input capture interrupt occurs
		}
}

/*
void TIM1_CC_IRQHandler(void)
{ 
	uint32_t uhCaptureNumber;
	//TIM_GET_ITSTATUS(TIM1, TIM_IT_CC1)
  if(TIM_GET_ITSTATUS(TIM1, TIM_IT_CC2) == SET) 
  {
    // Clear TIM1 Capture compare interrupt pending bit 
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
    if(uhCaptureNumber == 0)
    {
      // Get the Input Capture value
      uhIC3ReadValue1 = TIM_GetCapture2(TIM1);
      uhCaptureNumber = 1;
    }
    else if(uhCaptureNumber == 1)
    {
      // Get the Input Capture value 
      uhIC3ReadValue2 = TIM_GetCapture2(TIM1); 
      
      // Capture computation 
      if (uhIC3ReadValue2 > uhIC3ReadValue1)
      {
        uwCapture = (uhIC3ReadValue2 - uhIC3ReadValue1); 
      }
      else if (uhIC3ReadValue2 < uhIC3ReadValue1)
      {
        uwCapture = ((0xFFFF - uhIC3ReadValue1) + uhIC3ReadValue2); 
      }
      else
      {
        uwCapture = 0;
      }
      // Frequency computation 
      uwTIM1Freq = (uint32_t) SystemCoreClock / uwCapture;
      uhCaptureNumber = 0;
    }
  }
}*/

void TIM1_IRQHandler(void) {}

void TIM3_IRQHandler(void) {}
//
//End Timer 1 PWM input capture
//






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
		delay_ms(500);
		SANG_4_LED_OFF();
		delay_ms(500);
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
void SANG_4_LED_OFF()
{
		LED_D_12_LOW;
		LED_D_13_LOW;
		LED_D_14_LOW;
		LED_D_15_LOW;
}
void Acc_Led_Sang(int16_t Acc_X, int16_t Acc_Y, int16_t Acc_Z)
{
		int16_t defaulValue = 1000;
		LED_D_12_LOW;
		LED_D_13_LOW;
		LED_D_14_LOW;
		LED_D_15_LOW;			
		
		if(Acc_X < -defaulValue)
		{
			LED_D_12_LOW;
			LED_D_14_HIGH;
		}
		else if(Acc_X > defaulValue)
		{
			LED_D_14_LOW;				
			LED_D_12_HIGH;
		}

		if(Acc_Y < -defaulValue)
		{				
			LED_D_15_LOW;				
			LED_D_13_HIGH;
		}
		else if(Acc_Y > defaulValue)
		{				
			LED_D_13_LOW;				
			LED_D_15_HIGH;
		}		
		delay_ms(20);
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
