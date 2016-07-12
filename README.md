# quadrotor
*	Quadrotor build on KIT stm32f407
*	Author: Nguyen (xitrumuit1991)
*	Email: tranvokhoinguyen@gmail.com

--------------------------------------------------------------------------

Type X quadrotor
	(1)\   /(2)   ^^Head
        \ /		  ||	  y
         X		  ||	  |
        / \          x____|
    (4)/   \(3)				
(1)(3) motor
(2)(4) motor


--------------------------------------------------------------------------
Hardware: 
- 4 ESC(electric speed control) and 4 bruless motor
- RF module: devo 7 transmit and receiver
- STM32f407VG of ARM
- MPU6050 (gry sensor + accele sensor + temp sensor)


--------------------------------------------------------------------------
Output PWM TIMER 3:
-	PB4(channel 1) pwm 1 
-	PB5(channel 2) pwm 2
-	PB0(channel 3) pwm 3
-	PB1(channel 4) pwm 4


--------------------------------------------------------------------------
InputCaptrue PIN:
-	PE9 ->TIM1_CH1  //Throttle (can ga) tang giam toc do quay	keo len +(1900), keo xuong -(1100)
-	PA5 ->TIM2_CH1  //Rudder (xoay theo truc z) - goc Yaw		keo qua trai +(1900), keo qua phai -(1100)
-	PB8 ->TIM4_CH3  //Elevator (tien - lui) - goc Pitch. 		keo len la +, keo xuong la -
-	PA3 ->TIM5_CH4  //Aileron_TraiPhai (trai - phai) - goc Roll     keo qua trai +, keo qua phai -


--------------------------------------------------------------------------
Other PIN
-	PORT A: PA0 					=> Button User  
-	PORT B: PB6, PB7 				=> I2C1 cam bien 10 truc mpu6050 (PB6->I2C1_SCL,	PB7->I2C1_SDA) 	
-	PORT D: PD12, PD13, PD14, PD15  		=> LEDSang (PD12 GREEN, PD13 ORANGE, PD14 RED, PD15 BLUE)

--------------------------------------------------------------------------
