# quadrotor
quadrotor stm32f407

Hardware: 
- 4 ESC(electric speed control) and 4 bruless motor
- RF module: devo 7 transmit and receiver
- STM32f407VG of ARM
- MPU6050 (gry sensor + accele sensor + temp sensor)


PIN connected:
- Port A: PIN 0               (button user)
- PORT B: PIN 6, 7 						(I2C1 chip 10 truc mpu6050)
- PORT C: PIN 6, 7, 8, 9 			(timer 3 - PWM for 4 motor)
- Port D: PIN 12, 13, 14, 15  (for Led)
- PORT E: PIN 9, 11, 13, 14		(timer 1 - PWM input capture of 2.4G radio frequency)
