#include "stm32f4xx.h"
#include "filter.h"
#include <stdlib.h>
#include <math.h>

GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

void Threads(void);
void TIM2_Configuration(void);
void Delay(__IO uint32_t nCount);

#define PI 3.141592653589793f
int32_t nowtime,lasttime;
float lengthtime;
volatile float _random,_sin,random_LPF_HPF,random_kalman,random[3],signal_LPF,signal_HPF,signal_kalman,signal[3];
int main(void)
{
  TIM2_Configuration();
  SysTick_Config(SystemCoreClock / 999);
  while (1)
  {
		
  }
}

void Threads(void)
{
  int i;
  static float x=0;
  lasttime = TIM_GetCounter(TIM2);
  
  _sin = sin(x)*1000;
  x +=2*PI/1000;
  _random = (float)rand()/1000000;
  
  random_LPF_HPF = _random + _sin;
  signal_LPF = LPF(random_LPF_HPF,1,1000);
  signal_HPF = HPF(random_LPF_HPF,10,1000);
  
  random_kalman = (float)rand()/1000000;
  signal_kalman = kalman_single(random_kalman, 500, 10);
  
  for(i=0;i<3;i++)
    random[i] = (float)rand()/1000000;
  kalman((float*)random,(float*)signal,500,10);
  
  nowtime = TIM_GetCounter(TIM2);
  lengthtime = (float)(nowtime - lasttime)/84; // us
}

void TIM2_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_Cmd(TIM2, ENABLE);
}

void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}


#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  while (1)
  {}
}
#endif
