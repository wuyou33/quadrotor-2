u8 GetAFreeTimer(u32 unTime)
{
    u8 ucTmrSearchWith = 0;    
     
    u8 ucTimerIndex = 0;
     
    //find an idle timer
    if(unTime < 16384)//小于16.383s
    {
        ucTmrSearchWith = 0;
    }
    else if(unTime > 16383 && unTime < 32768)
    {
        ucTmrSearchWith = 3;
    }
    else if(unTime > 32767 && unTime < 2147483647)
    {
        ucTmrSearchWith = 8;
    }
 
     
    for(; ucTmrSearchWith < 10; ucTmrSearchWith++)
    {
        if(0 == acTimerId[ucTmrSearchWith])
        {
            ucTimerIndex = acTmrOrder[ucTmrSearchWith];
        }
    }
     
    return ucTimerIndex;
}
/*设置定时器,最大计时2147483647ms(24.8天)，最小1ms，其中unTime值不能超过该值
unTime: 计时时间，单位ms
unCount: 0：连续计时；其它：计时次数，到达次数后计时器停止工作
pHandler                    计数完成后执行函数
ucPara                      执行函数的参数
返回值：TimerID or 0:失败
*/
u8 SetTimer(u32 unTime, u32 unCount, TIMERFUN pHandler,u32 ucPara)
{
    u8 ucTimerIndex = 0;
    u8 ucRslt = 0;
    u8 ucTmrSearchWith = 0;
    u32 unTimeUse = 0;
 
    //find an idle timer
     
    if(unTime < 16384)//小于16.383s
    {
        ucTmrSearchWith = 0;
    }
    else if(unTime > 16383 && unTime < 32768)
    {
        ucTmrSearchWith = 3;
    }
    else if(unTime > 32767 && unTime < 2147483647)
    {
        ucTmrSearchWith = 8;
    }
    else
    {
        return 0;
    }
     
     
    for(; ucTmrSearchWith < 10; ucTmrSearchWith++)
    {
        if(0 == acTimerId[ucTmrSearchWith])
        {
            ucTimerIndex = acTmrOrder[ucTmrSearchWith];
            acTimerId[ucTmrSearchWith] = ucTimerIndex;
            ucRslt = ucTimerIndex;
 
 
            if(0 != unCount)
            {
                unCount++;
            }
            anTimingCnt[ucTmrSearchWith] = unCount;
 
            unTimeUse = unTime;
            if(ucTmrSearchWith < 3)
            {
                unTimeUse *= 2;
            }
 
            if( 0 != TimerEnable(ucTimerIndex, unTimeUse * 2, 42000, pHandler, ucPara))
            {
                acTimerId[ucTmrSearchWith] = 0;
                ucRslt = 0;
                continue;
            }
            break;
        }
    }
 
    return ucRslt;
}
 
 
 
 
 
 
/*关闭定时器
tmrId:定时器ID
*/
void KillTimer(TIMER tmrId)
{
    TimerStop(tmrId,0);
    switch(tmrId)
    {
    case 2:
        acTimerId[0] = 0;
        break;
    case 3:
        acTimerId[1] = 0;
        break;
    case 4:
        acTimerId[2] = 0;
        break;
    case 5:
        acTimerId[3] = 0;
        break;
    case 9:
        acTimerId[4] = 0;
        break;
    case 10:
        acTimerId[5] = 0;
        break;
    case 11:
        acTimerId[6] = 0;
        break;
    case 12:
        acTimerId[7] = 0;
        break;
    case 13:
        acTimerId[8] = 0;
        break;
    case 14:
        acTimerId[9] = 0;
        break;
    default:
        break;
    }
}
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
//定时器2中断服务程序
void TIM2_IRQHandler(void)
{
    if(TIM2->SR&0X0001)//溢出中断
    {
        if(ucTmr2Efct)
        {
            if(Timer2Fun)
            {
                (*Timer2Fun)(ucTimer2Para);
            }
            if(anTimingCnt[8] > 1)          //判断计时次数
            {
                anTimingCnt[8] --;
            }
            else if(1 == anTimingCnt[8])    //计时次数完成
            {
                KillTimer(2);
            }
        }
        else
        {
            ucTmr2Efct = 1;
        }
    }
    TIM2->SR&=~(1<<0);//清除中断标志位
}
//定时器3中断服务程序
void TIM3_IRQHandler(void)
{
    if(TIM3->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr3Efct)
        {
            if(Timer3Fun)
            {
                (*Timer3Fun)(ucTimer3Para);
            }
            if(anTimingCnt[3] > 1)          //判断计时次数
            {
                anTimingCnt[3] --;
            }
            else if(1 == anTimingCnt[3])    //计时次数完成
            {
                KillTimer(3);
            }
        }
        else
        {
            ucTmr3Efct = 1;
        }
    }
    TIM3->SR&=~(1<<0);//清除中断标志位
}
//定时器4中断服务程序
void TIM4_IRQHandler(void)
{
    if(TIM4->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr4Efct)
        {
            if(Timer4Fun)
            {
                (*Timer4Fun)(ucTimer4Para);
            }
            if(anTimingCnt[4] > 1)          //判断计时次数
            {
                anTimingCnt[4] --;
            }
            else if(1 == anTimingCnt[4])    //计时次数完成
            {
                KillTimer(4);
            }
        }
        else
        {
            ucTmr4Efct = 1;
        }
    }
    TIM4->SR&=~(1<<0);//清除中断标志位
}
//定时器5中断服务程序
void TIM5_IRQHandler(void)
{
    if(TIM5->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr5Efct)
        {
            if(Timer5Fun)
            {
                (*Timer5Fun)(ucTimer5Para);
            }
            if(anTimingCnt[9] > 1)          //判断计时次数
            {
                anTimingCnt[9] --;
            }
            else if(1 == anTimingCnt[9])    //计时次数完成
            {
                KillTimer(5);
            }
        }
        else
        {
            ucTmr5Efct = 1;
        }
    }
    TIM5->SR&=~(1<<0);//清除中断标志位
}
//定时器9中断服务程序
void TIM1_BRK_TIM9_IRQHandler(void)
{
    if(TIM9->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr9Efct)
        {
            if(Timer9Fun)
            {
                (*Timer9Fun)(ucTimer9Para);
            }
            if(anTimingCnt[0] > 1)          //判断计时次数
            {
                anTimingCnt[0] --;
            }
            else if(1 == anTimingCnt[0])    //计时次数完成
            {
                KillTimer(9);
            }
        }
        else
        {
            ucTmr9Efct = 1;
        }
        TIM9->SR&=~(1<<0);//清除中断标志位
    }
}
 
//定时器10中断服务程序
void TIM1_UP_TIM10_IRQHandler(void)
{
    if(TIM10->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr10Efct)
        {
            if(Timer10Fun)
            {
                (*Timer10Fun)(ucTimer10Para);
            }
            if(anTimingCnt[1] > 1)          //判断计时次数
            {
                anTimingCnt[1] --;
            }
            else if(1 == anTimingCnt[1])    //计时次数完成
            {
                KillTimer(10);
            }
        }
        else
        {
            ucTmr10Efct = 1;
        }
        TIM10->SR&=~(1<<0);//清除中断标志位
    }
}
 
 
//定时器11中断服务程序
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    if(TIM11->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr11Efct)
        {
            if(Timer11Fun)
            {
                (*Timer11Fun)(ucTimer11Para);
            }
            if(anTimingCnt[2] > 1)          //判断计时次数
            {
                anTimingCnt[2] --;
            }
            else if(1 == anTimingCnt[2])    //计时次数完成
            {
                KillTimer(11);
            }
        }
        else
        {
            ucTmr11Efct = 1;
        }
        TIM11->SR&=~(1<<0);//清除中断标志位
    }
}
 
 
 
//定时器12中断服务程序
void TIM8_BRK_TIM12_IRQHandler(void)
{
    if(TIM12->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr12Efct)
        {
            if(Timer12Fun)
            {
                (*Timer12Fun)(ucTimer12Para);
            }
            if(anTimingCnt[5] > 1)          //判断计时次数
            {
                anTimingCnt[5] --;
            }
            else if(1 == anTimingCnt[5])    //计时次数完成
            {
                KillTimer(12);
            }
        }
        else
        {
            ucTmr12Efct = 1;
        }
        TIM12->SR&=~(1<<0);//清除中断标志位
    }
}
 
 
//定时器13中断服务程序
void TIM8_UP_TIM13_IRQHandler(void)
{
    if(TIM13->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr13Efct)
        {
            if(Timer13Fun)
            {
                (*Timer13Fun)(ucTimer13Para);
            }
            if(anTimingCnt[6] > 1)          //判断计时次数
            {
                anTimingCnt[6] --;
            }
            else if(1 == anTimingCnt[6])    //计时次数完成
            {
                KillTimer(13);
            }
        }
        else
        {
            ucTmr13Efct = 1;
        }
        TIM13->SR&=~(1<<0);//清除中断标志位
    }
}
 
 
//定时器14中断服务程序
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    if(TIM14->SR&0X0001)//溢出中断
    {
        if(1 == ucTmr14Efct)
        {
            if(Timer14Fun)
            {
                (*Timer14Fun)(ucTimer14Para);
            }
            if(anTimingCnt[7] > 1)          //判断计时次数
            {
                anTimingCnt[7] --;
            }
            else if(1 == anTimingCnt[7])    //计时次数完成
            {
                KillTimer(14);
            }
        }
        else
        {
            ucTmr14Efct = 1;
        }
        TIM14->SR&=~(1<<0);//清除中断标志位
    }
}
 
 
 
 
 
/*使能基本计时器
    ucTimerIdx:计时器编号 范围：2~5 9~14
    unCount                     自动重装值。
    uwPsc                       计数器分频 2-65535 计时时间 = unCount * uwPsc / 84M(168M if ucTimerIdx in{9, 10, 11})
    pHandler                    计数完成后执行函数
    ucPara                      执行函数的参数
 
 
返回值:
0:成功
1:ucOpt unRunningFreqency unCount参数错误
2:定时器编号参数错误
3:定时器已经被占用
4:16位定时器传入的计数参数超阈值*/
u8 TimerEnable(u8 ucTimerIdx, u32 unCount, u16 uwPsc,TIMERFUN pHandler,u8 ucPara)
{
    //参数检查
    if(unCount < 2 || uwPsc < 2 || uwPsc > 65535)                                                                  //参数错误:重装值、分频数设置错误
    {
        return 1;
    }
    if(ucTimerIdx < 2 || ucTimerIdx > 14 || (ucTimerIdx > 5 && ucTimerIdx < 9) )                                //参数错误:不是2-5 9-14定时器
    {
        return 2;
    }
 
 
 
    switch(ucTimerIdx)
    {
    case 2:
        if(0x0001 & TIM2 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
//        if(0 == ucOpt)
//        {
//            TIM2 ->CR1 |= 1 << 3;
//        }
//        else if(1 == ucOpt)
//        {
//            TIM2 ->CR1 &= ~(1 << 3);
//        }
        RCC ->APB1ENR |= 1;
        TIM2 ->ARR = unCount - 1;
        TIM2 ->PSC = uwPsc - 1;
        TIM2 ->DIER |= 1;
        Timer2Fun = pHandler;
        ucTimer2Para = ucPara;
        MY_NVIC_Init(2,0,TIM2_IRQn,2);
        TIM2->SR = 0;//清除中断标志位
        TIM2 ->CR1 |= 0x01;
        break;
    case 3:
        if(unCount > 0x0000FFFF)
        {
            return 4;
        }
        if(0x0001 & TIM3 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 1;                //使能定时器时钟
        TIM3 ->ARR = unCount - 1;
        TIM3 ->PSC = uwPsc - 1;
        TIM3 ->DIER |= 1;
        Timer3Fun = pHandler;
        ucTimer3Para = ucPara;
        MY_NVIC_Init(0,3,TIM3_IRQn,2);
        TIM3->SR = 0;//清除中断标志位
        TIM3 ->CR1 |= 0x01;
        break;
    case 4:
        if(unCount > 0x0000FFFF)
        {
            return 4;
        }
        if(0x0001 & TIM4 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 2;                //使能定时器时钟
        TIM4 ->ARR = unCount - 1;
        TIM4 ->PSC = uwPsc - 1;
        TIM4 ->DIER |= 1;
 
        Timer4Fun = pHandler;
        ucTimer4Para = ucPara;
        MY_NVIC_Init(1, 0 , TIM4_IRQn, 2);
        TIM4->SR = 0;//清除中断标志位
        TIM4 ->CR1 |= 0x01;
        break;
    case 5:
        if(0x0001 & TIM5 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 3;                //使能定时器时钟
        TIM5 ->ARR = unCount - 1;
        TIM5 ->PSC = uwPsc - 1;
        TIM5 ->DIER |= 1;
 
        Timer5Fun = pHandler;
        ucTimer5Para = ucPara;
        MY_NVIC_Init(2,1,TIM5_IRQn,2);
        TIM5->SR = 0;//清除中断标志位
        TIM5 ->CR1 |= 0x01;
        break;
    case 9:
        if(0x0001 & TIM9 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB2ENR |= 1 << 16;                //使能定时器时钟
        TIM9 ->ARR = unCount - 1;
        TIM9 ->PSC = uwPsc - 1;
        TIM9 ->DIER |= 1;
 
        Timer9Fun = pHandler;
        ucTimer9Para = ucPara;
        MY_NVIC_Init(0, 0 ,TIM1_BRK_TIM9_IRQn,2);
        TIM9->SR = 0;//清除中断标志位
        TIM9 ->CR1 |= 0x01;
        break;
    case 10:
        if(0x0001 & TIM10 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB2ENR |= 1 << 17;                //使能定时器时钟
        TIM10 ->ARR = unCount - 1;
        TIM10 ->PSC = uwPsc - 1;
        TIM10 ->DIER |= 1;
 
        Timer10Fun = pHandler;
        ucTimer10Para = ucPara;
        MY_NVIC_Init(0, 1 ,TIM1_UP_TIM10_IRQn,2);
        TIM10->SR = 0;//清除中断标志位
        TIM10 ->CR1 |= 0x01;
        break;
    case 11:
        if(0x0001 & TIM11 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB2ENR |= 1 << 18;                //使能定时器时钟
        TIM11 ->ARR = unCount - 1;
        TIM11 ->PSC = uwPsc - 1;
        TIM11 ->DIER |= 1;
 
        Timer11Fun = pHandler;
        ucTimer11Para = ucPara;
        MY_NVIC_Init(0, 2 ,TIM1_TRG_COM_TIM11_IRQn,2);
        TIM11->SR = 0;//清除中断标志位
        TIM11 ->CR1 |= 0x01;
        break;
    case 12:
        if(0x0001 & TIM12 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 6;                //使能定时器时钟
        TIM12 ->ARR = unCount - 1;
        TIM12 ->PSC = uwPsc - 1;
        TIM12 ->DIER |= 1;
 
        Timer12Fun = pHandler;
        ucTimer12Para = ucPara;
        MY_NVIC_Init(1, 1 ,TIM8_BRK_TIM12_IRQn,2);
        TIM12->SR = 0;//清除中断标志位
        TIM12 ->CR1 |= 0x01;
        break;
    case 13:
        if(0x0001 & TIM13 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 7;                //使能定时器时钟
        TIM13 ->ARR = unCount - 1;
        TIM13 ->PSC = uwPsc - 1;
        TIM13 ->DIER |= 1;
 
        Timer13Fun = pHandler;
        ucTimer13Para = ucPara;
        MY_NVIC_Init(1, 2 ,TIM8_UP_TIM13_IRQn,2);
        TIM13->SR = 0;//清除中断标志位
        TIM13 ->CR1 |= 0x01;
        break;
    case 14:
        if(0x0001 & TIM14 ->CR1)                 //定时器正在被使用
        {
            return 3;
        }
        RCC ->APB1ENR |= 1 << 8;                //使能定时器时钟
        TIM14 ->ARR = unCount - 1;
        TIM14 ->PSC = uwPsc - 1;
        TIM14 ->DIER |= 1;
 
        Timer14Fun = pHandler;
        ucTimer14Para = ucPara;
        MY_NVIC_Init(1, 3 ,TIM8_TRG_COM_TIM14_IRQn,2);
        TIM14->SR = 0;//清除中断标志位
        TIM14 ->CR1 |= 0x01;
        break;
    default:
        break;
    }
    return 0;           //返回成功
}
 
 
/*停止基本计时器  使能位置0，停止其时钟
    ucTimerIdx:计时器编号 范围：2~5
    ucOpt   可选参数  保留不用
*/
void TimerStop(u8 ucTimerIdx,u8 ucOpt)
{
    switch(ucTimerIdx)
    {
    case 2:
        TIM2 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 0);                    //关闭时钟
        Timer2Fun = 0;
        ucTimer2Para = 0;
        ucTmr2Efct = 0;
        break;
    case 3:
        TIM3 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 1);                    //关闭时钟
        Timer3Fun = 0;
        ucTimer3Para = 0;
        ucTmr3Efct = 0;
        break;
    case 4:
        TIM4 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 2);                    //关闭时钟
        Timer4Fun = 0;
        ucTimer4Para = 0;
        ucTmr4Efct = 0;
        break;
    case 5:
        TIM5 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 3);                    //关闭时钟
        Timer5Fun = 0;
        ucTimer5Para = 0;
        ucTmr5Efct = 0;
        break;
    case 9:
        TIM9 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB2ENR &= ~(1 << 16);                   //关闭时钟
        Timer9Fun = 0;
        ucTimer9Para = 0;
        ucTmr9Efct = 0;
        break;
    case 10:
        TIM10 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB2ENR &= ~(1 << 17);                    //关闭时钟
        Timer10Fun = 0;
        ucTimer10Para = 0;
        ucTmr10Efct = 0;
        break;
    case 11:
        TIM11 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB2ENR &= ~(1 << 18);                    //关闭时钟
        Timer11Fun = 0;
        ucTimer11Para = 0;
        ucTmr11Efct = 0;
        break;
    case 12:
        TIM12 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 6);                     //关闭时钟
        Timer12Fun = 0;
        ucTimer12Para = 0;
        ucTmr12Efct = 0;
        break;
    case 13:
        TIM13 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 7);                     //关闭时钟
        Timer13Fun = 0;
        ucTimer13Para = 0;
        ucTmr13Efct = 0;
        break;
    case 14:
        TIM14 ->CR1 &= ~(1 << 0);                       //关闭使能
        RCC ->APB1ENR &= ~(1 << 8);                     //关闭时钟
        Timer14Fun = 0;
        ucTimer14Para = 0;
        ucTmr14Efct = 0;
        break;
    default:
        break;
    }
}