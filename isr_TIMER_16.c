#include "stm32f30x.h"
#include "Init.h"
//extern TypedefStateBLDC BLDC;
extern struct _stateBLDC StateBLDC;
     
void TIM1_UP_TIM16_IRQHandler()
{  
  if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_8) == Bit_RESET)
  {
    uint32_t t = TIM_GetCapture1(TIM16);
    if (t < IMPULS_MID_FRSKY)
    {
      t = (IMPULS_MID_FRSKY - IMPULS_MIN_FRSKY) - (t - IMPULS_MIN_FRSKY);
     if (!StateBLDC.revers)
      {
        k1_close;k2_close;k3_close;k4_close;k5_close;k6_close;
        k5_open;
        StateBLDC.position = 0;
        StateBLDC.revers = 1; // реверс
      }

    } 
    else
    {
      t -= IMPULS_MID_FRSKY;
      if (StateBLDC.revers)
      {
        k1_close;k2_close;k3_close;k4_close;k5_close;k6_close;
        k4_open;
        StateBLDC.position = 0;
        StateBLDC.revers = 0; // не реверс
      }
      
    }
    t = t/StateBLDC.k;
    t =t;///2;
    if (t > StateBLDC.MaxGas)
      StateBLDC.gas = StateBLDC.MaxGas;
    else
      StateBLDC.gas = t;
  
  TIM1->CCR1 = StateBLDC.gas;
  TIM1->CCR2 = StateBLDC.gas;
  TIM1->CCR3 = StateBLDC.gas;
    
    
  }
  TIM16->CNT = 0;
  
    
  TIM_ClearFlag(TIM16, TIM_FLAG_CC1);
}