#include "stm32f30x.h"
#include "Init.h"

extern struct _stateBLDC StateBLDC;
extern uint16_t *sence1,*sence2;
extern int32_t CountRate;
extern uint16_t *Threshold;
void TIM2_IRQHandler()
{

CountRate++;
//////////////////////////////////////////////////////////////////////////////////////new
if (StateBLDC.state == start)
  TIM2->CNT = 0;
//////////////////////////////////////////////////////////////////////////////////////new
  

  if (StateBLDC.position == 5) StateBLDC.position = 0;
  else StateBLDC.position++;
  
  
//Отключаем прерывание глобально для того,чтобы при коммутации не уйти в прерывание по превышению тока,если оно произойдет
  void irq_disable();
  if (!StateBLDC.revers)
    {
      switch(StateBLDC.position)
        { 
          case 0 : k6_close;  delay(); k5_open; sence1 = phasa_C; sence2 = Threshold;break;//phasa_C free к + sence PA3
          case 1 : k1_close;  delay(); k3_open; sence2 = phasa_A; sence1 = Threshold;break;//phasa_A free к - sence PA0
          case 2 : k5_close;  delay(); k4_open; sence1 = phasa_B; sence2 = Threshold;break;//phasa_B free к + sence PA1
          case 3 : k3_close;  delay(); k2_open; sence2 = phasa_C; sence1 = Threshold;break;//phasa_C free к - sence PA3
          case 4 : k4_close;  delay(); k6_open; sence1 = phasa_A; sence2 = Threshold;break;//phasa_A free к + sence PA0
          case 5 : k2_close;  delay(); k1_open; sence2 = phasa_B; sence1 = Threshold;break;//phasa_B free к - sence PA1
        default : k1_close;k2_close;k3_close;k4_close;k5_close;k6_close;break;
        }
    }
  else  //реверс
  {
        switch(StateBLDC.position)
      { 
        case 0 : k6_close;  delay(); k4_open; sence1 = phasa_C; sence2 = Threshold;break;//phasa_C free к + sence PA3
        case 1 : k2_close;  delay(); k3_open; sence2 = phasa_B; sence1 = Threshold;break;//phasa_B free к - sence PA1
        case 2 : k4_close;  delay(); k5_open; sence1 = phasa_A; sence2 = Threshold;break;//phasa_A free к + sence PA0
        case 3 : k3_close;  delay(); k1_open; sence2 = phasa_C; sence1 = Threshold;break;//phasa_C free к - sence PA3
        case 4 : k5_close;  delay(); k6_open; sence1 = phasa_B; sence2 = Threshold;break;//phasa_B free к + sence PA1
        case 5 : k1_close;  delay(); k2_open; sence2 = phasa_A; sence1 = Threshold;break;//phasa_A free к - sence PA0
      default : k1_close;k2_close;k3_close;k4_close;k5_close;k6_close;break;
      }
  
  }
  
void irq_enable();

if (0)//(StateBLDC.state == start)
{
  while (TIM2->CNT < 60000);
     switch(StateBLDC.position)
     {
      case 0 : k1_close;break;
      case 1 : k5_close;break;
      case 2 : k3_close;break;
      case 3 : k4_close;break;
      case 4 : k2_close;break;
      case 5 : k6_close;break;      
     }

}


if (StateBLDC.Cnt_start < CNT_START)
  StateBLDC.Cnt_start++;



StateBLDC.SenceEnable = 2;//1

TIM_ClearFlag(TIM2, TIM_IT_CC1);
TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
TIM1->SR = ~TIM_SR_CC4IF;
TIM1->DIER |= TIM_DIER_CC4IE;
NVIC->ICPR[0] = 1<<TIM2_IRQn;


}