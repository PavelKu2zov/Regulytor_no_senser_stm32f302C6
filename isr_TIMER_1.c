#include "stm32f30x.h"
#include "Init.h"


extern struct _stateBLDC StateBLDC;
extern uint16_t *sence1,*sence2;
extern uint16_t *Threshold;



void TIM1_CC_IRQHandler()
{
 

   TIM1->SR &= ~TIM_SR_CC1IF; 
 if( StateBLDC.SenceEnable == 0 ) 
 {

    do 
    {
        ADC1->ISR = ADC_ISR_JEOS; //clear JEOC flag
        ADC1->CR |= ADC_CR_JADSTART; //start convertion
        while ((ADC1->ISR & ADC_ISR_JEOS) != ADC_ISR_JEOS); 
 
        if ( *sence1 > *sence2 )         
          { 
//            if (( GPIOB->ODR & GPIO_ODR_12) == GPIO_ODR_12) 
//            {GPIOB->BSRR = GPIO_BSRR_BR_12;}
//            else
//            {GPIOB->BSRR = GPIO_BSRR_BS_12;}
            for(int i = 0; i < 10; i++);//без этой задержки не старатует почему не €сно??????


            
            
     //        StateBLDC.time_sence =  TIM2->CNT>>1;
             if ( StateBLDC.state == run )
              {
               
               if (StateBLDC.index_mem > 15)
                   StateBLDC.index_mem = 0;
               
               StateBLDC.time_sence_mem[StateBLDC.index_mem++] = TIM2->CNT>>1;
                 StateBLDC.time_sence = TIM2->CNT>>1;
               TIM2->CNT = 0;
              // StateBLDC.time_sence=0;                
               
//               for (int i=0;i<16;i++)
//                  StateBLDC.time_sence += StateBLDC.time_sence_mem[i];
               
             

               TIM2->CCR1 = StateBLDC.time_sence;//-StateBLDC.Operegenie;
              }
             TIM1->DIER &= ~TIM_DIER_CC4IE;
             TIM1->SR = ~TIM_SR_CC4IF;        
             
//             if ((StateBLDC.time_sence < 18000) && (Threshold != Threshold_Vp_div2))// 2000 об/мин 18000
//             {
//                TIM1->CCR4 = 200;
//                Threshold = Threshold_Vp_div2;
//                
//             }
             if (StateBLDC.time_sence < 12000)// 
               StateBLDC.use_delta_gas=0;
             break; 
          }

    }while((TIM1->SR & TIM_SR_CC1IF) != TIM_SR_CC1IF);
 
 }
 else 
   StateBLDC.SenceEnable--;
  
    TIM1->SR = ~TIM_SR_CC4IF;
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
    
    
}