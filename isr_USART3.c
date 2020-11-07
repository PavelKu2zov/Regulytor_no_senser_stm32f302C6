#include "stm32f30x.h"
#include "Init.h"
extern Circular_buffTypeDef   Circular_buff;

void USART3_IRQHandler()
{
  uint32_t status = USART3->ISR;
  
  if ( (status & USART_FLAG_TXE) == USART_FLAG_TXE)
  {
    if ( Circular_buff.p_rd != Circular_buff.p_wr )
      USART3->TDR = *Circular_buff.p_rd;
    
    if ( Circular_buff.p_rd == (Circular_buff.StartAdr + Circular_buff.Size))
      Circular_buff.p_rd = Circular_buff.StartAdr;
    else
      Circular_buff.p_rd++;
    
    if ( Circular_buff.p_rd == Circular_buff.p_wr )
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }
USART_ClearITPendingBit(USART3, USART_IT_TXE);
}