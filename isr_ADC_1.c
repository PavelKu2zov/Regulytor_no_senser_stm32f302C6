#include "stm32f30x.h"
#include "Init.h"


//�������� ���������� AWD
void ADC1_IRQHandler()
{
//��������� ������ �����
//  k4_close;
//  k5_close;
//  k6_close;
  GPIOB->BSRR = GPIO_BSRR_BS_12;//�������� ���������

}