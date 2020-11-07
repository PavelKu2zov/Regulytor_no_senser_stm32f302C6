#include "stm32f30x.h"
#include "Init.h"


//источник прерывания AWD
void ADC1_IRQHandler()
{
//Отключаем нижние ключи
//  k4_close;
//  k5_close;
//  k6_close;
  GPIOB->BSRR = GPIO_BSRR_BS_12;//Зажигаем светодиод

}