#include "stm32f30x.h"
#include "arm_math.h"
#include "Init.h"
void Init();
void Start_BLDC();

struct _stateBLDC StateBLDC;
uint16_t *sence1,*sence2;
uint16_t gas_tm;
uint32_t CountRate=0;
uint32_t ADCValueCurrentMax =0;
uint16_t BuffTxSpi3[SIZE_BUFF_SPI_DMA];
uint16_t BuffRxSpi3[SIZE_BUFF_SPI_DMA];
uint8_t buff_usart3_rx[SIZE_BUFF_USART3_TX];
uint16_t *Threshold;

uint16_t Threshold_PWM_off = 10;
uint32_t timer_read_spi =0;
uint32_t timer_read_adc=0;

uint16_t ADC_ch10=0;
uint16_t ADC_ch10_index=0;
uint32_t ADC_ch10_sum=0;

void  main(void)
{
   Init();
  //Threshold = &Threshold_PWM_off;//&Threshold_value;
  Threshold = Threshold_Vp_div2;
  //TIM1->CCR4 = Autoreload_Tim1-100;//100;//установка момента времени начала преобразования АЦП
  TIM1->CCR4 = 100;//200

   
  k1_close;
  k2_close;
  k3_close;
  k4_close;
  k5_close;
  k6_close;
  
//  while(1)
//  {
//     
//     ADC_ch10 = ADC_GetConversionValue(ADC1);
//  }
  

  StateBLDC.MaxGas = Autoreload_Tim1;
  StateBLDC.SenceEnable = 1;
  StateBLDC.Operegenie = OperegenieDefine;
  StateBLDC.counter = 0;//циклический счетчик пакетов
  StateBLDC.revers = 0;
  StateBLDC.index_mem=0;
  StateBLDC.use_delta_gas =1;
  for (int i=0;i<16;i++)
    StateBLDC.time_sence_mem[i] = 0;
    
  GPIOB->BSRR = GPIO_BSRR_BS_12;//Зажигаем светодиод
  TIM2->ARR = 0xFFFFFFFF;
  TIM2->CNT = 0;
  while (TIM2->CNT < 24000000); //Ожидаем 1 сек
  GPIOB->BSRR = GPIO_BSRR_BR_12;//Гасим светодиод
  TIM2->ARR = Autoreload_Tim2;
  TIM2->CNT = 0;
  
  TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
  TIM_ITConfig(TIM1,TIM_IT_CC4,DISABLE);//ENABLE
  
  
  Stop_BLCD();

  //определяем напряжение питания
 CalculateVDC();

  
 TIM_Cmd(TIM6,ENABLE);
 timer_read_adc = 500;
 timer_read_spi = 100;
 
 
 /////////////////////////////////////////////////////////////////////
//StateBLDC.state = run;
//TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
//TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
//    TIM2->CCR1 = 1000;
//    TIM2->CNT = 0;
////////////////////////////////////////////////////////////////////
#ifdef USE_FRSKY
StateBLDC.MaxGas = Autoreload_Tim1;
TIM_ITConfig(TIM16, TIM_IT_CC1, ENABLE);
StateBLDC.k = ((IMPULS_MAX_FRSKY-IMPULS_MID_FRSKY))/StateBLDC.MaxGas;// Расчет коэффициента для перерасчета значения GASa 
#elif USE_SBUS
StateBLDC.MaxGas = Autoreload_Tim1;
StateBLDC.k = ((SBUS_message_GASvalue_max)/(Autoreload_Tim1))-1;// Расчет коэффициента для перерасчета значения GASa по SBUS
#elif USE_FRSKY  
  StateBLDC.k = ((Spi_message_value_max)/(Autoreload_Tim1))-1;// Расчет коэффициента для перерасчета значения GASa по SPI 
#endif
  
#endif 
 
 while(1)
  {
    if (TIM6->CNT > MS_1) //прошла 1 ms
    {

      TIM6->CNT = 0;
      if (timer_read_adc > 0)
        timer_read_adc--;
      if (timer_read_spi > 0)
        timer_read_spi--;
    }
    
  //перегрузка по току
    if (ADC_ch10_index >= 10)
    {      
      ADC_ch10 = ADC_ch10_sum/ADC_ch10_index;
      ADC_ch10_index = 0;
      ADC_ch10_sum = 0;      
    }
    
     ADC_ch10_sum +=  ADC_GetConversionValue(ADC1);
     ADC_ch10_index++;
      
    if (0)//(ADC_ch10 > 1800)// 1000~ 5A; было 1800
    {
      GPIOB->BSRR = GPIO_BSRR_BS_12;
      Stop_BLCD();
      while(StateBLDC.gas > 50);
      GPIOB->BSRR = GPIO_BSRR_BR_12;
    }
//    else
//    {
//       GPIOB->BSRR = GPIO_BSRR_BR_12;
//    }

    
    //анализ вращения есть или нет
    if ( timer_read_adc == 0 )
     {
       timer_read_adc =  500;
      if ((*phasa_A + *phasa_B + *phasa_C) < 1)//(StateBLDC.gas < 50)
      { 
        StateBLDC.state = stop;
        //GPIOB->BSRR = GPIO_BSRR_BS_12;
        CalculateVDC();
      }
     }
     if ((StateBLDC.state == stop) && (StateBLDC.gas > 50))
     {
        //Start_BLDC();
        
        TIM2->ARR = 0xffffffff;
        TIM2->CCR1 = 0x5;
        TIM2->CNT = 0;
        StateBLDC.Cnt_start = 1000;
        StateBLDC.state = run;
        TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);

        //GPIOB->BSRR = GPIO_BSRR_BR_12;
     }
    
#ifndef FRSKY_USE
      // Считываем данные из SPI
    if ( timer_read_spi == 0 )
    {
      if ((StateBLDC.state == run) /*&& (StateBLDC.Cnt_start == CNT_START)*/)
        timer_read_spi = 5;
      else
       timer_read_spi = 100;  
      ReadSPI();

    }
#endif
      

  }
  
  
}

/*------------------------------------Stop-----------------------------------*/
void Stop_BLCD()
{
  //Отключаем прерывание для коммутации обмоток
    TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
    k1_close;
    k2_close;
    k3_close;
    k4_close;
    k5_close;
    k6_close;
    //GPIOB->BSRR = GPIO_BSRR_BS_12;//Зажигаем светодиод
    StateBLDC.state = stop;
    CalculateVDC();
}

/*------------------------------------Start----------------------------------*/
  void Start_BLDC()
  {
    
    StateBLDC.state = start;
    //Отключаем прерывание для коммутации обмоток
    TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
    k1_open;
    k2_close;
    k3_close;
    k4_close;
    k5_close;
    k6_close;
    StateBLDC.use_delta_gas=1;
   
//позиционируем ротор в положение 0
    StateBLDC.position = 5;

    StateBLDC.gas = 40;//60;
    TIM1->CCR1 = StateBLDC.gas;
    TIM1->CCR2 = StateBLDC.gas;
    TIM1->CCR3 = StateBLDC.gas;
    TIM2->CCR1 = 0x6ddd00;
    TIM2->CNT = 0;
    TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
    //Включаем прерывание для коммутации обмоток
    TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
    while ( StateBLDC.position == 5 );
    TIM_ITConfig(TIM2,TIM_IT_CC1,DISABLE);
    TIM2->ARR = 0xffffffff;
    TIM2->CCR1 = 0x112a880;
    TIM2->CNT = 0;
    TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
    while (TIM_GetFlagStatus(TIM2, TIM_FLAG_CC1) == RESET ); // ждем 250 ms
    k1_close;
//конец позиционирования ротора

    TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);//этого не было
    
     StateBLDC.gas = StateBLDC.start_gas_value;//100;//100;
     TIM1->CCR1 = StateBLDC.gas;
     TIM1->CCR2 = StateBLDC.gas;
     TIM1->CCR3 = StateBLDC.gas;
     TIM2->CCR1 = 350000;
     TIM2->CNT = 0;
     TIM_ClearFlag(TIM2, TIM_FLAG_CC1);
     TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
     uint8_t  tm,tm1 = 0;

     // увеличиваем обороты 
     for (int i = 1;i!=3;i++)//for (int i = 1;i!=15;i++)
//     {
//       tm =0;
//       tm1 =0;
//     while (tm != 3)
//     { 
//       tm1 = StateBLDC.position;
//       if ( StateBLDC.position == 0 )
//         tm++;
//       while ( tm1 == StateBLDC.position );
//     }
//      ReadSPI();
//      if (StateBLDC.gas < 50 )
//      {
//         Stop_BLCD();
//        return ;
//      }
//
//     if (( i > 2 )&&( i < 30 ))
//       TIM2->CCR1 = TIM2->CCR1 - 25000;
//    /* else if ( i < 50 )
//       TIM2->ARR =  TIM2->ARR - 5000;
//     else if ( i < 50 )
//       TIM2->ARR =  TIM2->ARR - 1800;
//     else if ( i < 60 )
//       TIM2->ARR =  TIM2->ARR - 1200;
//     else if ( i < 70 )
//       TIM2->ARR =  TIM2->ARR - 700;*/
//
//    }
    StateBLDC.Cnt_start = 0;
    StateBLDC.state = run;
  }


void delay()
{

}


uint8_t ReadSPI()
{
    if (( DMA_GetFlagStatus(DMA1_FLAG_TC2) == SET ) && 
         ( GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == Bit_SET ))
                                                        //если закончилась транзакция на прием
    { 
      //GPIOB->BSRR = GPIO_BSRR_BS_12;
      SPI_Cmd(SPI3, DISABLE);
      uint16_t data = SPI3->DR;
      data = SPI3->DR;
      data = SPI3->DR;
      data = SPI3->DR;
      DMA_Cmd(DMA1_Channel2, DISABLE);
      DMA_Cmd(DMA1_Channel3, DISABLE);
        
      gas_tm = BuffRxSpi3[0]/StateBLDC.k;   
      if ( gas_tm > StateBLDC.MaxGas )
        {
          StateBLDC.gas = StateBLDC.MaxGas;
        }
      else
      {
        StateBLDC.gas = gas_tm;
      }

      if ( StateBLDC.state == run )
        {
          if (StateBLDC.Cnt_start < CNT_START)////////////////////////////////////////
        {
          TIM1->CCR1 = GAS_5 ;
          TIM1->CCR2 = GAS_5 ;
          TIM1->CCR3 = GAS_5 ;
        }
      else 
         {
           if (StateBLDC.use_delta_gas)////////////////////////////////////////
           {
             uint32_t temp = TIM1->CCR1;
             if ( temp < StateBLDC.gas )
               {
                 if ( (StateBLDC.gas-temp) > DELTA_GAS) 
                    temp+=DELTA_GAS;
                 else
                    temp = StateBLDC.gas;
               }
             else if ( temp > StateBLDC.gas )
                    {
                      if ( (temp-StateBLDC.gas) > DELTA_GAS) 
                        temp-=DELTA_GAS;
                     else
                        temp = StateBLDC.gas;
                    }
              TIM1->CCR1 = temp;
              TIM1->CCR2 = temp;
              TIM1->CCR3 = temp;
           }
           else
           {
            TIM1->CCR1 = StateBLDC.gas;
            TIM1->CCR2 = StateBLDC.gas;
            TIM1->CCR3 = StateBLDC.gas;      
           }
         }
        }
      //формируем данные на передачу

      //запись данных в SPI
      BuffTxSpi3[0] = (uint16_t)StateBLDC.counter++;
      BuffTxSpi3[1] = (uint16_t)StateBLDC.time_sence<<1;//time_sence = время коммутации/2
      BuffTxSpi3[2] =  (uint16_t)ADC1->DR;//ток потребления
      
      
      DMA_ClearFlag( DMA1_FLAG_TC2 );
      DMA_ClearFlag( DMA1_FLAG_TC3 );
      DMA_SetCurrDataCounter(DMA1_Channel2, SIZE_BUFF_SPI_DMA);
      DMA_SetCurrDataCounter(DMA1_Channel3, SIZE_BUFF_SPI_DMA);
      
      
      //GPIOB->BSRR = GPIO_BSRR_BR_12;
      
      DMA_Cmd(DMA1_Channel2, ENABLE);
      DMA_Cmd(DMA1_Channel3, ENABLE);
      SPI_Cmd(SPI3, ENABLE);
      
      
      return 1;
    }

else return 0;

}

//определение напряжения питания
void CalculateVDC()
{
  uint16_t tm=0;
  uint16_t max_val_adc=0; 
  for (int i=0;i<16;i++)
  {
   ADC1->ISR = ADC_ISR_JEOS; //clear JEOC flag
   ADC1->CR |= ADC_CR_JADSTART; //start convertion
   while ((ADC1->ISR & ADC_ISR_JEOS) != ADC_ISR_JEOS); 
    tm += *Threshold_Vp_div2;
  }
  tm >>=4;
  
  if (((ADC1->CFGR>>3)&3) == 0)//12 bit resolution
    max_val_adc = 4095;
  else if (((ADC1->CFGR>>3)&3) == 1)//10 bit resolution 
    max_val_adc = 1023;
  else if (((ADC1->CFGR>>3)&3) == 2)//8 bit resolution 
  max_val_adc = 255;
    else if (((ADC1->CFGR>>3)&3) == 3)//6 bit resolution 
  max_val_adc = 63;
  
  StateBLDC.Vdc = (3.3*tm)/(K_Rdiv*max_val_adc); 
  
   //выбор значение gas для страрта
 if (StateBLDC.Vdc > 11)
   StateBLDC.start_gas_value = 2*Autoreload_Tim1/100;//2%
 else  if (StateBLDC.Vdc > 10)
   StateBLDC.start_gas_value = 3*Autoreload_Tim1/100;//3%  
 else 
   StateBLDC.start_gas_value = 6*Autoreload_Tim1/100;//6%   

}
