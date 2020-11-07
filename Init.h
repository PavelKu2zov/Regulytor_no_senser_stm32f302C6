//#define DBG                     
#define PCLK2                           72000000
#define PCLK1                           36000000//PCLK1 может быть максимум 36 MHz ref.manual page 115
#define PrescalerTim1                   1       //значение на которое делится Fclk
#define PrescalerTim2                   2
#define PrescalerTim6                   3600
#define PrescalerTim16                  3
#define Fpwm                            48000//36000                         // Задать частоту ШИМ в Hz 
#define Autoreload_Tim1                 (PCLK2)/(PrescalerTim1 * Fpwm)
#define GAS_5                           (Autoreload_Tim1*5)/100
#define Autoreload_Tim2                 0xffffffff
#define Autoreload_Tim6                 0xffff
#define Autoreload_Tim16                0xffff
#define Spi_message_value_max           0xffff
#define AWD_HighThreshold               39 //2 ампера for 12bit resolution ADC 
#define AWD_LowThreshold                0
#define OperegenieDefine                0x360                     
#define SIZE_BUFF_SPI_DMA               3
#define MS_1                            20
#define DELTA_GAS                       90 //4,5%; max приращение газа 4,5% с частотой 10 Hz для плавного старта
#define K_Rdiv                          (0.1176)//коэф. от делителя напряжения  

#define T_MAX_IMPULS_FRSKY              2000        //us
#define T_MID_IMPULS_FRSKY              1500        //us
#define T_MIN_IMPULS_FRSKY              1000        //us
#define IMPULS_MAX_FRSKY                (((PCLK2/1000000)/PrescalerTim16)*T_MAX_IMPULS_FRSKY)
#define IMPULS_MID_FRSKY                (((PCLK2/1000000)/PrescalerTim16)*T_MID_IMPULS_FRSKY)
#define IMPULS_MIN_FRSKY                (((PCLK2/1000000)/PrescalerTim16)*T_MIN_IMPULS_FRSKY)
#define USE_FRSKY

//ADC_1
#define phasa_A     ((uint16_t *)0x50000080) //ADC_JDR1   phasa_A PA0   ADC_Channel_1
#define phasa_B     ((uint16_t *)0x50000084) //ADC_JDR2   phasa_B PA1   ADC_Channel_2
#define phasa_C     ((uint16_t *)0x50000088) //ADC_JDR3   phasa_C PA3   ADC_Channel_4
#define Threshold_Vp_div2   ((uint16_t *)0x5000008C) //ADC_JDR4   Threshold PA4 ADC_Channel_5



#define ADC1_ISR    (*(uint32_t *)0x0x500000) 
#define ADC1_CR     (*(uint32_t *)0x0x500008)

#define SWSTART      ((uint32_t)0x00000004)
#define JADSTART     ((uint32_t)0x00000008)

//Timer1
//0x40012C00
#define TIM1_CNT   (*(uint32_t *)0x40012C24)
#define TIM1_SR    (*(uint32_t *)0x40012C10) 
#define TIM1_DIER  (*(uint32_t *)0x40012C0C)
#define TIM1_CCER  (*(uint32_t *)0x40012C20) 
#define TIM1_CCR1  (*(uint32_t *)0x40012C34)
#define TIM1_CCR2  (*(uint32_t *)0x40012C38)
#define TIM1_CCR3  (*(uint32_t *)0x40012C3C)
#define TIM1_CCR4  (*(uint32_t *)0x40012C40)

/*Timer 2*/
#define TIM2_CNT  (*(uint32_t *)0x40000024) 
#define TIM2_CCR1 (*(uint32_t *)0x40000034)
#define TIM2_ARR  (*(uint32_t *)0x4000002C)




#define k1_open   TIM1->CCER |= (1<<0);
#define k1_close  TIM1->CCER &=~(1<<0);
#define k2_open   TIM1->CCER |= (1<<4);
#define k2_close  TIM1->CCER &=~(1<<4);
#define k3_open   TIM1->CCER |= (1<<8);
#define k3_close  TIM1->CCER &=~(1<<8);
#define k4_close  GPIOB->BSRR = GPIO_BSRR_BR_1;
#define k5_close  GPIOB->BSRR = GPIO_BSRR_BR_7;
#define k6_close  GPIOB->BSRR = GPIO_BSRR_BR_6;
#define k4_open   GPIOB->BSRR = GPIO_BSRR_BS_1;
#define k5_open   GPIOB->BSRR = GPIO_BSRR_BS_7;
#define k6_open   GPIOB->BSRR = GPIO_BSRR_BS_6;

#define SIZE_BUFF_USART3_TX     100
#define CNT_START       650

enum _state
{
start,
run,
stop
};



struct _stateBLDC

{
  uint8_t       position; // номер позиции коммутации
  uint8_t       revers;// 1 - вращение в обратную сторону
  uint8_t       Wdt_com;//
  uint32_t      time_sence_mem[16];
  uint8_t       index_mem;
  uint8_t       use_delta_gas;
  uint32_t      time_sence;// время между нулями
  uint32_t      time_sence_start;
  uint32_t      gas;// величина ручки газа
  uint32_t      a;//значение с токового датчика
  uint16_t      counter;//циклический счетчик пакетов
  uint16_t      start_gas_value;
  float         Vdc;//напряжение питания
  //служебные параметры
  uint32_t       k;//коэффициент перерасчета значения GASa по SPI
  uint32_t      MaxGas;// максимальное значение счетчика TIM1
  uint8_t       SenceEnable;
  uint32_t      Operegenie;
  uint32_t      Cnt_start;
  enum _state       state;
};



void delay(void);
uint8_t ReadSPI();
void Stop_BLCD(void);
void  CalculateVDC(void);
