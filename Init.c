#include "stm32f30x.h"
#include "Init.h"
//#include "io_macros.h"
extern uint8_t BuffTxSpi3[SIZE_BUFF_SPI_DMA];
extern uint8_t BuffRxSpi3[SIZE_BUFF_SPI_DMA];

void Init()
{
  /* включение clock для переферии*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12,ENABLE);
  RCC_PCLK1Config(RCC_HCLK_Div2);//PCLK1 может быить максимум 36 MHz ref.manual page 115
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div4);//10
 
 
  /*----------------------конфигурация пинов----------------------------------*/
  
  /*X_Low ключи */
  GPIO_InitTypeDef GPIO_InitStructre;
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7 ;
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructre.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructre);
  
  /*X-High ключи в режиме ШИМ*/
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 ;
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &GPIO_InitStructre);
  
  /*Настройка альтернотивной функции A-High Tim_CHx*/
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_6);
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_6);
  
  /*Настройка пинов  АЦП для ОС и для Операционного усилителя */
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ;//GPIO_Pin_4 сгорел, вместо него GPIO_Pin_7
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructre.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructre);
  
  /*Нстройка PB14 как аналоговый для ОУ*/
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_14 ;
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructre.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructre);
  
  /*Настройка пинов USART3*/
//  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOB, &GPIO_InitStructre);
  
  /*Настройка пина для захвата импульса управления TIM16_CH1*/
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_8 ;
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructre);
  
//  /* Настройка альтернативной функции пинов PB8,PB9*/
//  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9, GPIO_AF_7); 
//  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_7);
  
    /* Настройка альтернативной функции пинов PB8*/
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource8, GPIO_AF_1);
  
  /*Настройка пинов SPI3*/
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_15;//SS
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructre);
  
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource15, GPIO_AF_6); 
  
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_5;//MOSI
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructre); 
 
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_3;//SCK
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStructre); 
  
  GPIO_InitStructre.GPIO_Pin = GPIO_Pin_4;//MISO
  GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructre. GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructre.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructre); 
 
  
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource3, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4, GPIO_AF_6);
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_6);
  
  /*Настройка PB12 на выход CurrentOverflow*/
  GPIO_InitStructre.GPIO_Pin=GPIO_Pin_12;
  GPIO_InitStructre.GPIO_Mode=GPIO_Mode_OUT;
  GPIO_InitStructre. GPIO_Speed=GPIO_Speed_10MHz;
  GPIO_InitStructre.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStructre);
  
  
  /*----------------------------Настройка USART3----------------------------*/
  USART_InitTypeDef USART_InitStruct;
  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength=USART_WordLength_8b;
  USART_InitStruct.USART_StopBits=USART_StopBits_1;
  USART_InitStruct.USART_Parity=USART_Parity_No;
  USART_InitStruct.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
  USART_Init(USART3,&USART_InitStruct);
  //USART_OverSampling8Cmd(USART3, ENABLE);
  
  /*----------------------------Настройка SPI_3------------------------------*/
  SPI_InitTypeDef SPI_InitStruct;
  SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStruct.SPI_Mode = SPI_Mode_Slave; 
  SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
  SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStruct.SPI_NSS = SPI_NSS_Hard;
  SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStruct.SPI_CRCPolynomial = 1;
  SPI_Init(SPI3, &SPI_InitStruct);
  
  //SPI_LastDMATransferCmd(SPI3, uint16_t SPI_LastDMATransfer);

  /*---------------------------Настройка Tim1--------------------------------*/
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseInitStruct.TIM_Prescaler = PrescalerTim1-1;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = Autoreload_Tim1;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
  
  TIM_OCInitTypeDef TIM_OCInitStruct;
  TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStruct.TIM_Pulse = 0;
  TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCPolarity_High; 
  TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set; 
  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Set; 
  TIM_OC1Init(TIM1, &TIM_OCInitStruct);
  TIM_OC2Init(TIM1, &TIM_OCInitStruct);
  TIM_OC3Init(TIM1, &TIM_OCInitStruct);
  
  TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
  TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
  TIM_OCInitStruct.TIM_OutputNState=TIM_OutputNState_Disable;
  TIM_OCInitStruct.TIM_Pulse=0;
  TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
  TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCNPolarity_High;
  TIM_OCInitStruct.TIM_OCIdleState=TIM_OCIdleState_Set;
  TIM_OCInitStruct.TIM_OCNIdleState=TIM_OCNIdleState_Set;
  TIM_OC4Init(TIM1, &TIM_OCInitStruct);
  
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  
  /*---------------------------Настройка Tim2--------------------------------*/
  /* по событию Tim2 (32 битный) генерируется коммутация обмоток двигателя, а так же 
    одновременно для измерения времени между переходами через ноль 
    источник частоты PCLK1 = 36000000 Mhz, Fclk_tim2 = PCLK1*2 = 72000000*/
  
  TIM_TimeBaseInitStruct.TIM_Prescaler =  PrescalerTim2-1 ;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = Autoreload_Tim2;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
  TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Disable);//Можно менять CCR1 не дожидаясь event
  TIM_ARRPreloadConfig(TIM2,DISABLE);
  
    /*---------------------------Настройка Tim6--------------------------------*/
 
  
  TIM_TimeBaseInitStruct.TIM_Prescaler =  PrescalerTim6-1;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = Autoreload_Tim6;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;  
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);
  TIM_ARRPreloadConfig(TIM6,DISABLE);
  
  /*---------------------------Настройка Tim16--------------------------------*/
  TIM_TimeBaseInitStruct.TIM_Prescaler = PrescalerTim16-1 ;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = Autoreload_Tim16;
  TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;  
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseInitStruct);
  TIM_ARRPreloadConfig(TIM16,DISABLE);
  
  TIM_ICInitTypeDef TIM_ICInitStruct;
  TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;                                 
  TIM_ICInitStruct.TIM_ICFilter = 0;
  TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
  TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI; 
  TIM_ICInit(TIM16,&TIM_ICInitStruct);

  TIM_ITConfig(TIM16, TIM_IT_CC1, DISABLE);
  
  
  /*--------------------------Настройка DMA1----------------------------------*/
  DMA_DeInit( DMA1_Channel2 );//SPI3_RX
  DMA_DeInit( DMA1_Channel3 );//SPI3_TX
  //Настройка DMA1 для SPI3_TX
  DMA_InitTypeDef DMA_InitStruct;
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;//адрес SPI3_DR
  DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&BuffTxSpi3;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStruct.DMA_BufferSize = SIZE_BUFF_SPI_DMA;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel3, &DMA_InitStruct);
  
  //Настройка DMA1 для SPI3_RX
  DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI3->DR;//адрес SPI3_DR
  DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&BuffRxSpi3;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStruct.DMA_BufferSize = SIZE_BUFF_SPI_DMA;
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh ;
  DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStruct);
  
  
  /*--------------------------Настройка OPAMP_2--------------------------------*/
  OPAMP_InitTypeDef OPAMP_InitStruct;
  OPAMP_InitStruct.OPAMP_InvertingInput = OPAMP_InvertingInput_PGA;//OPAMP_InvertingInput_IO2;//
  OPAMP_InitStruct.OPAMP_NonInvertingInput = OPAMP_NonInvertingInput_IO2;
  OPAMP_Init(OPAMP_Selection_OPAMP2, &OPAMP_InitStruct);
  OPAMP_PGAConfig(OPAMP_Selection_OPAMP2, OPAMP_OPAMP_PGAGain_16,OPAMP_PGAConnect_No);//OPAMP_PGAConnect_IO2
  
  /*--------------------------Настройка АЦП_1-------------------------------*/
  /*Общие настройки АЦП*/
  ADC_CommonInitTypeDef ADC_CommonInitStruct;
  ADC_CommonInitStruct.ADC_Clock = ADC_Clock_AsynClkMode; //HCLK/4
  ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStruct.ADC_DMAMode = ADC_DMAMode_OneShot;
  ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStruct.ADC_TwoSamplingDelay = 0;
  ADC_CommonInit(ADC1, &ADC_CommonInitStruct);
  
  ADC_InitTypeDef  ADC_InitStruct;
  ADC_InitStruct.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;// беспрерывное преобразование, только для регулярного канала
  ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;//ADC_Resolution_8b;//;//при смене resolution надо изменить коэф. для определения напряжение питания
  ADC_InitStruct.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
  ADC_InitStruct.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStruct.ADC_OverrunMode = ADC_OverrunMode_Enable;
  ADC_InitStruct.ADC_AutoInjMode = ADC_AutoInjec_Disable;// Injchanel при enable автоматически будет делать преобразование сразу после преобразования регул.канала
  ADC_InitStruct.ADC_NbrOfRegChannel = 1;
  ADC_Init(ADC1, &ADC_InitStruct);
 
  ADC_InjectedInitTypeDef ADC_InjectedInitStruct;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecConvEvent = ADC_ExternalTrigInjecConvEvent_0;
  ADC_InjectedInitStruct.ADC_ExternalTrigInjecEventEdge = ADC_ExternalTrigInjecEventEdge_None;
  ADC_InjectedInitStruct.ADC_NbrOfInjecChannel = 4;
  ADC_InjectedInitStruct.ADC_InjecSequence1 = ADC_InjectedChannel_1;//PA0 1
  ADC_InjectedInitStruct.ADC_InjecSequence2 = ADC_InjectedChannel_2;//PA1
  ADC_InjectedInitStruct.ADC_InjecSequence3 = ADC_InjectedChannel_4;//PA3
  ADC_InjectedInitStruct.ADC_InjecSequence4 = ADC_InjectedChannel_5;//PA4  5 было ADC_InjectedChannel_5
  ADC_InjectedInit(ADC1, &ADC_InjectedInitStruct); 
  
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_InjectedChannel_1,ADC_SampleTime_7Cycles5);
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_InjectedChannel_2,ADC_SampleTime_7Cycles5);
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_InjectedChannel_4,ADC_SampleTime_7Cycles5);
  ADC_InjectedChannelSampleTimeConfig(ADC1, ADC_InjectedChannel_5,ADC_SampleTime_7Cycles5);//было ADC_InjectedChannel_5
  
  ADC_RegularChannelConfig(ADC1,ADC_Channel_10, 1,  ADC_SampleTime_7Cycles5  );// Rank = 1 для OPAMP2
  ADC_RegularChannelSequencerLengthConfig(ADC1, 1); //SequencerLength = 1
  
  
  
  /*-----------------------Настройка ADC_AnalogWatchdog-----------------------*/
  ADC_AnalogWatchdog1SingleChannelConfig(ADC1, ADC_Channel_10);
  ADC_AnalogWatchdog1ThresholdsConfig(ADC1, AWD_HighThreshold, AWD_LowThreshold);

  
  /*Настройка прерываний*/
  
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = ADC1_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM1_UP_TIM16_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  
  TIM_Cmd(TIM1,ENABLE);
  TIM_Cmd(TIM2,ENABLE);
  TIM_Cmd(TIM6,ENABLE);
  TIM_Cmd(TIM16,ENABLE);

    DMA_Cmd(DMA1_Channel3, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);  
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    SPI_Cmd(SPI3, ENABLE);
  
  USART_Cmd(USART3, ENABLE);
  ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
  
  
  //Калибровка ADC_1
  //ADC_DisableCmd(ADC1); 
  ADC_VoltageRegulatorCmd(ADC1, ENABLE);
  TIM2->CNT = 0;
  TIM2->ARR = 0xFFFFFFFF;
  while (TIM2->CNT <0xafc80); //Ожидаем 10 ms
  ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
  ADC_StartCalibration(ADC1);
  while ( ADC_GetCalibrationStatus(ADC1) != RESET );
  TIM2->CNT = 0;
  TIM2->ARR = 0xFFFFFFFF;
  while (TIM2->CNT <0xafc80); //Ожидаем 10 ms
  ADC_Cmd(ADC1, ENABLE);
   TIM2->CNT = 0;
  TIM2->ARR = 0xFFFFFFFF;
  while (TIM2->CNT <0xafc80); //Ожидаем 10 ms
  ADC_StartConversion(ADC1);
  
  
  
  //Калибровка OPAMP_2
  //Calibrate the NMOS differential pair
  OPAMP_Cmd(OPAMP_Selection_OPAMP2, ENABLE);//вкл. OPAMP_2
//  OPAMP_OffsetTrimModeSelect(OPAMP_Selection_OPAMP2,OPAMP_Trimming_User);//Enable the user offset trimming by setting the USERTRIM bit
//  OPAMP_StartCalibration(OPAMP_Selection_OPAMP2,ENABLE);//Connect VM and VP to the internal reference voltage
//  OPAMP_VrefConfig(OPAMP_Selection_OPAMP2, OPAMP_Vref_90VDDA);//OPAMP internal reference =0.9 x VDDA
//  OPAMP_OffsetTrimConfig(OPAMP_Selection_OPAMP2, OPAMP_Input_Inverting, 0);//TRIMOFFSETN bits set zero
//  TIM2->CNT = 0;
//  TIM2->ARR = 0xFFFFFFFF;
//  TIM2->CNT = 0;
//  while (TIM2->CNT < 0xafc80); //Ожидаем 10 ms 
//  uint32_t tm =0;
//  while ( OPAMP_GetOutputLevel(OPAMP_Selection_OPAMP2) != OPAMP_OutputLevel_Low )
//  {
//    tm++;
//    OPAMP_OffsetTrimConfig(OPAMP_Selection_OPAMP2, OPAMP_Input_Inverting, (tm & 0x1f));
//    TIM2->CNT = 0;
//    while (TIM2->CNT < 0xafc80); //Ожидаем 10 ms 
//  }
//  
//  //Calibrate the PMOS differential pair
//  OPAMP_VrefConfig(OPAMP_Selection_OPAMP2, OPAMP_Vref_10VDDA);//OPAMP internal reference =0.9 x VDDA
//  tm =0;
// 
//  OPAMP_OffsetTrimConfig(OPAMP_Selection_OPAMP2, OPAMP_Input_NonInverting, 0);//TRIMOFFSETN bits set zero
//  TIM2->CNT = 0;
//  while (TIM2->CNT < 0xafc80); //Ожидаем 10 ms 
//  while ( OPAMP_GetOutputLevel(OPAMP_Selection_OPAMP2) != OPAMP_OutputLevel_Low )
//  {
//    tm++;  
//    OPAMP_OffsetTrimConfig(OPAMP_Selection_OPAMP2, OPAMP_Input_NonInverting , (tm & 0x1f));
//    TIM2->CNT = 0;
//    while (TIM2->CNT < 0xafc80); //Ожидаем 10 ms
//  }
//  
//  OPAMP_StartCalibration(OPAMP_Selection_OPAMP2,DISABLE);//calibration mode disabled
//  ADC_ClearFlag(ADC1, ADC_IT_AWD1); 
    //ADC_ITConfig(ADC1, ADC_IT_AWD1, ENABLE);
}



