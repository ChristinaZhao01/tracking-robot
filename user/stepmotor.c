#include "stm32f10x.h"
#include "stepmotor.h"

/*************************************************
  stepmotor_init(void)
**************************************************/
void stepmotor_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);

  //PD3-->AEN(低电平有效) PD6-->AMS1 PD5-->AMS2 PD4-->AMS3 PD7-->ADIR
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  //PE15-->BEN(低电平有效) PE14-->BMS1 PE13-->BMS2 PE12-->BMS3  PE11-->BDIR
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_ResetBits(GPIOD, GPIO_Pin_5|GPIO_Pin_6);    // AMS1 AMS2  Set HighLevel 4W1-2 Phase
  GPIO_SetBits(GPIOD, GPIO_Pin_4);             // AMS3
  GPIO_ResetBits(GPIOD, GPIO_Pin_3);                        // AEN=LOW Enable output
  GPIO_SetBits(GPIOD, GPIO_Pin_7);                          // ADIR 

  GPIO_ResetBits(GPIOE, GPIO_Pin_13|GPIO_Pin_14); // BMS1 BMS2  Set HighLevel 4W1-2 Phase
  GPIO_SetBits(GPIOE, GPIO_Pin_12);           // BMS3
  GPIO_ResetBits(GPIOE, GPIO_Pin_15);                       // BEN=LOW Enable output
  GPIO_SetBits(GPIOE, GPIO_Pin_11);                         // BDIR 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;                 //PB8 -->ASTEP  TIM4_CH3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;           
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;                 //PC6 -->BSTEP  TIM8_CH1 Remap -->TIM3_CH1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 7200 - 1;  // (Period + 1) * (Prescaler + 1) / 72M = 1ms  
  TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 3600;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);                

  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);           
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 7200 - 1;    
  TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1;  
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_Pulse = 3600;

  TIM_OC1Init(TIM3, &TIM_OCInitStructure);                    

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM3, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);  
}

/***************************END OF FILE**********************************************************************/








