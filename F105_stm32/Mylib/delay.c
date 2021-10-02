#include "main.h"
#include "delay.h"

int delayus_cnt;
int delayms_cnt;
/**
  * @brief  ����Tim6��ʱ������΢����ʱ
  * @param  None
  * @retval None
  */
void TIM6_Configration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 

    TIM_TimeBaseInitStructure.TIM_Period = 1; 	
    TIM_TimeBaseInitStructure.TIM_Prescaler = 35;  
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
		
		TIM_ARRPreloadConfig(TIM6,ENABLE);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM6, DISABLE); 
}

/**
  * @brief  TIM6��ʱ�ж�ִ������
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)
	{
        delayus_cnt--;
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
}

void delay_us(int tim) 
{
    delayus_cnt = tim;
    TIM_Cmd(TIM6, ENABLE);
    while(delayus_cnt > 0);
    TIM_Cmd(TIM6, DISABLE);
}

/**
  * @brief  ����Tim7��ʱ������������ʱ��׼ȷ��
  * @param  None
  * @retval None
  */

void TIM7_Configration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 

    TIM_TimeBaseInitStructure.TIM_Period = 3; 	
    TIM_TimeBaseInitStructure.TIM_Prescaler = 17999;  
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; 

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);
    TIM_ARRPreloadConfig(TIM7,ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM7, DISABLE); 
}

/**
  * @brief  TIM7��ʱ�ж�ִ������
  * @param  None
  * @retval None
  */
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)
	{
        delayms_cnt--;
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
}

void delay_ms(int tim)
{
    delayms_cnt = tim;
    TIM_Cmd(TIM7, ENABLE);
    while(delayms_cnt > 0);
    TIM_Cmd(TIM7, DISABLE);
}
