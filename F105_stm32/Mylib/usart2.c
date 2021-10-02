#include "main.h"
char receive[100];
char data[100];
extern struct Angle angle;
void uart_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef dma;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); 
	
	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

	USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���1 
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	
	dma.DMA_BufferSize=100;
	dma.DMA_DIR=DMA_DIR_PeripheralSRC;
	dma.DMA_MemoryBaseAddr=receive;
	dma.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
	dma.DMA_Mode=DMA_Mode_Circular;
	dma.DMA_PeripheralBaseAddr=&(USART2->DR);
	dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;
  dma.DMA_Priority=DMA_Priority_High;
	DMA_Init(DMA1_Channel6,&dma);
	DMA_Cmd(DMA1_Channel6,ENABLE);
}
void USART2_IRQHandler()
{
  
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
 	 {
	   short len;
     USART_Cmd(USART2,DISABLE);
		 DMA_Cmd(DMA1_Channel6,DISABLE);
		 USART2->SR;
		 USART2->DR;
		 len=100-DMA_GetCurrDataCounter(DMA1_Channel6);
		//��������
		
		
		
		 DMA_SetCurrDataCounter(DMA1_Channel6,100);
		 USART_Cmd(USART2,ENABLE);
		 DMA_Cmd(DMA1_Channel6,ENABLE);
	 }
}