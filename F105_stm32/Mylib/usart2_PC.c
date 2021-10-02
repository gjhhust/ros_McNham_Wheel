//#include "main.h"

//char PC_receive[PC_RECEIVE_SIZE];
//char PC_send[PC_SEND_SIZE];
//extern TaskHandle_t PCReceiveTaskHandle;

//void USART2_Configuration(u32 bound)
//{
//	//GPIO�˿�����
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	DMA_InitTypeDef dma;
//	NVIC_InitTypeDef NVIC_InitStructure;
//  
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��ʱ��
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); 
//	
//	//USART2_TX   GPIOA.2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO

//	//USART2_RX	  GPIOA.3��ʼ��
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.3  

//	//Usart2 NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

//	//DMA1���� NVIC ����
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//	
//	//USART ��ʼ������

//	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

//	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
//	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//�������ڽ����ж�
//	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
//	USART_DMACmd(USART2,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
//	
//	dma.DMA_BufferSize=PC_RECEIVE_SIZE;
//	dma.DMA_DIR=DMA_DIR_PeripheralSRC;
//	dma.DMA_MemoryBaseAddr=PC_receive;
//	dma.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
//	dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
//	dma.DMA_Mode=DMA_Mode_Circular;
//	dma.DMA_PeripheralBaseAddr=&(USART2->DR);
//	dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
//	dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;
//  dma.DMA_Priority=DMA_Priority_High;
//	DMA_Init(DMA1_Channel6,&dma);
//	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
//	DMA_Cmd(DMA1_Channel6,ENABLE);
//	
//	dma.DMA_BufferSize=PC_SEND_SIZE;
//	dma.DMA_DIR=DMA_DIR_PeripheralDST;
//	dma.DMA_MemoryBaseAddr=PC_send;
//	dma.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
//	dma.DMA_MemoryInc=DMA_MemoryInc_Enable;
//	dma.DMA_Mode=DMA_Mode_Circular;
//	dma.DMA_PeripheralBaseAddr=&(USART2->DR);
//	dma.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
//	dma.DMA_PeripheralInc=DMA_PeripheralInc_Disable ;
//  dma.DMA_Priority=DMA_Priority_High;
//	DMA_Init(DMA1_Channel7,&dma);
//	DMA_Cmd(DMA1_Channel7,DISABLE);
//	
//}

//void USART2_IRQHandler()
//{
//  BaseType_t pxHigherPriorityTaskWoken;
//	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
// 	 {
//	   short len;
//		 DMA_Cmd(DMA1_Channel6,DISABLE);
//		 USART2->SR;
//		 USART2->DR;
//     vTaskNotifyGiveFromISR(PCReceiveTaskHandle,&pxHigherPriorityTaskWoken);
//		 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
//		 DMA_SetCurrDataCounter(DMA1_Channel6,PC_RECEIVE_SIZE);
//		 DMA_Cmd(DMA1_Channel6,ENABLE);
//	 }
//}

//void DMA1_Channel6_IRQHandler(void)
//{
//   BaseType_t pxHigherPriorityTaskWoken;
//  if(DMA_GetITStatus(DMA_IT_TC)==SET)
//	{
//		 DMA_Cmd(DMA1_Channel6,DISABLE);
//		 DMA_ClearITPendingBit(DMA_IT_TC);
//		//��������
//	   vTaskNotifyGiveFromISR(PCReceiveTaskHandle,&pxHigherPriorityTaskWoken);
//		 portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
//		 
//		 DMA_Cmd(DMA1_Channel6,ENABLE);	
//	}
//}

