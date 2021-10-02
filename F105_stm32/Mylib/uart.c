#include "usart.h"



unsigned char JudgeReceiveBuffer[reBiggestSize*2];//接受最大内存
unsigned char JudgeSend[SendBiggestSize];//发送最大内存



/**********************************************************************************************************
*函 数 名: UART2_DMA_Init
*功能说明: uart2初始化(速度收发)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void USART2_Configuration(u32 bound)
{
//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); 
	
	//USART2_TX   GPIOA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO

	//USART2_RX	  GPIOA.3初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  

//	//Usart2 NVIC 配置
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 3;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器


	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口2 
	USART_DMACmd(USART2,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
	
	
	//RX DMA1 6通道
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel6);

		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)JudgeReceiveBuffer;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = reBiggestSize;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel6, &dma);
		DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);
	}
  
	//TX DMA1 7通道
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
	
		USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
		{
		  DMA_InitTypeDef 	dma;
			DMA_DeInit(DMA1_Channel7);
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
			dma.DMA_MemoryBaseAddr = (uint32_t)JudgeSend;
			dma.DMA_DIR = DMA_DIR_PeripheralDST;
			dma.DMA_BufferSize = SendBiggestSize;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Normal;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_M2M = DMA_M2M_Disable;

			DMA_Init(DMA1_Channel7,&dma);
			DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
			DMA_ITConfig(DMA1_Channel7, DMA1_FLAG_GL7,ENABLE);
			DMA_Cmd(DMA1_Channel7,DISABLE);
	 }		
}


//rx满中断
void DMA1_Channel6_IRQHandler(void)
{	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC6) == SET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);
		//接收处理函数
		JudgeBuffReceive(JudgeReceiveBuffer);
	}
}

//Tx满中断
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC7)!=RESET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC7 | DMA1_FLAG_GL7);
		DMA_ClearITPendingBit(DMA1_IT_TC7 | DMA1_FLAG_GL7);
		
		DMA_Cmd(DMA1_Channel7,DISABLE);
	}	
}
