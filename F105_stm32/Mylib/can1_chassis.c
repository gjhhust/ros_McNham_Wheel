#include "main.h"

CanRxMsg rx0,rx1;
extern chassis F105;
short NowAngleLeftFront[2],NowAngleRightFront[2],NowAngleLeftBack[2],NowAngleRightBack[2];
void CAN1_Configuration(void)
{
	  CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
		
		nvic.NVIC_IRQChannel=CAN1_RX0_IRQn;  //中断配置，指定中断种类
		nvic.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
		nvic.NVIC_IRQChannelSubPriority=0;//响应优先级0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//使能中断
		NVIC_Init(&nvic);//初始化
		
		nvic.NVIC_IRQChannel=CAN1_RX1_IRQn;  //中断配置，指定中断种类
		nvic.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
		nvic.NVIC_IRQChannelSubPriority=0;//响应优先级为0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//使能中断
		NVIC_Init(&nvic);//初始化
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//开时钟can
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//开时钟
		
    GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap


		gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		gpio.GPIO_Speed=GPIO_Speed_50MHz;
		gpio.GPIO_Pin=GPIO_Pin_8;
		GPIO_Init(GPIOB,&gpio);
		
		gpio.GPIO_Mode=GPIO_Mode_AF_PP;
		gpio.GPIO_Speed=GPIO_Speed_50MHz;
		gpio.GPIO_Pin=GPIO_Pin_9;
		GPIO_Init(GPIOB,&gpio);
		
		CAN_DeInit(CAN1);
    CAN_StructInit(&can); 	
		can.CAN_ABOM=ENABLE;//检测到信息后can总线自动退出关闭状态
		can.CAN_AWUM=ENABLE;//检测到can信息后，自动退出睡眠状态
		can.CAN_BS1=CAN_BS1_5tq;
		can.CAN_BS2=CAN_BS2_3tq;
		can.CAN_Mode=CAN_Mode_Normal;//正常模式
		can.CAN_NART=DISABLE;//错误时自动重发送信息
		can.CAN_Prescaler=4;  //波特率计算 36/3/(1+3+8)=1M
		can.CAN_RFLM=DISABLE;//接收FIFO溢出后不会锁定报文
		can.CAN_SJW=CAN_SJW_1tq;
		can.CAN_TTCM=DISABLE;//禁止时间触发通信模式
		can.CAN_TXFP=ENABLE;//发送优先级由请求顺序决定
		CAN_Init(CAN1,&can);
		
		
		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//为此过滤器接收到的信息分配FIFO
		can_filter.CAN_FilterScale=CAN_FilterScale_16bit;//一组四个过滤器为16位
		can_filter.CAN_FilterMode=CAN_FilterMode_IdList;//每位必须严格一致
		can_filter.CAN_FilterIdLow=0x201<<5 ;//接收全为0标展帧
		can_filter.CAN_FilterIdHigh=0x202<<5;//需要的标识符
		can_filter.CAN_FilterMaskIdHigh=0x203<<5;//接收全为0标准帧
		can_filter.CAN_FilterMaskIdLow=0x204<<5 ;//接收全为0的标准帧
		can_filter.CAN_FilterActivation=ENABLE; //使能此过滤器
		CAN_FilterInit(&can_filter);//初始化
		
		CAN_ITConfig(CAN1,CAN_IT_FMP1|CAN_IT_FMP0,ENABLE);
                                             //CAN1初始化配置，每一行配置都要写上对应的注释
}
void CAN2_Configuration(void)
{
	  CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
		
		nvic.NVIC_IRQChannel=CAN2_RX0_IRQn;  //中断配置，指定中断种类
		nvic.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级0
		nvic.NVIC_IRQChannelSubPriority=0;//响应优先级0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//使能中断
		NVIC_Init(&nvic);//初始化
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);//开时钟can
		
		gpio.GPIO_Mode=GPIO_Mode_IN_FLOATING;
		gpio.GPIO_Speed=GPIO_Speed_50MHz;
		gpio.GPIO_Pin=GPIO_Pin_12;
		GPIO_Init(GPIOB,&gpio);
		
		gpio.GPIO_Mode=GPIO_Mode_AF_PP;
		gpio.GPIO_Speed=GPIO_Speed_50MHz;
		gpio.GPIO_Pin=GPIO_Pin_13;
		GPIO_Init(GPIOB,&gpio);
		
		CAN_DeInit(CAN2);
    CAN_StructInit(&can); 	
		can.CAN_ABOM=ENABLE;//检测到信息后can总线自动退出关闭状态
		can.CAN_AWUM=ENABLE;//检测到can信息后，自动退出睡眠状态
		can.CAN_BS1=CAN_BS1_5tq;
		can.CAN_BS2=CAN_BS2_3tq;
		can.CAN_Mode=CAN_Mode_Normal;//正常模式
		can.CAN_NART=DISABLE;//错误时自动重发送信息
		can.CAN_Prescaler=4;  //波特率计算 36/3/(1+3+8)=1M
		can.CAN_RFLM=DISABLE;//接收FIFO溢出后不会锁定报文
		can.CAN_SJW=CAN_SJW_1tq;
		can.CAN_TTCM=DISABLE;//禁止时间触发通信模式
		can.CAN_TXFP=ENABLE;//发送优先级由请求顺序决定
		CAN_Init(CAN2,&can);
			
		can_filter.CAN_FilterNumber=15;
		can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//为此过滤器接收到的信息分配FIFO
		can_filter.CAN_FilterScale=CAN_FilterScale_16bit;//一组四个过滤器为16位
		can_filter.CAN_FilterMode=CAN_FilterMode_IdList;//每位必须严格一致
		can_filter.CAN_FilterIdLow=0x101<<5 ;//接收全为0标展帧
		can_filter.CAN_FilterIdHigh=0x100<<5;//需要的标识符
		can_filter.CAN_FilterMaskIdHigh=0x000<<5;//接收全为0标准帧
		can_filter.CAN_FilterMaskIdLow=0x000<<5 ;//接收全为0的标准帧
		can_filter.CAN_FilterActivation=ENABLE; //使能此过滤器
		CAN_FilterInit(&can_filter);//初始化
		
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
                                             //CAN1初始化配置，每一行配置都要写上对应的注释
}

void CAN1_RX0_IRQHandler(void)
{
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)==SET)
	{
	 CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	 CAN_Receive(CAN1,CAN_FIFO0,&rx0);
   CAN_DataReceive(&rx0);
	 	 
	}
}


void CAN2_RX0_IRQHandler(void)
{
  if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)==SET)
	{
	 CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
	 CAN_Receive(CAN2,CAN_FIFO0,&rx1);
	 CAN_DataReceive(&rx1);	 
	}

}

void CAN_DataReceive(CanRxMsg* rx0)
{
   switch(rx0->StdId)
	 {
	 case 0x201:
	 F105.PIDwheel[0].RealPoint=(short)(rx0->Data[2]<<8|rx0->Data[3]);
	 NowAngleLeftFront[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]);//前进增加
	 break;
	 
	 case 0x202:
	 F105.PIDwheel[1].RealPoint=(short)(rx0->Data[2]<<8|rx0->Data[3]);
	 NowAngleLeftBack[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]);
	 break;
	 
	 case 0x203:
	 F105.PIDwheel[2].RealPoint=(short)(rx0->Data[2]<<8|rx0->Data[3]);
	 NowAngleRightBack[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]);
	 break;
	 
	 case 0x204:
	 F105.PIDwheel[3].RealPoint=(short)(rx0->Data[2]<<8|rx0->Data[3]);
	 NowAngleRightFront[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]); //前进减少
	 break;
	 
	 case 0x101:
	 memcpy(&F105.YawAngle,rx0->Data,4);
   memcpy(&F105.YawAngleSpeed,(rx0->Data)+4,4);
	 break;
	 }

}



