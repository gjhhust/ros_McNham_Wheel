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
		
		nvic.NVIC_IRQChannel=CAN1_RX0_IRQn;  //�ж����ã�ָ���ж�����
		nvic.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
		nvic.NVIC_IRQChannelSubPriority=0;//��Ӧ���ȼ�0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//ʹ���ж�
		NVIC_Init(&nvic);//��ʼ��
		
		nvic.NVIC_IRQChannel=CAN1_RX1_IRQn;  //�ж����ã�ָ���ж�����
		nvic.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
		nvic.NVIC_IRQChannelSubPriority=0;//��Ӧ���ȼ�Ϊ0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//ʹ���ж�
		NVIC_Init(&nvic);//��ʼ��
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//��ʱ��can
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//��ʱ��
		
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
		can.CAN_ABOM=ENABLE;//��⵽��Ϣ��can�����Զ��˳��ر�״̬
		can.CAN_AWUM=ENABLE;//��⵽can��Ϣ���Զ��˳�˯��״̬
		can.CAN_BS1=CAN_BS1_5tq;
		can.CAN_BS2=CAN_BS2_3tq;
		can.CAN_Mode=CAN_Mode_Normal;//����ģʽ
		can.CAN_NART=DISABLE;//����ʱ�Զ��ط�����Ϣ
		can.CAN_Prescaler=4;  //�����ʼ��� 36/3/(1+3+8)=1M
		can.CAN_RFLM=DISABLE;//����FIFO����󲻻���������
		can.CAN_SJW=CAN_SJW_1tq;
		can.CAN_TTCM=DISABLE;//��ֹʱ�䴥��ͨ��ģʽ
		can.CAN_TXFP=ENABLE;//�������ȼ�������˳�����
		CAN_Init(CAN1,&can);
		
		
		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//Ϊ�˹��������յ�����Ϣ����FIFO
		can_filter.CAN_FilterScale=CAN_FilterScale_16bit;//һ���ĸ�������Ϊ16λ
		can_filter.CAN_FilterMode=CAN_FilterMode_IdList;//ÿλ�����ϸ�һ��
		can_filter.CAN_FilterIdLow=0x201<<5 ;//����ȫΪ0��չ֡
		can_filter.CAN_FilterIdHigh=0x202<<5;//��Ҫ�ı�ʶ��
		can_filter.CAN_FilterMaskIdHigh=0x203<<5;//����ȫΪ0��׼֡
		can_filter.CAN_FilterMaskIdLow=0x204<<5 ;//����ȫΪ0�ı�׼֡
		can_filter.CAN_FilterActivation=ENABLE; //ʹ�ܴ˹�����
		CAN_FilterInit(&can_filter);//��ʼ��
		
		CAN_ITConfig(CAN1,CAN_IT_FMP1|CAN_IT_FMP0,ENABLE);
                                             //CAN1��ʼ�����ã�ÿһ�����ö�Ҫд�϶�Ӧ��ע��
}
void CAN2_Configuration(void)
{
	  CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;
		
		nvic.NVIC_IRQChannel=CAN2_RX0_IRQn;  //�ж����ã�ָ���ж�����
		nvic.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�0
		nvic.NVIC_IRQChannelSubPriority=0;//��Ӧ���ȼ�0
	  nvic.NVIC_IRQChannelCmd=ENABLE;//ʹ���ж�
		NVIC_Init(&nvic);//��ʼ��
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);//��ʱ��can
		
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
		can.CAN_ABOM=ENABLE;//��⵽��Ϣ��can�����Զ��˳��ر�״̬
		can.CAN_AWUM=ENABLE;//��⵽can��Ϣ���Զ��˳�˯��״̬
		can.CAN_BS1=CAN_BS1_5tq;
		can.CAN_BS2=CAN_BS2_3tq;
		can.CAN_Mode=CAN_Mode_Normal;//����ģʽ
		can.CAN_NART=DISABLE;//����ʱ�Զ��ط�����Ϣ
		can.CAN_Prescaler=4;  //�����ʼ��� 36/3/(1+3+8)=1M
		can.CAN_RFLM=DISABLE;//����FIFO����󲻻���������
		can.CAN_SJW=CAN_SJW_1tq;
		can.CAN_TTCM=DISABLE;//��ֹʱ�䴥��ͨ��ģʽ
		can.CAN_TXFP=ENABLE;//�������ȼ�������˳�����
		CAN_Init(CAN2,&can);
			
		can_filter.CAN_FilterNumber=15;
		can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//Ϊ�˹��������յ�����Ϣ����FIFO
		can_filter.CAN_FilterScale=CAN_FilterScale_16bit;//һ���ĸ�������Ϊ16λ
		can_filter.CAN_FilterMode=CAN_FilterMode_IdList;//ÿλ�����ϸ�һ��
		can_filter.CAN_FilterIdLow=0x101<<5 ;//����ȫΪ0��չ֡
		can_filter.CAN_FilterIdHigh=0x100<<5;//��Ҫ�ı�ʶ��
		can_filter.CAN_FilterMaskIdHigh=0x000<<5;//����ȫΪ0��׼֡
		can_filter.CAN_FilterMaskIdLow=0x000<<5 ;//����ȫΪ0�ı�׼֡
		can_filter.CAN_FilterActivation=ENABLE; //ʹ�ܴ˹�����
		CAN_FilterInit(&can_filter);//��ʼ��
		
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
                                             //CAN1��ʼ�����ã�ÿһ�����ö�Ҫд�϶�Ӧ��ע��
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
	 NowAngleLeftFront[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]);//ǰ������
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
	 NowAngleRightFront[0]=(short)(rx0->Data[0]<<8|rx0->Data[1]); //ǰ������
	 break;
	 
	 case 0x101:
	 memcpy(&F105.YawAngle,rx0->Data,4);
   memcpy(&F105.YawAngleSpeed,(rx0->Data)+4,4);
	 break;
	 }

}



