#ifndef DATA_H
#define DATA_H
#include "main.h"


/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/


#define START   0X11
//����λ
#define Reset 0x01
#define Stop  0x02


//�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
typedef union 
{
	short d;
	unsigned char data[2];
}sendData;

//�������ٿ����ٶȹ�����
typedef union
{
	short d;
	unsigned char data[2];
}receiveData;


//���Ƴ���ṹ��
typedef struct{
	int X_SpeedSet;//m/s
	int Y_SpeedSet;
	int W_SpeedSet;
	unsigned char crtlFlag;
}Ctrl_information;


//void DATA_DISPOSE_task(void *pvParameters);
	
//���ڽ����ж��н��պ���
void usartReceiveData(void);

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag);   


//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux   -32767 32767
extern void usartSendData(short x_speed, short y_speed,short w_speed,short angle);


//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize); 


//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 


void JudgeBuffReceive(unsigned char ReceiveBuffer[]);
#endif
