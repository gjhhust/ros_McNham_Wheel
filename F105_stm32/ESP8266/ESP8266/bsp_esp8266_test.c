#include "main.h"


#define LED_CMD_NUMBER   3
char *ledCmd[LED_CMD_NUMBER] = { "LED1_ON","LED2_ON","LED_ALLOFF" };

volatile uint8_t ucTcpClosedFlag = 0;
char contol=0;

/**
  * @brief  ��ȡ����������ֺʹ��ڵ������ַ�������Ϣ
  * @param  ��
  * @retval ��
  */
//void Get_ESP82666_Cmd( char * cmd)
//{
//	uint8_t i;
//	for(i = 0;i < LED_CMD_NUMBER; i++)
//	{
//     if(( bool ) strstr ( cmd, ledCmd[i] ))
//		 break;
//	}
//	switch(i)
//    {
//      case 0:
//        LED1_ON;
//        break;
//      
//      case 1:
//        LED2_ON;
//        break;
//      
//      case 2:
//        LED1_OFF
//        LED2_OFF;
//        break;
//      
//      default:
//        break;      
//    }   
//}


/**
  * @brief  ESP8266 StaTcpClient Unvarnish ���ò��Ժ���
  * @param  ��
  * @retval ��
  */
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void)
{
//  printf( "\r\n�������� ESP8266 ......\r\n" );
//  printf( "\r\nʹ�� ESP8266 ......\r\n" );
	macESP8266_CH_ENABLE();
	while( ! ESP8266_AT_Test() );
  
//  printf( "\r\n�������ù���ģʽ STA ......\r\n" );
	while( ! ESP8266_Net_Mode_Choose ( STA ) );

//  printf( "\r\n�������� WiFi ......\r\n" );
  while( ! ESP8266_JoinAP ( macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd ) );	
	
//  printf( "\r\n��ֹ������ ......\r\n" );
	while( ! ESP8266_Enable_MultipleId ( DISABLE ) );
	
//  printf( "\r\n�������� Server ......\r\n" );
	while( !	ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );
	
//  printf( "\r\n����͸������ģʽ ......\r\n" );
	while( ! ESP8266_UnvarnishSend () );
	
//	printf( "\r\n���� ESP8266 ���\r\n" );
//	printf ( "\r\n��ʼ͸��......\r\n" );
  
}


/**
  * @brief  ESP8266 ����Ƿ���յ������ݣ�������Ӻ͵�������
  * @param  ��
  * @retval ��
  */
void ESP8266_CheckRecvDataTest(void)
{
  uint8_t ucStatus;
  uint16_t i;
  
//  /* ������յ��˴��ڵ������ֵ����� */
//  if(strUSART_Fram_Record.InfBit.FramFinishFlag == 1)
//  {
//    for(i = 0;i < strUSART_Fram_Record.InfBit.FramLength; i++)
//    {
//       USART_SendData( macESP8266_USARTx ,strUSART_Fram_Record.Data_RX_BUF[i]); //ת����ESP82636
//       while(USART_GetFlagStatus(macESP8266_USARTx,USART_FLAG_TC)==RESET){}      //�ȴ��������
//    }
//    strUSART_Fram_Record .InfBit .FramLength = 0;                                //�������ݳ�������
//    strUSART_Fram_Record .InfBit .FramFinishFlag = 0;                            //���ձ�־����
//  }
  
  /* ������յ���ESP8266������ */
  if(strEsp8266_Fram_Record.InfBit.FramFinishFlag)
  {                                                      
//		for(i = 0;i < strEsp8266_Fram_Record .InfBit .FramLength; i++)               
//		{
//			 while(USART_GetFlagStatus(DEBUG_USARTx,USART_FLAG_TC)==RESET){}
//		} //ת����USART1
		 contol = strEsp8266_Fram_Record.Data_RX_BUF[0];
     strEsp8266_Fram_Record .InfBit .FramLength = 0;                             //�������ݳ�������
     strEsp8266_Fram_Record.InfBit.FramFinishFlag = 0;                           //���ձ�־����

  }
  if ( ucTcpClosedFlag )                                             //����Ƿ�ʧȥ����
  {
    ESP8266_ExitUnvarnishSend ();                                    //�˳�͸��ģʽ
    
    do ucStatus = ESP8266_Get_LinkStatus ();                         //��ȡ����״̬
    while ( ! ucStatus );
    
    if ( ucStatus == 4 )                                             //ȷ��ʧȥ���Ӻ�����
    {
//      printf ( "\r\n���������ȵ�ͷ����� ......\r\n" );
      
      while ( ! ESP8266_JoinAP ( macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd ) );
      
      while ( !	ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );
      
//      printf ( "\r\n�����ȵ�ͷ������ɹ�\r\n" );

    }
    
    while ( ! ESP8266_UnvarnishSend () );		
    
  }
}

void ESP8266_SendInf(void)
{
	char data[] ="ab2323";
	ESP8266_SendString ( ENABLE, data, 6, Multiple_ID_0 );
}
