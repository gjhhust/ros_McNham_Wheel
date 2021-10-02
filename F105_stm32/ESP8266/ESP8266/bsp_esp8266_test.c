#include "main.h"


#define LED_CMD_NUMBER   3
char *ledCmd[LED_CMD_NUMBER] = { "LED1_ON","LED2_ON","LED_ALLOFF" };

volatile uint8_t ucTcpClosedFlag = 0;
char contol=0;

/**
  * @brief  获取网络调试助手和串口调试助手发来的信息
  * @param  无
  * @retval 无
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
  * @brief  ESP8266 StaTcpClient Unvarnish 配置测试函数
  * @param  无
  * @retval 无
  */
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void)
{
//  printf( "\r\n正在配置 ESP8266 ......\r\n" );
//  printf( "\r\n使能 ESP8266 ......\r\n" );
	macESP8266_CH_ENABLE();
	while( ! ESP8266_AT_Test() );
  
//  printf( "\r\n正在配置工作模式 STA ......\r\n" );
	while( ! ESP8266_Net_Mode_Choose ( STA ) );

//  printf( "\r\n正在连接 WiFi ......\r\n" );
  while( ! ESP8266_JoinAP ( macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd ) );	
	
//  printf( "\r\n禁止多连接 ......\r\n" );
	while( ! ESP8266_Enable_MultipleId ( DISABLE ) );
	
//  printf( "\r\n正在连接 Server ......\r\n" );
	while( !	ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );
	
//  printf( "\r\n进入透传发送模式 ......\r\n" );
	while( ! ESP8266_UnvarnishSend () );
	
//	printf( "\r\n配置 ESP8266 完毕\r\n" );
//	printf ( "\r\n开始透传......\r\n" );
  
}


/**
  * @brief  ESP8266 检查是否接收到了数据，检查连接和掉线重连
  * @param  无
  * @retval 无
  */
void ESP8266_CheckRecvDataTest(void)
{
  uint8_t ucStatus;
  uint16_t i;
  
//  /* 如果接收到了串口调试助手的数据 */
//  if(strUSART_Fram_Record.InfBit.FramFinishFlag == 1)
//  {
//    for(i = 0;i < strUSART_Fram_Record.InfBit.FramLength; i++)
//    {
//       USART_SendData( macESP8266_USARTx ,strUSART_Fram_Record.Data_RX_BUF[i]); //转发给ESP82636
//       while(USART_GetFlagStatus(macESP8266_USARTx,USART_FLAG_TC)==RESET){}      //等待发送完成
//    }
//    strUSART_Fram_Record .InfBit .FramLength = 0;                                //接收数据长度置零
//    strUSART_Fram_Record .InfBit .FramFinishFlag = 0;                            //接收标志置零
//  }
  
  /* 如果接收到了ESP8266的数据 */
  if(strEsp8266_Fram_Record.InfBit.FramFinishFlag)
  {                                                      
//		for(i = 0;i < strEsp8266_Fram_Record .InfBit .FramLength; i++)               
//		{
//			 while(USART_GetFlagStatus(DEBUG_USARTx,USART_FLAG_TC)==RESET){}
//		} //转发给USART1
		 contol = strEsp8266_Fram_Record.Data_RX_BUF[0];
     strEsp8266_Fram_Record .InfBit .FramLength = 0;                             //接收数据长度置零
     strEsp8266_Fram_Record.InfBit.FramFinishFlag = 0;                           //接收标志置零

  }
  if ( ucTcpClosedFlag )                                             //检测是否失去连接
  {
    ESP8266_ExitUnvarnishSend ();                                    //退出透传模式
    
    do ucStatus = ESP8266_Get_LinkStatus ();                         //获取连接状态
    while ( ! ucStatus );
    
    if ( ucStatus == 4 )                                             //确认失去连接后重连
    {
//      printf ( "\r\n正在重连热点和服务器 ......\r\n" );
      
      while ( ! ESP8266_JoinAP ( macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd ) );
      
      while ( !	ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );
      
//      printf ( "\r\n重连热点和服务器成功\r\n" );

    }
    
    while ( ! ESP8266_UnvarnishSend () );		
    
  }
}

void ESP8266_SendInf(void)
{
	char data[] ="ab2323";
	ESP8266_SendString ( ENABLE, data, 6, Multiple_ID_0 );
}
