#ifndef  __ESP8266_TEST_H
#define	 __ESP8266_TEST_H



#include "main.h"


/********************************** �û���Ҫ���õĲ���**********************************/
#define      macUser_ESP8266_ApSsid                       "MERCURY_502"                //Ҫ���ӵ��ȵ������
#define      macUser_ESP8266_ApPwd                        "18893106672"           //Ҫ���ӵ��ȵ����Կ

#define      macUser_ESP8266_TcpServer_IP                 "192.168.0.102"      //Ҫ���ӵķ������� IP
#define      macUser_ESP8266_TcpServer_Port               "8000"               //Ҫ���ӵķ������Ķ˿�



/********************************** �ⲿȫ�ֱ��� ***************************************/
extern volatile uint8_t ucTcpClosedFlag;



/********************************** ���Ժ������� ***************************************/
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);
void ESP8266_CheckRecvDataTest(void);
void ESP8266_SendInf(void);
#endif

