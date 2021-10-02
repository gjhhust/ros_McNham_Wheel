#ifndef  __ESP8266_TEST_H
#define	 __ESP8266_TEST_H



#include "main.h"


/********************************** 用户需要设置的参数**********************************/
#define      macUser_ESP8266_ApSsid                       "MERCURY_502"                //要连接的热点的名称
#define      macUser_ESP8266_ApPwd                        "18893106672"           //要连接的热点的密钥

#define      macUser_ESP8266_TcpServer_IP                 "192.168.0.102"      //要连接的服务器的 IP
#define      macUser_ESP8266_TcpServer_Port               "8000"               //要连接的服务器的端口



/********************************** 外部全局变量 ***************************************/
extern volatile uint8_t ucTcpClosedFlag;



/********************************** 测试函数声明 ***************************************/
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);
void ESP8266_CheckRecvDataTest(void);
void ESP8266_SendInf(void);
#endif

