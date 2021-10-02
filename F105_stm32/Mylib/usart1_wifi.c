#include "main.h"


void USART1_Configuration(u32 bound)
{
   ESP8266_Init(bound);
	 ESP8266_CheckRecvDataTest();
}