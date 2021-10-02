#include "main.h"
extern chassis F105; 

//红灯亮是泊车模式  蓝灯是遥控模式
 void LedTask()
 {
 while(1)
 {
 if(F105.Mode==RemoteMode)
 {
 GPIO_ResetBits(LED_GPIO,LED_PIN1);
 GPIO_SetBits(LED_GPIO,LED_PIN2);
 }
 if(F105.Mode==KeyMouseMode)
 {
  GPIO_SetBits(LED_GPIO,LED_PIN1);
 GPIO_ResetBits(LED_GPIO,LED_PIN2);
 }
 if(F105.Mode==PowerOffMode)
 {
 GPIO_SetBits(LED_GPIO,LED_PIN1);
 GPIO_SetBits(LED_GPIO,LED_PIN2);
 }
 
 vTaskDelay(50);
}

}



void LED_Configuration()
{
 GPIO_InitTypeDef gpio;
 RCC_APB2PeriphClockCmd(RCC_GPIO,ENABLE);
 
 gpio.GPIO_Mode=GPIO_Mode_Out_PP;
 gpio.GPIO_Pin=LED_PIN1;
 gpio.GPIO_Speed=GPIO_Speed_50MHz;
 GPIO_Init(LED_GPIO,&gpio);
 
 gpio.GPIO_Pin=LED_PIN2;
 GPIO_Init(LED_GPIO,&gpio);
 
 GPIO_SetBits(LED_GPIO,LED_PIN2);
 GPIO_SetBits(LED_GPIO,LED_PIN1);
 
} 