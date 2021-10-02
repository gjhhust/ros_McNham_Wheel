#include "main.h"
extern RC_Ctl_t RC_Ctl;;
extern chassis F105;
extern TaskHandle_t chassisTaskHandle;
extern TaskHandle_t AutoTaskHandle;
int suspend_remote=0;
int suspend_auto=0;
/**********************************************************************************************************
*����      S1               S2 
*      1 �� ң��           1 
*      3 �� �Զ�����       3
*      2 �� ����           2
**********************************************************************************************************/

void ActionTask()
{
 while(1)
{
	switch(RC_Ctl.s1)
	{
	case 1:
	F105.Mode=RemoteMode;
	if(!suspend_auto)
	{
	 suspend_auto=1;
	 vTaskSuspend(AutoTaskHandle);//���𲴳�����
	}
	if(suspend_remote)
	{
	  suspend_remote=0;
		vTaskResume(chassisTaskHandle);//�������chassis
	}
	break;
	
	case 2:
	F105.Mode=PowerOffMode;
	if(!suspend_remote)
	{
	suspend_remote=1;
	vTaskSuspend(chassisTaskHandle);//����chassis����
	}
  if(!suspend_auto)
	{
	 suspend_auto=1;
	 vTaskSuspend(AutoTaskHandle);//���𲴳�����
	}
	break;
	case 3:
	F105.Mode=KeyMouseMode;
	if(!suspend_remote)
	{
	suspend_remote=1;
	vTaskSuspend(chassisTaskHandle);//����chassis����
	}
	if(suspend_auto)
	{
	  suspend_auto=0;
		vTaskResume(AutoTaskHandle);//�������Auto
	}
	break;
	}

    vTaskDelay(50);  //��ʱ50ms

 }
}