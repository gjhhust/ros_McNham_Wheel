#include "main.h"
extern RC_Ctl_t RC_Ctl;;
extern chassis F105;
extern TaskHandle_t chassisTaskHandle;
extern TaskHandle_t AutoTaskHandle;
int suspend_remote=0;
int suspend_auto=0;
/**********************************************************************************************************
*拨杆      S1               S2 
*      1 上 遥控           1 
*      3 中 自动泊车       3
*      2 下 掉电           2
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
	 vTaskSuspend(AutoTaskHandle);//挂起泊车任务
	}
	if(suspend_remote)
	{
	  suspend_remote=0;
		vTaskResume(chassisTaskHandle);//解除挂起chassis
	}
	break;
	
	case 2:
	F105.Mode=PowerOffMode;
	if(!suspend_remote)
	{
	suspend_remote=1;
	vTaskSuspend(chassisTaskHandle);//挂起chassis任务
	}
  if(!suspend_auto)
	{
	 suspend_auto=1;
	 vTaskSuspend(AutoTaskHandle);//挂起泊车任务
	}
	break;
	case 3:
	F105.Mode=KeyMouseMode;
	if(!suspend_remote)
	{
	suspend_remote=1;
	vTaskSuspend(chassisTaskHandle);//挂起chassis任务
	}
	if(suspend_auto)
	{
	  suspend_auto=0;
		vTaskResume(AutoTaskHandle);//解除挂起Auto
	}
	break;
	}

    vTaskDelay(50);  //延时50ms

 }
}