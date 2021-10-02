#include "main.h" 
extern TaskHandle_t AutoTaskHandle;
void PCReceiveTask()
{
  while(1)
	{
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	
	taskENTER_CRITICAL();
	
	//数据处理
	
	taskEXIT_CRITICAL();
	if(1)//收到自动泊车的命令
	xTaskNotifyGive(AutoTaskHandle);
	
	//vTaskDelay(1);
	}


}