#include "main.h" 
extern TaskHandle_t AutoTaskHandle;
void PCReceiveTask()
{
  while(1)
	{
	ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	
	taskENTER_CRITICAL();
	
	//���ݴ���
	
	taskEXIT_CRITICAL();
	if(1)//�յ��Զ�����������
	xTaskNotifyGive(AutoTaskHandle);
	
	//vTaskDelay(1);
	}


}