# include "main.h" 

chassis F105;
volatile uint32_t CPU_RunTime = 0;

TaskHandle_t ActionTaskHandle;
#define ActionTaskPriority 14

TaskHandle_t chassisTaskHandle;
#define chassisTaskPriority 11

TaskHandle_t LedTaskHandle;
#define LedTaskPriority  13

TaskHandle_t PCReceiveTaskHandle;
#define PCReceiveTaskPriority 15

TaskHandle_t AutoTaskHandle;
#define AutoTaskPriority 12

void BSP_Init();
void Sys_Init();

int main()
{
  BSP_Init();
	delay_ms(5000);//等待初始化
  Sys_Init();
	//创建任务
	xTaskCreate(ActionTask,"actionTask",512,NULL,ActionTaskPriority,&ActionTaskHandle);
	xTaskCreate(ChassisTask,"chassisTask",512,NULL,chassisTaskPriority,&chassisTaskHandle);
	xTaskCreate(LedTask,"LedTask",512,NULL,LedTaskPriority,&LedTaskHandle);
	xTaskCreate(PCReceiveTask,"PCTASK",512,NULL,PCReceiveTaskPriority,&PCReceiveTaskHandle);
	xTaskCreate(AutoTask,"AutoTask",512,NULL,AutoTaskPriority,&AutoTaskHandle);
	vTaskStartScheduler();
	while(1)
	{

	}
	
	
}
void BSP_Init()
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	TIM6_Configration(); //TIM6,7用于延时
	TIM7_Configration();
  LED_Configuration();
	CAN1_Configuration();
	CAN2_Configuration();
	USART1_Configuration(115200);
	USART2_Configuration(115200);
	USART3_Configuration();
	USART5_Configuration(115200);
}
void Sys_Init()
{
 chassis_PID_Init();

}