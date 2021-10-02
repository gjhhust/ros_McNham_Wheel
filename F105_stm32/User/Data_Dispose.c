#include "Data_Dispose.h"

//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
/*--------------------------------发送协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------接收协议-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//数据头55aa + 数据字节数size + 数据（利用共用体） + 校验crc8 + 数据尾0d0a
//注意：这里数据中预留了一个字节的控制位，其他的可以自行扩展，更改size和数据
--------------------------------------------------------------------------*/

/*--------------------------------控制位-----------------------------------

8位由高到底分别是
软件复位 左轮方向 右轮方向 
--------------------------------------------------------------------------*/

/**************************************************************************
通信的发送函数和接收函数必须的一些常量、变量、共用体对象
**************************************************************************/

extern unsigned char JudgeSend[SendBiggestSize];//发送最大内存

unsigned char SaveBuffer[200];//接受双缓存区

Ctrl_information chassis_ctrl ={0,0,0,0}; //接受上位机控制信息
extern Chassis F103RC_chassis;//底盘实时数据
         


//发送数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
union sendData
{
	short d;
	unsigned char data[2];
}x_Actual,y_Actual,w_Actual,angleNow;

//左右轮速控制速度共用体
union receiveData
{
	short d;
	unsigned char data[2];
}x_Ctrl,y_Ctrl,z_Ctrl;



/**********************************************************************************************************
*函 数 名: JudgeBuffReceive
*功能说明: 接收函数
*形    参: ReceiveBuffer[]
*返 回 值: 无
**********************************************************************************************************/
void JudgeBuffReceive(unsigned char ReceiveBuffer[])
{
	uint16_t cmd_id;//识别信息内容，暂未开启
	short k=0;
	short PackPoint;
	memcpy(&SaveBuffer[reBiggestSize],&ReceiveBuffer[0],reBiggestSize);		//把ReceiveBuffer[0]地址拷贝到SaveBuffer[13], 依次拷贝13个, 把这一次接收的存到数组后方
	for(PackPoint=0;PackPoint<reBiggestSize;PackPoint++)		//先处理前半段数据(在上一周期已接收完成)
	{
		if(SaveBuffer[PackPoint]==header[0] && SaveBuffer[PackPoint + 1]== header[1]) //包头检测
		{	
			short dataLength  = SaveBuffer[PackPoint + 2]    ;
			unsigned char checkSum = getCrc8(&SaveBuffer[PackPoint], 3 + dataLength);
				// 检查信息校验值
			if (checkSum == SaveBuffer[PackPoint +3 + dataLength]) //SaveBuffer[PackPoint开始的校验位]
			{
				//说明数据核对成功，开始提取数据
				 for(k = 0; k < 2; k++)
					{
						x_Ctrl.data[k]  = SaveBuffer[PackPoint + k + 3]; //SaveBuffer[3]  SaveBuffer[4]
						y_Ctrl.data[k] = SaveBuffer[PackPoint + k + 5]; //SaveBuffer[5]  SaveBuffer[6]
						z_Ctrl.data[k]  = SaveBuffer[PackPoint + k + 7]; //SaveBuffer[7]  SaveBuffer[8]
					}				
					
					//速度赋值操作
					chassis_ctrl.X_SpeedSet = (int)(x_Ctrl.d);
					chassis_ctrl.Y_SpeedSet = (int)(y_Ctrl.d);
					chassis_ctrl.W_SpeedSet = (int)z_Ctrl.d;
					//ctrlFlag
					chassis_ctrl.crtlFlag = SaveBuffer[PackPoint + 3 + dataLength - 1];                //SaveBuffer[9]
				
			}
		}	
	memcpy(&SaveBuffer[0],&SaveBuffer[reBiggestSize],reBiggestSize);		//把SaveBuffer[13]地址拷贝到SaveBuffer[0], 依次拷贝13个，把之前存到后面的数据提到前面，准备处理
	}
}
/**************************************************************************
函数功能：将左右轮速和角度数据、控制信号进行打包，通过串口发送给Linux
入口参数：实时左轮轮速、实时右轮轮速、实时角度、控制信号（如果没有角度也可以不发）
返回  值：无
**************************************************************************/
void usartSendData(short x_speed, short y_speed,short w_speed,short angle)
{
	int i, length = 0;

	// 计算左右轮期望速度
	x_Actual.d  = x_speed;
	y_Actual.d = y_speed;
	w_Actual.d = w_speed;
	angleNow.d    = angle;
	
	// 设置消息头
	for(i = 0; i < 2; i++)
		JudgeSend[i] = header[i];                      // buf[0] buf[1] 
	
	// 设置机器人左右轮速度、角度
	length = 8;
	JudgeSend[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		JudgeSend[i + 3] = x_Actual.data[i];         // buf[3] buf[4]
		JudgeSend[i + 5] = y_Actual.data[i];        // buf[5] buf[6]
		JudgeSend[i + 7] = w_Actual.data[i]; 					// buf[7] buf[8]
		JudgeSend[i + 9] = angleNow.data[i];           // buf[9] buf[10]
	}
	// 预留控制指令
	
	// 设置校验值、消息尾
	JudgeSend[3 + length] = getCrc8(JudgeSend, 3 + length);  // buf[11]
	JudgeSend[3 + length + 1] = ender[0];              // buf[12]
	JudgeSend[3 + length + 2] = ender[1];              // buf[13]
	
	//发送字符串数据
		/*****数据上传*****/

//	USART_ClearFlag(USART2,USART_FLAG_TC);
//	for(i=0;i<13;i++)
//	{
//	  USART_SendData(USART2,JudgeSend[i]);
//	  while (USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET); //等待之前的字符发送完成
//	}
	DMA1_Channel7->CNDTR = SendBiggestSize; 
	DMA_Cmd(DMA1_Channel7,ENABLE);
}

/**************************************************************************
函数功能：计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
入口参数：数组地址、数组大小
返回  值：无
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







/**********************************************************************************************************
*函 数 名: Gyro_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//void DATA_DISPOSE_task(void *pvParameters){

//	
//	while(1)
//	{
// 
//		
//		
//		
//		//IMUupdate(gyrox, gyroy, gyroz, aacx, aacy,aacz);
//			
//		vTaskDelay(20); 

//	}

//}
