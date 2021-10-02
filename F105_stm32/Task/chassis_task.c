#include "main.h"
extern RC_Ctl_t RC_Ctl;
extern chassis F105;
static float carSpeedx,carSpeedy,carSpeedw;
extern CanTxMsg tx;
Pid_Typedef Park_x,Park_y,Park_theta;
/*      车轮 0    3
             1    2                  */
						 
void chassis_PID_Init()
{ 
 //                               p      i     d    积分限幅    总限幅
  PID_init(&F105.PIDwheel[0] ,     8.0  , 0.0 , 1.0 , 100.0    , 16384.0 );
	PID_init(&F105.PIDwheel[1] ,     8.0  , 0.0 , 1.0 , 100.0    , 16384.0 );
	PID_init(&F105.PIDwheel[2] ,     8.0  , 0.0 , 1.0 , 100.0    , 16384.0 );
	PID_init(&F105.PIDwheel[3] ,     8.0  , 0.0 , 1.0 , 100.0    , 16384.0 );
	PID_init(&Park_x              ,  5.0   , 0.0 , 0.0 , 100.0    , 8192.0 );
	PID_init(&Park_y              ,  5.0   , 0.0 , 0.0 , 100.0    , 8192.0 );
	PID_init(&Park_theta          ,  1   , 0.0 , 0.0 , 100.0    , 360.0 );
 
}

extern Pid_Typedef position;
void ChassisTask()
{
  float k; //不同模式输入量转化为输出量的比例系数
	tx.StdId=0x200;      //设置标识符
	tx.IDE=CAN_ID_STD;   //标准帧
	tx.RTR=CAN_RTR_DATA; //数据帧
	tx.DLC=8;            //数据长度
	k=5;
	while(1)
	{
	Cal_speed();
		
	//发送数据
	Send_to_PC();
		
	position.RealPoint = 0;
  carSpeedx= 1024-RC_Ctl.ch1; 
	carSpeedy= 1024-RC_Ctl.ch0;
	carSpeedw= 1024-RC_Ctl.ch2;
	

	
  F105.PIDwheel[0].SetPoint = limit_max_min(k * (+ carSpeedx + carSpeedy - carSpeedw),2000,-2000);
  F105.PIDwheel[1].SetPoint =	limit_max_min(k * (+ carSpeedx - carSpeedy - carSpeedw),2000,-2000);
  F105.PIDwheel[2].SetPoint = limit_max_min(k * (- carSpeedx - carSpeedy - carSpeedw),2000,-2000); 
  F105.PIDwheel[3].SetPoint = limit_max_min(k * (- carSpeedx + carSpeedy - carSpeedw),2000,-2000);

  F105.carSpeed[0]=PID_Cal(&F105.PIDwheel[0]);
	F105.carSpeed[1]=PID_Cal(&F105.PIDwheel[1]);
	F105.carSpeed[2]=PID_Cal(&F105.PIDwheel[2]);
	F105.carSpeed[3]=PID_Cal(&F105.PIDwheel[3]);
	
	//CAN通信发送四个轮子电流

  tx.Data[0]= (F105.carSpeed[0]>>8)&0xff;
	tx.Data[1]=  F105.carSpeed[0]&0xff;
	tx.Data[2]= (F105.carSpeed[1]>>8)&0xff;
	tx.Data[3]=  F105.carSpeed[1]&0xff;
  tx.Data[4]= (F105.carSpeed[2]>>8)&0xff;
	tx.Data[5]=  F105.carSpeed[2]&0xff;
  tx.Data[6]= (F105.carSpeed[3]>>8)&0xff;
	tx.Data[7]=  F105.carSpeed[3]&0xff;
	CAN_Transmit(CAN1,&tx);
	
	vTaskDelay(10);
	}
}

