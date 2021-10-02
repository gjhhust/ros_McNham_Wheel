#include "main.h"
float x,y=35,theta,w;
extern chassis F105;
CanTxMsg tx;
short x_park_flag,y_park_flag,w_park_flag;
short init_flag;
short init_Angle[4];
float carSpeedx,carSpeedy,carSpeedw;
extern Pid_Typedef Park_x,Park_y,Park_theta;
extern short  NowAngleLeftFront[2],NowAngleRightFront[2],NowAngleLeftBack[2],NowAngleRightBack[2];
short deltLeft,deltRight;
short inverse;
typedef struct{
	short V_x;
	short V_y;
	short V_w;
	short yaw;
}Chassis_send;
Chassis_send Send_data={0};
extern Ctrl_information chassis_ctrl;

//标定
Pid_Typedef position;
float time=0;
float time_chaju=0;
float time_last = 0;
void AutoTask()
{
  tx.StdId=0x200;      //设置标识符
	tx.IDE=CAN_ID_STD;   //标准帧
	tx.RTR=CAN_RTR_DATA; //数据帧
	tx.DLC=8;            //数据长度

//ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	
	
	x=0;
	y=0;
	w=0;
	theta=14;
		
  tx.Data[0]= 0;
	tx.Data[1]= 0;
	tx.Data[2]= 0;
	tx.Data[3]= 0;
  tx.Data[4]= 0;
	tx.Data[5]= 0;
  tx.Data[6]= 0;
	tx.Data[7]= 0;
	CAN_Transmit(CAN1,&tx);
while(1)
{
	//goto_1m();
	
	//time_chaju = time-time_last;
	Cal_speed();
	
	//发送数据
	Send_to_PC();
	
	//自动泊车的pid计算相关操作
	
  carSpeedx=speed2Current(limit_max_min(chassis_ctrl.Y_SpeedSet,60,-60));//cm/s转变为电流值
	carSpeedy=speed2Current(limit_max_min(chassis_ctrl.X_SpeedSet,60,-60));
	carSpeedw=limit_max_min(chassis_ctrl.W_SpeedSet,2,-2);

  F105.PIDwheel[0].SetPoint = limit_max_min(3.5*(+ carSpeedx - carSpeedy )- 1000*carSpeedw,4000,-4000);
  F105.PIDwheel[1].SetPoint = limit_max_min(3.5*(- carSpeedx - carSpeedy )- 1000*carSpeedw,4000,-4000);
  F105.PIDwheel[2].SetPoint = limit_max_min(3.5*(- carSpeedx + carSpeedy )- 1000*carSpeedw,4000,-4000);
  F105.PIDwheel[3].SetPoint = limit_max_min(3.5*(+ carSpeedx + carSpeedy )- 1000*carSpeedw,4000,-4000);
	
	F105.carSpeed[0]= limit_max_min(PID_Cal(&F105.PIDwheel[0]),3000,-3000);
	F105.carSpeed[1]= limit_max_min(PID_Cal(&F105.PIDwheel[1]),3000,-3000);
	F105.carSpeed[2]= limit_max_min(PID_Cal(&F105.PIDwheel[2]),3000,-3000);
	F105.carSpeed[3]= limit_max_min(PID_Cal(&F105.PIDwheel[3]),3000,-3000);
	
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
	

	vTaskDelay(20);
}

}
/**********************************************************************************************************
*函 数 名: goto_1m
*功能说明: 直线走一米
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static int set_des_flag=0;
float y_show=0;
float positionNow = 0;
void goto_1m(void){
	position.RealPoint += y*0.02;
	if(set_des_flag == 0){
		time = xTaskGetTickCount();
		position.SetPoint = 1000;//1m
		//位置环
		position.P = 2.0f;
		position.I = 0;
		position.D = 2;
		position.IMax = 200.0f;
		position.OutMax = 500;	//cm/s
		set_des_flag = 1;
	}	
	y_show = PID_Cal(&position);
	//Chassis_speed_R.SetPoint = Chassis_speed_L.SetPoint;
	
}
/**********************************************************************************************************
*函 数 名: void Cal_speed(void)
*功能说明: 计算xy速度
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Cal_speed(void){
	/*
		R=[	3.5			-3.5	-1000
				-3.5		-3.5	-1000
				-3.5		3.5		-1000
				3.5			3.5		-1000]
		xishu = inv(R' * R) * R'      matlab计算
	
		V = xishu * W（W是四个轮速）
	
	
	*/
	F105.V_x_now = Current2speed(0.0714*F105.PIDwheel[0].RealPoint - 0.0714*F105.PIDwheel[1].RealPoint - 0.0714*F105.PIDwheel[2].RealPoint + 0.0714*F105.PIDwheel[3].RealPoint);
	F105.V_y_now = Current2speed(-0.0714*F105.PIDwheel[0].RealPoint - 0.0714*F105.PIDwheel[1].RealPoint + 0.0714*F105.PIDwheel[2].RealPoint + 0.0714*F105.PIDwheel[3].RealPoint);
	
}


void Send_to_PC(void)
{
	//发送底盘数据
	Send_data.V_x = F105.V_x_now;
	Send_data.V_y = F105.V_y_now;
	Send_data.V_w = F105.YawAngleSpeed;
	Send_data.yaw = 88*F105.YawAngle;
	
	usartSendData(Send_data.V_y,Send_data.V_x,Send_data.V_w,Send_data.yaw);
}
//void AutoTask()
//{
//  tx.StdId=0x200;      //设置标识符
//	tx.IDE=CAN_ID_STD;   //标准帧
//	tx.RTR=CAN_RTR_DATA; //数据帧
//	tx.DLC=8;            //数据长度
//	
//ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	
//	
//	x=0;
//	y=0;
//	theta=0;
//		
//Park_x.SetPoint=x* 19*8192/2/3.14159/0.07;
//Park_y.SetPoint=y* 17.85*8192/2/3.14159/0.07;
//Park_theta.SetPoint=theta;

//while(1)
//{

//taskENTER_CRITICAL();  //进入临界区
//if(!init_flag)
//{
//init_flag=1;
//NowAngleLeftFront[1]=NowAngleLeftFront[0];
//}
//deltLeft=NowAngleLeftFront[0]-NowAngleLeftFront[1];
//NowAngleLeftFront[1]=NowAngleLeftFront[0];
////读数据
//taskEXIT_CRITICAL();
////自动泊车的pid计算相关操作
//  
//	//过零检测
//	if(deltLeft>8192*0.7)   deltLeft = -8192+deltLeft;
//	if(deltLeft<-(8192*0.7))  deltLeft= 8192+deltLeft;

////   旋转
//	if(!w_park_flag)
//	{
//	 Park_theta.RealPoint=F105.YawAngle;
//	 carSpeedw=PID_Cal(&Park_theta);
//  if(fabs(Park_theta.RealPoint-Park_theta.SetPoint)<2.0)
//	{
//	w_park_flag=1;
//	init_flag=0;
//	carSpeedw=0;
//	}
//	
//}
//	
////   左右移动	
//	else if(!x_park_flag)
//	{
//	Park_x.RealPoint+=deltLeft;
//	carSpeedx=PID_Cal(&Park_x);
//	if(fabs(Park_x.RealPoint-Park_x.SetPoint)<56)
//	{
//	x_park_flag=1;
//	init_flag=0;
//	carSpeedx=0;
//	}
//	}
//	
////   前后移动	
//	else if(!y_park_flag)
//	{
//	Park_y.RealPoint+=deltLeft;
//	carSpeedy=PID_Cal(&Park_y);
//	if(fabs(Park_y.RealPoint-Park_y.SetPoint)<56)
//	 {
//	 y_park_flag=1;
//	 init_flag=0;
//	 carSpeedy=0;
//	 } 
//	
//	}
//  F105.PIDwheel[0].SetPoint = limit_max_min(14*(+ carSpeedx + carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[1].SetPoint = limit_max_min(14*(- carSpeedx + carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[2].SetPoint = limit_max_min(14*(- carSpeedx - carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[3].SetPoint = limit_max_min(14*(+ carSpeedx - carSpeedy )+ 20*carSpeedw,1000,-1000);
//	
//	F105.carSpeed[0]= limit_max_min(PID_Cal(&F105.PIDwheel[0]),3000,-3000);
//	F105.carSpeed[1]= limit_max_min(PID_Cal(&F105.PIDwheel[1]),3000,-3000);
//	F105.carSpeed[2]= limit_max_min(PID_Cal(&F105.PIDwheel[2]),3000,-3000);
//	F105.carSpeed[3]= limit_max_min(PID_Cal(&F105.PIDwheel[3]),3000,-3000);
//	
//	//CAN通信发送四个轮子电流

//  tx.Data[0]= (F105.carSpeed[0]>>8)&0xff;
//	tx.Data[1]=  F105.carSpeed[0]&0xff;
//	tx.Data[2]= (F105.carSpeed[1]>>8)&0xff;
//	tx.Data[3]=  F105.carSpeed[1]&0xff;
//  tx.Data[4]= (F105.carSpeed[2]>>8)&0xff;
//	tx.Data[5]=  F105.carSpeed[2]&0xff;
//  tx.Data[6]= (F105.carSpeed[3]>>8)&0xff;
//	tx.Data[7]=  F105.carSpeed[3]&0xff;
//	CAN_Transmit(CAN1,&tx);
//	

//vTaskDelay(1);
//}

//}





//void AutoTask()
//{
//  tx.StdId=0x200;      //设置标识符
//	tx.IDE=CAN_ID_STD;   //标准帧
//	tx.RTR=CAN_RTR_DATA; //数据帧
//	tx.DLC=8;            //数据长度
//	
////ulTaskNotifyTake(pdTRUE,portMAX_DELAY);	
//	
//Park_x.SetPoint=0*8192/2/3.14159/0.07;
//Park_y.SetPoint=1.13*8192/2/3.14159/0.07;
//Park_theta.SetPoint=90.0;

//while(1)
//{
//taskENTER_CRITICAL();
//if(!init_flag)
//{
//init_flag=1;
//NowAngleLeftFront[1]=NowAngleLeftFront[0];
//NowAngleRightFront[1]=NowAngleRightFront[0];
//NowAngleLeftBack[1]=NowAngleLeftBack[0];
//NowAngleRightBack[1]=NowAngleRightBack[0];
//}

//deltLeft=((NowAngleLeftFront[0]-NowAngleRightBack[0])-(NowAngleLeftFront[1]-NowAngleRightBack[1]))/2;
//deltRight=((NowAngleRightFront[0]-NowAngleLeftBack[0])-(NowAngleRightFront[1]-NowAngleLeftBack[1]))/2;

//NowAngleLeftFront[1]=NowAngleLeftFront[0];
//NowAngleRightFront[1]=NowAngleRightFront[0];
//NowAngleLeftBack[1]=NowAngleLeftBack[0];
//NowAngleRightBack[1]=NowAngleRightBack[0];

////读数据
//taskEXIT_CRITICAL();
////自动泊车的pid计算相关操作
//  
//	//过零检测
//	if(deltLeft>8192*0.5)   deltLeft = -8192+deltLeft;
//	if(deltLeft<-(8192*0.5))  deltLeft = 8192+deltLeft;
//  if(deltRight>8192*0.5)  deltRight= -8192+deltRight;
//	if(deltRight<-(8192*0.5)) deltRight= 8192+deltRight;
//	
//	Park_y.RealPoint+=(deltLeft-deltRight)/2;
//	Park_x.RealPoint+=(deltLeft+deltRight)/2;
//	
//	//carSpeedx=PID_Cal(&Park_x);
//	carSpeedy=PID_Cal(&Park_y);
//	carSpeedx=0;
//	
//  F105.PIDwheel[0].SetPoint = limit_max_min(14*(+ carSpeedx + carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[1].SetPoint = limit_max_min(14*(- carSpeedx + carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[2].SetPoint = limit_max_min(14*(- carSpeedx - carSpeedy )+ 20*carSpeedw,1000,-1000);
//  F105.PIDwheel[3].SetPoint = limit_max_min(14*(+ carSpeedx - carSpeedy )+ 20*carSpeedw,1000,-1000);

//  F105.carSpeed[0]=limit_max_min(PID_Cal(&F105.PIDwheel[0]),3000,-3000);
//	F105.carSpeed[1]=limit_max_min(PID_Cal(&F105.PIDwheel[1]),3000,-3000);
//	F105.carSpeed[2]=limit_max_min(PID_Cal(&F105.PIDwheel[2]),3000,-3000);
//	F105.carSpeed[3]=limit_max_min(PID_Cal(&F105.PIDwheel[3]),3000,-3000);
//	
//		if((fabs(Park_y.RealPoint-Park_y.SetPoint)<60)&&(fabs(Park_x.RealPoint-Park_x.SetPoint)<60))
//	{
//  F105.carSpeed[0]=0;
//	F105.carSpeed[1]=0;
//	F105.carSpeed[2]=0;
//	F105.carSpeed[3]=0;
// }	
//	//CAN通信发送四个轮子电流

//  tx.Data[0]= (F105.carSpeed[0]>>8)&0xff;
//	tx.Data[1]=  F105.carSpeed[0]&0xff;
//	tx.Data[2]= (F105.carSpeed[1]>>8)&0xff;
//	tx.Data[3]=  F105.carSpeed[1]&0xff;
//  tx.Data[4]= (F105.carSpeed[2]>>8)&0xff;
//	tx.Data[5]=  F105.carSpeed[2]&0xff;
//  tx.Data[6]= (F105.carSpeed[3]>>8)&0xff;
//	tx.Data[7]=  F105.carSpeed[3]&0xff;
//	CAN_Transmit(CAN1,&tx);
//	

//vTaskDelay(1);
//}

//}
