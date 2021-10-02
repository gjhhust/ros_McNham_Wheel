#include "main.h"

float dt=0.06;
struct Angle gyro,w,angle_acc,angle_w,angle,acc;
short  acc_pitch,acc_roll,acc_yaw;
short x_flag=0,y_flag=0,z_flag=0;
double sum_pitch,sum_yaw,sum_roll,sum;
double test[1000];
double s2[1000],S2,ave;
int t,i,u;
void MPU_6050Init()
{
	 //解除休眠状态
	 iic_write_REG(0x6B,0x00);
	//陀螺仪采样率 1KHz
	iic_write_REG(0x19,0x07);	
	//低通滤波器的设置 截止频率是1K 带宽是5K
	iic_write_REG(0x1A,0x06);	  
	//加计2G模式 不自检
	iic_write_REG(0x1C,0x00);
	//陀螺仪1000deg/s 不自检
	iic_write_REG(0x1B,0x10);  
}

void MPU_6050Read()
{  
  //x-roll  y-pitch z-yaw
	
	//求偏差
		if(y_flag<10)
	{
	 sum_pitch+=w.pitch;
   y_flag++;
	}
		if(z_flag<10)
	{
	 sum_yaw+=w.yaw;
   z_flag++;
	}
	if(x_flag<10)
	{
	 sum_roll+=w.roll;
   x_flag++;
	}

	     //零飘 除偏差
	w.pitch=w.pitch-(sum_pitch/y_flag);
	w.yaw=w.yaw-(sum_yaw/z_flag);
	w.roll=w.roll-(sum_roll/x_flag);
	
	angle_acc.roll=atan2(acc_pitch,acc_yaw)*57.3;
	angle_acc.pitch=atan2(acc_roll,acc_yaw)*57.3;
	
	
	 if(t<1000)  //标定加速度计用
	{
	 test[t]=acc_pitch+16384;
	 ave+=test[t]/1000.0;
	 t++;
	}
	
//  if(t<1000)  //算R,测量误差的方差用
//	{
//	
//	 test[t]=angle_acc.pitch;
//	 sum+=angle_acc.pitch;
//	 t++;
//	}
//	if(t==1000)
//	{
//	ave=sum/1000;
//	for(i=0;i<1000;i++)
//	{
//	s2[u]+=(test[i]-ave)*(test[i]-ave)/999;
//	S2+=s2[u]/1000;
//	}
//	u++;
//	if(u<1000)
//	{
//	t=0;
//	sum=0;
//	}
//	if(u==1000)
//	{
//	t++;
//	}
//	}
//		
	angle_w.pitch=angle.pitch+w.pitch*dt;
	angle_w.roll =angle.roll+w.roll*dt;
	angle_w.yaw =angle.yaw+w.yaw*dt;
	
	//过零检测
	if(angle_w.pitch>180.0f)
	{
	angle_w.pitch=angle_w.pitch-360.0f;
	}
	else if(angle_w.pitch<-180.0f)
	{
	angle_w.pitch=angle_w.pitch+360.0f;
	}	
	
	
	if(angle_w.roll>180.0f)
	{
	angle_w.roll=angle_w.roll-360.0f;
	}
	else if(angle_w.roll<-180.0f)
	{
	angle_w.roll=angle_w.roll+360.0f;
	}	
	
	
	if(angle_w.yaw>180.0f)
	{
	angle.yaw=angle_w.yaw-360.0f;
	}
	else if(angle_w.yaw<-180.0f)
	{
	angle.yaw=angle_w.yaw+360.0f;
	}
	else {
	angle.yaw=angle_w.yaw;
	}
	
	
	
	angle.pitch=Kalman_Filter(angle_acc.pitch,angle_w.pitch,1);
	angle.roll=Kalman_Filter(angle_acc.roll,angle_w.roll,0);
	
}

