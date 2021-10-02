#ifndef __PID_H__
#define __PID_H__

typedef struct  
{
		float SetPoint;			//设定目标值
	  float RealPoint;      //实际值
		
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float err[2];		    //误差
		float SumError;			//积分误差
	
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	
	  float Out;
		float OutMax;	
	
}Pid_Typedef;


float PID_Cal(Pid_Typedef * p);
void PID_init(Pid_Typedef * pid,float p,float i,float d,float imax,float outmax);


#endif
