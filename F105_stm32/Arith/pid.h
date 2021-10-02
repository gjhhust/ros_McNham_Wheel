#ifndef __PID_H__
#define __PID_H__

typedef struct  
{
		float SetPoint;			//�趨Ŀ��ֵ
	  float RealPoint;      //ʵ��ֵ
		
		float P;						//��������
		float I;						//���ֳ���
		float D;						//΢�ֳ���
		
		float err[2];		    //���
		float SumError;			//�������
	
		float IMax;					//��������
		
		float POut;					//�������
		float IOut;					//�������
		float DOut;					//΢�����
	
	  float Out;
		float OutMax;	
	
}Pid_Typedef;


float PID_Cal(Pid_Typedef * p);
void PID_init(Pid_Typedef * pid,float p,float i,float d,float imax,float outmax);


#endif
