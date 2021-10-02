#include "main.h"

float dt;//ע�⣺dt��ȡֵΪkalman�˲�������ʱ��
double P[2][2] = {{ 1, 0 },{ 0, 1 }};
double Q[2][2]={{0.1,0},{0,0.5}};
double Pdot[4] ={0,0,0,0};
double R[3]={0.1,0.095};//����ͳ�ƣ�roll�����������ֵΪ0.1 pitchΪ0.095


//�������˲�
double Kalman_Filter(float angle_acc, float angle_w,int mode)//angleAx �� gyroGy
{
  double theta;
	theta=angle_w+(Pdot[0]/(Pdot[0]+R[mode]))*(angle_acc-angle_w);
	P[0][0]=Pdot[0]*R[mode]/(Pdot[0]+R[mode]);
	P[0][1]=Pdot[1]*R[mode]/(Pdot[0]+R[mode]);
	P[1][0]=Pdot[2]-Pdot[0]*Pdot[2]/(Pdot[0]+R[mode]);
	P[1][1]=Pdot[3]-Pdot[1]*Pdot[2]/(Pdot[0]+R[mode]);
	Pdot[0]=P[0][0]+P[1][0]*dt+P[0][1]*dt+P[1][1]*dt*dt+Q[0][0];
	Pdot[1]=P[0][1]+P[1][1]*dt+Q[0][1];
	Pdot[2]=P[1][0]+P[1][1]*dt+Q[1][0];
	Pdot[3]=P[1][1]+Q[1][1];
 
  return theta;

}