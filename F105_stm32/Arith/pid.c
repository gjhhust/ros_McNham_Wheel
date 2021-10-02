#include "main.h"




float PID_Cal(Pid_Typedef *P)
{
		P->err[0] = P->SetPoint-P->RealPoint;     
		P->SumError += P->err[0];

		P->POut = P->P * P->err[0];
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * (P->err[0]-P->err[1]);
		P->err[1]=P->err[0];
		
		if(fabs(P->IOut) >= P->IMax)
			{
			if(P->IOut>0)P->IOut = P->IMax;
			else 	P->IOut = -P->IMax;
			}
		P->Out= P->POut+P->IOut+P->DOut;
		if(fabs(P->Out)>P->OutMax)
    {
			if(P->Out>0) P->Out = P->OutMax;
    	else   P->Out = -P->OutMax;		
		}	
		return P->Out;
}

void PID_init(Pid_Typedef * pid,float p,float i,float d,float imax,float outmax)
{
  pid->D= d;
	pid->I= i;
	pid->P= p;
	pid->IMax= imax;
	pid->OutMax= outmax;
}

