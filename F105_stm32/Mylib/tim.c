#include "main.h"
extern struct Angle w,acc;
extern short  acc_pitch,acc_roll,acc_yaw;
short w_x,w_y,w_z;
void tim5_init()
{
  TIM_TimeBaseInitTypeDef tim;
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel=TIM5_IRQn;
	nvic.NVIC_IRQChannelSubPriority=0;
	nvic.NVIC_IRQChannelPreemptionPriority=1;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	tim.TIM_ClockDivision=TIM_CKD_DIV1;
	tim.TIM_CounterMode=TIM_CounterMode_Up;
	tim.TIM_Period=60000-1;
	tim.TIM_Prescaler=72-1;
	TIM_TimeBaseInit(TIM5,&tim);
	
	TIM_ARRPreloadConfig(TIM5,ENABLE);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5,DISABLE);
}
void TIM5_IRQHandler(void)
{
   if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET)
	 {
	   TIM_Cmd(TIM5,DISABLE);
	   TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	
   w_x=(iic_read_REG(MPU6050_GYRO_OUT)<<9|iic_read_REG(MPU6050_GYRO_OUT+1));
   w_y=(iic_read_REG(MPU6050_GYRO_OUT+2)<<9|iic_read_REG(MPU6050_GYRO_OUT+3));
	 w_z=(iic_read_REG(MPU6050_GYRO_OUT+4)<<9|iic_read_REG(MPU6050_GYRO_OUT+5));	
    		
	 w.roll=w_x*w_scale;
   w.pitch=w_y*w_scale;
	 w.yaw=w_z*w_scale;
			
		acc_roll=(iic_read_REG(MPU6050_ACC_OUT)<<9|iic_read_REG(MPU6050_ACC_OUT+1));
	  acc_pitch=(iic_read_REG(MPU6050_ACC_OUT+2)<<9|iic_read_REG(MPU6050_ACC_OUT+3));
	  acc_yaw=(iic_read_REG(MPU6050_ACC_OUT+4)<<9|iic_read_REG(MPU6050_ACC_OUT+5));
   
	 acc.yaw=acc_yaw-551;//补偿加速度计偏差
	 acc.roll=acc_roll-10;
	 acc.pitch=acc_pitch+441;
	 
		TIM_SetCounter(TIM5,0);
		TIM_Cmd(TIM5,ENABLE);
	 }
}