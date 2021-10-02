#ifndef _MPU6050_H_
#define _MPU6050_H_
  //x-roll  y-pitch z-yaw
#define MPU6050_WHO_AM_I        0x75    
#define MPU6050_GYRO_OUT        0x43              //MPU6050���������ݼĴ�����ַ
#define MPU6050_ACC_OUT         0x3B              //MPU6050���ٶ����ݼĴ�����ַ
#define w_scale                 0.03048780488f    //1/32.8  ��Ϊ�˶�ÿ�룬  1����Ϊ57.3��
void MPU_6050Init();
void MPU_6050Read();
struct Angle
{
double pitch;
double yaw;
double roll;
};

#endif