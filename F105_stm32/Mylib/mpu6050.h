#ifndef _MPU6050_H_
#define _MPU6050_H_
  //x-roll  y-pitch z-yaw
#define MPU6050_WHO_AM_I        0x75    
#define MPU6050_GYRO_OUT        0x43              //MPU6050陀螺仪数据寄存器地址
#define MPU6050_ACC_OUT         0x3B              //MPU6050加速度数据寄存器地址
#define w_scale                 0.03048780488f    //1/32.8  化为了度每秒，  1弧度为57.3°
void MPU_6050Init();
void MPU_6050Read();
struct Angle
{
double pitch;
double yaw;
double roll;
};

#endif