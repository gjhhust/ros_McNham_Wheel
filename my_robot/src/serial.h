#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include "robot_start.h" 







extern void serialInit();
extern void writeSpeed(short V_x, short V_y,short V_w,unsigned char ctrlFlag);
extern bool readSpeed(double &V_x_Actual,double &V_y_Actual,double &V_w_Actual,double &Angle);
unsigned char getCrc8(unsigned char *ptr, unsigned short len);

void odom_pub_calcu(void);
double sin_cal(double theta);
#endif
