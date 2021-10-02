#ifndef __MAIN_H_
#define __MAIN_H_

//标准库函数
#include "stm32f10x.h"
#include <math.h>
#include <stdio.h>  
#include <string.h>  
#include <stdbool.h>
#include <stdarg.h>

//FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//外设配置
#include "i2c_init.h"
#include "can1_chassis.h"
#include "usart1_wifi.h"
#include "usart2_PC.h"
#include "usart3_remote.h"
#include "usart5_calib.h"
#include "bsp_esp8266_test.h"
#include "Common.h"
#include "bsp_esp8266.h"
#include "delay.h"
#include "usart.h"
//任务
#include "Data_Dispose.h"
#include "chassis_task.h"
#include "action_task.h"
#include "led_task.h"
#include "PCReceiveTask.h"
#include "AutoTask.h"
//算法
#include "KF.h"
#include "pid.h"
#include "algorithmOfCRC.h"

typedef struct
{
char Mode;
short carSpeed[4];
float YawAngle;
float YawAngleSpeed;
Pid_Typedef PIDwheel[4];
Pid_Typedef SpeedPID[4];
Pid_Typedef PositionPID[4];
float V_x_now;
float V_y_now;	
} chassis;

#define limit_max_min(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

#endif



