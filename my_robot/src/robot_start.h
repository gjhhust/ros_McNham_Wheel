#ifndef ROBOT_START
#define ROBOT_START

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include "std_msgs/Float32.h" 
#include "turtlesim/Pose.h"  
#include "serial.h"
#include <vector>

#define speed_limit 35 //cm/s
#define angle_speed_limit 6 //cm/s
#define angle_change_rate 88 //角度从底盘以degree单位数值*88发送上来
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))
typedef struct{
    double x_speed;
    double y_speed;
    double w_speed;
}Chassis_ctrl_structure;

namespace robot
{
    class robot
    {
        public:
            bool init();                  
            bool task_start(double V_x, double V_y,double V_z);
            double wheel_distance;        //机器人底盘宽度
            double speed_radio;           //速度标定系数
            double sampling_time;         //速度采集频率
        
        private:
            void cal_Odom();               //里程计计算
            void pub_OdomAndTf();           //发布Odom和tf
        
        private:
            ros::Time current_time, last_time; //时间

            double position_x;                     //机器人位姿
            double position_y;
            double yaw;
            double position_x_last;                     //上一时间机器人位姿
            double position_y_last;
            double yaw_last;

            double V_x_Actual;                    //机器人实际左轮速度m/s
            double V_y_Actual;                    //机器人右轮速度
            double V_w_Actual;                   //机器人角速度
            unsigned char sensFlag;       //通信预留发送和接收标志位，可进行信号控制使用
            
            ros::NodeHandle n;
            ros::Publisher pub;
            ros::Publisher yaw_angle;
            tf::TransformBroadcaster odom_broadcaster;
    };
    
}

#endif