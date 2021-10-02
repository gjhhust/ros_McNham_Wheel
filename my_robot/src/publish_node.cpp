/**************************
 * 使用该代码以启动机器人
 * 
 * 
 * ************************/

#include "serial.h"

//test send value
Chassis_ctrl_structure Chassis_ctrl={0};

double wheel_distance=0;        //机器人底盘宽度
double speed_radio=0;           //速度标定系数
double sampling_time=0;         //速度采集频率



//pub odom
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom;
std_msgs::Float32 left_speed_msg,right_speed_msg;

/********************************************************
函数功能：订阅/cmd_vel控制 计算两个轮子的分别速度，计算控制位
入口参数：
出口参数：
********************************************************/  
void speed_vel_Callback(const geometry_msgs::Twist msg)  
{  
    // 将接收到的消息打印出来  
    //ROS_INFO("Turtle speed: x:%0.6f, z:%0.6f", msg.linear.x, msg.angular.z); 
    Chassis_ctrl.x_speed = msg.linear.x;
    Chassis_ctrl.y_speed = msg.linear.y;
    Chassis_ctrl.w_speed = msg.angular.z;


    if(Chassis_ctrl.x_speed>speed_limit){
        Chassis_ctrl.x_speed=speed_limit;
    }
    if(Chassis_ctrl.x_speed< -speed_limit){
        Chassis_ctrl.x_speed=-speed_limit;
    }
    if(Chassis_ctrl.y_speed>speed_limit){
        Chassis_ctrl.y_speed=speed_limit;
    }
    if(Chassis_ctrl.y_speed< -speed_limit){
        Chassis_ctrl.y_speed=-speed_limit;
    }
    if(Chassis_ctrl.w_speed >angle_speed_limit){
        Chassis_ctrl.w_speed = angle_speed_limit;
    }
    if(Chassis_ctrl.w_speed <-angle_speed_limit){
        Chassis_ctrl.w_speed =-angle_speed_limit;
    }

    //ROS_INFO("发布的速度: riht_speed:%d\tleft_speed:%d,wheel_distance:%0.5f",right_speed, left_speed,wheel_distance); 
}  



 
int main(int argc, char** argv)
{
    //初始化ROS节点
    ros::init(argc, argv, "robot_start");
    ros::NodeHandle nh;

    //初始化robot
    robot::robot myrobot;
    if(!myrobot.init())
        ROS_ERROR("myrobot start failed.");
    ROS_INFO("myrobot start successful");




    wheel_distance=myrobot.wheel_distance;        //机器人底盘宽度
    speed_radio=myrobot.speed_radio;           //速度标定系数
    sampling_time=myrobot.sampling_time;         //速度采集频率
    //ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);
    //创建一个订阅者订阅键盘控制节点
    ros::Subscriber speed_vel = nh.subscribe("/cmd_vel", 1, speed_vel_Callback);//subscribe("/cmd_vel", 10, speed_vel_Callback);  

    //循环运行
    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        ros::spinOnce();
        // 机器人控制
        myrobot.task_start(100*Chassis_ctrl.x_speed, 100*Chassis_ctrl.y_speed, 10 * Chassis_ctrl.w_speed);
        loop_rate.sleep();
    }

    return 0;
}
