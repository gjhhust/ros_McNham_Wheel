#include "robot_start.h"
using namespace std;


namespace robot
{   boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    /********************************************************
    函数功能：串口参数初始化、时间变量初始化、实例化发布对象
    入口参数：无
    出口参数：bool
    ********************************************************/
    bool robot::init()
    {
        // 串口初始化连接
        serialInit();
               
        ros::Time::init();
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        position_x_last=0; 
        position_y_last=0;
        yaw_last=0;
        sensFlag=0;


        wheel_distance = 0.185;
        sampling_time = 0.02;
        //定义发布消息的名称
        pub = n.advertise<nav_msgs::Odometry>("odom", 1);		
        yaw_angle = n.advertise<std_msgs::Float32>("yaw_angle", 1);
        
        task_start(0, 0, 0);
        return true;
    }
 
    /********************************************************
    函数功能：根据机器人线速度和角度计算机器人里程计
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::cal_Odom()
    {

        //cacular odom


        current_time = ros::Time::now();
        double theta = (yaw-yaw_last)/2 + yaw_last;
        double time = (current_time.toSec()-last_time.toSec());


        position_x = V_x_Actual*time;
        position_y = V_y_Actual*time;


        
        // ROS_INFO("center_d:%f,\t left_speed:%f \t time:%f\n",center_d,left_speed,time);



        position_x_last = position_x;
        position_y_last = position_y;
        yaw_last = yaw;
        last_time = current_time;
        // //打印位姿调试信息，不用的时候可以关闭
        // ROS_INFO("position_x:%f\n",position_x);
        // ROS_INFO("position_y:%f\n",position_y);
        // ROS_INFO("yaw:%f\n",yaw);
    }
    /********************************************************
    函数功能：发布机器人里程计和TF
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::pub_OdomAndTf()
    {
        ros::Time time = ros::Time::now();
        // 发布TF
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id  = "base_footprint";

        geometry_msgs::Quaternion odom_quat;
        odom_quat = tf::createQuaternionMsgFromYaw(yaw);
        odom_trans.transform.translation.x = position_x;
        odom_trans.transform.translation.y = position_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans);

        // 发布里程计消息
        nav_msgs::Odometry msgl;
        msgl.header.stamp = time;
        msgl.header.frame_id = "odom";

        msgl.pose.pose.position.x = position_x;
        msgl.pose.pose.position.y = position_y;
        msgl.pose.pose.position.z = 0.0;
        msgl.pose.pose.orientation = odom_quat;
        msgl.pose.covariance = odom_pose_covariance;

        msgl.child_frame_id = "base_footprint";
        msgl.twist.twist.linear.x = V_x_Actual;
        msgl.twist.twist.linear.y = V_y_Actual;
        msgl.twist.twist.angular.z = V_w_Actual;
        msgl.twist.covariance = odom_twist_covariance;
    
        pub.publish(msgl);

    }
    /********************************************************
    函数功能：自定义deal，实现整合，并且发布TF变换和Odom
    入口参数：机器人线速度和角速度，调用上面三个函数
    出口参数：bool
    ********************************************************/
    bool robot::task_start(double V_x, double V_y,double V_w)
    {

        // 向STM32发送对机器人的预期控制速度，以及预留信号控制位
        writeSpeed(V_x, V_y, V_w,sensFlag);
        // 从STM32读取机器人实际速度和角度，以及预留信号控制位
        readSpeed(V_x_Actual, V_y_Actual, V_w_Actual, yaw);//速度接收CM/S  角度为100倍  单位 度

        //标定角度使用
        //std_msgs::Float32 yaw1;
        //yaw1.data = yaw/88.0;
        //yaw_angle.publish(yaw1);

        //ROS_INFO("\t yaw:%f\n",yaw);

        ROS_INFO("\t V_x_Actual: %.3f   V_y_Actual: %.3f V_w_Actual: %.3f  yaw:%f\n",V_x_Actual,V_y_Actual,V_w_Actual,yaw/88);
        yaw =  - yaw * (0.0174532) / angle_change_rate;//化为弧度
        V_x_Actual = V_x_Actual * 0.01;//实际修正到控制m/s
        V_y_Actual = V_y_Actual * 0.01;
        V_w_Actual = V_y_Actual * 0.01;

        // 里程计计算
        cal_Odom();
        // 发布TF变换和Odom
        pub_OdomAndTf();
    }
}

