#include "sdk_V3_use.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"

#include <stdio.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

std::string  g_strLidarID = "";
std::string strPort = "/dev/ttyUSB0";              // For Linux OS
int iBaud = 115200;
std::string strLidarModel = "X2M";
bool inverted_=true;//是否反转

LstNodeDistQ3 lstG;

std::mutex mu;						//互斥锁
std::condition_variable cond;       //条件变量

void push_Scan(
                ros::Publisher *pub,
                std::string frame_id,
                ros::Time now,
                double scan_time,
                float angle_min, float angle_max,
                size_t node_count,
                float max_distance
                ){
	// tsNodeInfo msg= lstG.front();
	// lstG.pop_front();
	sensor_msgs::LaserScan scan_msg;

    scan_msg.header.frame_id = frame_id;
    scan_msg.header.stamp = now;

	scan_msg.angle_min =  angle_min;
    scan_msg.angle_max =  angle_max;

    scan_msg.angle_min =  M_PI - angle_min;
    scan_msg.angle_max =  M_PI - angle_max;
    scan_msg.angle_increment = DEG2RAD(0.9);


    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = max_distance;//8.0;

    scan_msg.intensities.resize(node_count);
    scan_msg.ranges.resize(node_count);

    for (size_t i = 0; i < node_count; i++) {
        float read_value = lstG.at(i).distance_q2/4.0f/1000;
        if (read_value == 0.0)
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[i] = read_value;
        scan_msg.intensities[i] = (float) (lstG.at(i).syn_quality );
    }
	
    pub->publish(scan_msg);
}
int rtn = 0;
bool bPollMode = true;
bool bDistQ2 = true;
bool bLoop = false;

bool ralidar_Init(void)
{

	std::string strVer = getSDKVersion();
    std::cout << "Main: SDK verion=" << strVer.c_str()<< std::endl;

    auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
	setSDKCallBackFunErrorCode(funErrorCode);

    auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
    setSDKCallBackFunSecondInfo(funSecondInfo);

    if(!bPollMode)//call back
    {
        auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
        setSDKCallBackFunPointCloud(funPointCloud);

        auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
        setSDKCallBackFunDistQ2(funDistQ2);
    }

	int iReadTimeoutms = 2;//10

	setSDKCircleDataMode();
	rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);

    if (rtn != 1)
    {
		hcSDKUnInit();
		ROS_INFO("Main: Init sdk failed!\n");
		getchar();
		exit(0);
		return false;
        
    }

	setSDKLidarPowerOn(true);
	setSDKCircleDataMode();

	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	g_strLidarID = getSDKLidarID();
	
	
	ROS_INFO( "Lidar ID=%s\n" , getSDKLidarID());
	ROS_INFO( "Factory Info:%s\n" , getSDKFactoryInfo());
	ROS_INFO( "Main: Firmware ver:%s\n", getSDKFirmwareVersion() );
	ROS_INFO( "Main: Hardware ver:%s\n", getSDKHardwareVersion());
	ROS_INFO( "Main: Lidar model:%s\n" , getSDKLidarModel() );

    return true;
}


short Data_Full_Flag=0;
void ralidar_RUN(void){

    while(lstG.size() <400){
        getSDKScanData(lstG, false);
    }

    //排序从小到大
    if(!inverted_){
        std::stable_sort(lstG.begin(), lstG.begin()+lstG.size(), lessmark_reverse);
        ROS_INFO("reverse\n");
    }else{
        std::stable_sort(lstG.begin(), lstG.begin()+lstG.size(), lessmark);
    }
}

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "HC_rplidar_node");

    std::string serial_port = "/dev/lidar";
    int serial_baudrate = 115200;
    std::string frame_id = "laser";
    bool inverted = false;//反转
    float max_distance = 8.0;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, true);
    

    strPort = serial_port;              // For Linux OS
    iBaud = serial_baudrate;
    inverted_ = inverted;


    ROS_INFO("HCralidar running on ROS package rplidar_ros. ");

    if(ralidar_Init()){
        ROS_INFO("Init successfully");
    }
    //打开串口

    

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;
    size_t   count=0;
    lstG.clear();
    while(lstG.size()<100) getSDKScanData(lstG, false);
    lstG.clear();

	//std::thread receiverThread(ralidar_RUN);
    while (ros::ok()) {    


        //push_Scan
        start_scan_time= ros::Time::now();
        while(lstG.size() <395){
            getSDKScanData(lstG, false);
        }
        end_scan_time = ros::Time::now();

        //排序从小到大
        if(inverted_){
            std::stable_sort(lstG.begin(), lstG.begin()+lstG.size(), lessmark_reverse);
        }else{
            std::stable_sort(lstG.begin(), lstG.begin()+lstG.size(), lessmark);
        }

        // printf("This is a new data\n");

        // for(int i=0;i<lstG.size();i++){
        //     printf("[%.3f]",(lstG.at(i)).angle_q6_checkbit/64.0f);
        // }
        // printf("\n");
        count = lstG.size();
        scan_duration = (end_scan_time - start_scan_time).toSec();


        int start_node = 0, end_node = count -1;
        // find the first valid node and last valid node
        while (lstG.at(++start_node).distance_q2 == 0);
        --start_node;
        while (lstG.at(--end_node).distance_q2 == 0);
        ++end_node;
        float angle_min = DEG2RAD((lstG.at(start_node).angle_q6_checkbit)/64.0f);
        float angle_max = DEG2RAD((lstG.at(end_node).angle_q6_checkbit)/64.0f);

        
        
        //ROS_INFO("SIZE:[%d] , scan_time:[%.4f], angle_min:[%.4f], angle_max:[%.4f]\n",count,scan_duration,angle_min,angle_max);
        push_Scan(&scan_pub,
                frame_id,
                start_scan_time,
                scan_duration,
                angle_min, angle_max,
                count,
                max_distance
                );
    
        //clear
        lstG.clear();
        ros::spinOnce();
    }

    // done!
	hcSDKUnInit();
    // drv->stop();
    // drv->stopMotor();
    // RPlidarDriver::DisposeDriver(drv);
    return 0;
}


//将temp内的node移动回原列表
void huigui_node(void){

    // while(lstG_temp.empty() != true){
    //     lstG.push_back(lstG_temp.back());//把最后一个转移
    //     //ROS_INFO("Nowing Huigui list node is=%0.4f  " ,(double)(lstG.back()).angle_q6_checkbit/64.0f);
    //     lstG_temp.pop_back();//删除
    // }
    //ROS_INFO("\n");
}

void angle_(int count){
    int angle_compensate_multiple = 1;

    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
    int angle_compensate_offset = 0;
    // LstNodeDistQ2 lstG_temp;

    // LstNodeDistQ2::iterator  p=lstG.begin();//定义迭代器 用于遍历
    // for(auto sInfo : lstG)
    // {
    //     float angle = (sInfo.angle_q6_checkbit)/64.0f;
    //     int angle_value = (int)(angle * angle_compensate_multiple);

    //      if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
    //     int i = 0, j = 0;
    //     //std::cout << "Main: Angle=" << (double)sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
    //     for (j = 0; j < angle_compensate_multiple; j++) {//只修正一个
    //             //angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
    //     }

    //     ++p;
    // }

    // int i = 0, j = 0;
    // for( ; i < count; i++ ) {
    //     if (nodes[i].dist_mm_q2 != 0) {//距离不是0
    //         //计算当前角度，如果角度比上次小则，记录
    //         float angle = getAngle(nodes[i]);
    //         int angle_value = (int)(angle * angle_compensate_multiple);
    //         if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
    //         for (j = 0; j < angle_compensate_multiple; j++) {//只修正一个
    //             angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
    //         }
    //     }
    // }
}