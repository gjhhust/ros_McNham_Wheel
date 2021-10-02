
#include "sdk_V3_use.h"



extern std::string  g_strLidarID;
extern std::string strPort;              // For Linux OS
extern int iBaud;
extern std::string strLidarModel;


void sdkCallBackFunErrorCode(int iErrorCode)
{
	char buff[128] = { 0 };
	sprintf(buff, "Error code=%d\n",iErrorCode);
	printf(buff);
}

void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	/*printf("Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%d,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%d\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS, sInfo.iPacketPerSecond, sInfo.iValid, sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);*/

	std::string strFile = "";
	if(g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "FPS_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "FPS_" + g_strLidarID + ".csv";
	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
	sprintf(buff, "%lld,%d,%d,%lld,%0.2f,%d,%d,%d,%lld\n",
		sInfo.u64TimeStampS, sInfo.iNumPerPacket, sInfo.iGrayBytes, sInfo.u64FPS
		, sInfo.dRMS,  sInfo.iValid, sInfo.iInvalid, sInfo.iPacketPerSecond
		, sInfo.u64ErrorPacketCount);

	outFile.write(buff, strlen(buff));
	outFile.close();

	printf(buff);
	
}

void sdkCallBackFunPointCloud(LstPointCloud lstG)
{
 
	std::string strFile = "";
	if (g_strLidarID.size() > DEFAULT_ID_LEN)
		strFile = "Raw_" + g_strLidarID.substr(g_strLidarID.size() - DEFAULT_ID_LEN, DEFAULT_ID_LEN) + ".csv";
	else
		strFile = "Raw_" + g_strLidarID + ".csv";

	std::ofstream outFile;
	outFile.open(strFile, std::ios::app);

	char buff[128] = { 0 };
    for(auto sInfo : lstG)
    {
		
		memset(buff, 0, 128);
		sprintf(buff, "%lld,%0.3f,%0.3f,%d,%d,%d,%d\n",
			HCHead::getCurrentTimestampUs(), sInfo.dAngle, sInfo.dAngleRaw, sInfo.u16Dist, sInfo.bValid, sInfo.u16Speed, sInfo.u16Gray);

		outFile.write(buff, strlen(buff));

		printf(buff);
		

		
    }
		
	outFile.close();

	sprintf(buff, "---------------------------------------  %d\n",lstG.size());

	printf(buff);
}

void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
    std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() <<std::endl;
    for(auto sInfo : lstG)
    {
        //std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
    }
}



std::string getLidarModel()
{
	printf("Please select Lidar model:\n");
	std::string str = "X2M";
	std::cin >> str;
	std::transform(str.begin(), str.end(), str.begin(), ::toupper);
	return str;
}


//用于sort排序
bool lessmark(const tsNodeInfo& stItem1, const tsNodeInfo& stItem2)
 {
     return stItem1.angle_q6_checkbit > stItem2.angle_q6_checkbit;
 }

 bool lessmark_reverse(const tsNodeInfo& stItem1, const tsNodeInfo& stItem2)
 {
     return stItem1.angle_q6_checkbit < stItem2.angle_q6_checkbit;
 }
// int rtn = 0;

// bool bPollMode = true;
// bool bDistQ2 = true;
// bool bLoop = false;
// bool ralidar_Init(void)
// {

// 	std::string strVer = getSDKVersion();
//     std::cout << "Main: SDK verion=" << strVer.c_str()<< std::endl;

//     auto funErrorCode = std::bind(sdkCallBackFunErrorCode, std::placeholders::_1);
// 	setSDKCallBackFunErrorCode(funErrorCode);

//     auto funSecondInfo = std::bind(sdkCallBackFunSecondInfo, std::placeholders::_1);
//     setSDKCallBackFunSecondInfo(funSecondInfo);

//     if(!bPollMode)//call back
//     {
//         auto funPointCloud = std::bind(sdkCallBackFunPointCloud, std::placeholders::_1);
//         setSDKCallBackFunPointCloud(funPointCloud);

//         auto funDistQ2 = std::bind(sdkCallBackFunDistQ2, std::placeholders::_1);
//         setSDKCallBackFunDistQ2(funDistQ2);
//     }

	


// 	int iReadTimeoutms = 2;//10

// 	setSDKCircleDataMode();
// 	rtn = hcSDKInitialize(strPort.c_str(), strLidarModel.c_str(), iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);

//     if (rtn != 1)
//     {
// 		hcSDKUnInit();
// 		printf("Main: Init sdk failed!\n");
// 		getchar();
// 		exit(0);
// 		return 0;
        
//     }

// 	setSDKLidarPowerOn(true);

// 	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
// 	/*if (!getSDKLidarInfo())
// 	{
// 		hcSDKUnInit();
// 		printf("Main: No lidar ID, please try again!\n");
// 		getchar();
// 		exit(0);
// 		return 0;
// 	}*/
// 	g_strLidarID = getSDKLidarID();
	
	
// 	printf( "Lidar ID=%s\n" , getSDKLidarID());
// 	printf( "Factory Info:%s\n" , getSDKFactoryInfo());
// 	printf( "Main: Firmware ver:%s\n", getSDKFirmwareVersion() );
// 	printf( "Main: Hardware ver:%s\n", getSDKHardwareVersion());
// 	printf( "Main: Lidar model:%s\n" , getSDKLidarModel() );

	

// 	hcSDKUnInit();
//     return true;

// }

// int iCount = 0;
// LidarTest  *lidarTest = nullptr;
// LstNodeDistQ2 lstG;
// void ralidar_RUN(void){
// 	if(bPollMode)
//         {
//             if(bDistQ2)
//             {
                
//                 getSDKScanData(lstG, false);
// 				// printf( "Main: Poll DistQ2 Rx Points=%d\n" ,lstG.size() );
//                 // for(auto sInfo : lstG)
//                 // {
// 				// 	printf("Main: Angle=%0.2f,Dist=%d\n" ,(double)sInfo.angle_q6_checkbit/64.0f  , sInfo.distance_q2/4 );
//                 // }
//             }
//             else
//             {
//                 LstPointCloud lstG;
// 				if (getSDKRxPointClouds(lstG))
// 				{
// 					if (lstG.size() > 0)
// 					{
// 						printf("Main: ------------------------------Poll Rx Points=%d\n", lstG.size());
// 						for (auto sInfo : lstG)
// 						{
// 							if(sInfo.dAngle>70 && sInfo.dAngle < 110)
// 								//printf("Main: Angle=%0.4f,Raw_Angle=%0.4f,Dist=%d\n", sInfo.dAngle, sInfo.dAngleRaw,sInfo.u16Dist);//
// 							printf( "Main: Angle=%0.4f,Dist=%d,Gray=%d\n", sInfo.dAngle , sInfo.u16Dist, sInfo.u16Gray);
// 						}
// 					}
					
// 				}
// 				else
// 				{
// 					int iError = getSDKLastErrCode();
// 					if (iError != LIDAR_SUCCESS)
// 					{
// 						printf( "Main: Poll Rx Points error code=%d\n", iError );
// 						switch (iError)
// 						{
// 						case ERR_SHARK_MOTOR_BLOCKED:
// 							break;
// 						case ERR_SHARK_INVALID_POINTS:
// 							break;
// 						case ERR_LIDAR_SPEED_LOW:
// 							break;
// 						case ERR_LIDAR_SPEED_HIGH:
// 							break;
// 						case ERR_DISCONNECTED:
// 							break;
// 						case ERR_LIDAR_FPS_INVALID:
// 							break;
// 						default:
// 							break;
// 						}
// 					}
// 				}
				                
//             }
//         }
//         int iSDKStatus = getSDKStatus();
// 		//printf("Main: SDK Status=%d\n" ,iSDKStatus );

// 		iCount++;
// 		if (iCount > 300)
// 		{
// 			iCount = 0;
// 			//g_strLidarID = getSDKLidarID();
// 			//printf("Lidar ID=%s\n", g_strLidarID.c_str());
// 		}
		

//         std::this_thread::sleep_for(std::chrono::milliseconds(10));
//         std::this_thread::yield();
// }

// void ralidar_delete(void){
// 	if (lidarTest) 
// 	{
// 		delete lidarTest;
// 		lidarTest = nullptr;
// 	}
// }
