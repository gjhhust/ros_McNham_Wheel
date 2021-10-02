#include "LidarTest.h"
#include <chrono>



LidarTest::LidarTest()
{
	printf("LidarTest: LidarTest()\n");
	m_device = nullptr;

	m_threadWork = std::thread(&LidarTest::initLidar, this);
}
LidarTest::~LidarTest()
{
	printf("LidarTest: ~LidarTest()\n");
	m_bRun = false;
	if (m_threadWork.joinable())
		m_threadWork.join();

	if (m_device)
	{
		m_device->unInit();

		delete m_device;
		m_device = nullptr;
	}
}


void LidarTest::sdkCallBackFunErrorCode(int iErrorCode)
{
	printf("HCSDK Main: sdkCallBackFunErrorCode ErrorCode=%d" , iErrorCode);
}

void LidarTest::sdkCallBackFunSecondInfo(tsSDKStatistic sInfo)
{
	printf("HCSDK Main: sdkCallBackFunSecondInfo time=%lld s,points=%d,GrayBytes=%d,FPS=%lld,speed=%0.2f,PPS=%d,valid=%d,invalid=%d,ErrorPacket=%lld\n" , 
		sInfo.u64TimeStampS , sInfo.iNumPerPacket, sInfo.iGrayBytes , sInfo.u64FPS
		,sInfo.dRMS , sInfo.iPacketPerSecond,sInfo.iValid , sInfo.iInvalid
		, sInfo.u64ErrorPacketCount);
}

void LidarTest::sdkCallBackFunPointCloud(LstPointCloud lstG)
{
	printf("HCSDK Main: sdkCallBackFunPointCloud Rx Points=%d\n",lstG.size());
	for (auto sInfo : lstG)
	{
		//std::cout << "Main: Angle=" << sInfo.dAngle  << ",AngleRaw=" << sInfo.dAngleRaw << ",Dist=" << sInfo.u16Dist << std::endl;
	}
}

void LidarTest::sdkCallBackFunDistQ2(LstNodeDistQ2 lstG)
{
	std::cout << "Main: sdkCallBackFunDistQ2 Rx Points=" << lstG.size() << std::endl;
	for (auto sInfo : lstG)
	{
		//std::cout << "Main: Angle=" << sInfo.angle_q6_checkbit / 64.0f << ",Dist=" << sInfo.distance_q2 / 4 << std::endl;
	}
}


void LidarTest::initLidar()
{
	HCLidar& device = HCLidar::getInstance();
	int rtn = 0;

	bool bPollMode = true;
	bool bDistQ2 = false;
	bool bLoop = false;

	printf("Main: SDK verion=%s\n" , device.getSDKVersion().c_str() );

	/*
	auto funErrorCode = std::bind(&LidarTest::sdkCallBackFunErrorCode, this,std::placeholders::_1);
	device.setCallBackFunErrorCode(funErrorCode);

	auto funSecondInfo = std::bind(&LidarTest::sdkCallBackFunSecondInfo, this,std::placeholders::_1);
	device.setCallBackFunSecondInfo(funSecondInfo);

	if (!bPollMode)//call back
	{
		auto funPointCloud = std::bind(&LidarTest::sdkCallBackFunPointCloud, this,std::placeholders::_1);
		device.setCallBackFunPointCloud(funPointCloud);

		auto funDistQ2 = std::bind(&LidarTest::sdkCallBackFunDistQ2, this,std::placeholders::_1);
		device.setCallBackFunDistQ2(funDistQ2);
	}
	*/



	int iBaud = 115200;
	int iReadTimeoutms = 10;//
	// ##### 1. Open serial port using valid COM id #####
#ifdef _WIN32
	rtn = device.initialize("//./com3", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);                     // For windows OS
#else
	rtn = device.initialize("/dev/ttyPort1", "X2M", iBaud, iReadTimeoutms, bDistQ2, bLoop, bPollMode);               // For Linux OS
#endif

	if (rtn != 1)
	{
		int iErrorCode = device.getLastErrCode();
		if (iErrorCode == ERR_SERIAL_INVALID_HANDLE || iErrorCode == ERR_SERIAL_READFILE_FAILED)
		{
			device.unInit();
			printf("LidarTest: Init sdk failed!\n");
			return ;
		}

	}
		
	//Power for lidar

	if (!device.getLidarInfo())
	{
		device.unInit();
		printf("LidarTest: Get lidar info failed!\n");
		return;
		
	}

	printf("LidarTest: Lidar ID=%s\n", device.getLidarID().c_str());
	printf("LidarTest: Factory Info:%s\n", device.getFactoryInfo().c_str());
	printf("LidarTest: Firmware ver:%s\n", device.getFirmwareVersion().c_str());
	printf("LidarTest: Hardware ver:%s\n", device.getHardwareVersion().c_str());
	printf("LidarTest: Lidar model:%s\n", device.getLidarModel().c_str());

	while (m_bRun)
	{

		if(bPollMode)
		{
		    if(bDistQ2)
		    {
		        LstNodeDistQ2 lstG;
		        device.getScanData(lstG, false);

		        std::cout << "Main: Poll DistQ2 Rx Points=" << lstG.size() <<std::endl;
		        for(auto sInfo : lstG)
		        {
		            //std::cout << "Main: Angle=" << (double)sInfo.angle_q6_checkbit/64.0f  << ",Dist=" << sInfo.distance_q2/4 << std::endl;
		        }
		    }
		    else
		    {
		        LstPointCloud lstG;
				if (device.getRxPointClouds(lstG))
				{
					printf("LidarTest: Poll Rx Points=%d\n", lstG.size());
					for (auto sInfo : lstG)
					{
						//printf("LidarTest: Angle=%0.4f,Dist=%d\n", sInfo.dAngle, sInfo.u16Dist);
					}
				}
				else
				{
					int iError = device.getLastErrCode();
					if (iError != LIDAR_SUCCESS)
					{
						printf("LidarTest: Poll Rx Points error code=%d" , iError );
						switch (iError)
						{
						case ERR_SHARK_MOTOR_BLOCKED:
							break;
						case ERR_SHARK_INVALID_POINTS:
							break;
						case ERR_LIDAR_SPEED_LOW:
							break;
						case ERR_LIDAR_SPEED_HIGH:
							break;
						case ERR_DISCONNECTED:
							break;
						case ERR_LIDAR_FPS_INVALID:
							break;
						default:
							break;
						}
					}
				}
					                
		    }
		}
		int iSDKStatus = device.getSDKStatus();


		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		std::this_thread::yield();
	}

	device.unInit();
}

