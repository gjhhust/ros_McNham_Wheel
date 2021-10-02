#ifndef _HCSDK_H_
#define _HCSDK_H_

#include "HcData.h"

#ifdef __cplusplus
extern "C" {
#endif

	bool hcSDKInitialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2, bool bGetLoopData, bool bPollMode);
	bool hcSDKUnInit();

	//set callback function for error code
	void setSDKCallBackFunErrorCode(CallBackFunErroCode fun);

	//set callback function for Statistic infomation
	void setSDKCallBackFunSecondInfo(CallBackFunSecondInfo fun);

	//set callback function for rx pointclouds
	void setSDKCallBackFunPointCloud(CallBackFunPointCloud fun);

	//set callback function for rx Distance Q2
	void setSDKCallBackFunDistQ2(CallBackFunDistQ2 fun);

	//get error code
	int getSDKLastErrCode();

	bool getSDKLidarInfo();

	//get SDK status
	int getSDKStatus();

	// get SDK Version
	char* getSDKVersion();

	//get lidar ID
	char* getSDKLidarID();


	// get factory infomation
	char* getSDKFactoryInfo();

	//get lidar model
	char* getSDKLidarModel();

	//get firmware version
	char* getSDKFirmwareVersion();

	//get hardware version
	char* getSDKHardwareVersion();

	// set work parameter
	void setSDKWorkPara(tsSDKPara& sSDKPara);

	//poll mode,get pointclouds
	bool getSDKRxPointClouds(LstPointCloud& lstG);

	//poll mode,get ScanData
	bool getSDKScanData(std::vector<tsNodeInfo>& dataList, bool bReverse = true);

	//Factory mode for test
	void setSDKFactoryMode(bool bFactoryMode = true);

	//start the factory mode 
	bool startSDKFactoryModeRun();
	
	//set one circle data to output 
	void setSDKCircleDataMode();

	//set the lidar status is powered
	void setSDKLidarPowerOn(bool bPowerOn);

	void setSDKLidarLowSpeed(bool bLow);

#ifdef __cplusplus
};
#endif

#endif

