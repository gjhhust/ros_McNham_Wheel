
#include "HcSDK.h"
#include <string>
#include <iostream>
#include "hclidar.h"

#ifdef __cplusplus
extern "C" {
#endif

std::string  g_strSDKVer = "";
std::string  g_strSDKLidarID = "";
std::string  g_strSDKFactoryInfo = "";
std::string  g_strSDKLidarModel = "";
std::string  g_strSDKFirmwareVersion = ""; 
std::string  g_strSDKHardwareVersion = ""; 

HCLidar& g_device = HCLidar::getInstance();

bool hcSDKInitialize(const char* chPort, const char* chLidarModel, int iBaud, int iReadTimeoutMs, bool bDistQ2, bool bGetLoopData, bool bPollMode)
{
	return g_device.initialize(chPort, chLidarModel, iBaud, iReadTimeoutMs, bDistQ2, bGetLoopData, bPollMode);
}

bool hcSDKUnInit()
{
	return g_device.unInit();
}
void setSDKCallBackFunErrorCode(CallBackFunErroCode fun)
{
	g_device.setCallBackFunErrorCode(fun);
}

void setSDKCallBackFunSecondInfo(CallBackFunSecondInfo fun)
{
	g_device.setCallBackFunSecondInfo(fun);
}

//set callback function for rx pointclouds
void setSDKCallBackFunPointCloud(CallBackFunPointCloud fun)
{
	g_device.setCallBackFunPointCloud(fun);
}

//set callback function for rx Distance Q2
void setSDKCallBackFunDistQ2(CallBackFunDistQ2 fun)
{
	g_device.setCallBackFunDistQ2(fun);
}

//get error code
int getSDKLastErrCode()
{
	return g_device.getLastErrCode();
}

bool getSDKLidarInfo()
{
	return g_device.getLidarInfo();
}

//get SDK status
int getSDKStatus()
{
	return g_device.getSDKStatus();
}

// get SDK Version
char* getSDKVersion()
{
	g_strSDKVer = g_device.getSDKVersion();
	return (char*)g_strSDKVer.c_str();
}

//get lidar ID
char* getSDKLidarID()
{
	g_strSDKLidarID = g_device.getLidarID();
	return (char*)g_strSDKLidarID.c_str();
}


// get factory infomation
char* getSDKFactoryInfo()
{
	g_strSDKFactoryInfo = g_device.getFactoryInfo();
	return (char*)g_strSDKFactoryInfo.c_str();
}

//get lidar model
char* getSDKLidarModel()
{
	g_strSDKLidarModel = g_device.getLidarModel();
	return (char*)g_strSDKLidarModel.c_str();
}

//get firmware version
char* getSDKFirmwareVersion()
{
	g_strSDKFirmwareVersion = g_device.getFirmwareVersion();
	return (char*)g_strSDKFirmwareVersion.c_str();
}

//get hardware version
char* getSDKHardwareVersion()
{
	g_strSDKHardwareVersion = g_device.getHardwareVersion();
	return (char*)g_strSDKHardwareVersion.c_str();
}

// set work parameter
void setSDKWorkPara(tsSDKPara& sSDKPara)
{
	g_device.setWorkPara(sSDKPara);
}

//poll mode,get pointclouds
bool getSDKRxPointClouds(LstPointCloud& lstG)
{
	return g_device.getRxPointClouds(lstG);
}

//poll mode,get ScanData
bool getSDKScanData(std::vector<tsNodeInfo>& dataList, bool bReverse)
{
	return g_device.getScanData(dataList, bReverse);
}


void setSDKFactoryMode(bool bFactoryMode)
{
	g_device.setFactoryMode(bFactoryMode);
}
bool startSDKFactoryModeRun()
{
	return g_device.startFactoryModeRun();
}

void setSDKCircleDataMode()
{
	g_device.setCircleDataMode();
}

void setSDKLidarPowerOn(bool bPowerOn)
{
	g_device.setLidarPowerOn(bPowerOn);
}

void setSDKLidarLowSpeed(bool bLow)
{
	g_device.setLidarLowSpeed(bLow);
}

#ifdef __cplusplus
};
#endif