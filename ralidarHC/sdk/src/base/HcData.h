#ifndef _HCDATA_H_
#define _HCDATA_H_

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <list>
#include <set>
#include <chrono>

typedef unsigned char              UCHAR;
typedef unsigned char              UINT8;
typedef unsigned short             USHORT;
typedef unsigned short             UINT16;
typedef unsigned long              ULONG;
typedef unsigned int               UINT;
typedef unsigned long              DWORD;
typedef int                        BOOL;
typedef unsigned char              BYTE;
typedef unsigned short             WORD;
typedef unsigned int               UINT32;
typedef unsigned long long int     UINT64;



#define TRUE                            1
#define FALSE                           0



typedef struct tsNodeInfo
{
	UINT16    syn_quality;
	UINT16    angle_q6_checkbit;
	UINT16    distance_q2;
	UINT16    isValid;// 1Valid,  0 invalid
   /* bool operator<(const tsNodeInfo& _Left) const
	{
		if (_Left.angle_q6_checkbit < this->angle_q6_checkbit)
		{
			return false;
		}

		return true;
	}*/
}tsNodeInfo;

typedef std::vector<tsNodeInfo> LstNodeDistQ2;

typedef struct tsSDKPara
{
	int           iNoDataMS;  //warning timeout ,ms, read data from serial port is null,default 1000ms
	int           iDisconnectMS;   //lidar disconnected��ms, defualt 3000ms
	int           iFPSContinueMS;  // FPS continue abnormal, ms ,default 5000ms
	int           iSpeedContinueMS; // speed continue abnormal, ms ,default 3500ms
	int           iCoverContinueMS;// covered  duration, ms ,default 3500ms
	int           iBlockContinueMS; // blocked  duration, ms ,default 3500ms
	int           iCoverPoints;  //
	int           iPollBuffSize;  //POLL mode ,buff max size
	int           iCallbackBuffSize; //Callback mode ,buff size
	int           iCirclesBuffSize;//POLL mode, circle max size
	int           iChangeSpeedMS; // change speed  duration, ms ,default 2500ms

	tsSDKPara()
	{
		iNoDataMS = 1000;
		iDisconnectMS = 3000;
		iFPSContinueMS = 5000;
		iSpeedContinueMS = 3500;
		iCoverContinueMS = 3500;
		iBlockContinueMS = 3500;
		iCoverPoints = 100;
		iPollBuffSize = 5000;
		iCallbackBuffSize = 50;
		iCirclesBuffSize = 3;
		iChangeSpeedMS = 2500;
	}
}tsSDKPara;


typedef enum enSDKStatus
{
	SDK_UNINIT = -1,
	SDK_INIT = 0,
	SDK_ID_TIMEOUT,
	SDK_NO_DATA,
	SDK_DISCONNECT,
	SDK_WORKING
}enSDKStatus;


enum LiDarErrorCode
{
	LIDAR_SUCCESS = 0,

	ERR_MODEL_NOT_EXISTS = -1000,//Lidar model not exists
	ERR_SERIAL_INVALID_HANDLE = -1001,//COM handle error
	ERR_SERIAL_SETCOMMTIMEOUTS_FAILED = -1002,//set COM timeouts
	ERR_SERIAL_READFILE_FAILED = -1003,//read COM Failed
	ERR_SERIAL_READFILE_ZERO = -1004,//read COM NULL
	ERR_FIND_HEAD_TIMEOUT = -1005,//Find packet header error  100ms
	ERR_CHECKDATA_INVALID = -1006,//packet cal failed
	ERR_GETPACKAGE_FAILED = -1007,//get packet failed
	ERR_DATABYTELENGTH_INVALID = -1008,//Lidar packet error
	ERR_RECEIVE_BUFFER_SMALL = -1009,//rx buffer too small
	ERR_NOT_ID = -1010,//init failed ,get ID Failed
	ERR_NO_DATA = -1011,//init failed ,no data rx
	ERR_MOTOR_BLOCKED = -1012,//motor blocked
	ERR_REBOOT_LIDAR = -1013,//lidar reboot
	ERR_SDK_HAD_BEEN_INIT = -1014,//SDK had been init
	ERR_SDK_INIT_PARA = -1015,//SDK init parameter error
	ERR_DISCONNECTED = -1016,//lidar disconnect
	ERR_CALLBACK_FUN = -1017,//Not register callback fun
	ERR_POLL_MODE = -1018,//poll/callback mode error
	ERR_BUFF_FULL = -1019,//poll  mode  pointclouds buff full
	ERR_START_INFO = -1020,//Start info error
	ERR_DEV_MODEL = -1021,//device model error
	ERR_RX_CONTINUE = -1022,//continues rx error packet

	ERR_SHARK_MOTOR_BLOCKED = -1515,//motor blocked for shark
	ERR_SHARK_INVALID_POINTS = -1516,//invalid points for shark


	//Lidar erro
	ERR_LIDAR_FPS_INVALID = -3001,//fps , continue 5000ms default.
	ERR_LIDAR_SPEED_LOW = -3002,//speed low, continue 3500ms default.
	ERR_LIDAR_SPEED_HIGH = -3003,//speed high, continue 3500ms default.
	ERR_LIDAR_NUMBER_INVALID = -3004,//pointcloud too little,1 circle valid < 50, and continue 50 circles
	ERR_LIDAR_SAME_ANGLE = -3005, //continue same a angle
	ERR_LIDAR_ENCODER = -3006,// Lidar encoder error,after powered 5s ,speed 0 and 1023 in a second,and no blocked msg
	ERR_LIDAR_SENSOR = -3007,// Lidar sensor error,after powered 5s ,FPS=0 and ERR_LIDAR_FPS_INVALID,and no blocked msg  in a second
	ERR_LIDAR_VOLTAGE = -3008,// Lidar voltage error, 5s no ID,no dist data
	ERR_LIDAR_PD_CURRENT = -3009,// Lidar current error, speed ok,FPS ok, no valid pointclouds
	ERR_LIDAR_CHANGE_SPEED = -3010,// Lidar change speed timeout,
};


typedef struct tsSDKStatistic
{
	UINT64        u64TimeStampS;
	UINT64        u64TSRxPacketFirst;
	UINT64        u64RxPacketCount;
	UINT64        u64ErrorPacketCount;
	int           iErrorCountContinue;
	int           iPortReadCount;
	UINT64        u64FPS;
	int           iPacketPerSecond;
	int           iNumPerPacket;
	int           iValid;
	int           iInvalid;
	float         dRMS;
	UINT64        u64CurrentS;
	int           iGrayBytes;    //
	tsSDKStatistic()
	{
		reset();
		iGrayBytes = 0;
	}
	void reset()
	{
		u64TimeStampS = 0;
		u64TSRxPacketFirst = 0;
		u64RxPacketCount = 0;
		u64ErrorPacketCount = 0;
		iPortReadCount = 0;
		u64FPS = 0;
		iPacketPerSecond = 0;
		iNumPerPacket = 0;
		iValid = 0;
		iInvalid = 0;
		dRMS = 0;
		u64CurrentS = 0;
		iErrorCountContinue = 0;
	}
}tsSDKStatistic;

//pointcloud
typedef struct tsPointCloud
{
	bool         bValid;       // true Valid point, false  invalid            
	double       dAngle;       // compensate angle    ,degree     
	double       dAngleRaw;    // raw angle     ,degree 
	double       dAngleDisp;   // don't care    
	UINT16       u16Dist;      // compensate distance  ,mm  
	UINT16       u16DistRaw;   // raw distance,mm
	UINT16       u16Speed;     // motor speed, RPM
	UINT16       u16Gray;      // luminance
	bool         bGrayTwoByte; // true  u16Gray 2byte ,false u16Gray 1byte 
	UINT64       u64TimeStampMs;    // timestamp ,ms  
	float        fTemperature;//�¶�
	tsPointCloud() :
		bValid(true),
		dAngle(0.),
		dAngleRaw(0.),
		dAngleDisp(0.),
		u16Dist(0),
		u16DistRaw(0),
		u16Speed(0),
		u16Gray(0),
		bGrayTwoByte(false),
		u64TimeStampMs(0),
		fTemperature(0)
	{}
}tsPointCloud;
typedef std::vector<tsPointCloud> LstPointCloud;


typedef std::function<void(int)>              CallBackFunErroCode;
typedef std::function<void(tsSDKStatistic)>   CallBackFunSecondInfo;
typedef std::function<void(LstPointCloud)>    CallBackFunPointCloud;
typedef std::function<void(LstNodeDistQ2)>    CallBackFunDistQ2;

typedef std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds> MicroClock_type;


#endif

