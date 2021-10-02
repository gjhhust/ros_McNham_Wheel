#ifndef HCHEAD_H
#define HCHEAD_H

#include <set>
#include <chrono>
#include <algorithm>
#include <vector>
#include <list>
#include <iostream>
#include <cmath>
#include <ctime> 
#include <stdarg.h> 

#include "HcData.h"

#define SDK_VER                    (char*)"3.0.9d"

#define SHARK_ENABLE               0

#define  DEBUG_INFO                0

#define  DEFAULT_ID                     (char*)"00000000000000000000000000000000"
#define  DEFAULT_FACTORY                (char*)"CS"
#define  DEFAULT_FIRMWARE               (char*)"00.00.00.00"
#define  DEFAULT_HARDWARE               (char*)"00.00.00.00"

#define  X1B                            "X1B"
#define  X1D                            "X1D"
#define  X1E                            "X1E"
#define  X1F                            "X1F"
#define  X1G                            "X1G"
#define  X1K                            "X1K"
#define  X1L                            "X1L"
#define  X1M                            "X1M"
#define  X1N                            "X1N"
#define  X1S                            "X1S"
#define  X2A                            "X2A"
#define  X2B                            "X2B"
#define  X2C                            "X2C"
#define  X2D                            "X2D"
#define  X2E                            "X2E"
#define  X2F                            "X2F"
#define  X2M                            "X2M"
#define  X2N                            "X2N"
#define  X2Y                            "X2Y"
#define  T1A                            "T1A"

#define  PI_HC                          3.141592653589793

#define  READ_BUFF_SIZE                 256
#define  READ_TIMEOUT_MS                10
#define  DEFAULT_ID_LEN                 14

#define  FACT_NEW_LEN                   6
#define  FACT_NEW_RESERVE_LEN           6
#define  FACT_NEW_CAL_LEN               2
#define  FACT_NEW_HARD_LEN              3

#define  FAC_INFO_LEN                   12
#define  RESERVER_LEN                   4
#define  VER_LEN                        3
#define  ID_LEN                         4
#define  DIST_BYTES                     2

#define  MSG_ID                         1
#define  MSG_CMD                        2
#define  MSG_POINTCLOUD                 3

#define  FPS_1800_NOR                   1800
#define  FPS_2000_NOR                   2088 //2088  // 2100
#define  FPS_3000_NOR                   3048 //3012 
#define  FPS_TOF_NOR                    3300
#define  FPS_1800_RANGE                 50
#define  FPS_2000_RANGE                 100
#define  FPS_3000_RANGE                 100
#define  FPS_TOF_RANGE                  200
#define  FPS_1800_MAX                   (FPS_1800_NOR+FPS_1800_RANGE)
#define  FPS_1800_MIN                   (FPS_1800_NOR-FPS_1800_RANGE)
#define  FPS_2000_MAX                   (FPS_2000_NOR+FPS_2000_RANGE)
#define  FPS_2000_MIN                   (FPS_2000_NOR-FPS_2000_RANGE)
#define  FPS_3000_MAX                   (FPS_3000_NOR+FPS_3000_RANGE)
#define  FPS_3000_MIN                   (FPS_3000_NOR-FPS_3000_RANGE)
#define  FPS_TOF_MAX                    (FPS_TOF_NOR+FPS_TOF_RANGE)
#define  FPS_TOF_MIN                    (FPS_TOF_NOR-FPS_TOF_RANGE)

#define  ANGLE_RESOLV_1800              1.09
#define  ANGLE_RESOLV_2000              0.92  //0.92
#define  ANGLE_RESOLV_3000              0.72 //0.75
#define  ANGLE_RESOLV_TOF               0.7
#define  ANGLE_RESOLV_NARWAL_NOR        0.65 //
#define  ANGLE_RESOLV_NARWAL_LOW        0.55 //

#define  CICRLE_MAX_1800                370
#define  CICRLE_MAX_2000                415  // 5.2hz
#define  CICRLE_MAX_3000                515  // 6hz
#define  CICRLE_MAX_TOF                 550  // 6hz
#define  CICRLE_MAX_NARWAL_NOR          585  // 5.3hz  571
#define  CICRLE_MAX_NARWAL_LOW          760  // 4.2hz   727

#define  SPEED_250_NOR                  250
#define  SPEED_300_NOR                  300
#define  SPEED_312_NOR                  312
#define  SPEED_315_NOR                  315
#define  SPEED_360_NOR                  360
#define  SPEED_TOF_NOR                  360
#define  SPEED_250_RANGE                10
#define  SPEED_300_RANGE                10
#define  SPEED_312_RANGE                6
#define  SPEED_315_RANGE                10
#define  SPEED_360_RANGE                10
#define  SPEED_TOF_RANGE                10
#define  SPEED_250_MAX                  (SPEED_250_NOR+SPEED_250_RANGE)
#define  SPEED_250_MIN                  (SPEED_250_NOR-SPEED_250_RANGE)
#define  SPEED_300_MAX                  (SPEED_300_NOR+SPEED_300_RANGE)
#define  SPEED_300_MIN                  (SPEED_300_NOR-SPEED_300_RANGE)
#define  SPEED_312_MAX                  (SPEED_312_NOR+SPEED_312_RANGE)
#define  SPEED_312_MIN                  (SPEED_312_NOR-SPEED_312_RANGE)
#define  SPEED_315_MAX                  (SPEED_315_NOR+SPEED_315_RANGE)
#define  SPEED_315_MIN                  (SPEED_315_NOR-SPEED_315_RANGE)
#define  SPEED_360_MAX                  (SPEED_360_NOR+SPEED_360_RANGE)
#define  SPEED_360_MIN                  (SPEED_360_NOR-SPEED_360_RANGE)
#define  SPEED_TOF_MAX                  (SPEED_TOF_NOR+SPEED_TOF_RANGE)
#define  SPEED_TOF_MIN                  (SPEED_TOF_NOR-SPEED_TOF_RANGE)

#define  MCU_BLOCK_TIME_MS              1500
#define  INIT_TIMEOUT_MS                2000
#define  LESS_THAN_NUMBER               32  
#define  VALID_NUMBER_COUNT             50 
#define  NUMBER_CONTINUE_CIRCLE         50  
#define  NUMBER_CONTINUE_ERROR_PACKET   10  

#define  SENSOR_ERROR_SECOND            5  
#define  SENSOR_ERROR_TIME_MS           1000  
#define  ENCODER_ERROR_SECOND           5  
#define  ENCODER_ERROR_TIME_MS          1000  
#define  LDS_VOLTAGE_ERROR_SECOND       5  
#define  PD_ERROR_TIME_MS               3000  

union Fp32
{
	uint32_t u;
	float f;
};



#pragma pack(push)
#pragma pack(1)


////rock  konyun 2021-04-28
typedef struct tsUID 
{
	UINT16 UID_0;
	UINT16 UID_1;
	UINT16 UID_2;
	UINT16 UID_3;
	UINT16 UID_4;
	UINT16 UID_5;
	UINT16 UID_6;
	UINT16 UID_7;
	UINT16 UID_8;
}tsUID;

typedef struct tsMCUType 
{
	UINT16 word_0;
	UINT16 word_1;
	UINT16 word_2;
	UINT16 word_3;
	UINT16 word_4;
	UINT16 word_5;
}tsMCUType;

typedef struct tsLidarAttr
{
	UINT16    u16PacketSize;  //size 2
	UINT16    u16Version; //size 2
	UINT16    u16UIDSize; //size 2
	tsUID     sUid; //size 18
	UINT16    u16AngleOffset; //size 2
	tsMCUType sMCUType; //size 12
	UINT16    u16LightPlane; //size 2
	UINT16    u16Power; //size 2
	UINT16    u16Calibration[20]; //size 40
	UINT16    u16CheckSum; //size 2
}tsLidarAttr;

typedef union unLidarInfo
{
	tsLidarAttr sAttr;
	UINT8       u8DataOctal[84];
	UINT16      u16DataHex[42];
}unLidarInfo;

#define RCV_SIZE             sizeof(unLidarInfo)*2
#define LDS_INFO_START       0xAA


typedef union unDevID
{
    UINT32       u32ID;
    UCHAR        u8ID[ID_LEN];
}unDevID;


typedef struct tsSDKIDNew
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FACT_NEW_LEN];
    UCHAR        u8FacReserve[FACT_NEW_RESERVE_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8CalVer[FACT_NEW_CAL_LEN];
    UCHAR        u8HardVer[FACT_NEW_HARD_LEN];
    UCHAR        u8ID[ID_LEN];
}tsSDKIDNew;

typedef struct tsIDX2
{
    UINT16       u16Head;
    UCHAR        u8Ver[VER_LEN];
    UCHAR        u8ID[ID_LEN];
    UCHAR        u8Cal;
}tsIDX2;

typedef struct tsIDX1
{
    UINT16       u16Head;
    UCHAR        u8ID[ID_LEN];
    UINT16       u16Cal;
}tsIDX1;

typedef struct tsCmdInfo
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
    UCHAR        u8FacInfo[FAC_INFO_LEN];
    UINT16       u16Ang;
    UCHAR        u8Direction;
    UCHAR        u8AngleCorrection;
    UCHAR        u8ReserverInfo[RESERVER_LEN];
}tsCmdInfo;

typedef struct tsCmdStart
{
    UINT16       u16Head;
    UINT16       u16Len;
    UINT16       u16Cal;
    UCHAR        u8Type;
}tsCmdStart;

typedef struct tsPointCloudHead
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Num;
    UINT16       u16Speed;
    UINT16       u16FirstAng;
}tsPointCloudHead;

typedef struct tsPointCloudHeadTof
{
	UINT16       u16Head;
	UCHAR        u8Info;
	UCHAR        u8Num;
	UINT16       u16Speed;
	UINT16       u16FirstAng;
	UINT16       u16LastAng;
	UINT16       u16Temperature;
}tsPointCloudHeadTof;

typedef struct tsBlockMessage
{
    UINT16       u16Head;
    UCHAR        u8Info;
    UCHAR        u8Len;
    UCHAR        u8MsgID;
    UCHAR        u8Code;
    UCHAR        u8Reserve;
    UCHAR        u8CheckSum;
}tsBlockMessage;


typedef struct tsPointCloudTail
{
    UINT16       u16LastAng;
    UINT16       u16CheckSum;
}tsPointCloudTail;

typedef struct tsPointCloudTailTof
{
	UINT16       u16CheckSum;
}tsPointCloudTailTof;

/*
typedef struct tsPointCloud
{
    UCHAR        u8Dist[DIST_BYTES];
    UINT16       u16Signal;
}tsPointCloud;
*/


#pragma pack(pop)


class HCHead
{
public:
    HCHead();

    static UINT64 getCurrentTimestampUs();
	static UINT64 getCurrentTimestampMs();

    static void eraseBuff(std::vector<UCHAR>& lstG,int iLen);
    static void eraseRangeData(LstPointCloud& lstG,int iLen);

	static double getAngleFromXY(const double x, const double y);
	static double getAngleFromAB(const double a, const double b);
	static double getDistFromAB(const double a, const double b);

	static float uint6_cov_float(UINT16 value);

	static UINT16 float_cov_uint16(float value);
};

bool nodeComparator(const tsNodeInfo& s1, const tsNodeInfo& s2);
bool newComparator(const tsPointCloud& s1, const tsPointCloud& s2);


static char*  print_curr_time()
{
	time_t now = time(nullptr);
	tm* curr_tm = localtime(&now);

	//tm curr_tm;
	//localtime_s(&curr_tm,&now);
	static char time[80] = { 0 };
	strftime(time, 80, "%Y-%m-%d %H:%M:%S    ", curr_tm);
	//printf(time);
	return time;
}


#ifdef __linux__
#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)
#endif
#if defined (_WIN32) || defined( _WIN64)
#define __FILENAME__ (strrchr(__FILE__, '\\') ? (strrchr(__FILE__, '\\') + 1):__FILE__)
#endif

#define __MY_DATE__ (print_curr_time())




#define LOG_WARNING (printf("HCSDK W:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 
#define LOG_INFO    (printf("HCSDK I:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 
#define LOG_ERROR   (printf("HCSDK E:%s %s:%u:\t", __MY_DATE__, __FILENAME__, __LINE__), printf) 


#endif // HCHEAD_H
