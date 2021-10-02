#ifndef LIDARTEST_H
#define LIDARTEST_H

#include "base/hclidar.h"

#include <stdio.h>



class LidarTest
{
public:
    LidarTest();
	~LidarTest();

	void initLidar();


private:
	void sdkCallBackFunErrorCode(int iErrorCode);
	void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo);
	void sdkCallBackFunPointCloud(LstPointCloud lstG);
	void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG);

private:
	HCLidar *m_device;
	std::thread  m_threadWork;
	bool     m_bRun = true;
};


#endif // LIDARTEST_H
