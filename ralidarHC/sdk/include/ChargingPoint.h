#ifndef _CHARGINGPOINT_H_
#define _CHARGINGPOINT_H_

#include "ComDataType.h"

#ifdef __cplusplus
extern "C" {
#endif
	bool findBSInfo(std::vector<tsADIR>& lstAdir, std::vector<tsBSInfo>& lstBSInfo);
	char* getVersion();
#ifdef __cplusplus
};
#endif

#endif

