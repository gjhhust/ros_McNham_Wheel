#ifndef SDK_V3_H
#define SDK_V3_H
#include <stdio.h>
#include <iostream>
#include <string> 
#include <sstream>
#include <algorithm>
#include <fstream>
#include "base/hclidar.h"
#include <stdio.h>
#include "base/HcData.h"
#include "base/HcSDK.h"
#include "LidarTest.h"

#include<bits/stdc++.h>             //万能头文件
#include<list>
typedef std::vector<tsNodeInfo>  LstNodeDistQ3;
bool ralidar_Init(void);
void ralidar_RUN(void);
void ralidar_delete(void);
void sdkCallBackFunDistQ2(LstNodeDistQ2 lstG);
void sdkCallBackFunPointCloud(LstPointCloud lstG);
void sdkCallBackFunSecondInfo(tsSDKStatistic sInfo);
void sdkCallBackFunErrorCode(int iErrorCode);
void huigui_node(void);

void xunachu_360(void);
bool lessmark(const tsNodeInfo& stItem1, const tsNodeInfo& stItem2);
bool lessmark_reverse(const tsNodeInfo& stItem1, const tsNodeInfo& stItem2);
#endif