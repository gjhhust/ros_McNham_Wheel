#ifndef _COMDATATYPE_H_
#define _COMDATATYPE_H_

#include <vector>
#include <iostream>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#ifndef float32_t
typedef float float32_t;
#endif


typedef struct tsBSInfo {
	double      dConfidence;
	double      dMx;
	double      dMy;
	double      dLx;
	double      dLy;
	double      dRx;
	double      dRy;
	tsBSInfo()
	{
		dConfidence = 0;
		dMx = 0;
		dMy = 0;
		dLx = 0;
		dLy = 0;
		dRx = 0;
		dRy = 0;
	}
} tsBSInfo;


typedef struct tsADIR 
{
	float32_t angle;
	int32_t distance;
	int32_t intensity;
	int32_t rpm;
} tsADIR;

#endif

