#ifndef _CAN1_CHASSIS_H_
#define _CAN1_CHASSIS_H_
#include "main.h"

void CAN1_Configuration(void);
void CAN_DataReceive(CanRxMsg* rx);
void CAN2_Configuration(void);

typedef struct
{
 short current[4];
 
} Chassis;


#endif
