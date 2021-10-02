#ifndef _AUTOTASK_H_
#define _AUTOTASK_H_
#define speed2Current(x)  ((x)*8)
#define Current2speed(x)  ((x)/8)

void AutoTask(void);
void goto_1m(void);
void Cal_speed(void);
void Send_to_PC(void);
#endif
