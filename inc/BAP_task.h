#ifndef BAP_TASK_H
#define BAP_TASK_H
#include "BAP_define.h"

void BAP_TaskModuleInit(void);
void BAP_TaskRecvCmd(void* p);
void BAP_TaskPlateControl(void* p);
void BAP_TaskMotorControl(void* p);
void BAP_TaskTesting(void* p);

#endif