#ifndef BAP_TASK_H
#define BAP_TASK_H
#include "BAP_define.h"

#define BAP_BPOS_STR_D                          "BPos"
#define BAP_CTRL_STR_D                          "Ctrl"
#define BAP_MODE_STR_D                          "Mode"
#define BAP_PID_STR_D                           "PID"
#define BAP_SMC_STR_D                           "SMC"
#define BAP_CIR_STR_D                           "Cir"
#define BAP_REC_STR_D                           "Rec"
#define BAP_FSE_STR_D                           "FSe"

void BAP_TaskModuleInit(void);
void BAP_TaskRecvCmd(void* p);
void BAP_TaskPlateControl(void* p);
void BAP_TaskMotorControl(void* p);
void BAP_TaskCommunicate(void* p);
void BAP_TaskTrajectoryControl(void* p);

#endif
