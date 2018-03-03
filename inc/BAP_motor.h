#ifndef BAP_MOTOR_H
#define BAP_MOTOR_H

#include "BAP_define.h"

void BAP_MotorModuleInit(void);
uint32_t BAP_MotorReadEncoder(uint32_t tim);
BAP_RESULT_E BAP_MotorChangePWMPeriod(uint32_t tim, enum tim_oc_id oc_id, int period);
uint32_t BAP_MotorGetPos(BAP_MOTOR_E motor);

#endif