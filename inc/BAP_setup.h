#ifndef BAP_SETUP_H
#define BAP_SETUP_H
#include "BAP_define.h"

void BAPClockSetup(void);
void BAPGPIOSetup(void);
BAP_RESULT_E BAPUSARTWithDMASetup(uint32_t uart, int baudrate,bool withDMA);
BAP_RESULT_E BAPPWMTimerSetup(uint32_t tim, uint32_t prescaler, uint32_t period);
BAP_RESULT_E BAPPWMOutputSetup(uint32_t tim);
BAP_RESULT_E BAPPWMSetup(uint32_t tim, uint32_t prescaler, uint32_t period, enum tim_oc_id oc_id);
void BAPSemaphoreInit(void);

#endif