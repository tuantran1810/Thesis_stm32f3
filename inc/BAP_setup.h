#ifndef BAP_SETUP_H
#define BAP_SETUP_H
#include "BAP_define.h"

void BAP_SetupClock(void);
void BAP_SetupGPIO(void);
BAP_RESULT_E BAP_SetupUSARTWithDMA(uint32_t uart, int baudrate,bool withDMA);
BAP_RESULT_E BAP_SetupPWM(uint32_t tim, uint32_t prescaler, uint32_t period);
BAP_RESULT_E BAP_SetupEncoder(uint32_t tim);
BAP_RESULT_E BAP_SetupPWMOutputEnable(uint32_t tim, enum tim_oc_id oc_id);
void BAP_SetupModuleInit(void);

#endif