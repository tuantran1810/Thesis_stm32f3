#ifndef BAP_UART_H
#define BAP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "BAP_define.h"

BAP_RESULT_E BAP_UART_InterSemaphoreInit(void);
BAP_RESULT_E BAP_UART_SendString(uint32_t uart, char* str, int strlen); 
BAP_RESULT_E BAP_UART_RecvString(uint32_t uart, char* str, int strlen);
BAP_RESULT_E BAP_UARTSendDMA(uint32_t uart, char *data, int size);
BAP_RESULT_E BAP_UARTRecvDMA(uint32_t uart, char *data, int size);


#endif