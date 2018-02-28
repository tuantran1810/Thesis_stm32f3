#ifndef BAP_DEFINE_H
#define BAP_DEFINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define BAP_SYSTEM_CLOCK_HZ_D					64000000
#define BAP_UART_STARTMESSAGE_LENGTH_D          11
#define BAP_UART_STARTMESSAGE_STR_LENGTH_D      BAP_UART_STARTMESSAGE_LENGTH_D - 2
#define BAP_UART_CMDMESS_STR_LENGTH_D           7
#define BAP_MAX_UART_MESSAGE_LENGTH_D           50

#define BAP_MAX_TICK_TO_WAIT_MESSAGE_D          (TickType_t)20

#define BAP_STARTNEWTURN_STR_D                  "StartNewTurn"
#define BAP_RECVOK_STR_D                        "RecvOK"
#define BAP_RECVNG_STR_D                        "RecvNG"

#define BAP_STARTMESS_STR_D                     "StartMess"
#define BAP_CMDMESS_STR_D                       "CmdMess"
#define BAP_BPOS_STR_D                          "BPos"
#define BAP_SETPOINT_STR_D                      "SetPoint"
#define BAP_CTRL_STR_D                          "Ctrl"


#define UART_CMD_CH_D                           USART2
#define UART_DEBUG_CH_D                         USART2

//type define
typedef enum
{
    BAP_SUCCESS,
    BAP_FAILED_NULL_PTR,
    BAP_FAILED_WRONG_PAR,
    BAP_FAILED_TIMEOUT
}BAP_RESULT_E;

//Macro functions
#define BAP_LOG_DEBUG(MESS)                     BAP_UART_SendString(UART_DEBUG_CH_D, MESS, strlen(MESS))
#define BAP_CLEAN_BUFFER(BUFFER)                memset(BUFFER, 0, sizeof(BUFFER))
#define BAP_SemTake(sem, time)                  xSemaphoreTake(sem, time)
#define BAP_SemTakeMax(sem)                     xSemaphoreTake(sem, portMAX_DELAY)
#define BAP_SemGive(sem)                        xSemaphoreGive(sem)
#define BAP_MutexLock(mutex)                    xSemaphoreTake(mutex, portMAX_DELAY)
#define BAP_MutexUnlock(mutex)                  xSemaphoreGive(mutex)

//extern variables
extern SemaphoreHandle_t CMDUART_Send_Se;

extern SemaphoreHandle_t CMDUART_RecvStartMess_Se;
extern char CMDUART_RecvStartMess_Buffer[BAP_UART_STARTMESSAGE_LENGTH_D];

extern SemaphoreHandle_t CMDUART_Recv_Se;
extern char CMDUART_Recv_Buffer[BAP_MAX_UART_MESSAGE_LENGTH_D];

#endif