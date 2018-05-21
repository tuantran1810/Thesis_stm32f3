#ifndef BAP_DEFINE_H
#define BAP_DEFINE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#define BAP_SYSTEM_CLOCK_HZ_D                   64000000
#define BAP_UART_BAUDRATE_D                     230400
#define BAP_MOTOR_START_POS_D					0x8000
#define BAP_MOTOR_DISCRETE_TIME_INTERVAL        0.004       //4ms
#define BAP_DISCRETE_TIME_INTERVAL              0.020       //20ms

#define BAP_UART_STARTMESSAGE_STR_LENGTH_D      BAP_UART_STARTMESSAGE_LENGTH_D - 2
#define BAP_UART_CMDMESS_STR_LENGTH_D           7
#define BAP_MAX_UART_MESSAGE_LENGTH_D           40
#define BAP_UART_BPOS_XY_LENGTH_D               3

#define BAP_MAX_TICK_TO_WAIT_MESSAGE_D          (TickType_t)20

#define BAP_STARTNEWTURN_STR_D                  "2StartNewTurn3"
#define BAP_RECVOK_STR_D                        "2RecvOK3"
#define BAP_RECVNG_STR_D                        "2RecvNG3"

#define BAP_CMDMESS_STR_D                       "Cmnd"
#define BAP_BPOS_STR_D                          "BPos"
#define BAP_SETPOINT_STR_D                      "SPnt"
#define BAP_CTRL_STR_D                          "Ctrl"


#define BAP_UART_CMD_CH_D                       USART2
#define BAP_UART_DEBUG_CH_D                     UART5

#define BAP_PWM_TIMER_D                         TIM4
#define BAP_PWM_XAXISTMOTOR_FORWARD_OUT_D       TIM_OC1
#define BAP_PWM_XAXISTMOTOR_BACKWARD_OUT_D      TIM_OC2
#define BAP_PWM_YAXISTMOTOR_FORWARD_OUT_D       TIM_OC3
#define BAP_PWM_YAXISTMOTOR_BACKWARD_OUT_D      TIM_OC4
#define BAP_MAX_PWM_PULSEWIDTH_D				999

#define BAP_XAXISTMOTOR_ENCODER_TIMER_D         TIM1
#define BAP_YAXISTMOTOR_ENCODER_TIMER_D         TIM8
#define BAP_ENCODER_PULSE_PER_ROUND_D			3072

//Macro functions
#define BAP_LOG_DEBUG(MESS)                     BAP_UART_SendString(BAP_UART_DEBUG_CH_D, MESS, strlen(MESS))
#define BAP_CLEAN_BUFFER(BUFFER)                memset(BUFFER, 0, strlen(BUFFER))
#define BAP_SemCreateBin(sem)                   sem = xSemaphoreCreateBinary()
#define BAP_SemTake(sem, time)                  xSemaphoreTake(sem, time)
#define BAP_SemTakeMax(sem)                     xSemaphoreTake(sem, portMAX_DELAY)
#define BAP_SemGive(sem)                        xSemaphoreGive(sem)

//type define
typedef enum
{
    BAP_SUCCESS,
    BAP_FAILED_NULL_PTR,
    BAP_FAILED_WRONG_PAR,
    BAP_FAILED_TIMEOUT
}BAP_RESULT_E;

typedef enum
{
    BAP_MOTORDIR_FORWARD,
    BAP_MOTORDIR_BACKWARD,
    BAP_MOTORDIR_BUTT
}BAP_MOTOR_DIR_E;

typedef struct TaskSharedVars_RecvPos_S
{
    int x;
    int y;
}TaskSharedVars_RecvPos_S;

typedef struct TaskSharedVars_MotorPos_S
{
    float x;
    float y;
}TaskSharedVars_MotorPos_S;

typedef struct TaskSharedVars_S
{
    TaskSharedVars_RecvPos_S RecvPos;
    TaskSharedVars_MotorPos_S MotorPos;
}TaskSharedVars_S;

//extern variables
extern SemaphoreHandle_t CMDUART_Send_Se;

extern SemaphoreHandle_t CMDUART_Recv_Se;

extern TaskSharedVars_S SharedVars;
#endif
