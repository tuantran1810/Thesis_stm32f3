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

#define BAP_UART_CMD_CH_D                       USART2
#define BAP_UART_DEBUG_CH_D                     USART3

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

typedef enum
{
    BAP_PLATE_CONTROLLER_PID,
    BAP_PLATE_CONTROLLER_SMC,
    BAP_PLATE_CONTROLLER_BUTT
}BAP_PLATE_CONTROLLER_E;

typedef enum
{
    BAP_PLATE_MODE_CIRCLE,
    BAP_PLATE_MODE_RECTANGLE,
    BAP_PLATE_MODE_FREESET,
    BAP_PLATE_MODE_BUTT
}BAP_PLATE_MODE_E;

typedef struct TaskSharedVars_RecvPos_S
{
    int x;
    int y;
    BAP_PLATE_CONTROLLER_E controller;
}TaskSharedVars_RecvPos_S;

typedef struct TaskSharedVars_MotorPos_S
{
    float x;
    float y;
}TaskSharedVars_MotorPos_S;

typedef struct TaskSharedVars_TrjCircle_S
{
    unsigned int x;
    unsigned int y;
    unsigned int R;
}TaskSharedVars_TrjCircle_S;

typedef struct TaskSharedVars_TrjRectangle_S
{
    unsigned int topleft_x;
    unsigned int topleft_y;
    unsigned int botright_x;
    unsigned int botright_y;
}TaskSharedVars_TrjRectangle_S;

typedef struct TaskSharedVars_TrjFreeSet_S
{
    unsigned int x;
    unsigned int y;
}TaskSharedVars_TrjFreeSet_S;

typedef struct TaskSharedVars_Trajectory_S
{
    BAP_PLATE_MODE_E mode;
    TaskSharedVars_TrjCircle_S circle;
    TaskSharedVars_TrjRectangle_S rectangle;
    TaskSharedVars_TrjFreeSet_S freeset;
}TaskSharedVars_Trajectory_S;

typedef struct TaskSharedVars_S
{
    TaskSharedVars_RecvPos_S RecvPos;
    TaskSharedVars_MotorPos_S MotorPos;
    TaskSharedVars_Trajectory_S Trajectory;
}TaskSharedVars_S;

//extern variables
extern SemaphoreHandle_t CMDUART_Send_Se;
extern SemaphoreHandle_t DEBUGUART_Send_Se;

extern SemaphoreHandle_t CMDUART_Recv_Se;
extern SemaphoreHandle_t DEBUGUART_Recv_Se;

extern TaskSharedVars_S SharedVars;

#endif
