#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "BAP_task.h"
#include "BAP_define.h"
#include "BAP_UART.h"
#include "BAP_setup.h"
#include "BAP_motor.h"

SemaphoreHandle_t InterSharedVar_RecvPos_Se;
SemaphoreHandle_t InterSharedVars_Se;

BAP_Motor_S BAP_xAxistMotor;
BAP_Motor_S BAP_yAxistMotor;

void BAP_TaskMotorConfig(void);

void BAP_TaskModuleInit(void)
{
    BAP_SemCreateBin(InterSharedVars_Se);
    BAP_SemGive(InterSharedVars_Se);
    BAP_SemCreateBin(InterSharedVar_RecvPos_Se);
    BAP_SemGive(InterSharedVar_RecvPos_Se);
    BAP_TaskMotorConfig();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskRecvCmd(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;

    char CMDUART_Recv_Buffer1[BAP_MAX_UART_MESSAGE_LENGTH_D];
    char CMDUART_Recv_Buffer2[BAP_MAX_UART_MESSAGE_LENGTH_D];
    char* CMDUART_Recv_Buffer_Write = CMDUART_Recv_Buffer1;
    char* CMDUART_Recv_Buffer_Read = CMDUART_Recv_Buffer2;
    
    char x_chr[BAP_UART_BPOS_XY_LENGTH_D+1];
    char y_chr[BAP_UART_BPOS_XY_LENGTH_D+1];
    char length_ch[3];
    int x, y, length;

    BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Write);
    BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Read);
    BAP_CLEAN_BUFFER(x_chr);
    BAP_CLEAN_BUFFER(y_chr);
    BAP_CLEAN_BUFFER(length_ch);

    BAP_UARTRecvDMA(BAP_UART_CMD_CH_D, CMDUART_Recv_Buffer_Write, BAP_MAX_UART_MESSAGE_LENGTH_D);

    while(1)
    {
        x = 0;
        y = 0;
        length = 0;
        if(BAP_SemTake(CMDUART_Recv_Se, BAP_MAX_TICK_TO_WAIT_MESSAGE_D) == pdPASS)
        {
            if(CMDUART_Recv_Buffer1 == CMDUART_Recv_Buffer_Write)
            {
                CMDUART_Recv_Buffer_Write = CMDUART_Recv_Buffer2;
                CMDUART_Recv_Buffer_Read = CMDUART_Recv_Buffer1;                
            }            
            else
            {
                CMDUART_Recv_Buffer_Write = CMDUART_Recv_Buffer1;
                CMDUART_Recv_Buffer_Read = CMDUART_Recv_Buffer2;                
            }
            BAP_UARTRecvDMA(BAP_UART_CMD_CH_D, CMDUART_Recv_Buffer_Write, BAP_MAX_UART_MESSAGE_LENGTH_D);
            // command format: [length(2)][CmdStr(4)][par par par ...]
            memcpy(length_ch, &CMDUART_Recv_Buffer_Read[1], 2); //Number with 2 digits
            length = atoi(length_ch);

            if (length <= BAP_MAX_UART_MESSAGE_LENGTH_D && length > 12)
            {
                if(memcmp(&CMDUART_Recv_Buffer_Read[5], BAP_BPOS_STR_D, strlen(BAP_BPOS_STR_D)) == 0)
                {
                    //Receive BPos command with syntax: [xx][BPos][xxx yyy]
                    memcpy(x_chr, &CMDUART_Recv_Buffer_Read[11], BAP_UART_BPOS_XY_LENGTH_D);
                    memcpy(y_chr, &CMDUART_Recv_Buffer_Read[15], BAP_UART_BPOS_XY_LENGTH_D);

                    x = atoi(x_chr);
                    y = atoi(y_chr);

                    BAP_LOG_DEBUG(x_chr);
                    BAP_SemTakeMax(InterSharedVars_Se);
                    pSharedVars->RecvPos.x = x;
                    pSharedVars->RecvPos.y = y;
                    BAP_SemGive(InterSharedVars_Se);
                    BAP_SemGive(InterSharedVar_RecvPos_Se);
                }
                else if(memcmp(&CMDUART_Recv_Buffer_Read[strlen(BAP_CMDMESS_STR_D)], BAP_CTRL_STR_D, strlen(BAP_CTRL_STR_D)) == 0)
                {
                    //TODO
                }
                else if(memcmp(&CMDUART_Recv_Buffer_Read[strlen(BAP_CMDMESS_STR_D)], BAP_SETPOINT_STR_D, strlen(BAP_SETPOINT_STR_D)) == 0)
                {
                    //TODO
                }
                else
                {

                }
            }
        }

        BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Read);
        BAP_CLEAN_BUFFER(x_chr);
        BAP_CLEAN_BUFFER(y_chr);
        BAP_CLEAN_BUFFER(length_ch);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskMotorControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    int x = 0;
    int y = 0;
    while(1)
    {
        if(BAP_SemTakeMax(InterSharedVar_RecvPos_Se) == pdPASS)
        {
            BAP_SemTakeMax(InterSharedVars_Se);
            x = pSharedVars->RecvPos.x;
            y = pSharedVars->RecvPos.y;
            BAP_SemGive(InterSharedVars_Se);
            char str[40];
            sprintf(str, "x = %d", x);
            BAP_LOG_DEBUG(str);
        }
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskTesting(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    char str[50] = {0};
    float deg = 0;
    float pid_out = 0;
    uint32_t enc = 0;
    while(1)
    {
        memset(str, 0, 50);
        BAP_MotorGetPosDegree(&BAP_xAxistMotor, &deg);
        sprintf(str, "deg = %f", deg);
        BAP_LOG_DEBUG(str);
        // BAP_MotorGetPIDPosOutput(&BAP_xAxistMotor, &pid_out);
        // int tmp = (int)pid_out;
        // BAP_MotorChangeSpeedPWM(&BAP_xAxistMotor, tmp);
        vTaskDelay(20);
    }
}

void BAP_TaskMotorConfig(void)
{
    memset(&BAP_xAxistMotor, 0, sizeof(BAP_xAxistMotor));
    memset(&BAP_yAxistMotor, 0, sizeof(BAP_yAxistMotor));

    BAP_MotorInit_S input;

    //for xAxist motor
    input.enc_tim = BAP_XAXISTMOTOR_ENCODER_TIMER_D;
    input.pwm_tim = BAP_PWM_TIMER_D;
    input.forward_output = BAP_PWM_XAXISTMOTOR_FORWARD_OUT_D;
    input.backward_output = BAP_PWM_XAXISTMOTOR_BACKWARD_OUT_D;

    input.KP = 3;
    input.KI = 0;
    input.KD = 0;
    input.dT = 0.02;
    input.k = 1;
    input.setpoint = 0;

    BAP_MotorInit(&BAP_xAxistMotor, &input);

    //for yAxist motor
    input.enc_tim = BAP_YAXISTMOTOR_ENCODER_TIMER_D;
    input.pwm_tim = BAP_PWM_TIMER_D;
    input.forward_output = BAP_PWM_YAXISTMOTOR_FORWARD_OUT_D;
    input.backward_output = BAP_PWM_YAXISTMOTOR_BACKWARD_OUT_D;

    BAP_MotorInit(&BAP_xAxistMotor, &input);
}
