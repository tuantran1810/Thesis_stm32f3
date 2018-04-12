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

SemaphoreHandle_t InterSharedVars_Se;

void BAP_TaskModuleInit(void)
{
    BAP_SemCreateBin(InterSharedVars_Se);
    BAP_SemGive(InterSharedVars_Se);
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
            // command format: UUUUUUUUUU[length(2)][CmdStr(4)][par par par ...]
            memcpy(length_ch, &CMDUART_Recv_Buffer_Read[11], 2); //Number with 2 digits
            length = atoi(length_ch);

            if (length <= BAP_MAX_UART_MESSAGE_LENGTH_D && length > 22)
            {
                if(memcmp(&CMDUART_Recv_Buffer_Read[15], BAP_BPOS_STR_D, strlen(BAP_BPOS_STR_D)) == 0)
                {
                    //Receive BPos command with syntax: UUUUUUUUUU[xx][BPos][xxx yyy]
                    memcpy(x_chr, &CMDUART_Recv_Buffer_Read[21], BAP_UART_BPOS_XY_LENGTH_D);
                    memcpy(y_chr, &CMDUART_Recv_Buffer_Read[25], BAP_UART_BPOS_XY_LENGTH_D);
                    // BAP_LOG_DEBUG(CMDUART_Recv_Buffer_Read);
                    BAP_LOG_DEBUG(x_chr);
                    // BAP_LOG_DEBUG(y_chr);

                    x = atoi(x_chr);
                    y = atoi(y_chr);

                    BAP_SemTakeMax(InterSharedVars_Se);
                    pSharedVars->RecvPos.x = x;
                    pSharedVars->RecvPos.y = y;
                    pSharedVars->RecvPos.new_flag = 1;
                    BAP_SemGive(InterSharedVars_Se);
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
    while(1)
    {
        // BAP_SemTakeMax(InterSharedVars_Se);
        // if(pSharedVars->flag)
        // {
        //     BAP_LOG_DEBUG("OK\n\r");
        //     pSharedVars->flag = 0;
        //     BAP_MotorChangePosSetpoint(BAP_MOTOR1, (float)pSharedVars->PWM);
        // }
        // BAP_SemGive(InterSharedVars_Se);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskTesting(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    char str[50] = {0};
    while(1)
    {
        memset(str, 0, 50);
        float deg = BAP_MotorGetPosDegree(BAP_MOTOR1);
        sprintf(str, "deg = %f\n\r", deg);
        BAP_LOG_DEBUG(str);
        float pid_out = BAP_MotorGetPIDPosOutput(BAP_MOTOR1);
        int tmp = (int)pid_out;
        BAP_MotorChangeSpeed(BAP_MOTOR1, tmp);
        vTaskDelay(20);
    }
}
