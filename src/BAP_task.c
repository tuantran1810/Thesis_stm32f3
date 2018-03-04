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
    TaskSharedVars_s* pSharedVars = (TaskSharedVars_s*)p;
    while(1)
    {
        BAP_SemTakeMax(CMDUART_RecvStartMess_Se);
        
        BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_STARTNEWTURN_STR_D, strlen(BAP_STARTNEWTURN_STR_D));
        BAP_UARTRecvDMA(BAP_UART_CMD_CH_D, CMDUART_RecvStartMess_Buffer, BAP_UART_STARTMESSAGE_LENGTH_D);

        if(BAP_SemTake(CMDUART_Recv_Se, portMAX_DELAY) == pdPASS)
        {
            if(memcmp(CMDUART_RecvStartMess_Buffer, BAP_STARTMESS_STR_D, BAP_UART_STARTMESSAGE_STR_LENGTH_D) == 0)
            {
                int length = 0;
                length = atoi(&CMDUART_RecvStartMess_Buffer[BAP_UART_STARTMESSAGE_STR_LENGTH_D]);

                BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_RECVOK_STR_D, strlen(BAP_RECVOK_STR_D));
                BAP_UARTRecvDMA(BAP_UART_CMD_CH_D, CMDUART_Recv_Buffer, (int)length);
                if(BAP_SemTake(CMDUART_Recv_Se, portMAX_DELAY) == pdPASS)
                {
                    BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_RECVOK_STR_D, strlen(BAP_RECVOK_STR_D));

                    if(memcmp(CMDUART_Recv_Buffer, BAP_CMDMESS_STR_D, strlen(BAP_CMDMESS_STR_D)) == 0)
                    {
                        if(memcmp(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D)], BAP_BPOS_STR_D, strlen(BAP_BPOS_STR_D)) == 0)
                        {
                            BAP_LOG_DEBUG("BAP_BPOS_STR_D");
                            BAP_LOG_DEBUG(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D)]);
                        }
                        else if(memcmp(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D)], BAP_CTRL_STR_D, strlen(BAP_CTRL_STR_D)) == 0)
                        {
                            int tmp = atoi(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D) + strlen(BAP_CTRL_STR_D)]);
                            BAP_SemTakeMax(InterSharedVars_Se);
                            pSharedVars->PWM = tmp;
                            pSharedVars->flag = 1;
                            BAP_SemGive(InterSharedVars_Se);

                        }
                        else if(memcmp(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D)], BAP_SETPOINT_STR_D, strlen(BAP_SETPOINT_STR_D)) == 0)
                        {
                            BAP_LOG_DEBUG("BAP_SETPOINT_STR_D");
                            BAP_LOG_DEBUG(&CMDUART_Recv_Buffer[strlen(BAP_CMDMESS_STR_D)]);
                        }
                        else
                        {
                            //TODO
                        }
                    }
                }
                else
                {
                    BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_RECVNG_STR_D, strlen(BAP_RECVNG_STR_D));
                }
            }
            else
            {
                BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_RECVNG_STR_D, strlen(BAP_RECVNG_STR_D));
            }
        }
        else
        {
            BAP_UART_SendString(BAP_UART_CMD_CH_D, BAP_RECVNG_STR_D, strlen(BAP_RECVNG_STR_D));
        }
        BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer);
        BAP_CLEAN_BUFFER(CMDUART_RecvStartMess_Buffer);
        BAP_SemGive(CMDUART_RecvStartMess_Se);
        
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskMotorControl(void* p)
{
    TaskSharedVars_s* pSharedVars = (TaskSharedVars_s*)p;
    while(1)
    {
        BAP_SemTakeMax(InterSharedVars_Se);
        if(pSharedVars->flag)
        {
            BAP_LOG_DEBUG("OK\n\r");
            pSharedVars->flag = 0;
            BAP_MotorChangePosSetpoint(BAP_MOTOR1, (float)pSharedVars->PWM);
        }
        BAP_SemGive(InterSharedVars_Se);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskTesting(void* p)
{
    TaskSharedVars_s* pSharedVars = (TaskSharedVars_s*)p;
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
