#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

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
#include "BAP_DiscreteFunction.h"
#include "BAP_SMC.h"
#include "pid_controller.h"

#define BAP_INPUT_LPF_COEFFICIENT       100
#define BAP_PLATE_OUTPUT_LIMIT_ANGLE    12
#define BAP_PLATE_PIXEL                 320.0
#define BAP_PLATE_WIDTH                 0.26
#define BAP_PI                          3.14159

SemaphoreHandle_t InterSharedVar_RecvPos_Se;
SemaphoreHandle_t InterSharedVars_Se;
SemaphoreHandle_t InterController_Se;

BAP_Motor_S BAP_xAxistMotor;
BAP_Motor_S BAP_yAxistMotor;

BAP_SecondOrderLF_S BAP_xAxistLPF;
BAP_SecondOrderLF_S BAP_yAxistLPF;

BAP_SMC_S BAP_xAxistSMC;
BAP_SMC_S BAP_yAxistSMC;

PID_Controller BAP_xAxistPID;
PID_Controller BAP_yAxistPID;

void BAP_TaskMotorConfig(void);
void BAP_TaskPlateConfigPID(void);
void BAP_TaskPlateConfigSMC(void);
void BAP_TaskUpdateSetPoints(unsigned int pixel_x, unsigned int pixel_y);
float BAP_TaskPixel2M(float pixel);
float BAP_TaskRad2Deg(float rad);
float BAP_TaskDeg2Rad(float deg);

void BAP_TaskModuleInit(void)
{
    BAP_SemCreateBin(InterSharedVars_Se);
    BAP_SemGive(InterSharedVars_Se);
    BAP_SemCreateBin(InterSharedVar_RecvPos_Se);
    BAP_SemCreateBin(InterController_Se);
    BAP_SemGive(InterController_Se);
    BAP_TaskMotorConfig();
    BAP_TaskPlateConfigPID();
    BAP_TaskPlateConfigSMC();
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//UART recving task - receive from RPi
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
        if(BAP_SemTakeMax(CMDUART_Recv_Se) == pdPASS)
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

                    BAP_SemTakeMax(InterSharedVars_Se);
                    pSharedVars->RecvPos.x = x;
                    pSharedVars->RecvPos.y = y;
                    pSharedVars->Streaming.x_realpos = x;
                    pSharedVars->Streaming.y_realpos = y;                    
                    BAP_SemGive(InterSharedVars_Se);
                    BAP_SemGive(InterSharedVar_RecvPos_Se);
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
//Plate controlling task
void BAP_TaskPlateControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    int x = 0;
    int y = 0;
    float x_out = 0;
    float y_out = 0;
    BAP_PLATE_CONTROLLER_E controller = BAP_PLATE_CONTROLLER_PID;
    while(1)
    {
        if(BAP_SemTakeMax(InterSharedVar_RecvPos_Se) == pdPASS)
        {
            BAP_SemTakeMax(InterSharedVars_Se);
            x = pSharedVars->RecvPos.x;
            y = pSharedVars->RecvPos.y;
            controller = pSharedVars->RecvPos.controller;
            BAP_SemGive(InterSharedVars_Se);

            BAP_SemTakeMax(InterController_Se);
            if(controller == BAP_PLATE_CONTROLLER_PID)
            {
                x_out = PID_Get_Output(&BAP_xAxistPID, x);
                y_out = PID_Get_Output(&BAP_yAxistPID, y);
            }
            else
            {
                BAP_SMCUpdateParam(&BAP_xAxistSMC, BAP_TaskPixel2M(x));
                BAP_SMCUpdateParam(&BAP_yAxistSMC, BAP_TaskPixel2M(y));

                BAP_SMCOuputCal(&BAP_xAxistSMC, &x_out); //setpoint angle for xAxist motor
                BAP_SMCOuputCal(&BAP_yAxistSMC, &y_out); //setpoint angle for yAxist motor
                x_out = BAP_TaskRad2Deg(x_out);
                y_out = BAP_TaskRad2Deg(y_out);
            }
            BAP_SemGive(InterController_Se);

            BAP_SemTakeMax(InterSharedVars_Se);
            pSharedVars->MotorPos.x = x_out;
            pSharedVars->MotorPos.y = y_out;
            BAP_SemGive(InterSharedVars_Se);
        }
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//Motor controlling task
void BAP_TaskMotorControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    float x = 0;
    float y = 0;
    float x_out = 0;
    float y_out = 0;
    float tmpx = 0, tmpy = 0;
    while(1)
    {
        BAP_SemTakeMax(InterSharedVars_Se);
        x = pSharedVars->MotorPos.x;
        y = pSharedVars->MotorPos.y;
        BAP_SemGive(InterSharedVars_Se);

        BAP_MotorChangePosSetpoint(&BAP_xAxistMotor, x);
        BAP_MotorChangePosSetpoint(&BAP_yAxistMotor, y);

        BAP_MotorGetPIDPosOutput(&BAP_xAxistMotor, &x_out); //PWM signal
        BAP_MotorGetPIDPosOutput(&BAP_yAxistMotor, &y_out); //PWM signal

        BAP_MotorChangeSpeedPWM(&BAP_xAxistMotor, (int)x_out);
        BAP_MotorChangeSpeedPWM(&BAP_yAxistMotor, (int)y_out);

        BAP_MotorGetPosDegree(&BAP_xAxistMotor, &tmpx);
        BAP_MotorGetPosDegree(&BAP_yAxistMotor, &tmpy);
        vTaskDelay(4);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskCommunicate(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;

    char CMDUART_Recv_Buffer1[BAP_MAX_UART_MESSAGE_LENGTH_D];
    char CMDUART_Recv_Buffer2[BAP_MAX_UART_MESSAGE_LENGTH_D];
    char* CMDUART_Recv_Buffer_Write = CMDUART_Recv_Buffer1;
    char* CMDUART_Recv_Buffer_Read = CMDUART_Recv_Buffer2;
    
    char f_num = 0;
    char f_dec[4] = {0};
    char ui_num[3] = {0};
    char length_ch[2] = {0};

    float recv_PID_Kp = 0, recv_PID_Ki = 0, recv_PID_Kd = 0;
    float recv_SMC_k1 = 0, recv_SMC_K = 0;
    unsigned int tmp1 = 0, tmp2 = 0, tmp3 = 0, tmp4 = 0;
    unsigned int length = 0;

    BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Write);
    BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Read);

    BAP_UARTRecvDMA(BAP_UART_DEBUG_CH_D, CMDUART_Recv_Buffer_Write, BAP_MAX_UART_MESSAGE_LENGTH_D);

    while(1)
    {
        length = 0;
        if(BAP_SemTakeMax(DEBUGUART_Recv_Se) == pdPASS)
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
            BAP_UARTRecvDMA(BAP_UART_DEBUG_CH_D, CMDUART_Recv_Buffer_Write, BAP_MAX_UART_MESSAGE_LENGTH_D);
            memcpy(length_ch, &CMDUART_Recv_Buffer_Read[1], 2); //Number with 2 digits
            length = atoi(length_ch);
            BAP_LOG_DEBUG(CMDUART_Recv_Buffer_Read);

            if (length <= BAP_MAX_UART_MESSAGE_LENGTH_D && length > 12)
            {
                if(memcmp(&CMDUART_Recv_Buffer_Read[5], BAP_CTRL_STR_D, strlen(BAP_CTRL_STR_D)) == 0)
                {
                    if((memcmp(&CMDUART_Recv_Buffer_Read[11], BAP_PID_STR_D, strlen(BAP_PID_STR_D)) == 0) &&
                        (CMDUART_Recv_Buffer_Read[17] == '.' && CMDUART_Recv_Buffer_Read[24] == '.' && CMDUART_Recv_Buffer_Read[31] == '.'))
                    {
                        f_num = CMDUART_Recv_Buffer_Read[16];
                        memcpy(f_dec, &CMDUART_Recv_Buffer_Read[18], 4);
                        recv_PID_Kp = (float)(f_num - 48) + (float)(atoi(f_dec))/10000.0;

                        f_num = CMDUART_Recv_Buffer_Read[23];
                        memcpy(f_dec, &CMDUART_Recv_Buffer_Read[25], 4);
                        recv_PID_Ki = (float)(f_num - 48) + (float)(atoi(f_dec))/10000.0;

                        f_num = CMDUART_Recv_Buffer_Read[30];
                        memcpy(f_dec, &CMDUART_Recv_Buffer_Read[32], 4);
                        recv_PID_Kd = (float)(f_num - 48) + (float)(atoi(f_dec))/10000.0;

                        BAP_SemTakeMax(InterController_Se);
                        PID_Update(&BAP_xAxistPID, recv_PID_Kp, recv_PID_Ki, recv_PID_Kd, BAP_DISCRETE_TIME_INTERVAL, -1);
                        PID_Update(&BAP_yAxistPID, recv_PID_Kp, recv_PID_Ki, recv_PID_Kd, BAP_DISCRETE_TIME_INTERVAL, 1);
                        BAP_SemGive(InterController_Se);

                        BAP_SemTakeMax(InterSharedVars_Se);
                        pSharedVars->RecvPos.controller = BAP_PLATE_CONTROLLER_PID;
                        BAP_SemGive(InterSharedVars_Se);
                    }
                    else if((memcmp(&CMDUART_Recv_Buffer_Read[11], BAP_SMC_STR_D, strlen(BAP_SMC_STR_D)) == 0) &&
                        (CMDUART_Recv_Buffer_Read[17] == '.' && CMDUART_Recv_Buffer_Read[24] == '.'))
                    {
                        f_num = CMDUART_Recv_Buffer_Read[16];
                        memcpy(f_dec, &CMDUART_Recv_Buffer_Read[18], 4);
                        recv_SMC_k1 = (float)(f_num - 48) + (float)(atoi(f_dec))/10000.0;

                        f_num = CMDUART_Recv_Buffer_Read[23];
                        memcpy(f_dec, &CMDUART_Recv_Buffer_Read[25], 4);
                        recv_SMC_K = (float)(f_num - 48) + (float)(atoi(f_dec))/10000.0;

                        BAP_SemTakeMax(InterController_Se);
                        BAP_SMCUpdate(&BAP_xAxistSMC, recv_SMC_k1, recv_SMC_K);
                        BAP_SMCUpdate(&BAP_yAxistSMC, recv_SMC_K, recv_SMC_K);
                        BAP_SemGive(InterController_Se);

                        BAP_SemTakeMax(InterSharedVars_Se);
                        pSharedVars->RecvPos.controller = BAP_PLATE_CONTROLLER_SMC;
                        BAP_SemGive(InterSharedVars_Se);
                    }
                }
                else if(memcmp(&CMDUART_Recv_Buffer_Read[5], BAP_MODE_STR_D, strlen(BAP_MODE_STR_D)) == 0)
                {
                    if(memcmp(&CMDUART_Recv_Buffer_Read[11], BAP_CIR_STR_D, strlen(BAP_CIR_STR_D)) == 0)
                    {
                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[16], 3);
                        tmp1 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[20], 3);
                        tmp2 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[24], 3);
                        tmp3 = atoi(ui_num);

                        BAP_SemTakeMax(InterSharedVars_Se);
                        pSharedVars->Trajectory.mode = BAP_PLATE_MODE_CIRCLE;
                        pSharedVars->Trajectory.circle.x = tmp2;
                        pSharedVars->Trajectory.circle.y = tmp3;
                        pSharedVars->Trajectory.circle.R = tmp1;
                        BAP_SemGive(InterSharedVars_Se);
                    }
                    else if(memcmp(&CMDUART_Recv_Buffer_Read[11], BAP_REC_STR_D, strlen(BAP_REC_STR_D)) == 0)
                    {
                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[16], 3);
                        tmp1 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[20], 3);
                        tmp2 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[24], 3);
                        tmp3 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[28], 3);
                        tmp4 = atoi(ui_num);

                        BAP_SemTakeMax(InterSharedVars_Se);
                        pSharedVars->Trajectory.mode = BAP_PLATE_MODE_RECTANGLE;
                        pSharedVars->Trajectory.rectangle.topleft_x = tmp1;
                        pSharedVars->Trajectory.rectangle.topleft_y = tmp2;
                        pSharedVars->Trajectory.rectangle.botright_x = tmp3;
                        pSharedVars->Trajectory.rectangle.botright_y = tmp4;
                        BAP_SemGive(InterSharedVars_Se);
                    }
                    else if(memcmp(&CMDUART_Recv_Buffer_Read[11], BAP_FSE_STR_D, strlen(BAP_FSE_STR_D)) == 0)
                    {
                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[16], 3);
                        tmp1 = atoi(ui_num);

                        memcpy(ui_num, &CMDUART_Recv_Buffer_Read[20], 3);
                        tmp2 = atoi(ui_num);

                        BAP_SemTakeMax(InterSharedVars_Se);
                        pSharedVars->Trajectory.mode = BAP_PLATE_MODE_FREESET;
                        pSharedVars->Trajectory.freeset.x = tmp1;
                        pSharedVars->Trajectory.freeset.y = tmp2;
                        BAP_SemGive(InterSharedVars_Se);
                    }
                }
            }
        }
        BAP_CLEAN_BUFFER(CMDUART_Recv_Buffer_Read);
        BAP_CLEAN_BUFFER(length_ch);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskTrajectoryControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    TaskSharedVars_Trajectory_S traj;

    BAP_PLATE_MODE_E pre_mode = BAP_PLATE_MODE_FREESET;
    unsigned int count = 0;
    unsigned int max_count = 0;
    unsigned int circle_x = 0, circle_y = 0;

    BAP_TaskUpdateSetPoints(160, 160);

    BAP_SemTakeMax(InterSharedVars_Se);
    pSharedVars->Trajectory.mode = BAP_PLATE_MODE_FREESET;
    pSharedVars->Trajectory.freeset.x = 160;
    pSharedVars->Trajectory.freeset.y = 160;
    BAP_SemGive(InterSharedVars_Se);

    while(1)
    {
        BAP_SemTakeMax(InterSharedVars_Se);
        memcpy(&traj, &(pSharedVars->Trajectory), sizeof(TaskSharedVars_Trajectory_S));
        BAP_SemGive(InterSharedVars_Se);

        if(traj.mode == BAP_PLATE_MODE_CIRCLE)
        {
            if(pre_mode != BAP_PLATE_MODE_CIRCLE)
            {
                count = 0;
                pre_mode = BAP_PLATE_MODE_CIRCLE;
            }

            circle_y = (float)traj.circle.y + (float)traj.circle.R*sin(BAP_TaskDeg2Rad(count/2));
            circle_x = (float)traj.circle.x + (float)traj.circle.R*cos(BAP_TaskDeg2Rad(count/2));

            BAP_TaskUpdateSetPoints(circle_x, circle_y);

            count++;
            if(count == 500)
            {
                count = 0;
            }
        }
        else if(traj.mode == BAP_PLATE_MODE_RECTANGLE)
        {
            if(pre_mode != BAP_PLATE_MODE_RECTANGLE)
            {
                max_count = (traj.rectangle.botright_x - traj.rectangle.topleft_x + traj.rectangle.botright_y - traj.rectangle.topleft_y)*2*5; //5 counts per pixel
                count = 0;
                pre_mode = BAP_PLATE_MODE_RECTANGLE;
            }

            if(count < max_count/4)
            {
                BAP_TaskUpdateSetPoints(traj.rectangle.topleft_x, traj.rectangle.topleft_y);
            }

            if(count >= max_count/4 && count < 2*max_count/4)
            {
                BAP_TaskUpdateSetPoints(traj.rectangle.botright_x, traj.rectangle.topleft_y);
            }

            if(count >= 2*max_count/4 && count < 3*max_count/4)
            {
                BAP_TaskUpdateSetPoints(traj.rectangle.botright_x, traj.rectangle.botright_y);
            }

            if(count >= 3*max_count/4)
            {
                BAP_TaskUpdateSetPoints(traj.rectangle.topleft_x, traj.rectangle.botright_y);
            }

            count++;
            if(count == max_count)
            {
                count = 0;
            }
        }
        else if(traj.mode == BAP_PLATE_MODE_FREESET)
        {
            BAP_TaskUpdateSetPoints(traj.freeset.x, traj.freeset.y);
            pre_mode = BAP_PLATE_MODE_FREESET;
        }
        vTaskDelay(4);
    }
}

void BAP_TaskStreaming(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Ultility functions
void BAP_TaskMotorConfig(void)
{
    memset(&BAP_xAxistMotor, 0, sizeof(BAP_xAxistMotor));
    memset(&BAP_yAxistMotor, 0, sizeof(BAP_yAxistMotor));

    BAP_MotorInit_S input;

    //for xAxist motor
    //using Ziegler–Nichols method with Ku = 70, Tu = 120ms, Kp = 0.6Ku, Ki = 1.2Ku/Tu, Kp = 0.075*Ku*Tu
    input.KP = 14; 
    input.KI = 230;
    input.KD = 0.56;
    input.dT = BAP_MOTOR_DISCRETE_TIME_INTERVAL;
    input.k = 1;
    input.setpoint = 0;
    input.enc_tim = BAP_XAXISTMOTOR_ENCODER_TIMER_D;
    input.pwm_tim = BAP_PWM_TIMER_D;
    input.forward_output = BAP_PWM_XAXISTMOTOR_FORWARD_OUT_D;
    input.backward_output = BAP_PWM_XAXISTMOTOR_BACKWARD_OUT_D;
    BAP_MotorInit(&BAP_xAxistMotor, &input);
    BAP_MotorSetPIDOutputLimit(&BAP_xAxistMotor, -400.0, 400.0);
    BAP_MotorChangeSpeedPWM(&BAP_xAxistMotor, 0);

    //for yAxist motor
    //using Ziegler–Nichols method with Ku = 90, Tu = 120ms, Kp = 0.6Ku, Ki = 1.2Ku/Tu, Kp = 0.075*Ku*Tu
    input.KP = 18;
    input.KI = 300;
    input.KD = 0.72;
    input.dT = BAP_MOTOR_DISCRETE_TIME_INTERVAL;
    input.k = -1;
    input.setpoint = 0;
    input.enc_tim = BAP_YAXISTMOTOR_ENCODER_TIMER_D;
    input.pwm_tim = BAP_PWM_TIMER_D;
    input.forward_output = BAP_PWM_YAXISTMOTOR_FORWARD_OUT_D;
    input.backward_output = BAP_PWM_YAXISTMOTOR_BACKWARD_OUT_D;
    BAP_MotorInit(&BAP_yAxistMotor, &input);
    BAP_MotorSetPIDOutputLimit(&BAP_yAxistMotor, -400.0, 400.0);
    BAP_MotorChangeSpeedPWM(&BAP_yAxistMotor, 0);
}

void BAP_TaskPlateConfigPID(void)
{
    memset(&BAP_xAxistPID, 0, sizeof(PID_Controller));
    memset(&BAP_yAxistPID, 0, sizeof(PID_Controller));

    PID_Init(&BAP_xAxistPID, 0.05, 0, 0.03, BAP_DISCRETE_TIME_INTERVAL, -1, 160);
    PID_Init(&BAP_yAxistPID, 0.05, 0, 0.03, BAP_DISCRETE_TIME_INTERVAL, 1, 160);
    PID_OutputLimit_Enable(&BAP_xAxistPID, -BAP_PLATE_OUTPUT_LIMIT_ANGLE, BAP_PLATE_OUTPUT_LIMIT_ANGLE);  
    PID_OutputLimit_Enable(&BAP_yAxistPID, -BAP_PLATE_OUTPUT_LIMIT_ANGLE, BAP_PLATE_OUTPUT_LIMIT_ANGLE);  
}

void BAP_TaskPlateConfigSMC(void)
{
    memset(&BAP_xAxistSMC, 0, sizeof(BAP_SMC_S));
    memset(&BAP_yAxistSMC, 0, sizeof(BAP_SMC_S));
    memset(&BAP_xAxistLPF, 0, sizeof(BAP_SecondOrderLF_S));
    memset(&BAP_yAxistLPF, 0, sizeof(BAP_SecondOrderLF_S));

    BAP_SMCSecondOrderLFInit(&BAP_xAxistLPF, 0.004, BAP_INPUT_LPF_COEFFICIENT);
    BAP_SMCSecondOrderLFInit(&BAP_yAxistLPF, 0.004, BAP_INPUT_LPF_COEFFICIENT);

    BAP_SMCInit(&BAP_xAxistSMC, 2, BAP_DISCRETE_TIME_INTERVAL, 10, -10, 2.5, 3, 0, -1);
    BAP_SMCInit(&BAP_yAxistSMC, 2, BAP_DISCRETE_TIME_INTERVAL, 10, -10, 2.5, 3, 0, 1);

    BAP_SMCOutputLimitEnable(&BAP_xAxistSMC, -BAP_TaskDeg2Rad(BAP_PLATE_OUTPUT_LIMIT_ANGLE), BAP_TaskDeg2Rad(BAP_PLATE_OUTPUT_LIMIT_ANGLE));
    BAP_SMCOutputLimitEnable(&BAP_yAxistSMC, -BAP_TaskDeg2Rad(BAP_PLATE_OUTPUT_LIMIT_ANGLE), BAP_TaskDeg2Rad(BAP_PLATE_OUTPUT_LIMIT_ANGLE));
}

void BAP_TaskUpdateSetPoints(unsigned int pixel_x, unsigned int pixel_y)
{
    float x_out = 0, y_out = 0;

    BAP_SemTakeMax(InterController_Se);
    BAP_FuncSampleAppend(&(BAP_xAxistLPF.input), BAP_TaskPixel2M(pixel_x));
    BAP_FuncSampleAppend(&(BAP_yAxistLPF.input), BAP_TaskPixel2M(pixel_y));
    BAP_SMCSecondOrderLFGetOutput(&BAP_xAxistLPF, &x_out);
    BAP_SMCSecondOrderLFGetOutput(&BAP_yAxistLPF, &y_out);

    PID_Update_Setpoint(&BAP_xAxistPID, pixel_x);
    PID_Update_Setpoint(&BAP_yAxistPID, pixel_y);

    BAP_SMCChangeSetPoint(&BAP_xAxistSMC, x_out);
    BAP_SMCChangeSetPoint(&BAP_yAxistSMC, y_out);
    BAP_SemGive(InterController_Se);
}

float BAP_TaskPixel2M(float pixel)
{
    return (pixel/BAP_PLATE_PIXEL)*BAP_PLATE_WIDTH;
}

float BAP_TaskRad2Deg(float rad)
{
    return (rad/BAP_PI)*180.0;
}

float BAP_TaskDeg2Rad(float deg)
{
    return (deg/180.0)*BAP_PI;
}
