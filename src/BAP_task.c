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
#include "BAP_DiscreteFunction.h"
#include "BAP_SMC.h"
#include "pid_controller.h"

#define BAP_INPUT_LPF_COEFFICIENT   0.05    //LPF(s) = 1/(as + 1)^2

const float sin_sig[150] = 
    {
   -0.0000,   -0.8431,   -1.6848,   -2.5234,   -3.3575,   -4.1857,   -5.0065,   -5.8183,   -6.6198,   -7.4095,   -8.1861,   -8.9481,
   -9.6941,  -10.4230,  -11.1333,  -11.8238,  -12.4933,  -13.1406,  -13.7645,  -14.3640,  -14.9379,  -15.4853,  -16.0051,  -16.4964,
  -16.9585,  -17.3904,  -17.7913,  -18.1607,  -18.4977,  -18.8019,  -19.0726,  -19.3094,  -19.5119,  -19.6797,  -19.8125,  -19.9100,
  -19.9722,  -19.9989,  -19.9900,  -19.9456,  -19.8657,  -19.7505,  -19.6001,  -19.4149,  -19.1953,  -18.9414,  -18.6539,  -18.3333,
  -17.9800,  -17.5948,  -17.1782,  -16.7312,  -16.2544,  -15.7487,  -15.2150,  -14.6542,  -14.0674,  -13.4556,  -12.8198,  -12.1613,
  -11.4811,  -10.7805,  -10.0608,   -9.3232,   -8.5690,   -7.7995,   -7.0162,   -6.2204,   -5.4136,   -4.5971,   -3.7725,   -2.9411,
   -2.1045,   -1.2642,   -0.4217,    0.4217,    1.2642,    2.1045,    2.9411,    3.7725,    4.5971,    5.4136,    6.2204,    7.0162,
    7.7995,    8.5690,    9.3232,   10.0608,   10.7805,   11.4811,   12.1613,   12.8198,   13.4556,   14.0674,   14.6542,   15.2150,
   15.7487,   16.2544,   16.7312,   17.1782,   17.5948,   17.9800,   18.3333,   18.6539,   18.9414,   19.1953,   19.4149,   19.6001,
   19.7505,   19.8657,   19.9456,   19.9900,   19.9989,   19.9722,   19.9100,   19.8125,   19.6797,   19.5119,   19.3094,   19.0726,
   18.8019,   18.4977,   18.1607,   17.7913,   17.3904,   16.9585,   16.4964,   16.0051,   15.4853,   14.9379,   14.3640,   13.7645,
   13.1406,   12.4933,   11.8238,   11.1333,   10.4230,    9.6941,    8.9481,    8.1861,    7.4095,    6.6198,    5.8183,    5.0065,
    4.1857,    3.3575,    2.5234,    1.6848,    0.8431,    0.0000
    };

const float cos_sig[150] =
    {
  -20.0000,  -19.9822,  -19.9289,  -19.8402,  -19.7162,  -19.5571,  -19.3632,  -19.1350,  -18.8727,  -18.5768,  -18.2480,  -17.8866,
  -17.4935,  -17.0693,  -16.6147,  -16.1306,  -15.6178,  -15.0773,  -14.5099,  -13.9167,  -13.2988,  -12.6573,  -11.9932,  -11.3078,
  -10.6024,   -9.8780,   -9.1361,   -8.3780,   -7.6049,   -6.8184,   -6.0197,   -5.2103,   -4.3917,   -3.5652,   -2.7324,   -1.8948,
   -1.0537,   -0.2108,    0.6324,    1.4746,    2.3141,    3.1495,    3.9793,    4.8021,    5.6163,    6.4205,    7.2133,    7.9932,
    8.7590,    9.5092,   10.2425,   10.9575,   11.6531,   12.3280,   12.9809,   13.6108,   14.2165,   14.7969,   15.3510,   15.8778,
   16.3763,   16.8458,   17.2853,   17.6940,   18.0713,   18.4165,   18.7289,   19.0081,   19.2534,   19.4645,   19.6410,   19.7826,
   19.8890,   19.9600,   19.9956,   19.9956,   19.9600,   19.8890,   19.7826,   19.6410,   19.4645,   19.2534,   19.0081,   18.7289,
   18.4165,   18.0713,   17.6940,   17.2853,   16.8458,   16.3763,   15.8778,   15.3510,   14.7969,   14.2165,   13.6108,   12.9809,
   12.3280,   11.6531,   10.9575,   10.2425,    9.5092,    8.7590,    7.9932,    7.2133,    6.4205,    5.6163,    4.8021,    3.9793,
    3.1495,    2.3141,    1.4746,    0.6324,   -0.2108,   -1.0537,   -1.8948,   -2.7324,   -3.5652,   -4.3917,   -5.2103,   -6.0197,
   -6.8184,   -7.6049,   -8.3780,   -9.1361,   -9.8780,  -10.6024,  -11.3078,  -11.9932,  -12.6573,  -13.2988,  -13.9167,  -14.5099,
  -15.0773,  -15.6178,  -16.1306,  -16.6147,  -17.0693,  -17.4935,  -17.8866,  -18.2480,  -18.5768,  -18.8727,  -19.1350,  -19.3632,
  -19.5571,  -19.7162,  -19.8402,  -19.9289,  -19.9822,  -20.0000
    };

SemaphoreHandle_t InterSharedVar_RecvPos_Se;
SemaphoreHandle_t InterSharedVars_Se;

BAP_Motor_S BAP_xAxistMotor;
BAP_Motor_S BAP_yAxistMotor;

BAP_SecondOrderLF_S BAP_xAxistLPF;
BAP_SecondOrderLF_S BAP_yAxistLPF;

BAP_SMC_S BAP_xAxistSMC;
BAP_SMC_S BAP_yAxistSMC;

PID_Controller tmp_x;
PID_Controller tmp_y;

void BAP_TaskMotorConfig(void);

void BAP_TaskModuleInit(void)
{
    // BAP_SemCreateBin(InterSharedVars_Se);
    InterSharedVars_Se = xSemaphoreCreateMutex();
    BAP_SemGive(InterSharedVars_Se);
    BAP_SemCreateBin(InterSharedVar_RecvPos_Se);
    BAP_TaskMotorConfig();
    BAP_SMCSecondOrderLFInit(&BAP_xAxistLPF, BAP_DISCRETE_TIME_INTERVAL, BAP_INPUT_LPF_COEFFICIENT);
    BAP_SMCSecondOrderLFInit(&BAP_yAxistLPF, BAP_DISCRETE_TIME_INTERVAL, BAP_INPUT_LPF_COEFFICIENT);
    BAP_SMCInit(&BAP_xAxistSMC, 2, BAP_DISCRETE_TIME_INTERVAL, 1, -1, 2.5, 10, 0);
    BAP_SMCInit(&BAP_yAxistSMC, 2, BAP_DISCRETE_TIME_INTERVAL, 1, -1, 2.5, 10, 0);
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
void BAP_TaskPlateControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    int x = 0;
    int y = 0;
    float x_out = 0;
    float y_out = 0;
    int count = 0;
    while(1)
    {
        if(BAP_SemTakeMax(InterSharedVar_RecvPos_Se) == pdPASS)
        {
            BAP_SemTakeMax(InterSharedVars_Se);
            x = pSharedVars->RecvPos.x;
            y = pSharedVars->RecvPos.y;
            BAP_SemGive(InterSharedVars_Se);

            // BAP_FuncSampleAppend(&(BAP_xAxistLPF.input), (float)x);
            // BAP_FuncSampleAppend(&(BAP_yAxistLPF.input), (float)y);

            // BAP_SMCSecondOrderLFGetOutput(&BAP_xAxistLPF, &x_out);
            // BAP_SMCSecondOrderLFGetOutput(&BAP_yAxistLPF, &y_out);

            // BAP_SMCUpdateParam(&BAP_xAxistSMC, x_out);
            // BAP_SMCUpdateParam(&BAP_yAxistSMC, y_out);

            // BAP_SMCOuputCal(&BAP_xAxistSMC, &x_out); //setpoint angle for xAxist motor
            // BAP_SMCOuputCal(&BAP_yAxistSMC, &y_out); //setpoint angle for yAxist motor

            x_out = PID_Get_Output(&tmp_x, x);
            y_out = PID_Get_Output(&tmp_y, y);

            BAP_SemTakeMax(InterSharedVars_Se);
            pSharedVars->MotorPos.x = x_out;
            pSharedVars->MotorPos.y = y_out;
            BAP_SemGive(InterSharedVars_Se);
        }
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskMotorControl(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    float x = 0;
    float y = 0;
    float x_out = 0;
    float y_out = 0;
    float tmpx = 0, tmpy = 0;
    char str[50] = {0};
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
        sprintf(str, "x, y = %f, %f, %f, %f\n\r", x, tmpx, y, tmpy);
        BAP_LOG_DEBUG(str);
        vTaskDelay(4);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskTesting(void* p)
{
    TaskSharedVars_S* pSharedVars = (TaskSharedVars_S*)p;
    float x_out = 0;
    float y_out = 0;
    float x_deg = 0;
    float y_deg = 0;
    char str[100];
    int count = 0;
    vTaskDelay(1000);
    while(1)
    {
        BAP_MotorGetPosDegree(&BAP_xAxistMotor, &x_deg);
        BAP_MotorGetPosDegree(&BAP_yAxistMotor, &y_deg);
        BAP_MotorChangePosSetpoint(&BAP_xAxistMotor, sin_sig[count]);
        BAP_MotorChangePosSetpoint(&BAP_yAxistMotor, cos_sig[count]);

        BAP_MotorGetPIDPosOutput(&BAP_xAxistMotor, &x_out); //PWM signal
        BAP_MotorGetPIDPosOutput(&BAP_yAxistMotor, &y_out); //PWM signal

        BAP_MotorChangeSpeedPWM(&BAP_xAxistMotor, (int)x_out);
        BAP_MotorChangeSpeedPWM(&BAP_yAxistMotor, (int)y_out);
        count++;
        if (count == 151)
        {
            count = 0;
        }
        vTaskDelay(20);
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void BAP_TaskMotorConfig(void)
{
    memset(&BAP_xAxistMotor, 0, sizeof(BAP_xAxistMotor));
    memset(&BAP_yAxistMotor, 0, sizeof(BAP_yAxistMotor));

    BAP_MotorInit_S input;

    //for xAxist motor
    //using Ziegler–Nichols method with Ku = 70, Tu = 120ms, Kp = 0.6Ku, Ki = 1.2Ku/Tu, Kp = 0.075*Ku*Tu
    // input.KP = 18; 
    // input.KI = 0.06;
    // input.KD = 0.04;
    input.KP = 14; 
    input.KI = 0.06;
    input.KD = 0.04;
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
    input.KI = 0.06;
    input.KD = 0.04;
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

    memset(&tmp_x, 0, sizeof(PID_Controller));
    memset(&tmp_y, 0, sizeof(PID_Controller));

    PID_Init(&tmp_x, 0.04, 0, 0.03, 0.02, -1, 200);
    PID_Init(&tmp_y, 0.04, 0, 0.03, 0.02, 1, 200);
    PID_OutputLimit_Enable(&tmp_x, -10, 10);  
    PID_OutputLimit_Enable(&tmp_y, -10, 10);  
}
