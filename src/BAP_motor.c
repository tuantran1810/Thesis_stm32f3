#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include "BAP_define.h"
#include "BAP_motor.h"
#include "pid_controller.h"

PID_Controller Motor1Pos_PID;
PID_Controller Motor2Pos_PID;

void BAP_MotorModuleInit(void)
{
    memset(&Motor1Pos_PID, 0, sizeof(Motor1Pos_PID));
    memset(&Motor2Pos_PID, 0, sizeof(Motor2Pos_PID));
    PID_Init(&Motor1Pos_PID, 3, 0, 0, 0.02, 1, 360*10);//(3, 0.005, 0.00001)
    PID_Init(&Motor2Pos_PID, 1, 1, 1, 0.02, 1000, 0);
}

BAP_RESULT_E BAP_MotorChangePosSetpoint(BAP_MOTOR_E motor, float setpoint)
{
    float ret = 0;
    switch(motor)
    {
        case BAP_MOTOR1:
            ret = PID_Update_Setpoint(&Motor1Pos_PID, setpoint);
            break;
        case BAP_MOTOR2:
            ret = PID_Update_Setpoint(&Motor2Pos_PID, setpoint);
            break;
        default:
            return ret;
    }
    return ret;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
BAP_RESULT_E BAP_MotorChangeSpeed(BAP_MOTOR_E motor, int speed)
{
    const uint32_t tim = BAP_PWM_TIMER_D;
    int tmp_speed = 0;
    enum tim_oc_id tmp_oc = 0;
    enum tim_oc_id zero_oc = 0;
    switch(motor)
    {
        case BAP_MOTOR1:
            if(speed > 0)
            {
                tmp_speed = speed;
                tmp_oc = BAP_PWM_MOTOR1_FORWARD_OUT_D;
                zero_oc = BAP_PWM_MOTOR1_BACKWARD_OUT_D;
            }
            else
            {
                tmp_speed = -(speed);
                tmp_oc = BAP_PWM_MOTOR1_BACKWARD_OUT_D;
                zero_oc = BAP_PWM_MOTOR1_FORWARD_OUT_D;
            }
            break;
        case BAP_MOTOR2:
            if(speed > 0)
            {
                tmp_speed = speed;
                tmp_oc = BAP_PWM_MOTOR2_FORWARD_OUT_D;
                zero_oc = BAP_PWM_MOTOR2_BACKWARD_OUT_D;
            }
            else
            {
                tmp_speed = -(speed);
                tmp_oc = BAP_PWM_MOTOR2_BACKWARD_OUT_D;
                zero_oc = BAP_PWM_MOTOR2_FORWARD_OUT_D;
            }
            break;
        default:
            return BAP_FAILED_WRONG_PAR;
    }

    tmp_speed = (tmp_speed > BAP_MAX_PWM_PULSEWIDTH_D) ? BAP_MAX_PWM_PULSEWIDTH_D : tmp_speed;

    BAP_MotorChangePWMPeriod(tim, tmp_oc, tmp_speed);
    BAP_MotorChangePWMPeriod(tim, zero_oc, 0);
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_MotorChangePWMPeriod(uint32_t tim, enum tim_oc_id oc_id, int period)
{
    timer_set_oc_value(tim, oc_id, period);
    return BAP_SUCCESS;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
float BAP_MotorGetPIDPosOutput(BAP_MOTOR_E motor)
{
    float ret = 0;
    switch(motor)
    {
        case BAP_MOTOR1:
            ret = PID_Get_Output(&Motor1Pos_PID, BAP_MotorGetPosDegree(motor));
            break;
        case BAP_MOTOR2:
            ret = PID_Get_Output(&Motor2Pos_PID, BAP_MotorGetPosDegree(motor));
            break;
        default:
            return ret;
    }
    return ret;
}

float BAP_MotorGetPosDegree(BAP_MOTOR_E motor)
{
    float deg = 0;
    uint32_t pos = BAP_MotorGetPos(motor);
    deg = (float)(pos - BAP_MOTOR_START_POS_D);
    deg = (deg/(float)BAP_ENCODER_PULSE_PER_ROUND_D)*360.0;
    return deg;
}

uint32_t BAP_MotorGetPos(BAP_MOTOR_E motor)
{
    static uint32_t Motor1_Pos = BAP_MOTOR_START_POS_D;
    static uint32_t Motor2_Pos = BAP_MOTOR_START_POS_D;
    static uint32_t Motor1_prepos16 = 0;
    static uint32_t Motor2_prepos16 = 0;

    uint32_t* ret = NULL;
    uint32_t* prepos = NULL;
    uint32_t presentpos = 0;

    switch(motor)
    {
        case BAP_MOTOR1:
            presentpos = BAP_MotorReadEncoder(BAP_MOTOR1_ENCODER_TIMER_D);
            prepos = &Motor1_prepos16;
            ret = &Motor1_Pos;
            break;
        case BAP_MOTOR2:
            presentpos = BAP_MotorReadEncoder(BAP_MOTOR2_ENCODER_TIMER_D);
            prepos = &Motor2_prepos16;
            ret = &Motor2_Pos;
            break;
        default:
        return 0;
    }

    if((*prepos < 0x1000) && (presentpos > 0xf000))
        *ret = ((*ret - (1<<16))&0xffff0000)|presentpos;
    else if((*prepos > 0xf000) && (presentpos < 0x1000))
        *ret = ((*ret + (1<<16))&0xffff0000)|presentpos;
    else
        *ret = (*ret&0xffff0000)|presentpos;

    *prepos = presentpos;
    return (*ret);
}

uint32_t BAP_MotorReadEncoder(uint32_t tim)
{
    return timer_get_counter(tim);
}
