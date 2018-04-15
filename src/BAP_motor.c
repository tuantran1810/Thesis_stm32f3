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

void BAP_MotorInit(BAP_Motor_S* motor, BAP_MotorInit_S* input);
uint32_t BAP_MotorReadEncoder(BAP_Motor_S* motor);
uint32_t BAP_MotorGetPos(BAP_Motor_S* motor);
BAP_RESULT_E BAP_MotorChangePWMPeriod(BAP_Motor_S* motor, BAP_MOTOR_DIR_E dir, int period);

void BAP_MotorModuleInit(void)
{
    
}

void BAP_MotorInit(BAP_Motor_S* motor, BAP_MotorInit_S* input)
{
    memset(motor, 0, sizeof(motor));
    motor->encoder.tim = input->enc_tim;
    motor->encoder.present_counter = 0;
    motor->encoder.past_counter = 0;
    motor->encoder.value = BAP_MOTOR_START_POS_D;

    motor->pwm.tim = input->pwm_tim;
    motor->pwm.forward = input->forward_output;
    motor->pwm.backward = input->backward_output;

    PID_Init(&(motor->pid), input->KP, input->KI, input->KD, input->dT, input->k, input->setpoint);
}

BAP_RESULT_E BAP_MotorChangePosSetpoint(BAP_Motor_S* motor, float setpoint)
{
    if(motor == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    PID_Update_Setpoint(&(motor->pid), setpoint);
    return BAP_SUCCESS;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
BAP_RESULT_E BAP_MotorChangeSpeedPWM(BAP_Motor_S* motor, int speed)
{
    if(motor == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    int abs_speed = (speed > 0) ? speed : -speed;
    abs_speed = (abs_speed > BAP_MAX_PWM_PULSEWIDTH_D) ? BAP_MAX_PWM_PULSEWIDTH_D : abs_speed;

    if(speed > 0)   //forward case
    {
        BAP_MotorChangePWMPeriod(motor, BAP_MOTORDIR_FORWARD, abs_speed);
        BAP_MotorChangePWMPeriod(motor, BAP_MOTORDIR_BACKWARD, 0);
    }
    else            //backward case
    {
        BAP_MotorChangePWMPeriod(motor, BAP_MOTORDIR_FORWARD, 0);
        BAP_MotorChangePWMPeriod(motor, BAP_MOTORDIR_BACKWARD, abs_speed);
    }
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_MotorChangePWMPeriod(BAP_Motor_S* motor, BAP_MOTOR_DIR_E dir, int period)
{
    if(dir < BAP_MOTORDIR_BUTT)
    {
        if(dir == BAP_MOTORDIR_FORWARD)
        {
            timer_set_oc_value(motor->pwm.tim, motor->pwm.forward, period);
        }
        else if(dir == BAP_MOTORDIR_BACKWARD)
        {
            timer_set_oc_value(motor->pwm.tim, motor->pwm.backward, period);
        }
        else
        {
            return BAP_FAILED_WRONG_PAR;
        }
    }
    else
    {
        return BAP_FAILED_WRONG_PAR;
    }
    return BAP_SUCCESS;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
BAP_RESULT_E BAP_MotorGetPIDPosOutput(BAP_Motor_S* motor, float* ret_PIDout)
{
    if(motor == NULL || ret_PIDout == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }
    float deg = 0;
    *ret_PIDout = PID_Get_Output(&(motor->pid), BAP_MotorGetPosDegree(motor, &deg));
    return BAP_SUCCESS;
}

BAP_RESULT_E BAP_MotorGetPosDegree(BAP_Motor_S* motor, float* ret_deg)
{
    if(motor == NULL || ret_deg == NULL)
    {
        return BAP_FAILED_NULL_PTR;
    }

    float deg = 0;
    uint32_t pos = BAP_MotorGetPos(motor);
    deg = (float)(pos - BAP_MOTOR_START_POS_D);
    *ret_deg = (deg/(float)BAP_ENCODER_PULSE_PER_ROUND_D)*360.0;
    return BAP_SUCCESS;
}

uint32_t BAP_MotorGetPos(BAP_Motor_S* motor)
{
    uint32_t present = BAP_MotorReadEncoder(motor);
    uint32_t past = motor->encoder.past_counter;
    uint32_t enc_value = motor->encoder.value;


    if((past < 0x1000) && (present > 0xf000))
    {
        enc_value = ((enc_value - (1<<16)) & 0xffff0000) | present;
    }
    else if((past > 0xf000) && (present < 0x1000))
    {
        enc_value = ((enc_value + (1<<16)) & 0xffff0000) | present;
    }
    else
    {
        enc_value = (enc_value & 0xffff0000) | present;
    }

    motor->encoder.value = enc_value;
    return enc_value;
}

uint32_t BAP_MotorReadEncoder(BAP_Motor_S* motor)
{
    motor->encoder.past_counter = motor->encoder.present_counter;
    motor->encoder.present_counter = timer_get_counter(motor->encoder.tim);
    return motor->encoder.present_counter;
}
