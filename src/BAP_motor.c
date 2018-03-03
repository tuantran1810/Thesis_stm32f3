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

PID_Controller Motor2_PID;
PID_Controller Motor1_PID;

void BAP_MotorModuleInit(void)
{
    memset(&Motor1_PID, 0, sizeof(Motor1_PID));
    memset(&Motor2_PID, 0, sizeof(Motor2_PID));
}

uint32_t BAP_MotorReadEncoder(uint32_t tim)
{
    return timer_get_counter(tim);
}

BAP_RESULT_E BAP_MotorChangePWMPeriod(uint32_t tim, enum tim_oc_id oc_id, int period)
{
    timer_set_oc_value(tim, oc_id, period);
    return BAP_SUCCESS;
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
