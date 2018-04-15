#include "pid_controller.h"

bool PID_Update_Param(PID_Controller* PID);

bool PID_Update_Param(PID_Controller* PID){
    if ((PID->KP == 0)&&(PID->KI == 0)&&(PID->KD == 0)) return 0;
    if (PID->dT == 0) return 0;
    PID->a = (PID->KP) + (PID->KI*PID->dT/2) + (PID->KD/PID->dT);
    PID->b = (-PID->KP) + (PID->KI*PID->dT/2) - (2*PID->KD/PID->dT);
    PID->c = PID->KD/PID->dT;
    return 1;
}

bool PID_Update(PID_Controller* PID, float KP, float KI, float KD, float dT, float k){
    PID->KP = KP;
    PID->KD = KD;
    PID->KI = KI;
    PID->dT = dT;
    PID->k = k;
    return PID_Update_Param(PID);
}

bool PID_Update_Setpoint(PID_Controller* PID, float setpoint)
{
    PID->setpoint = setpoint;
    return 1;
}

bool PID_Init(PID_Controller* PID, float KP, float KI, float KD, float dT, float k, float setpoint){
    PID->KP = KP;
    PID->KD = KD;
    PID->KI = KI;
    PID->dT = dT;
    PID->setpoint = setpoint;
    PID->k = k;
    PID->uk_1 = 0;
    PID->ek_1 = 0;
    PID->ek_2 = 0;
    PID->with_limit = false;
    return PID_Update_Param(PID);
}

bool PID_OutputLimit_Enable(PID_Controller* PID, float lower, float upper)
{
    PID->lower_limit = lower;
    PID->upper_limit = upper;
    PID->with_limit = true;
    return true;
}

bool PID_OutputLimit_Disable(PID_Controller* PID)
{
    PID->lower_limit = 0;
    PID->upper_limit = 0;
    PID->with_limit = false;
    return true;
}


float PID_Get_Output(PID_Controller* PID, float respond){
    float ek = PID->setpoint - respond;
    float uk = PID->uk_1 + PID->a*ek + PID->b*PID->ek_1 + PID->c*PID->ek_2;
    PID->uk_1 = uk;
    PID->ek_2 = PID->ek_1;
    PID->ek_1 = ek;
    float ret = uk*PID->k;

    if(PID->with_limit)
    {
        if(ret > PID->upper_limit)
        {
            return PID->upper_limit;
        }
        else if (ret < PID->lower_limit)
        {
            return PID->lower_limit;
        }
    }
    return ret;
}
