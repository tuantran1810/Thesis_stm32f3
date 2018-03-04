#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "math.h"

typedef struct PID_Controller{
    float KP;
    float KI;
    float KD;
    float dT;
    float a;
    float b;
    float c;
    float k;
    float setpoint;
    float uk_1;
    float ek_1;
    float ek_2;
}PID_Controller;

bool PID_Update(PID_Controller* PID, float KP, float KI, float KD, float dT, float k);
bool PID_Init(PID_Controller* PID, float KP, float KI, float KD, float dT, float k, float setpoint);
float PID_Get_Output(PID_Controller* PID, float input);
bool PID_Update_Setpoint(PID_Controller* PID, float setpoint);
