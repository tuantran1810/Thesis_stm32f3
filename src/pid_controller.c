#include "pid_controller.h"

bool PID_Update_Param(struct PID_Controller* PID);

bool PID_Update_Param(struct PID_Controller* PID){
	if ((PID->KP == 0)&&(PID->KI == 0)&&(PID->KD == 0)) return 0;
	if (PID->dT == 0) return 0;
	PID->a = (PID->KP) + (PID->KI*PID->dT/2) + (PID->KD/PID->dT);
	PID->b = (-PID->KP) + (PID->KI*PID->dT/2) - (2*PID->KD/PID->dT);
	PID->c = PID->KD/PID->dT;
	return 1;
}

bool PID_Update(struct PID_Controller* PID, float KP, float KI, float KD, float dT, float setpoint){
	PID->KP = KP;
	PID->KD = KD;
	PID->KI = KI;
	PID->dT = dT;
	PID->setpoint = setpoint;
	return PID_Update_Param(PID);
}

bool PID_Init(struct PID_Controller* PID, float KP, float KI, float KD, float dT, float setpoint){
	PID->KP = KP;
	PID->KD = KD;
	PID->KI = KI;
	PID->dT = dT;
	PID->setpoint = setpoint;
	PID->uk_1 = 0;
	PID->ek_1 = 0;
	PID->ek_2 = 0;
	return PID_Update_Param(PID);
}

float PID_Get_Output(struct PID_Controller* PID, float respond){
	float ek = PID->setpoint - respond;
	float uk = PID->uk_1 + PID->a*ek +PID->b*PID->ek_1 + PID->c*PID->ek_2;
	PID->uk_1 = uk;
	PID->ek_2 = PID->ek_1;
	PID->ek_1 = ek;
	return uk;
}
