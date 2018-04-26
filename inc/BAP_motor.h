#ifndef BAP_MOTOR_H
#define BAP_MOTOR_H

#include <libopencm3/stm32/timer.h>

#include "BAP_define.h"
#include "pid_controller.h"

typedef struct BAP_MotorEncoder_S
{
	uint32_t tim;
	uint32_t present_counter;
	uint32_t past_counter;
	uint32_t value;
}BAP_MotorEncoder_S;

typedef struct BAP_MotorPos_S
{
	float pos_degree;
}BAP_MotorPos_S;

typedef struct BAP_MotorPWM_S
{
	uint32_t tim;
	enum tim_oc_id backward;
	enum tim_oc_id forward;
}BAP_MotorPWM_S;

typedef struct BAP_Motor_S
{
	BAP_MotorPWM_S pwm;
	BAP_MotorEncoder_S encoder;
	BAP_MotorPos_S pos;
	int speed;
	PID_Controller pid;
}BAP_Motor_S;

typedef struct BAP_MotorInit_S
{
	//for timer
	uint32_t enc_tim;
	uint32_t pwm_tim;
	enum tim_oc_id backward_output;
	enum tim_oc_id forward_output;
	//for PID
	float KP;
	float KI;
	float KD;
	float dT;
	float k;
	float setpoint;
}BAP_MotorInit_S;

void BAP_MotorModuleInit(void);
void BAP_MotorInit(BAP_Motor_S* motor, BAP_MotorInit_S* input);

BAP_RESULT_E BAP_MotorGetPosDegree(BAP_Motor_S* motor, float *ret_deg);
BAP_RESULT_E BAP_MotorGetPIDPosOutput(BAP_Motor_S* motor, float *ret_PIDout);
BAP_RESULT_E BAP_MotorChangeSpeedPWM(BAP_Motor_S* motor, int speed);
BAP_RESULT_E BAP_MotorChangePosSetpoint(BAP_Motor_S* motor, float setpoint);

#endif