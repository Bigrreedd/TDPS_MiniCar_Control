#include "FanMotor_SafetyLock.h"

static uint16_t g_fan_motor_duty = FAN_MOTOR_SAFE_DUTY_OFF;

void FanMotor_SafetyLock_Init(void)
{
	g_fan_motor_duty = FAN_MOTOR_SAFE_DUTY_OFF;
}

void FanMotor_SafetyLock_ForceOff(void)
{
	g_fan_motor_duty = FAN_MOTOR_SAFE_DUTY_OFF;
}

uint16_t FanMotor_SafetyLock_GetDutyCycle(void)
{
	return g_fan_motor_duty;
}