#ifndef __FAN_MOTOR_SAFETY_LOCK_H
#define __FAN_MOTOR_SAFETY_LOCK_H

#include "stm32f10x.h"

#define FAN_MOTOR_SAFE_DUTY_OFF        0u

void FanMotor_SafetyLock_Init(void);
void FanMotor_SafetyLock_ForceOff(void);
uint16_t FanMotor_SafetyLock_GetDutyCycle(void);

#endif