#ifndef __FAN_MOTOR_SAFETY_LOCK_H
#define __FAN_MOTOR_SAFETY_LOCK_H

#include "stm32f10x.h"

#define FAN_MOTOR_SAFE_DUTY_OFF        0u

/* 解锁开关（2026-06-05 功率审查）：=1 前必须完成 M3PWM.h 注释的三个硬件条件
 * （换≥20A续流二极管 / 双层板铜皮温升实测 / 电流采样）。解锁后仍受双层钳位：
 * RequestDuty 轮子优先门控 + M3PWM_SetDutyCycle 底层 ABS_CAP。 */
#ifndef FAN_MOTOR_UNLOCKED
#define FAN_MOTOR_UNLOCKED             0
#endif

void FanMotor_SafetyLock_Init(void);
void FanMotor_SafetyLock_ForceOff(void);
uint16_t FanMotor_SafetyLock_GetDutyCycle(void);

/* 轮子优先门控：请求风扇占空比(0-1000)，按当前双轮占空比(0-10000)裁决授权值。
 * 锁定状态恒返回 0；解锁后：双轮均≤FAN_WHEEL_CRUISE_DUTY(巡航)→上限 FAN_DUTY_ABS_CAP(50)，
 * 任一轮更高(发车/深弯/堵转)→上限 FAN_DUTY_CONCURRENT_CAP(20)。 */
uint16_t FanMotor_RequestDuty(uint16_t req_duty, uint16_t wheel_duty_l, uint16_t wheel_duty_r);

#endif