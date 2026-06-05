#include "FanMotor_SafetyLock.h"
#include "M3PWM.h"   /* FAN_DUTY_ABS_CAP / FAN_DUTY_CONCURRENT_CAP / FAN_WHEEL_CRUISE_DUTY */

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

/* 轮子优先门控（2026-06-05 功率审查）。预算：XT30 15A 总线，双轮@SAFE_MAX 满电堵转
 * 12.36A+逻辑0.5A=12.86A 优先扣除 → 风扇并发余额 2.14A → 20/1000；巡航工况(双轮均
 * ≤1500/10000，实测运行电流~3A)余额放宽，但 SS54FSH 二极管钳到 50/1000。 */
uint16_t FanMotor_RequestDuty(uint16_t req_duty, uint16_t wheel_duty_l, uint16_t wheel_duty_r)
{
#if !FAN_MOTOR_UNLOCKED
	(void)req_duty; (void)wheel_duty_l; (void)wheel_duty_r;
	g_fan_motor_duty = FAN_MOTOR_SAFE_DUTY_OFF;
	return FAN_MOTOR_SAFE_DUTY_OFF;
#else
	uint16_t cap = ((wheel_duty_l <= FAN_WHEEL_CRUISE_DUTY) && (wheel_duty_r <= FAN_WHEEL_CRUISE_DUTY))
	               ? FAN_DUTY_ABS_CAP : FAN_DUTY_CONCURRENT_CAP;
	if (req_duty > cap) req_duty = cap;
	g_fan_motor_duty = req_duty;
	return req_duty;
#endif
}