/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "MPU6050_Config.h"
#include "pose.h"

#define RAD_TO_DEG (57.2957795f)
/* 偏航积分静止死区：|gz| 低于此值(rad/s)不积分，消除零偏残差导致的航向漂移
 * clone MPU6050 静止零偏较大，0.01 太小压不住，上电后角度缓慢自增；
 * 提到 0.04(≈2.3°/s)止住漂移，代价是慢速转动的小角速度会被吃掉一点 */
#define GYRO_YAW_DEADBAND_RADS (0.04f)
/* 偏航角标定系数：补偿 clone 芯片灵敏度偏差/积分微损。
 * 标定方法：把车精确转 360°(或 720°)，读 OLED 角度显示值 measured，
 *   新系数 = 旧系数 × (真实角度 / measured)。
 *   例：转 360° 显示 350°，则 GYRO_YAW_SCALE = 1.000 × 360/350 ≈ 1.029。
 * 仅影响显示积分角，不改 PID 用的瞬时 gz_rads(转向阻尼不受影响)。 */
#ifndef GYRO_YAW_SCALE
#define GYRO_YAW_SCALE (1.0f)
#endif

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  while (1) {}
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

volatile float add_angle = 0;
volatile float add_angle_deg_360 = 0;
volatile uint32_t add_angle_num = 0;
volatile uint8_t g_manual_drive_active = 0;
volatile uint16_t g_manual_drive_ticks_remaining = 0;
volatile uint8_t g_control_tick = 0;
volatile int16_t position_get = 0;
volatile uint32_t g_millis = 0;
extern volatile uint8_t is_racing;

uint32_t Millis_Get(void)
{
	return g_millis;
}

/*
 * 上板（传感）SysTick：仅 IMU 读取 + 航向积分 + 通知主循环采样。
 * 不含电机/编码器/风扇/串口看门狗（这些都在下板）。
 */
void SysTick_Handler(void)
{
	float dt = 0.002f;
	g_millis += 2u;

	// 1. IMU 读取（MPU6050 软件I2C）
	if (!MPU6050_IsAutoReadSuspended())
	{
		static uint8_t imu_div = 0u; if (++imu_div >= 5u) { imu_div = 0u; MPU6050_ReadData(&MPU6050_data);
		MPU6050_ConvertToPhysics(&MPU6050_data); }
	}

	// 2. 角度积分（陀螺仪零偏已在 MPU6050 标定）
	// 静止死区：|gz| 低于阈值视为零偏残差噪声，不积分，消除静止时航向缓慢漂移
	// GYRO_YAW_SCALE：补偿 clone 芯片灵敏度偏差，让显示角度对得上真实转角
	{
		float gz = MPU6050_data.gz_rads * GYRO_YAW_SCALE;
		float gz_abs = (gz < 0.0f) ? -gz : gz;
		if (gz_abs >= GYRO_YAW_DEADBAND_RADS)
		{
			add_angle += gz * dt;
			add_angle_deg_360 += gz * dt * RAD_TO_DEG;
		}
	}
	if (add_angle_deg_360 >= 360.0f)
		add_angle_deg_360 -= 360.0f;
	else if (add_angle_deg_360 < 0.0f)
		add_angle_deg_360 += 360.0f;
	add_angle_num++;

	// 3. 通知主循环执行 LineSensor + BlackPoint + 发送
	g_control_tick = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
