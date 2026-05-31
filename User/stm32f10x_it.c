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
#include "ABEncoder.h"
#include "MPU6050_Config.h"
#include "pose.h"
#include "Motor_ctr.h"
#include "FanMotor_SafetyLock.h"

#define RAD_TO_DEG (57.2957795f)
/* 偏航积分静止死区：|gz| 低于此值(rad/s)不积分，消除零偏残差导致的航向漂移 */
#define GYRO_YAW_DEADBAND_RADS (0.01f)

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
extern volatile uint16_t uart_rx_timeout;
extern volatile uint8_t is_racing;

uint32_t Millis_Get(void)
{
	return g_millis;
}

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

	// 2. 串口看门狗（仅在等待串口指令的待机状态下生效，自动循迹模式下禁用）
	if(!g_manual_drive_active && !is_racing)
	{
		uart_rx_timeout ++;
		if(uart_rx_timeout > 250)
		{
			FanMotor_SafetyLock_ForceOff();
			is_racing = 0;
			uart_rx_timeout = 250;
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 3. 角度积分（陀螺仪零偏已在 MPU6050 标定）
	// 静止死区：|gz| 低于阈值视为零偏残差噪声，不积分，消除静止时航向缓慢漂移
	{
		float gz = MPU6050_data.gz_rads;
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

	// 4. 手动驾驶计时
	if(g_manual_drive_active)
	{
		if(g_manual_drive_ticks_remaining > 0)
			g_manual_drive_ticks_remaining--;
		if(g_manual_drive_ticks_remaining == 0)
		{
			g_manual_drive_active = 0;
			FanMotor_SafetyLock_ForceOff();
			is_racing = 0;
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 5. 编码器测速
	ABEncoder_UpdateSpeed();

	// 6. 通知主循环执行 LineSensor + BlackPoint + PID
	g_control_tick = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/