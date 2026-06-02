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
#include "pose.h"
#include "Motor_ctr.h"

#define RAD_TO_DEG (57.2957795f)

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

/* 上板经串口下发的角速度（rad/s），供位置环陀螺项使用（默认未开启） */
volatile float g_link_gz_rads = 0.0f;

uint32_t Millis_Get(void)
{
	return g_millis;
}

/*
 * 下板（电机/脚）SysTick：编码器测速 + 串口看门狗 + 手动驾驶计时 + 通知主循环。
 * 本板无本地 IMU；航向积分用上板经串口下发的 g_link_gz_rads。
 */
void SysTick_Handler(void)
{
	float dt = 0.002f;
	g_millis += 2u;

	// 1. 串口看门狗：上板停发 SENSOR_DATA 时及时停车（自动循迹运行中也生效，
	//    因为运行所需的位置数据完全依赖上板串口；链路断开必须停）。
	uart_rx_timeout++;
	if(uart_rx_timeout > 250)
	{
		uart_rx_timeout = 250;
		is_racing = 0;
		if(!g_manual_drive_active)
		{
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 2. 航向积分（角速度来自上板串口；零偏已在上板 MPU6050 标定）
	add_angle += g_link_gz_rads * dt;
	add_angle_deg_360 += g_link_gz_rads * dt * RAD_TO_DEG;
	if (add_angle_deg_360 >= 360.0f)
		add_angle_deg_360 -= 360.0f;
	else if (add_angle_deg_360 < 0.0f)
		add_angle_deg_360 += 360.0f;
	add_angle_num++;

	// 3. 手动驾驶计时（开环自检用）
	if(g_manual_drive_active)
	{
		if(g_manual_drive_ticks_remaining > 0)
			g_manual_drive_ticks_remaining--;
		if(g_manual_drive_ticks_remaining == 0)
		{
			g_manual_drive_active = 0;
			is_racing = 0;
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 4. 编码器测速
	ABEncoder_UpdateSpeed();

	// 5. 通知主循环执行 PID 控制
	g_control_tick = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
