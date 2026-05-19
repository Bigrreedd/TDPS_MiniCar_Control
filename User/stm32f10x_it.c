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
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Motor_ctr.h"
#include "M3PWM.h"

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
extern volatile uint16_t uart_rx_timeout;
extern volatile uint8_t is_racing;

void SysTick_Handler(void)
{
	float dt = 0.002f;

	// 1. IMU 读取（SPI 传输，保持实时性）
	LSM6DSR_ReadData(&LSM6DSR_data);
	LSM6DSR_ConvertToPhysics(&LSM6DSR_data);

	// 2. 串口看门狗（仅在等待串口指令的待机状态下生效，自动循迹模式下禁用）
	if(!g_manual_drive_active && !is_racing)
	{
		uart_rx_timeout ++;
		if(uart_rx_timeout > 250)
		{
			M3PWM_SetDutyCycle(0);
			is_racing = 0;
			uart_rx_timeout = 250;
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 3. 角度积分（带漏积分防漂移）
	// 纯陀螺仪积分会因零偏累积漂移，使用漏积分因子衰减长期漂移
	add_angle += LSM6DSR_data.gz_rads * dt;
	add_angle *= 0.9999f;  // 漏积分因子，约 10 秒衰减 1%
	add_angle_deg_360 += LSM6DSR_data.gz_rads * dt * RAD_TO_DEG;
	add_angle_deg_360 *= 0.9999f;  // 漏积分，与 add_angle 保持一致
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
			M3PWM_SetDutyCycle(0);
			is_racing = 0;
			Motor_StopAll();
			Motor_Disable();
		}
	}

	// 5. 编码器测速
	ABEncoder_UpdateSpeed();

	// 6. 通知主循环执行 ADC + BlackPoint + PID
	g_control_tick = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/