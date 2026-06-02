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

uint32_t Millis_Get(void)
{
	return g_millis;
}

/*
 * 下板（纯执行器）SysTick：编码器测速 + 串口看门狗 + 通知主循环。
 * 无本地 IMU/航向积分/手动驾驶；一切控制在上板。
 */
void SysTick_Handler(void)
{
	g_millis += 2u;

	// 1. 串口看门狗：上板停发 MOTOR_CMD（链路断/上板复位）超时立即停车下电
	//    100 tick = 200ms，缩短上板重烧时的窜车窗口
	uart_rx_timeout++;
	if(uart_rx_timeout > 100)
	{
		uart_rx_timeout = 100;
		is_racing = 0;
		Motor_StopAll();
		Motor_Disable();
	}

	add_angle_num++;

	// 2. 编码器测速
	ABEncoder_UpdateSpeed();

	// 3. 通知主循环（编码器回传节拍）
	g_control_tick = 1;
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/******************************************************************************/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
