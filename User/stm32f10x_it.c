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
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "ABEncoder.h"
#include "ADC_get.h"
#include "LSM6DSR_Config.h"
#include "pose.h"
#include "Motor_ctr.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "M3PWM.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
float add_angle = 0;
float add_angle_num = 0;
int16_t position_get = 0.0f;
BlackPointResult_t result_BlackPoint;
extern uint16_t uart_rev_tiem;
extern uint8_t star_car;
void SysTick_Handler(void)
{
	float dt = 0.001f;
	static uint32_t lose_time = 0;
	LSM6DSR_ReadData(&LSE6DSR_data);
	LSM6DSR_ConvertToPhysics(&LSE6DSR_data);
	
	uart_rev_tiem ++;
	if(uart_rev_tiem > 250)
	{
		M3PWM_SetDutyCycle(0);
		star_car = 0; 
		uart_rev_tiem = 250;
		Motor_Disable();
	}
//	// 更新旧的角度计算（用于兼容性，使用转换后的角速度）
	add_angle += LSE6DSR_data.gy_rads * dt;  // 使用弧度/秒，正确积分
	add_angle_num += 1.0f;
	
	// 保留旧的姿态解算调用（可选，用于对比）
//	prepare_data();
//	imuupdate(&gyr_rad, &acc_g, &att_angle);
	
	ABEncoder_UpdateSpeed();
	MuxADC_SampleAll();
	BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint);
	if(result_BlackPoint.found)
	{
			// 找到黑点，位置在 result.position
			position_get = result_BlackPoint.precise_position * 10.0f;//BlackPoint_Finder_Search(g_mux_adc_values, &result_BlackPoint)*10.0f;
			if(lose_time > 0)
			{
					lose_time--;
			}
			// 使用 position 进行后续控制
	}
	else
	{
			
			lose_time ++;
			if(lose_time > 500)
			{
				lose_time = 500;
				Motor_Disable();
				star_car = 0; 
			}
		// 未找到黑点，使用上一个位置 result.position
	}
	PID_Control_Update();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
