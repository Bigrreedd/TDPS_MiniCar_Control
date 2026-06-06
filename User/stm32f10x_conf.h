/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "stm32f10x_adc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_can.h"
#include "stm32f10x_cec.h"
#include "stm32f10x_crc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dbgmcu.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_iwdg.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_sdio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_wwdg.h"
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
/* #define USE_FULL_ASSERT    1 */

/* ===== TDPS 项目全局编译开关 ===== */

/* USART3 调试串口占用 PB10/PB11：
 *   =1 (旧双层板默认): PB10=USART3_TX, PB11=USART3_RX，LineSensor ch6(光电管)停用
 *   =0 (2合1上层板)  : PB10 归还 LineSensor ch6，恢复 7 路灰度；USART3 不启用
 *   切换后重编译即可，无需改其他文件。 */
#define USART3_DEBUG_ON_PB10  0

/* 2合1 上层板（Schematic7-U7）调试遥测改走 USART2（PA2/PA3 → J5"通信"口，板物理右上角）：
 *   该板唯一串口引出 = J5；USART1 PA9/PA10、USART3 PB10/PB11 均无连接器（PB10=S7 灰度）。
 *   =1 (本分支默认): 调试遥测帧明文经 USART2 发 PC；无下板心跳时停发电机协议帧防二进制刷屏
 *                    （下板 100Hz 主动心跳，链路接通 g_link_alive 置位后协议帧自动恢复）。
 *   =0             : 还原旧双层板行为（遥测走 USART3，协议帧无条件发送）。 */
#define TELEMETRY_ON_USART2   1

/* ESP32S3-1 雷达上位机接入 USART2/J5（同一物理口，06-06 与队友定版 A5 5A 协议）：
 *   接线：J5 TX(PA2)→ESP32 GPIO18(RX)，J5 RX(PA3)←ESP32 GPIO17(TX)，共地，115200 8N1。
 *   =1 (本分支默认,06-06 晚雷达板接上后切换): 遥测文本封 0x07 帧(A5 5A|ver|type|seq|len|
 *                          payload|xor)发 ESP32，ESP32 原样转 BLE A003→小程序"车端串口日志"
 *                          面板；USART2 RX 改喂 ESP32 帧解析(0x11 DECISION/0x30 ARCH_PASSED)。
 *   =0 (PC 调试):          明文遥测直发 PC，RX 走旧 0xAA Proto——拔 ESP32 插 USB-TTL 时回 0。
 *   两套解析互斥（双解析会在对方 payload 内伪同步并反向发 ACK 污染链路），只能二选一。
 *   注意：0x07 帧的内容源自遥测构建，置 1 时须保持 TELEMETRY_ON_USART2=1 不动。 */
#define ESP32_ON_USART2   1

/* 单板架构（2合1 单板 = 一颗 MCU 干完控制+电机驱动+编码器+风扇）：
 *   =1 (本分支 2in1-single): 电机本地 TIM1 直驱(Motor_ctr)、编码器本地 TIM3/TIM4 读(ABEncoder)、
 *                            风扇本地 TIM2_CH4(M3PWM)；删串口电机指令转发与远端编码器回传。
 *   =0 (双层板上层): 电机/编码器经 USART2 协议帧与远端下板交互(Proto_SendMotorCmd/OnEncFeedback)。
 *   网表实证:电机 MOTOR1/2H/L+EN=PA8~12+DRV8701、编码器 PA6/7+PB6/7、风扇 PB11 全在本 MCU。 */
#define SINGLE_BOARD_LOCAL_DRIVE  1

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports 
  *         the name of the source file and the source line number of the call 
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
