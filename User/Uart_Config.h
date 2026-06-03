#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>

// USART2: PA2 (TX), PA3 (RX) — 板间通信（二进制协议帧）
// USART3: PB10 (TX), PB11 (RX) — 调试串口（接电脑，printf 重定向目标）

void Uart2_Init(uint32_t baudrate);
void Uart2_SendByte(uint8_t byte);
void Uart2_SendBuf(const uint8_t *buf, uint16_t len);
void Uart2_SendString(const char *str);
uint8_t Uart2_ReadByteBlocking(void);
int Uart2_BytesAvailable(void);

void Uart3_Init(uint32_t baudrate);
void Uart3_SendByte(uint8_t byte);
void Uart3_SendBuf(const uint8_t *buf, uint16_t len);
void Uart3_SendString(const char *str);
uint8_t Uart3_ReadByteBlocking(void);
int Uart3_BytesAvailable(void);

#endif
