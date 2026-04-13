#ifndef __UART_CONFIG_H__
#define __UART_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>

// USART2: PA2 (TX), PA3 (RX)

void Uart2_Init(uint32_t baudrate);
void Uart2_SendByte(uint8_t byte);
void Uart2_SendBuf(const uint8_t *buf, uint16_t len);
void Uart2_SendString(const char *str);
uint8_t Uart2_ReadByteBlocking(void);
int Uart2_BytesAvailable(void);

#endif

