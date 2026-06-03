#include "Uart_Config.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#ifndef USART2_RX_BUFFER_SIZE
#define USART2_RX_BUFFER_SIZE 512
#endif

static volatile uint8_t s_usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
static volatile uint16_t s_usart2_rx_head = 0;
static volatile uint16_t s_usart2_rx_tail = 0;

static void Uart2_PutChar(uint8_t ch)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ch);
}

void Uart2_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;

	if(baudrate == 0) baudrate = 115200;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// PA2: TX 复用推挽
	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);

	// PA3: RX 浮空输入
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	// USART2 参数
	USART_StructInit(&usart);
	usart.USART_BaudRate = baudrate;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &usart);

	// 使能接收中断，环形缓冲
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	// NVIC 配置
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_Cmd(USART2, ENABLE);
}

extern volatile uint16_t uart_rx_timeout;
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uart_rx_timeout = 0;
		uint8_t data = (uint8_t)USART_ReceiveData(USART2);
		uint16_t next = (uint16_t)((s_usart2_rx_head + 1) % USART2_RX_BUFFER_SIZE);
		if(next != s_usart2_rx_tail)
		{
			s_usart2_rx_buffer[s_usart2_rx_head] = data;
			s_usart2_rx_head = next;
		}
		// 否则丢弃字节以避免覆盖
	}
}

void Uart2_SendByte(uint8_t byte)
{
	Uart2_PutChar(byte);
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void Uart2_SendBuf(const uint8_t *buf, uint16_t len)
{
	if(buf == 0 || len == 0) return;
	while(len--)
	{
		Uart2_PutChar(*buf++);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

void Uart2_SendString(const char *str)
{
	if(!str) return;
	while(*str)
	{
		Uart2_PutChar((uint8_t)*str++);
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

uint8_t Uart2_ReadByteBlocking(void)
{
	while(s_usart2_rx_head == s_usart2_rx_tail);
	uint8_t ch = s_usart2_rx_buffer[s_usart2_rx_tail];
	s_usart2_rx_tail = (uint16_t)((s_usart2_rx_tail + 1) % USART2_RX_BUFFER_SIZE);
	return ch;
}

int Uart2_BytesAvailable(void)
{
	if(s_usart2_rx_head >= s_usart2_rx_tail)
		return (int)(s_usart2_rx_head - s_usart2_rx_tail);
	else
		return (int)(USART2_RX_BUFFER_SIZE - (s_usart2_rx_tail - s_usart2_rx_head));
}

/* ========== USART3 调试串口 (PB10 TX, PB11 RX, 115200) ==========
 * PB10 原为 LineSensor ch6，PB11 原为 M3PWM（未使用）。
 * USART3 直连电脑 USB-TTL，printf 重定向到此，USART2 只走二进制协议。 */

#ifndef USART3_RX_BUFFER_SIZE
#define USART3_RX_BUFFER_SIZE 256
#endif

static volatile uint8_t s_usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
static volatile uint16_t s_usart3_rx_head = 0;
static volatile uint16_t s_usart3_rx_tail = 0;

static void Uart3_PutChar(uint8_t ch)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, ch);
}

void Uart3_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef usart;

	if(baudrate == 0) baudrate = 115200;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#if USART3_DEBUG_ON_PB10
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	// PB10: TX 复用推挽
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &gpio);

	// PB11: RX 浮空输入
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &gpio);
#endif

	USART_StructInit(&usart);
	usart.USART_BaudRate = baudrate;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &usart);

	// 使能接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)
{
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART3);
		uint16_t next = (uint16_t)((s_usart3_rx_head + 1) % USART3_RX_BUFFER_SIZE);
		if(next != s_usart3_rx_tail)
		{
			s_usart3_rx_buffer[s_usart3_rx_head] = data;
			s_usart3_rx_head = next;
		}
	}
}

void Uart3_SendByte(uint8_t byte)
{
	Uart3_PutChar(byte);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void Uart3_SendBuf(const uint8_t *buf, uint16_t len)
{
	if(buf == 0 || len == 0) return;
	while(len--)
	{
		Uart3_PutChar(*buf++);
	}
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void Uart3_SendString(const char *str)
{
	if(!str) return;
	while(*str)
	{
		Uart3_PutChar((uint8_t)*str++);
	}
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

uint8_t Uart3_ReadByteBlocking(void)
{
	while(s_usart3_rx_head == s_usart3_rx_tail);
	uint8_t ch = s_usart3_rx_buffer[s_usart3_rx_tail];
	s_usart3_rx_tail = (uint16_t)((s_usart3_rx_tail + 1) % USART3_RX_BUFFER_SIZE);
	return ch;
}

int Uart3_BytesAvailable(void)
{
	if(s_usart3_rx_head >= s_usart3_rx_tail)
		return (int)(s_usart3_rx_head - s_usart3_rx_tail);
	else
		return (int)(USART3_RX_BUFFER_SIZE - (s_usart3_rx_tail - s_usart3_rx_head));
}

// printf 重定向 → USART3（调试串口接电脑）
// USART2 是板间二进制协议链路，printf 不能污染它。
// 仅当 USART3_DEBUG_ON_PB10=1 时 USART3 外设已初始化且有 clock 才安全。
#if USART3_DEBUG_ON_PB10
int fputc(int ch, FILE *f)
{
	(void)f;
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	USART_SendData(USART3, (uint8_t)ch);
	return ch;
}
#endif
