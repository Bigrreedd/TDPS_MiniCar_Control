#include "stm32f10x.h"

// GCC 兼容：Keil 的 __nop() 在 GCC 中不存在，映射为内联汇编
#ifndef __nop
#define __nop() __asm("nop")
#endif

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	while(xus -- )
	{
		for(uint16_t i = 6; i > 0; i --)
		{
			__nop();
		}
	}
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
