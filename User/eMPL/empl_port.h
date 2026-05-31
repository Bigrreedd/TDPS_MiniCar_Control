#ifndef __EMPL_PORT_H__
#define __EMPL_PORT_H__

/*
 * eMPL/DMP 平台移植层（STM32F10x 标准外设库版本）
 *
 * 把 InvenSense eMPL 驱动需要的平台相关接口统一映射到本项目：
 *   - i2c_write / i2c_read : 复用 MPU6050_Config.c 的 PB8/PB9 软件 I2C
 *   - mdelay               : 复用 System/Delay.c 的毫秒延时
 *   - empl_get_ms          : 复用 SysTick 的毫秒计数 (stm32f10x_it.c)
 *
 * 这些都使用 7 位从机地址（eMPL 内部传入的就是 0x68）。
 */

#include <stdint.h>
#include "MPU6050_Config.h"
#include "Delay.h"
#include "stm32f10x_it.h"

/* eMPL 的 i2c_write/i2c_read 约定：返回 0 成功，非 0 失败 */
#define empl_i2c_write(addr, reg, len, data)  MPU6050_I2C_WriteLen((addr), (reg), (len), (data))
#define empl_i2c_read(addr, reg, len, data)   MPU6050_I2C_ReadLen((addr), (reg), (len), (data))

/* 毫秒延时 */
#define empl_mdelay(ms)   Delay_ms((uint32_t)(ms))

/* 毫秒时基：eMPL 通过 get_ms(unsigned long*) 取当前毫秒数 */
static inline void empl_get_ms(unsigned long *count)
{
	if (count)
	{
		*count = (unsigned long)Millis_Get();
	}
}

#endif /* __EMPL_PORT_H__ */
