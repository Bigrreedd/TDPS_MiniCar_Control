#ifndef __MPU6050_CONFIG_H__
#define __MPU6050_CONFIG_H__

#include "stm32f10x.h"
#include <stdint.h>

// 新主板使用MPU6050 I2C：PB9=SDA，PB8=SCL；保留MPU6050_*接口名以兼容现有控制代码

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	float ax_g;     // 加速度 X轴 (g)
	float ay_g;     // 加速度 Y轴 (g)
	float az_g;     // 加速度 Z轴 (g)
	float gx_rads;  // 角速度 X轴 (弧度/秒)
	float gy_rads;  // 角速度 Y轴 (弧度/秒)
	float gz_rads;  // 角速度 Z轴 (弧度/秒)
}MPU6050_DATA_T;
extern volatile MPU6050_DATA_T MPU6050_data;
// 函数声明
uint8_t MPU6050_Init(void);
uint8_t MPU6050_ReadID(void);
void MPU6050_ReadData(volatile MPU6050_DATA_T *physics);
void MPU6050_ConvertToPhysics(volatile MPU6050_DATA_T *physics);
uint8_t MPU6050_ReadReg(uint8_t reg);
void MPU6050_WriteReg(uint8_t reg, uint8_t value);
void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len);

#endif

