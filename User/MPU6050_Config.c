#include "MPU6050_Config.h"
#include "Delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

volatile MPU6050_DATA_T MPU6050_data;

#define MPU6050_SCL_PORT GPIOB
#define MPU6050_SCL_PIN  GPIO_Pin_8
#define MPU6050_SDA_PORT GPIOB
#define MPU6050_SDA_PIN  GPIO_Pin_9

#define MPU6050_ADDR_LOW      0xD0u
#define MPU6050_ADDR_HIGH     0xD2u
#define MPU6050_WHO_AM_I      0x75u
#define MPU6050_PWR_MGMT_1    0x6Bu
#define MPU6050_PWR_MGMT_2    0x6Cu
#define MPU6050_SMPLRT_DIV    0x19u
#define MPU6050_CONFIG        0x1Au
#define MPU6050_GYRO_CONFIG   0x1Bu
#define MPU6050_ACCEL_CONFIG  0x1Cu
#define MPU6050_INT_ENABLE    0x38u
#define MPU6050_ACCEL_XOUT_H  0x3Bu

static uint8_t g_mpu6050_addr = MPU6050_ADDR_LOW;
static uint8_t g_mpu6050_ready = 0u;

static void MPU6050_I2C_Delay(void)
{
	volatile uint8_t i;
	for (i = 0; i < 12u; i++)
	{
	}
}

static void MPU6050_W_SCL(BitAction bit)
{
	GPIO_WriteBit(MPU6050_SCL_PORT, MPU6050_SCL_PIN, bit);
	MPU6050_I2C_Delay();
}

static void MPU6050_W_SDA(BitAction bit)
{
	GPIO_WriteBit(MPU6050_SDA_PORT, MPU6050_SDA_PIN, bit);
	MPU6050_I2C_Delay();
}

static uint8_t MPU6050_R_SDA(void)
{
	uint8_t bit;
	bit = (uint8_t)GPIO_ReadInputDataBit(MPU6050_SDA_PORT, MPU6050_SDA_PIN);
	MPU6050_I2C_Delay();
	return bit;
}

static void MPU6050_I2C_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_Out_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = MPU6050_SCL_PIN | MPU6050_SDA_PIN;
	GPIO_Init(GPIOB, &gpio);

	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_SET);
}

static void MPU6050_I2C_Start(void)
{
	MPU6050_W_SDA(Bit_SET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_RESET);
	MPU6050_W_SCL(Bit_RESET);
}

static void MPU6050_I2C_Stop(void)
{
	MPU6050_W_SDA(Bit_RESET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_SET);
}

static uint8_t MPU6050_I2C_WaitAck(void)
{
	uint16_t timeout = 0u;

	MPU6050_W_SDA(Bit_SET);
	MPU6050_W_SCL(Bit_SET);
	while (MPU6050_R_SDA())
	{
		timeout++;
		if (timeout > 1000u)
		{
			MPU6050_W_SCL(Bit_RESET);
			return 0u;
		}
	}
	MPU6050_W_SCL(Bit_RESET);
	return 1u;
}

static void MPU6050_I2C_SendAck(uint8_t ack)
{
	MPU6050_W_SDA(ack ? Bit_SET : Bit_RESET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SCL(Bit_RESET);
	MPU6050_W_SDA(Bit_SET);
}

static uint8_t MPU6050_I2C_SendByte(uint8_t byte)
{
	uint8_t i;

	for (i = 0; i < 8u; i++)
	{
		MPU6050_W_SDA((byte & 0x80u) ? Bit_SET : Bit_RESET);
		MPU6050_W_SCL(Bit_SET);
		MPU6050_W_SCL(Bit_RESET);
		byte <<= 1;
	}
	return MPU6050_I2C_WaitAck();
}

static uint8_t MPU6050_I2C_ReadByte(uint8_t ack)
{
	uint8_t i;
	uint8_t byte = 0u;

	MPU6050_W_SDA(Bit_SET);
	for (i = 0; i < 8u; i++)
	{
		byte <<= 1;
		MPU6050_W_SCL(Bit_SET);
		if (MPU6050_R_SDA())
		{
			byte |= 0x01u;
		}
		MPU6050_W_SCL(Bit_RESET);
	}
	MPU6050_I2C_SendAck(ack ? 0u : 1u);
	return byte;
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
	uint8_t data;

	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte(g_mpu6050_addr))
	{
		MPU6050_I2C_Stop();
		return 0xFFu;
	}
	if (!MPU6050_I2C_SendByte(reg))
	{
		MPU6050_I2C_Stop();
		return 0xFFu;
	}
	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte((uint8_t)(g_mpu6050_addr | 0x01u)))
	{
		MPU6050_I2C_Stop();
		return 0xFFu;
	}
	data = MPU6050_I2C_ReadByte(0u);
	MPU6050_I2C_Stop();
	return data;
}

void MPU6050_WriteReg(uint8_t reg, uint8_t value)
{
	MPU6050_I2C_Start();
	if (MPU6050_I2C_SendByte(g_mpu6050_addr))
	{
		if (MPU6050_I2C_SendByte(reg))
		{
			(void)MPU6050_I2C_SendByte(value);
		}
	}
	MPU6050_I2C_Stop();
}

static uint8_t MPU6050_ReadRegsChecked(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	if (buf == 0 || len == 0u)
	{
		return 0u;
	}

	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte(g_mpu6050_addr))
	{
		MPU6050_I2C_Stop();
		return 0u;
	}
	if (!MPU6050_I2C_SendByte(reg))
	{
		MPU6050_I2C_Stop();
		return 0u;
	}
	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte((uint8_t)(g_mpu6050_addr | 0x01u)))
	{
		MPU6050_I2C_Stop();
		return 0u;
	}
	for (i = 0; i < len; i++)
	{
		buf[i] = MPU6050_I2C_ReadByte((uint8_t)(i + 1u < len));
	}
	MPU6050_I2C_Stop();
	return 1u;
}

void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	(void)MPU6050_ReadRegsChecked(reg, buf, len);
}

uint8_t MPU6050_ReadID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_Init(void)
{
	uint8_t id;

	g_mpu6050_ready = 0u;
	MPU6050_I2C_Init();
	Delay_ms(50);

	g_mpu6050_addr = MPU6050_ADDR_LOW;
	id = MPU6050_ReadID();
	if (id != 0x68u)
	{
		g_mpu6050_addr = MPU6050_ADDR_HIGH;
		id = MPU6050_ReadID();
	}
	if (id != 0x68u && id != 0x69u)
	{
		g_mpu6050_addr = MPU6050_ADDR_LOW;
		return 0u;
	}

	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80u);
	Delay_ms(100);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01u);
	Delay_ms(10);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00u);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09u);
	MPU6050_WriteReg(MPU6050_CONFIG, 0x03u);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x10u);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00u);
	MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x00u);
	Delay_ms(10);
	g_mpu6050_ready = 1u;
	return 1u;
}

void MPU6050_ReadData(volatile MPU6050_DATA_T *physics)
{
	uint8_t buf[14];

	if (physics == 0)
	{
		return;
	}
	if (!g_mpu6050_ready)
	{
		physics->ax = 0;
		physics->ay = 0;
		physics->az = 0;
		physics->gx = 0;
		physics->gy = 0;
		physics->gz = 0;
		return;
	}
	if (!MPU6050_ReadRegsChecked(MPU6050_ACCEL_XOUT_H, buf, 14u))
	{
		return;
	}

	physics->ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
	physics->ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
	physics->az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);
	physics->gx = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
	physics->gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
	physics->gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
}

void MPU6050_ConvertToPhysics(volatile MPU6050_DATA_T *physics)
{
	const float gyro_lsb_to_rad_per_sec = 1879.299f;

	if (physics == 0)
	{
		return;
	}
	physics->ax_g = (float)physics->ax / 16384.0f;
	physics->ay_g = (float)physics->ay / 16384.0f;
	physics->az_g = (float)physics->az / 16384.0f;
	physics->gx_rads = (float)physics->gx / gyro_lsb_to_rad_per_sec;
	physics->gy_rads = (float)physics->gy / gyro_lsb_to_rad_per_sec;
	physics->gz_rads = (float)physics->gz / gyro_lsb_to_rad_per_sec;
}
