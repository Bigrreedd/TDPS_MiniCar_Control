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
static volatile uint8_t g_imu_autoread_suspended = 0u;

/* 陀螺仪零偏（静止时的原始读数平均），上电标定一次，每次读取时扣除 */
static int16_t g_gyro_off_x = 0;
static int16_t g_gyro_off_y = 0;
static int16_t g_gyro_off_z = 0;
static void MPU6050_CalibrateGyro(void);

static void MPU6050_I2C_Delay(void)
{
	/* 软件 I2C 位延时：取一个既不超过 400kHz、又不会拖垮 2ms SysTick 中断的折中值。
	   80 会让一次 14 字节读取逼近 2ms 饿死主循环(OLED 黑屏)，故收回到 16 */
	volatile uint16_t i;
	for (i = 0; i < 16u; i++)
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

static void MPU6050_SDA_OUT(void)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = MPU6050_SDA_PIN;
	GPIO_Init(MPU6050_SDA_PORT, &gpio);
}

static void MPU6050_SDA_IN(void)
{
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = MPU6050_SDA_PIN;
	GPIO_Init(MPU6050_SDA_PORT, &gpio);
}

static void MPU6050_I2C_Init(void)
{
	GPIO_InitTypeDef gpio;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_Pin = MPU6050_SCL_PIN;
	GPIO_Init(GPIOB, &gpio);

	MPU6050_SDA_OUT();

	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_SET);
}

static void MPU6050_I2C_Start(void)
{
	MPU6050_SDA_OUT();
	MPU6050_W_SDA(Bit_SET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_RESET);
	MPU6050_W_SCL(Bit_RESET);
}

static void MPU6050_I2C_Stop(void)
{
	MPU6050_SDA_OUT();
	MPU6050_W_SDA(Bit_RESET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SDA(Bit_SET);
}

static uint8_t MPU6050_I2C_WaitAck(void)
{
	uint16_t timeout = 0u;

	MPU6050_SDA_IN();
	MPU6050_W_SCL(Bit_SET);
	while (MPU6050_R_SDA())
	{
		timeout++;
		if (timeout > 50u)
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
	MPU6050_SDA_OUT();
	MPU6050_W_SDA(ack ? Bit_SET : Bit_RESET);
	MPU6050_W_SCL(Bit_SET);
	MPU6050_W_SCL(Bit_RESET);
	MPU6050_W_SDA(Bit_SET);
}

static uint8_t MPU6050_I2C_SendByte(uint8_t byte)
{
	uint8_t i;

	MPU6050_SDA_OUT();
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

	MPU6050_SDA_IN();
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

void MPU6050_I2C_PinInit(void)
{
	MPU6050_I2C_Init();
}

/* DMP 占用软件 I2C 期间，挂起 SysTick 中断里的 IMU 自动读，避免总线竞争 */
void MPU6050_SuspendAutoRead(void)
{
	g_imu_autoread_suspended = 1u;
}

void MPU6050_ResumeAutoRead(void)
{
	g_imu_autoread_suspended = 0u;
}

uint8_t MPU6050_IsAutoReadSuspended(void)
{
	return g_imu_autoread_suspended;
}

/* eMPL/DMP 平台胶水：7 位地址连续写。返回 0 成功，非 0 失败 */
int MPU6050_I2C_WriteLen(uint8_t slave_addr7, uint8_t reg, uint8_t len, const uint8_t *data)
{
	uint8_t i;
	uint8_t addr_w = (uint8_t)(slave_addr7 << 1);

	if (data == 0 && len != 0u)
	{
		return -1;
	}

	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte(addr_w))
	{
		MPU6050_I2C_Stop();
		return -1;
	}
	if (!MPU6050_I2C_SendByte(reg))
	{
		MPU6050_I2C_Stop();
		return -1;
	}
	for (i = 0; i < len; i++)
	{
		if (!MPU6050_I2C_SendByte(data[i]))
		{
			MPU6050_I2C_Stop();
			return -1;
		}
	}
	MPU6050_I2C_Stop();
	return 0;
}

/* eMPL/DMP 平台胶水：7 位地址连续读。返回 0 成功，非 0 失败 */
int MPU6050_I2C_ReadLen(uint8_t slave_addr7, uint8_t reg, uint8_t len, uint8_t *buf)
{
	uint8_t i;
	uint8_t addr_w = (uint8_t)(slave_addr7 << 1);
	uint8_t addr_r = (uint8_t)((slave_addr7 << 1) | 0x01u);

	if (buf == 0 || len == 0u)
	{
		return -1;
	}

	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte(addr_w))
	{
		MPU6050_I2C_Stop();
		return -1;
	}
	if (!MPU6050_I2C_SendByte(reg))
	{
		MPU6050_I2C_Stop();
		return -1;
	}
	MPU6050_I2C_Start();
	if (!MPU6050_I2C_SendByte(addr_r))
	{
		MPU6050_I2C_Stop();
		return -1;
	}
	for (i = 0; i < len; i++)
	{
		buf[i] = MPU6050_I2C_ReadByte((uint8_t)(i + 1u < len));
	}
	MPU6050_I2C_Stop();
	return 0;
}

uint8_t MPU6050_ReadID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

uint8_t MPU6050_Init(void)
{
	uint8_t id = 0x00u;
	uint8_t tries;

	g_mpu6050_ready = 0u;
	MPU6050_I2C_Init();
	Delay_ms(50);

	/* clone 芯片(WHO_AM_I=0x70)上电后第一笔 I2C 事务常失败，需多次重试。
	   低地址(0xD0)与高地址(0xD2)各试几次，读到非 0x00/0xFF 即认为器件在线 */
	for (tries = 0u; tries < 8u; tries++)
	{
		g_mpu6050_addr = (tries & 1u) ? MPU6050_ADDR_HIGH : MPU6050_ADDR_LOW;
		id = MPU6050_ReadID();
		if (id != 0x00u && id != 0xFFu)
		{
			break;
		}
		Delay_ms(5);
	}
	if (id == 0x00u || id == 0xFFu)
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
	MPU6050_CalibrateGyro();
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
	physics->gx = (int16_t)((int16_t)(((uint16_t)buf[8] << 8) | buf[9]) - g_gyro_off_x);
	physics->gy = (int16_t)((int16_t)(((uint16_t)buf[10] << 8) | buf[11]) - g_gyro_off_y);
	physics->gz = (int16_t)((int16_t)(((uint16_t)buf[12] << 8) | buf[13]) - g_gyro_off_z);
}

/* 静止零偏标定：连续采样陀螺仪原始值求平均，作为零偏存储。调用时务必保持车静止 */
static void MPU6050_CalibrateGyro(void)
{
	uint8_t buf[14];
	int32_t sx = 0, sy = 0, sz = 0;
	uint16_t i;
	const uint16_t samples = 200u;

	g_gyro_off_x = 0;
	g_gyro_off_y = 0;
	g_gyro_off_z = 0;
	for (i = 0u; i < samples; i++)
	{
		if (MPU6050_ReadRegsChecked(MPU6050_ACCEL_XOUT_H, buf, 14u))
		{
			sx += (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
			sy += (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
			sz += (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
		}
		Delay_ms(2);
	}
	g_gyro_off_x = (int16_t)(sx / (int32_t)samples);
	g_gyro_off_y = (int16_t)(sy / (int32_t)samples);
	g_gyro_off_z = (int16_t)(sz / (int32_t)samples);
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
