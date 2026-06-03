#include "LineSensor.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define LINE_SENSOR_ACTIVE_LOW 0
#define LINE_SENSOR_BLACK_VALUE 0u
#define LINE_SENSOR_WHITE_VALUE 4095u

volatile uint16_t g_line_sensor_values[SENSOR_COUNT] = {0};
volatile uint16_t g_battery_adc_value = 0u;

static uint16_t LineSensor_ReadDigital(uint8_t channel)
{
	BitAction bit;

	switch (channel)
	{
	case 0: bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1); break;
	case 1: bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4); break;
	case 2: bit = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5); break;
	case 3: bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0); break;
	case 4: bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1); break;
	case 5: bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2); break;
#if USART3_DEBUG_ON_PB10
	case 6: return LINE_SENSOR_WHITE_VALUE;  /* PB10 → USART3_TX，ch6 停用 */
#else
	case 6: bit = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10); break;
#endif
	default: return LINE_SENSOR_WHITE_VALUE;
	}
#if LINE_SENSOR_ACTIVE_LOW
	return (bit == Bit_RESET) ? LINE_SENSOR_BLACK_VALUE : LINE_SENSOR_WHITE_VALUE;
#else
	return (bit == Bit_SET) ? LINE_SENSOR_BLACK_VALUE : LINE_SENSOR_WHITE_VALUE;
#endif
}

static uint16_t BatteryADC_ReadPA0(void)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

void LineSensor_Init(void)
{
	GPIO_InitTypeDef gpio;
	ADC_InitTypeDef adc;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOA, &gpio);
#if USART3_DEBUG_ON_PB10
	/* PB10 让给 USART3_TX，ch6 光电管停用 */
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
#else
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_10;
#endif
	GPIO_Init(GPIOB, &gpio);

	gpio.GPIO_Mode = GPIO_Mode_AIN;
	gpio.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &gpio);

	ADC_DeInit(ADC1);
	adc.ADC_Mode = ADC_Mode_Independent;
	adc.ADC_ScanConvMode = DISABLE;
	adc.ADC_ContinuousConvMode = DISABLE;
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc.ADC_DataAlign = ADC_DataAlign_Right;
	adc.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &adc);

	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}

uint16_t LineSensor_ReadChannel(uint8_t channel)
{
	if (channel >= SENSOR_COUNT)
	{
		return LINE_SENSOR_WHITE_VALUE;
	}
	return LineSensor_ReadDigital(channel);
}

void LineSensor_SampleAll(void)
{
	uint8_t i;

	for(i = 0; i < SENSOR_COUNT; i++)
	{
		g_line_sensor_values[i] = LineSensor_ReadChannel(i);
	}
	g_battery_adc_value = BatteryADC_ReadPA0();
}
