#include "BlackPoint_Finder.h"
#include <math.h>

// 传感器配置数组
static SensorConfig_t sensor_config[SENSOR_COUNT];

// 上一次找到的黑点位置（初始化为中间位置）
static uint8_t last_position = SENSOR_COUNT / 2;

// 上一次找到的精确位置（初始化为中间位置）
static float last_precise_position = (float)(SENSOR_COUNT / 2);

/**
 * @brief 初始化寻点模块
 * @note 设置默认的最小值和最大值参数
 */
void BlackPoint_Finder_Init(void)
{
	uint8_t i;
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		sensor_config[i].min_value = 0;
		sensor_config[i].max_value = 4095;
	}
	// 初始化上一次位置为中间
	last_position = SENSOR_COUNT / 2;
	last_precise_position = (float)(SENSOR_COUNT / 2);
}

/**
 * @brief 设置指定传感器的最小值和最大值
 */
void BlackPoint_Finder_SetSensorConfig(uint8_t sensor_idx, uint16_t min_value, uint16_t max_value)
{
	if(sensor_idx < SENSOR_COUNT)
	{
		sensor_config[sensor_idx].min_value = min_value;
		sensor_config[sensor_idx].max_value = max_value;
	}
}

/**
 * @brief 获取指定传感器的配置
 */
void BlackPoint_Finder_GetSensorConfig(uint8_t sensor_idx, uint16_t *min_value, uint16_t *max_value)
{
	if(sensor_idx < SENSOR_COUNT && min_value != NULL && max_value != NULL)
	{
		*min_value = sensor_config[sensor_idx].min_value;
		*max_value = sensor_config[sensor_idx].max_value;
	}
}

/**
 * @brief 判断指定传感器是否为黑点
 * @return 1=黑点，0=白点
 */
uint8_t BlackPoint_Finder_IsBlackPoint(uint8_t sensor_idx, uint16_t adc_value)
{
	uint16_t min_val, max_val;
	uint16_t threshold;
	
	if(sensor_idx >= SENSOR_COUNT)
		return 0;
	
	min_val = sensor_config[sensor_idx].min_value;
	max_val = sensor_config[sensor_idx].max_value;
	
	// 计算阈值：(max - min) * 0.2 + min
	// 如果ADC值小于等于阈值，则认为是黑点
	threshold = min_val + (uint16_t)((max_val - min_val) * BLACK_POINT_THRESHOLD_PERCENT);
	
	// 黑点：ADC值小（接近最小值）
	// 白点：ADC值大（接近最大值）
	if(adc_value <= threshold)
		return 1;  // 是黑点
	else
		return 0;  // 是白点
}

/**
 * @brief 寻点主函数（数字版）：传感器为数字读取，每路仅有黑(接近0)/白(接近4095)两态。
 *        扫描所有判为黑点的通道，按通道索引求质心，得到 0~(SENSOR_COUNT-1) 的连续位置。
 *        无任何黑点时保持上一次位置并置 found=0。
 * @param adc_values: 传感器值数组（黑≈0，白≈4095）
 * @param result: 输出结果结构体指针
 * @return 黑点精确位置（浮点），未找到时返回上一次精确位置
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result)
{
	uint8_t i;
	uint8_t black_count = 0;
	uint32_t index_sum = 0;     /* 黑点通道索引之和 */
	uint8_t first_black = 0;
	float precise_pos;

	if(adc_values == NULL || result == NULL)
	{
		if(result != NULL)
		{
			result->found = 0;
			result->position = last_position;
			result->precise_position = last_precise_position;
		}
		return last_precise_position;
	}

	/* 扫描所有判为黑点的通道，统计数量、索引和、首个黑点位置 */
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			if(black_count == 0)
				first_black = i;
			black_count++;
			index_sum += i;
		}
	}

	/* 没有黑点：保持上一次位置 */
	if(black_count == 0)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}

	/* 质心 = 黑点索引平均值，结果必落在 [0, SENSOR_COUNT-1]，无需再钳位 */
	precise_pos = (float)index_sum / (float)black_count;

	result->found = 1;
	result->position = first_black;
	result->precise_position = precise_pos;
	last_position = first_black;
	last_precise_position = precise_pos;

	return precise_pos;
}

/**
 * @brief 获取上一次找到的黑点位置
 */
uint8_t BlackPoint_Finder_GetLastPosition(void)
{
	return last_position;
}

/**
 * @brief 重置上一次位置为中间位置
 */
void BlackPoint_Finder_ResetLastPosition(void)
{
	last_position = SENSOR_COUNT / 2;
	last_precise_position = (float)(SENSOR_COUNT / 2);
}

