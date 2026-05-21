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
 * @brief 寻点主函数：遍历所有点找归一化最小值，用左右两点插值提高精度
 * @param adc_values: ADC值数组
 * @param result: 输出结果结构体指针
 * @return 返回找到的黑点精确位置（浮点数），如果未找到则保持上一个精确位置
 */
float BlackPoint_Finder_Search(volatile uint16_t *adc_values, BlackPointResult_t *result)
{
	uint8_t i;
	uint16_t black_mask = 0u;
	uint8_t best_start = 0u;
	uint8_t best_end = 0u;
	uint8_t best_len = 0u;
	float best_center = last_precise_position;
	float best_distance = 9999.0f;
	uint8_t found_segment = 0u;
	uint8_t position;
	
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
	
	for(i = 0; i < SENSOR_COUNT; i++)
	{
		if(BlackPoint_Finder_IsBlackPoint(i, adc_values[i]))
		{
			black_mask |= (uint16_t)(1u << i);
		}
	}

	if(black_mask == 0u)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}

	i = 0u;
	while(i < SENSOR_COUNT)
	{
		uint8_t start;
		uint8_t end;
		uint8_t len;
		float center;
		float distance;

		if((black_mask & (uint16_t)(1u << i)) == 0u)
		{
			i++;
			continue;
		}

		start = i;
		while(i < SENSOR_COUNT && (black_mask & (uint16_t)(1u << i)) != 0u)
		{
			i++;
		}
		end = (uint8_t)(i - 1u);
		len = (uint8_t)(end - start + 1u);
		center = ((float)start + (float)end) * 0.5f;
		distance = fabsf(center - last_precise_position);

		if(!found_segment || distance < best_distance ||
		   (fabsf(distance - best_distance) < 0.001f && len > best_len))
		{
			found_segment = 1u;
			best_start = start;
			best_end = end;
			best_len = len;
			best_center = center;
			best_distance = distance;
		}
	}

	if(!found_segment)
	{
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		return last_precise_position;
	}

	if(best_center < 0.0f)
		best_center = 0.0f;
	if(best_center > (float)(SENSOR_COUNT - 1))
		best_center = (float)(SENSOR_COUNT - 1);

	position = (uint8_t)(best_center + 0.5f);
	if(position > (SENSOR_COUNT - 1u))
		position = SENSOR_COUNT - 1u;

	result->found = 1;
	result->position = position;
	result->precise_position = best_center;
	last_position = position;
	last_precise_position = best_center;

	(void)best_start;
	(void)best_end;
	(void)best_len;

	return best_center;
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

