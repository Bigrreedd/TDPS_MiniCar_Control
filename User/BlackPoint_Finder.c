#include "BlackPoint_Finder.h"
#include <math.h>

// 传感器配置数组
static SensorConfig_t sensor_config[SENSOR_COUNT];

// 上一次找到的黑点位置（初始化为中间位置）
static uint8_t last_position = SENSOR_COUNT / 2;

// 上一次找到的精确位置（初始化为中间位置）
static float last_precise_position = (float)(SENSOR_COUNT / 2);

/* ===== 传感器滤波：简单移动平均（3帧） =====
 * 弯道时传感器噪声导致 pos 跳变 0↔50，增加滤波平滑读数。
 * 使用 3 帧平均：既能滤除高频噪声，又不过度延迟（6ms@500Hz）。 */
#define FILTER_WINDOW_SIZE 3
static uint16_t adc_history[SENSOR_COUNT][FILTER_WINDOW_SIZE] = {0};
static uint8_t filter_index = 0;

static void FilterSensorValues(uint16_t *adc_values)
{
	uint8_t i, j;
	// 更新历史缓冲
	for (i = 0; i < SENSOR_COUNT; i++) {
		adc_history[i][filter_index] = adc_values[i];
	}
	filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;

	// 计算移动平均
	for (i = 0; i < SENSOR_COUNT; i++) {
		uint32_t sum = 0;
		for (j = 0; j < FILTER_WINDOW_SIZE; j++) {
			sum += adc_history[i][j];
		}
		adc_values[i] = (uint16_t)(sum / FILTER_WINDOW_SIZE);
	}
}

/* 路口冻结连续帧计数：宽黑被判为路口时，precise_position 冻结为上次有效值。
 * 但真正的停止标志/线尾也会触发宽黑，故加超时上限——超过 JUNCTION_FREEZE_MAX_TICKS
 * 仍持续宽黑则放弃冻结(按全局质心走)，避免线尾被永久冻结在原地。
 * 控制环 500Hz(2ms/帧)，200 帧≈400ms，够穿过任何一条交叉支线。 */
#ifndef JUNCTION_FREEZE_MAX_TICKS
#define JUNCTION_FREEZE_MAX_TICKS 200u
#endif
static uint16_t g_junction_ticks = 0;

/* 单通道加权值：2026-06-05 拉平为均匀权重。
 * 旧边重权重(31/26/17)在"三路黑常态"(物理光斑限制)下造成：中心量化读数
 * {1,2,3}=1.85/{2,3,4}=3.15(±0.65且外偏粘滞)，每次18↔31翻转经增益曲线放大
 * 后首拍D踢≈293，是直线常驻噪声/边缘升级源(13:09三连发车实测)。
 * 拉平后中心±0.5、阶梯均匀0.5/步；深弯触发不受影响({4,5}→e2.0,{5}→e2.5仍≥1.9)，
 * 最深差速302不变(由BENCH 250钳位决定)，仅入弯过渡段corr -13%。 */
static uint8_t SensorWeight(uint8_t i)
{
	(void)i;
	return 20;                                          /* 均匀权重2.0 */
}

/* 对 [start,end] 闭区间内的黑点求加权质心（仅在已知该段全黑时调用） */
static float RunCentroid(uint8_t start, uint8_t end)
{
	uint32_t idx_sum = 0;
	uint32_t w_sum   = 0;
	uint8_t i;
	for (i = start; i <= end; i++)
	{
		uint8_t w = SensorWeight(i);
		idx_sum += (uint32_t)i * w;
		w_sum   += w;
	}
	if (w_sum == 0) return last_precise_position;  /* 防御：不应发生 */
	return (float)idx_sum / (float)w_sum;
}

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
	g_junction_ticks = 0;
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
	float precise_pos;

	if(adc_values == NULL || result == NULL)
	{
		if(result != NULL)
		{
			result->found = 0;
			result->position = last_position;
			result->precise_position = last_precise_position;
			result->is_junction = 0;
			result->black_count = 0;
			result->span = 0;
			result->run_count = 0;
			result->raw_centroid = last_precise_position;
			result->junction_ticks = g_junction_ticks;
		}
		return last_precise_position;
	}

	/* ===== 传感器滤波：平滑噪声，减少 pos 跳变 ===== */
	FilterSensorValues(adc_values);

	/* ===== 单遍扫描：收集黑点分布(计数/首末/连续段/全局质心) ===== */
	uint8_t  first_black = 0, last_black = 0;
	uint8_t  run_count = 0;       /* 连续黑段数 */
	uint8_t  prev_black = 0;
	uint32_t index_sum = 0;       /* 全局: Σ(索引×权重) */
	uint32_t weight_sum = 0;      /* 全局: Σ权重 */
	/* 各连续段端点(用于支线场景按连续性选段)；段数上限 = 通道数/2 + 1 */
	uint8_t  run_start[SENSOR_COUNT];
	uint8_t  run_end[SENSOR_COUNT];

	for(i = 0; i < SENSOR_COUNT; i++)
	{
		uint8_t b = BlackPoint_Finder_IsBlackPoint(i, adc_values[i]);
		if(b)
		{
			uint8_t weight = SensorWeight(i);
			if(black_count == 0) first_black = i;
			last_black = i;
			black_count++;
			index_sum  += (uint32_t)i * weight;
			weight_sum += weight;
			if(!prev_black)            /* 新段起点 */
			{
				run_start[run_count] = i;
				run_count++;
			}
			run_end[run_count - 1] = i;  /* 延伸当前段终点 */
		}
		prev_black = b;
	}

	/* 没有黑点：保持上一次位置，清路口标志 */
	if(black_count == 0)
	{
		g_junction_ticks = 0;
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		result->is_junction = 0;
		result->black_count = 0;
		result->span = 0;
		result->run_count = 0;
		result->raw_centroid = last_precise_position;
		result->junction_ticks = 0;
		return last_precise_position;
	}

	uint8_t  span = (uint8_t)(last_black - first_black + 1);
	float    raw_centroid = (float)index_sum / (float)weight_sum;

	/* ===== 路口/支线判定 =====
	 * 正常线/弯道: black_count≤4 且 span≤4 且单段。
	 * 宽黑(count≥5 或 span≥5)= 交叉/T 字路口 → 冻结质心走直，掐断质心被支线拽偏。
	 * 恢复OR逻辑（昨天配置）+ 阈值5（防止弯道4路误触发）→ 平衡鲁棒性和误触发。
	 * 排除全黑(count=SENSOR_COUNT)：那是丢线环境光干扰，不是路口。 */
	uint8_t junction = ((black_count >= 5u) || (span >= 5u)) && (black_count < SENSOR_COUNT);

	if(junction && g_junction_ticks < JUNCTION_FREEZE_MAX_TICKS)
	{
		/* 冻结：保持上次有效精确位置(≈走直)，不更新 last_*，置路口标志 */
		if(g_junction_ticks < 0xFFFFu) g_junction_ticks++;
		result->found = 1;             /* 仍看得见线，绝不能置 0(会触发丢线逻辑) */
		result->position = last_position;
		result->precise_position = last_precise_position;
		result->is_junction = 1;
		result->black_count = black_count;
		result->span = span;
		result->run_count = run_count;
		result->raw_centroid = raw_centroid;
		result->junction_ticks = g_junction_ticks;
		return last_precise_position;
	}

	/* 非路口(或冻结超时)：清路口计数 */
	g_junction_ticks = 0;

	/* 选取精确位置：
	 * - 多段(支线+主线): 选质心最接近上次位置的连续段(连续性跟踪，忽略内侧支线)。
	 * - 单段: 即全局质心(与历史行为一致，边界权重照常生效，弯道响应不变)。 */
	if(run_count >= 2u)
	{
		float best_pos = RunCentroid(run_start[0], run_end[0]);
		float best_err = fabsf(best_pos - last_precise_position);
		uint8_t k;
		for(k = 1; k < run_count; k++)
		{
			float c = RunCentroid(run_start[k], run_end[k]);
			float e = fabsf(c - last_precise_position);
			if(e < best_err) { best_err = e; best_pos = c; }
		}
		precise_pos = best_pos;
		first_black = run_start[0];   /* position 仍报首段起点，保持语义 */
	}
	else
	{
		precise_pos = raw_centroid;
	}

	result->found = 1;
	result->position = first_black;
	result->precise_position = precise_pos;
	result->is_junction = 0;
	result->black_count = black_count;
	result->span = span;
	result->run_count = run_count;
	result->raw_centroid = raw_centroid;
	result->junction_ticks = 0;
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
	g_junction_ticks = 0;
}

