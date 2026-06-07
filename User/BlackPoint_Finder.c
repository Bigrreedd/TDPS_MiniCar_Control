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

/* ===== 路口穿越计数（06-05 用户需求：记录走过几个 Y/交叉口并上串口） =====
 * 去抖上升沿计数：宽黑条件连续 CONFIRM 帧才计 1 次(滤单帧闪烁如全黑中单传感器闪白)，
 * 之后须连续 REARM 帧非路口才重新武装(路口内部 junc 抖动不会重复计数)。
 * 已知误报源(会+1，对照日志甄别)：U 弯两腿同时入视野(两端黑 span≥5)。
 * 用未截断的宽黑原始条件(非 is_junction)：冻结超时(>200帧)的长路口仍只计 1 次。 */
#ifndef JUNCTION_PASS_CONFIRM_TICKS
#define JUNCTION_PASS_CONFIRM_TICKS 10u    /* 500Hz 下 20ms */
#endif
#ifndef JUNCTION_PASS_REARM_TICKS
#define JUNCTION_PASS_REARM_TICKS 100u     /* 500Hz 下 200ms */
#endif
static uint16_t g_junction_pass_count = 0;
static uint16_t g_junc_on_streak  = 0;
static uint16_t g_junc_off_streak = 0;
static uint8_t  g_junc_counted    = 0;

extern volatile uint8_t is_racing;   /* jc 门控：停车/搬运期间不计数(第13轮 jc=4 搬运误计) */

static void JunctionPassUpdate(uint8_t junction_active)
{
	if(!is_racing)
	{
		return;   /* 非运行态冻结计数器(K1 发车时整组清零重新起算) */
	}
	if(junction_active)
	{
		g_junc_off_streak = 0;
		if(g_junc_on_streak < 0xFFFFu) g_junc_on_streak++;
		if(!g_junc_counted && g_junc_on_streak >= JUNCTION_PASS_CONFIRM_TICKS)
		{
			if(g_junction_pass_count < 0xFFFFu) g_junction_pass_count++;
			g_junc_counted = 1;
		}
	}
	else
	{
		g_junc_on_streak = 0;
		if(g_junc_off_streak < 0xFFFFu) g_junc_off_streak++;
		if(g_junc_counted && g_junc_off_streak >= JUNCTION_PASS_REARM_TICKS)
		{
			g_junc_counted = 0;
		}
	}
}

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
	g_junction_pass_count = 0;
	g_junc_on_streak = 0;
	g_junc_off_streak = 0;
	g_junc_counted = 0;
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
			result->junction_pass_count = g_junction_pass_count;
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
		JunctionPassUpdate(0);
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		result->is_junction = 0;
		result->black_count = 0;
		result->span = 0;
		result->run_count = 0;
		result->raw_centroid = last_precise_position;
		result->junction_ticks = 0;
		result->junction_pass_count = g_junction_pass_count;
		return last_precise_position;
	}

	/* C4-blind(06-06 21:18 Run1 实证): 全黑 = 丢线,不是"线在中心"。
	 * 冲出赛道后的深色地面 7 路全黑,旧逻辑落到正常线分支(found=1/质心=中心/corr=0/lost=0)
	 * → 以 33cps 直行盲驶 30cm+,375 丢线自停被绕过。改判丢线:瞬时全黑(起跑线/拱门阴影,
	 * ~16tick)丢线计数照常直行无感,持续全黑(出界)由 375 自停兜底。
	 * jc 不受影响——全黑本就被路口判据的 count<SENSOR_COUNT 排除。 */
	if(black_count >= SENSOR_COUNT)
	{
		g_junction_ticks = 0;
		JunctionPassUpdate(0);
		result->found = 0;
		result->position = last_position;
		result->precise_position = last_precise_position;
		result->is_junction = 0;
		result->black_count = black_count;
		result->span = (uint8_t)(last_black - first_black + 1);
		result->run_count = run_count;
		result->raw_centroid = last_precise_position;
		result->junction_ticks = 0;
		result->junction_pass_count = g_junction_pass_count;
		return last_precise_position;
	}

	uint8_t  span = (uint8_t)(last_black - first_black + 1);
	float    raw_centroid = (float)index_sum / (float)weight_sum;

	/* ===== 路口/支线判定 =====
	 * 正常线/弯道: black_count≤4 且 span≤4 且单段。
	 * 宽黑(count≥5 或 span≥5)= 交叉/T 字路口 → 冻结质心走直，掐断质心被支线拽偏。
	 * 恢复OR逻辑（昨天配置）+ 阈值5（防止弯道4路误触发）→ 平衡鲁棒性和误触发。
	 * 排除全黑(count=SENSOR_COUNT)：那是丢线环境光干扰，不是路口。 */
	/* R1(06-05 审查三方确认): span 支路加 run_count==1。U 弯两腿同入视野=跨度大但黑数少且
	 * 双段(span≥5,count2~4,run_count=2)，旧判据误判路口→强制走直冲出U弯(第10轮实测死因类)；
	 * 真T字/路口=单段宽黑仍触发；双段宽跨度交给下方连续性选段追最近腿。副效益:U腿jc误+1消失。 */
	/* A1(06-06 审计组双员收敛, 7路适配): count 绝对阈值 5 改相对 SENSOR_COUNT-1——
	 * 6路构建 ≡5(83%,旧行为逐位不变)；7路=6(86%)。动机:7路下 5/7=71%，深弯外侧 5 连管
	 * 压线即误判路口强制走直(枚举实证 {0..4}/{1..5}/{2..6} 三个连续5掩码)，重蹈U弯冲出死因；
	 * 真T/十字近满覆盖(6~7管)仍必中。jc 深弯误+1 同步消失。上路第15轮必验项。 */
	/* F44(06-08 方块区首测三跑实证,3独立分析师): 散布型宽黑补判——`black_count≥5 且 run_count≥2`
	 * = 垂直交叉/方块横边骑斜入(质心被拆成两段算花,如 S=1365,0,4095,1365,0,0,0=6黑两段,pos跳5/40)。
	 * 旧判据只认"单段宽黑(6黑/span5单段)",这类散布交叉漏网→质心垃圾→乱打舵脱轨(G1左/G3右随机)。
	 * 深弯外侧5连黑=单段(run_count==1)不触发,U腿双段但 black_count 仅2~4(<5)不触发——既有防误触行为不变。
	 * 红线安全:只决定"宽黑交叉时冻结直穿",不写死转向方向(方向仍由质心,红线b)。 */
	uint8_t junction = ((black_count >= (uint8_t)(SENSOR_COUNT - 1u))
	                    || ((span >= 5u) && (run_count == 1u))
	                    || ((black_count >= 5u) && (run_count >= 2u)))
	                   && (black_count < SENSOR_COUNT);
	JunctionPassUpdate(junction);   /* 穿越计数用未截断条件：超时长路口仍只计1次 */

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
		result->junction_pass_count = g_junction_pass_count;
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
	result->junction_pass_count = g_junction_pass_count;
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
	/* K1 发车会走到这里：每次运行的路口计数从 0 起 */
	g_junction_pass_count = 0;
	g_junc_on_streak = 0;
	g_junc_off_streak = 0;
	g_junc_counted = 0;
}

