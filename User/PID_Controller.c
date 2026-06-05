#include "PID_Controller.h"
#include "MPU6050_Config.h"
#include "Path.h"
#include "ABEncoder.h"
#include <math.h>
extern volatile int16_t position_get;
extern BlackPointResult_t result_BlackPoint;

#ifndef PID_GYRO_ENABLE
#define PID_GYRO_ENABLE 0
#endif
#ifndef WHEEL_BALANCE_ENABLE
#define WHEEL_BALANCE_ENABLE 1
#endif
#ifndef WHEEL_BALANCE_KP
#define WHEEL_BALANCE_KP 8.0f
#endif
#ifndef WHEEL_BALANCE_LIMIT
#define WHEEL_BALANCE_LIMIT 15.0f   /* 限幅15保证不破坏失速裕度(MIN_OUTPUT-decel_cap=20)。
                                      * 深弯时 110-110(decel_cap)-15=−15 仍>0，内侧轮不会反转。
                                      * 原300会让内侧−190→反转。足够补偿死区差(左820右940=120)。 */
#endif
/* 死区不对称前馈补偿：右电机死区 950，左电机 750，差 200。
 * 偏置动态缩放：低速时缩小避免单轮负占空比，高速时满偏置。
 * 公式: dz_bias = clamp(out-20, 0, MOTOR_DZ_BIAS_MAX) */
#ifndef MOTOR_DZ_BIAS_MAX
#define MOTOR_DZ_BIAS_MAX 100.0f
#endif
#ifndef POSITION_LOOP_ENABLE
#define POSITION_LOOP_ENABLE 1
#endif
#ifndef BENCH_POSITION_TEST_ENABLE
#define BENCH_POSITION_TEST_ENABLE 1
#endif
#ifndef BENCH_POSITION_TEST_CORRECTION_LIMIT
#define BENCH_POSITION_TEST_CORRECTION_LIMIT 250.0f
#endif
/* 架空台架速度闭环验证：1=固定速度目标，跳过 Path 状态机。
 * 原因：架空时 total_dist_cm 由编码器假累计推得飞快，Path 状态机会一路升档
 *      (SEARCH 90 -> STRAIGHT 140 -> LINE_FOLLOW 115 ...)，
 *      表现为"目标速度被一直推高"导致单向加速。
 * 上路前必须置 0，否则没有循迹速度规划。 */
#ifndef BENCH_FIXED_SPEED_ENABLE
#define BENCH_FIXED_SPEED_ENABLE 1
#endif
#ifndef BENCH_FIXED_TARGET_CPS
#define BENCH_FIXED_TARGET_CPS 20.0f
#endif
#ifndef SPEED_PID_MIN_OUTPUT
#define SPEED_PID_MIN_OUTPUT 110.0f
#endif

// ==================== 速度环PID实现 ====================

/**
 * @brief 初始化速度环PID控制器
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, 
                   float output_max, float output_min)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;  // 积分限幅设为输出上限的一半
	controller->param.integral_min = output_min * 0.5f;

	controller->last_error = 0.0f;
	controller->last_last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief 速度环PID计算（增量式）
 * 增量式PID公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2*e(k-1)+e(k-2)]
 * 简化版：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-e(k-1)]
 * 输出：u(k) = u(k-1) + Δu(k)
 */
float SpeedPID_Calculate(SpeedPID_Controller_t *controller, float target_speed, float current_speed)
{
	if(controller == NULL) return 0.0f;
	
	float error = target_speed - current_speed;
	float delta_output = 0.0f;
	float new_output = 0.0f;
	
	// 比例项：Kp * (e(k) - e(k-1))
	float p_term = controller->param.kp * (error - controller->last_error);
	
	// 积分项：Ki * e(k)
	float i_term = controller->param.ki * error;
	
	// 微分项：Kd * (e(k) - 2*e(k-1) + e(k-2))  —— 误差增量的增量
	float d_term = controller->param.kd * ((error - controller->last_error) - (controller->last_error - controller->last_last_error));

	// 计算增量输出
	delta_output = p_term + i_term + d_term;

		// 增量限幅：防止误差突变时单步跳变过大
		if (delta_output > 200.0f) delta_output = 200.0f;
		else if (delta_output < -200.0f) delta_output = -200.0f;

	// 新的输出 = 上一次输出 + 增量
	new_output = controller->last_output + delta_output;

	// 输出限幅
	if(new_output > controller->param.output_max)
		new_output = controller->param.output_max;
	else if(new_output < controller->param.output_min)
		new_output = controller->param.output_min;

	// 保存当前误差和输出
	controller->last_last_error = controller->last_error;
	controller->last_error = error;
	controller->last_output = new_output;
	
	return new_output;
}

/**
 * @brief 设置速度环PID参数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief 重置速度环PID控制器
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller)
{
	if(controller == NULL) return;
	
	controller->last_error = 0.0f;
	controller->last_last_error = 0.0f;
	controller->last_output = 0.0f;
	controller->integral = 0.0f;
}

// ==================== 位置环PID实现 ====================

/**
 * @brief 初始化位置环PID控制器
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
                      float output_max, float output_min, float target_position)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
	controller->param.gyro_kd = gyro_kd;
	controller->param.output_max = output_max;
	controller->param.output_min = output_min;
	controller->param.integral_max = output_max * 0.5f;
	controller->param.integral_min = output_min * 0.5f;
	controller->param.target_position = target_position;
	
	controller->last_error = 0.0f;
	controller->integral = 0.0f;
}

/**
 * @brief 位置环PID计算（位置式）
 * 位置式PID公式：u(k) = Kp*e(k) + Ki*Σe(k) + Kd*[e(k)-e(k-1)]
 * 误差定义：error = current_position - target_position
 * 输出：位置偏右时为正（左轮加速、右轮减速，车头左转修正），位置偏左时为负（左轮减速、右轮加速，车头右转修正）
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position)
{
	if(controller == NULL) return 0.0f;
	
	// 误差 = 当前位置 - 目标位置
	// 位置偏右（current_position > target_position）时，error为正，输出为正，左轮加速、右轮减速（车头左转修正回中心）
	float error = current_position - controller->param.target_position;
	float output = 0.0f;
	
	// 比例项：Kp * e(k)
	float p_term = controller->param.kp * error;
	
	// 积分项：Ki * Σe(k)
	controller->integral += error;
	
	// 积分限幅（防止积分饱和）
	if(controller->integral > controller->param.integral_max)
		controller->integral = controller->param.integral_max;
	else if(controller->integral < controller->param.integral_min)
		controller->integral = controller->param.integral_min;
	
	float i_term = controller->param.ki * controller->integral;
	
	// 微分项：低通滤波（压6路灰度阶梯量化产生的微分尖峰，α=0.4）
	// Kd保持固定550——动态Kd对"直线微偏vs入弯"区分不可靠，改用误差变化率本身
	// 微分项天然就是阻尼：慢漂(直线)de/dt小、快变(入弯)de/dt大，滤波后线性Kd已自适应
	float d_raw = error - controller->last_error;
	controller->d_filtered = 0.4f * d_raw + 0.6f * controller->d_filtered;
	float d_term = controller->param.kd * controller->d_filtered;
	float gyro_term = 0.0f;
#if PID_GYRO_ENABLE
	gyro_term = controller->param.gyro_kd * MPU6050_data.gz_rads;
	// gyro_kd * gz_rads 的典型量级远超 3500，需要按实际角速度范围重新标定限幅
	// 正常行驶 gz_rads ≈ ±5 rad/s，急转弯 ≈ ±20 rad/s
	// 限幅设为 output_max 的 40%，保证补偿有效但不过度
	float gyro_limit = controller->param.output_max * 0.4f;
	if(gyro_term >= gyro_limit)
	{
		gyro_term = gyro_limit;
	}
	else if(gyro_term <= -gyro_limit)
	{
		gyro_term = -gyro_limit;
	}
#endif
	// 输出偏差值（直接叠加到速度环输出）
	output = p_term + i_term + d_term - gyro_term;
	
	// 输出限幅
	if(output > controller->param.output_max)
		output = controller->param.output_max;
	else if(output < controller->param.output_min)
		output = controller->param.output_min;
	
	// 保存当前误差
	controller->last_error = error;
	
	return output;
}

/**
 * @brief 设置位置环PID参数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd)
{
	if(controller == NULL) return;
	
	controller->param.kp = kp;
	controller->param.ki = ki;
	controller->param.kd = kd;
}

/**
 * @brief 设置位置环目标位置
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position)
{
	if(controller == NULL) return;
	
	controller->param.target_position = target_position;
}

/**
 * @brief 重置位置环PID控制器
 */
void PositionPID_Reset(PositionPID_Controller_t *controller)
{
	if(controller == NULL) return;

	controller->last_error = 0.0f;
	controller->integral = 0.0f;
	controller->d_filtered = 0.0f;
}

// ==================== 电机控制辅助函数 ====================

/**
 * @brief 根据速度目标值设置电机（自动处理方向和速度）
 * @note 速度负值为后退，正值为前进
 */
/* 最近一次电机目标输出（带符号占空比），供二合一板上板转发到下板执行器 */
volatile float g_motor_target_l = 0.0f;
volatile float g_motor_target_r = 0.0f;
void Motor_SetSpeedWithDirection(uint8_t motor_id, float speed_target)
{
	uint16_t duty = 0;
	uint8_t direction = MOTOR_DIR_FORWARD;
	
	// 记录带符号目标，供上板(大脑)经串口转发给下板(执行器)
	if(motor_id == MOTOR_L) g_motor_target_l = speed_target;
	else if(motor_id == MOTOR_R) g_motor_target_r = speed_target;

	// 判断方向
	if(speed_target < 0.0f)
	{
		// 负值：后退
		direction = MOTOR_DIR_BACKWARD;
		duty = (uint16_t)(-speed_target);  // 取绝对值
	}
	else
	{
		// 正值：前进
		direction = MOTOR_DIR_FORWARD;
		duty = (uint16_t)speed_target;
	}
	
	// 限制占空比范围
	if(duty > MOTOR_DUTY_MAX)
		duty = MOTOR_DUTY_MAX;
	
	// 设置方向和速度
	Motor_SetDirection(motor_id, direction);
	Motor_SetSpeed(motor_id, duty);
}

SpeedPID_Controller_t g_speed_pid;
PositionPID_Controller_t g_position_pid;

// 在main函数中初始化
void PID_Init(void)
{
    // 速度环：控制平均速度。
    // 反馈单位已改为"计数/秒"(见 main.c OnEncFeedback)，与旧的"计数/2ms"
    // 相差约 500x，故增益须整体大幅下调。旧增益是针对恒≈0 的坏反馈调的，
    // 无可保留的有效整定值。下列为按新量纲推算的保守起点，需上车微调：
    //   反馈≈数十计数/秒，输出经 ClampMotorDuty 限到 ±1500。
    //   起点沿用下板低速架空调稳值，Kd 先置 0 避免放大低速量化噪声。
    // 基准增益在参考速度 80 cnt/s 下整定，运行时按 i_speed/80 自动缩放
    SpeedPID_Init(&g_speed_pid, 2.0f, 14.0f, 2.0f, 1000.0f, -1000.0f);

    // 位置环：Kp=48, Kd=550（配合平滑梯度权重）
    PositionPID_Init(&g_position_pid, 48.0f, 0.0f, 550.0f, 0.0f, 9000.0f, -9000.0f, (float)(SENSOR_COUNT - 1u) / 2.0f);
    g_position_pid.param.integral_max = 300.0f;  // 积分限幅降低
}
extern volatile uint8_t is_racing;
/* 速度样本就绪标志(定义于 main.c)：有新窗口速度时为 1 */
extern volatile uint8_t g_speed_sample_ready;
/* 速度环输出在样本间保持：无新样本时沿用上次输出 */
static float g_speed_output = 0.0f;
/* 丢线保护：连续丢线超过 1s → 强制停车 */
static uint16_t g_line_lost_ticks = 0;
/* 丢线寻线：保存上次有效修正值 */
static float g_last_valid_correction = 0.0f;
/* 深弯模式滞回状态：1=内侧轮停转模式。误差≥进入阈值置1，≤退出阈值清0，
 * 掐断弯道边缘质心量化噪声(4.19↔4.54)导致的内侧轮768↔0颤振。 */
static uint8_t g_deep_turn_mode = 0;
#ifndef PID_LINE_LOST_STOP_TICKS
#define PID_LINE_LOST_STOP_TICKS 375u  /* 750ms @ 500Hz control tick */
#endif
/* 当前速度环实际使用的目标(cnt/s)——暴露给调试遥测。
 * 在 BENCH_FIXED_SPEED_ENABLE 下可能与 Path_GetTargetSpeed() 不同。 */
static float g_current_target_speed = 0.0f;
void PID_Control_Update(void)
{
    /* 里程累积已移至 OnEncFeedback(每 10ms 收一帧)，避免 2ms 控制环重复累加同一个 10ms 增量 */
    Path_Update();  // 更新路径状态机
    float current_position;
    float avg_speed;
    float speed_output;
    float position_correction = 0.0f;  // 初始化，防止未定义行为
    float wheel_balance;
    float left_output,right_output;
    float g_raw_abs_err = 0.0f;  // 原始误差幅度(增益曲线放大前),供内侧轮自适应减速
		float i_speed = 0;
		if(!is_racing)
		{
			g_speed_output = 0.0f;
			g_current_target_speed = 0.0f;
			g_speed_sample_ready = 0;
			SpeedPID_Reset(&g_speed_pid);
			PositionPID_Reset(&g_position_pid);
			g_deep_turn_mode = 0;   /* 停车清深弯模式,避免重启残留 */
			Motor_StopAll();
			g_motor_target_l = 0.0f;
			g_motor_target_r = 0.0f;
			return;
		}
	    /* 丢线计数更新（寻线策略延后到位置环计算后） */
	    if (result_BlackPoint.found) {
		g_line_lost_ticks = 0;
	    } else {
		g_line_lost_ticks++;
		// 清零积分，防止丢线期间错误累积
		g_position_pid.integral = 0.0f;
	    }
    // 1. 获取当前位置（从你的position变量）
	    current_position = (float)position_get / 10.0f;
	// 误差增益曲线（二次型，三点定标）：0/5路大幅加猛、中心保持
	// gain = 1.232 - 0.686*|e| + 0.564*e²
	// 中心(e=0.5)→corr≈21；1/4(e=1.5)→corr≈90；0/5(e=2.5)→corr≈310（外侧轮420）
	    {
		float center = g_position_pid.param.target_position;
		float raw_err = current_position - center;
		float ae = fabsf(raw_err);
		g_raw_abs_err = ae;   /* 存原始误差幅度,供内侧轮自适应减速用(非放大后) */
		float gain = 1.232f - 0.686f * ae + 0.564f * ae * ae;
		current_position = center + raw_err * gain;
	    }
    
#if POSITION_LOOP_ENABLE && (!BENCH_FIXED_SPEED_ENABLE || BENCH_POSITION_TEST_ENABLE)
    if (result_BlackPoint.is_junction) {
        /* 路口/岔路：旁路位置环，强制走直(correction=0)。
         * 关键:不调用 PositionPID_Calculate → 环内 last_error/d_filtered 冻结在进路口前的值，
         * 且不更新 g_last_valid_correction，出路口时 d_raw≈0 无微分踢、无支线污染。
         * 深弯滞回在下方按 !is_junction 冻结(路口宽黑会把 raw_abs_err 顶到~2.5+误触发内侧停转)。 */
        position_correction = 0.0f;
    } else {
        // 2. 位置环计算（输出偏差值）
        position_correction = PositionPID_Calculate(&g_position_pid, current_position);

        // 保存有效修正值（供丢线寻线使用）
        g_last_valid_correction = position_correction;

#if BENCH_FIXED_SPEED_ENABLE && BENCH_POSITION_TEST_ENABLE
        if (position_correction > BENCH_POSITION_TEST_CORRECTION_LIMIT) {
            position_correction = BENCH_POSITION_TEST_CORRECTION_LIMIT;
        } else if (position_correction < -BENCH_POSITION_TEST_CORRECTION_LIMIT) {
            position_correction = -BENCH_POSITION_TEST_CORRECTION_LIMIT;
        }
#endif
    }
#else
    position_correction = 0.0f;
    PositionPID_Reset(&g_position_pid);
#endif

    /* 丢线寻线策略：丢线后继续保持上次修正方向 */
    if (!result_BlackPoint.found) {
        // 丢线 < 250ms（125 ticks）：继续寻线，保持上次修正方向
        if (g_line_lost_ticks <= 125u) {
            position_correction = g_last_valid_correction * 0.8f;  // 衰减 20%
        }

        // 丢线超过 1s：停车
        if (g_line_lost_ticks > PID_LINE_LOST_STOP_TICKS) {
            is_racing = 0;
            g_line_lost_ticks = 0;
            return;
        }
    }

skip_position_pid:  // 丢线寻线跳转标签（必须在条件编译块外）
    
    // 3. 计算平均速度
    avg_speed = ((float)speed_left + (float)speed_right) / 2.0f;

#if BENCH_FIXED_SPEED_ENABLE
			/* 架空台架：目标固定，不让 Path 状态机靠假里程一路升档 */
			i_speed = (float)BENCH_FIXED_TARGET_CPS;

			// 丢线时降速：给更多时间重新找线
			if (!result_BlackPoint.found && g_line_lost_ticks > 10u) {
				i_speed = 18.0f;  // 降速到 18 cnt/s
			}
#else
			i_speed = Path_GetTargetSpeed();
#endif
			g_current_target_speed = i_speed;

	    /* PID 增益按目标速度自动缩放：基准在 80 cnt/s 整定 */
	    {
		float s = i_speed / 80.0f;
		if (s < 0.3f) s = 0.3f;
		if (s > 1.5f) s = 1.5f;
		SpeedPID_SetParam(&g_speed_pid, 2.0f*s, 14.0f*s*s*s, 2.0f*s);
	    }
    // 4. 速度环计算（输出基础速度）
    //    速度反馈≈20Hz刷新，速度环只在有新样本时计算，避免500Hz重复积分陈旧值；
    //    无新样本时沿用上一次 speed_output，位置环仍每 2ms 更新保证循迹响应。
    if (g_speed_sample_ready)
    {
        g_speed_sample_ready = 0;
			speed_output = SpeedPID_Calculate(&g_speed_pid, i_speed, avg_speed);
			if (i_speed > 0.0f && speed_output < SPEED_PID_MIN_OUTPUT) {
				speed_output = SPEED_PID_MIN_OUTPUT;
				g_speed_pid.last_output = SPEED_PID_MIN_OUTPUT;
			}
			g_speed_output = speed_output;
    }
    else
    {
        speed_output = g_speed_output;   /* 沿用最近一次速度环输出 */
    }
    // 5. 叠加位置环偏差
#if WHEEL_BALANCE_ENABLE
    #if BENCH_FIXED_SPEED_ENABLE
        /* 台架模式：关闭轮速平衡，避免与 Ki 叠加震荡 */
        wheel_balance = 0.0f;
    #else
        /* 正常模式：开启轮速平衡补偿死区不对称 */
        wheel_balance = ((float)speed_right - (float)speed_left) * WHEEL_BALANCE_KP;
        if(wheel_balance > WHEEL_BALANCE_LIMIT)
        {
            wheel_balance = WHEEL_BALANCE_LIMIT;
        }
        else if(wheel_balance < -WHEEL_BALANCE_LIMIT)
        {
            wheel_balance = -WHEEL_BALANCE_LIMIT;
        }
    #endif
#else
    wheel_balance = 0.0f;
#endif
	    /* 速度自适应：曲率 κ ∝ position_correction / V。
	     * 保持相同曲率 → position_correction ∝ V。
	     * V0=60 cnt/s 为基准，下限 0.85× 保低速转向差速，上限 2.5× 防高速过激。 */
	    {
	        float speed_scale = i_speed / 60.0f;
	        if (speed_scale < 0.85f) speed_scale = 0.85f;
	        if (speed_scale > 2.5f) speed_scale = 2.5f;
	        position_correction *= speed_scale;

		/* 修正总幅限到 320（0/5路再加猛后的需求） */
		{
			float pc_max = 320.0f;
			if (position_correction > pc_max) position_correction = pc_max;
			else if (position_correction < -pc_max) position_correction = -pc_max;
		}
	    }

	    /* 内外侧拆分差速 + 自适应内侧减速：
	     * 直线(小误差)内侧轮限90保持前进不顿挫；
	     * 弯道(大误差)内侧轮减速量升到speed_output，让内侧轮停转→绕内轮急转，压小转弯半径。
	     * 死区是前馈相加，内侧目标≤0时ApplyDeadzone返回0(干净停转,不失速)。 */
	    {
		/* 用原始误差幅度(非增益放大后)+ 滞回开关决定内侧减速深度。
		 * 阈值按实测弯道质心定标:4/5路质心e≈2.0,3/4/5路e≈1.7。
		 * 关键:弯道边缘质心在4.19↔4.54量化跳变(e 1.69↔2.04),若ramp跟瞬时误差走
		 * 内侧轮会768↔0颤振(甩头抖动根源)。用滞回:误差过1.9才进深弯停内侧,
		 * 退到1.5才恢复,中间抖动不切换,内侧稳定停转→稳定绕转而非甩头。
		 * 路口冻结:is_junction 时不评估进/退阈值,保持进路口前的模式。
		 * 否则交叉/T字宽黑会把 g_raw_abs_err 顶到~2.5+(过1.9)误触发内侧停转→车头窜向支线。 */
		if (!result_BlackPoint.is_junction) {
			if (g_raw_abs_err >= 1.9f) {
				g_deep_turn_mode = 1;   /* 进入深弯:内侧轮停转 */
			} else if (g_raw_abs_err <= 1.5f) {
				g_deep_turn_mode = 0;   /* 退出深弯:内侧轮恢复前进 */
			}
		}
		/* decel_cap: 深弯模式内侧减速，否则正常前进(90)。
		 * 实测: 20仍不够，180°弯道差速不足。回退到0（昨天配置）：
		 * 内轮可完全停转，最大差速能力，转弯半径最小。 */
		#define MIN_INNER_WHEEL_SPEED 0.0f
		float decel_cap = g_deep_turn_mode ? speed_output : 90.0f;
		if (decel_cap < 90.0f) decel_cap = 90.0f;   /* 下限90（浅弯） */

		float inner_decel, outer_accel;

		if (position_correction >= 0.0f) {
			/* correction>0：右轮内侧(减速)，左轮外侧(加速)，车头右转 */
			inner_decel = (position_correction > decel_cap) ? decel_cap : position_correction;
			outer_accel = position_correction;  /* 外侧全额加速 */
			left_output  = speed_output + outer_accel + wheel_balance;
			right_output = speed_output - inner_decel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限，防止低速时 decel_cap 下限90仍让内轮过低 */
			if (g_deep_turn_mode && right_output < MIN_INNER_WHEEL_SPEED) {
				right_output = MIN_INNER_WHEEL_SPEED;
			}
		} else {
			/* correction<0：左轮内侧(减速)，右轮外侧(加速)，车头左转 */
			inner_decel = (-position_correction > decel_cap) ? decel_cap : (-position_correction);
			outer_accel = -position_correction;
			left_output  = speed_output - inner_decel + wheel_balance;
			right_output = speed_output + outer_accel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限 */
			if (g_deep_turn_mode && left_output < MIN_INNER_WHEEL_SPEED) {
				left_output = MIN_INNER_WHEEL_SPEED;
			}
		}
	    }
    
    // 6. 设置电机（直接传 float，避免 int16_t 强转溢出 UB）
    Motor_SetSpeedWithDirection(MOTOR_L, left_output);
    Motor_SetSpeedWithDirection(MOTOR_R, right_output);
}

float PID_GetCurrentTargetSpeed(void)
{
    return g_current_target_speed;
}

uint16_t PID_GetLineLostTicks(void)
{
    return g_line_lost_ticks;
}

uint8_t PID_GetDeepTurnMode(void)
{
    return g_deep_turn_mode;
}
