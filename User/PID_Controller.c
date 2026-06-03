#include "PID_Controller.h"
#include "MPU6050_Config.h"
#include "Path.h"
#include "ABEncoder.h"
#include <math.h>
extern volatile int16_t position_get;

#ifndef PID_GYRO_ENABLE
#define PID_GYRO_ENABLE 0
#endif
#ifndef WHEEL_BALANCE_ENABLE
#define WHEEL_BALANCE_ENABLE 1
#endif
#ifndef WHEEL_BALANCE_KP
#define WHEEL_BALANCE_KP 2.0f
#endif
#ifndef WHEEL_BALANCE_LIMIT
#define WHEEL_BALANCE_LIMIT 300.0f
#endif
#ifndef POSITION_LOOP_ENABLE
#define POSITION_LOOP_ENABLE 0
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
#define BENCH_FIXED_TARGET_CPS 80.0f
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
	
	// 微分项：Kd * [e(k) - e(k-1)]
	float d_term = controller->param.kd * (error - controller->last_error);
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
    SpeedPID_Init(&g_speed_pid, 6.0f, 0.5f, 0.0f, 8000.0f, -8000.0f);
    
    // 位置环：输出偏差值，叠加到速度环（循迹响应，未改量纲，保持原整定）
    PositionPID_Init(&g_position_pid, 198.0f, 0.0f, 2280.0f, 0.0f, 9000.0f, -9000.0f, (float)(SENSOR_COUNT - 1u) / 2.0f);
}
extern volatile uint8_t is_racing;
/* 速度样本就绪标志(定义于 main.c)：有新窗口速度时为 1 */
extern volatile uint8_t g_speed_sample_ready;
/* 速度环输出在样本间保持：无新样本时沿用上次输出 */
static float g_speed_output = 0.0f;
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
    float position_correction;
    float wheel_balance;
    float left_output,right_output;
		float i_speed = 0;
		static uint8_t first_set = 0; 
		static float start_speed = 0; 	
		if(!is_racing)
		{
			start_speed = 0;
			first_set = 0;
			g_speed_output = 0.0f;
			g_current_target_speed = 0.0f;
			g_speed_sample_ready = 0;
			SpeedPID_Reset(&g_speed_pid);
			PositionPID_Reset(&g_position_pid);
			Motor_StopAll();
			return;
		}
    // 1. 获取当前位置（从你的position变量）
    current_position = (float)position_get / 10.0f;
    
#if POSITION_LOOP_ENABLE
    // 2. 位置环计算（输出偏差值）
    position_correction = PositionPID_Calculate(&g_position_pid, current_position);
#else
    position_correction = 0.0f;
    PositionPID_Reset(&g_position_pid);
#endif
    
    // 3. 计算平均速度
    avg_speed = ((float)speed_left + (float)speed_right) / 2.0f;

#if BENCH_FIXED_SPEED_ENABLE
			/* 架空台架：目标固定，不让 Path 状态机靠假里程一路升档 */
			i_speed = (float)BENCH_FIXED_TARGET_CPS;
#else
			i_speed = Path_GetTargetSpeed();
#endif
			g_current_target_speed = i_speed;
    // 4. 速度环计算（输出基础速度）
    //    速度反馈≈20Hz刷新，速度环只在有新样本时计算，避免500Hz重复积分陈旧值；
    //    无新样本时沿用上一次 speed_output，位置环仍每 2ms 更新保证循迹响应。
    if (g_speed_sample_ready)
    {
        g_speed_sample_ready = 0;
			if(start_speed < i_speed && first_set == 0)
			{
				start_speed += 2.0f;
			}
			else
			{
				start_speed = i_speed;
				first_set  = 1;
			}
			speed_output = SpeedPID_Calculate(&g_speed_pid, start_speed, avg_speed);
			g_speed_output = speed_output;
    }
    else
    {
        speed_output = g_speed_output;   /* 沿用最近一次速度环输出 */
    }
    // 5. 叠加位置环偏差
#if WHEEL_BALANCE_ENABLE && !BENCH_FIXED_SPEED_ENABLE
		wheel_balance = ((float)speed_right - (float)speed_left) * WHEEL_BALANCE_KP;
		if(wheel_balance > WHEEL_BALANCE_LIMIT)
		{
			wheel_balance = WHEEL_BALANCE_LIMIT;
		}
		else if(wheel_balance < -WHEEL_BALANCE_LIMIT)
		{
			wheel_balance = -WHEEL_BALANCE_LIMIT;
		}
#else
		/* 架空：左右轮速度差是物理特性(死区/摩擦不同)，强行差分补偿会放大成 ±300，
		 * 表现为持续偏一侧。架空速度闭环阶段直接关掉，留到上路后再开。 */
		wheel_balance = 0.0f;
#endif
    left_output = speed_output + position_correction + wheel_balance;   // 左轮加（位置偏右→左轮加速→车头左转修正）
    right_output = speed_output - position_correction - wheel_balance;  // 右轮减（位置偏右→右轮减速→车头左转修正）
    
    // 6. 设置电机（直接传 float，避免 int16_t 强转溢出 UB）
    Motor_SetSpeedWithDirection(MOTOR_L, left_output);
    Motor_SetSpeedWithDirection(MOTOR_R, right_output);
}

float PID_GetCurrentTargetSpeed(void)
{
    return g_current_target_speed;
}
