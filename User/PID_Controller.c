#include "PID_Controller.h"
#include "LSM6DSR_Config.h"
#include <math.h>
 extern int16_t position_get;
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
	
	// 微分项：Kd * (e(k) - e(k-1))
	float d_term = controller->param.kd * (error - controller->last_error);
	 
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
 * 输出：位置偏右时为正（左轮减速、右轮加速），位置偏左时为负（左轮加速、右轮减速）
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position)
{
	if(controller == NULL) return 0.0f;
	
	// 误差 = 当前位置 - 目标位置
	// 位置偏右（current_position > target_position）时，error为正，输出为正，左轮减速、右轮加速
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
	float gyro_term = controller->param.gyro_kd * LSE6DSR_data.gz_rads;
	if(gyro_term >= 7000)
	{
		gyro_term = 7000;
	}
	else if(gyro_term <= -7000)
	{
		gyro_term = -7000;
	}
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
void Motor_SetSpeedWithDirection(uint8_t motor_id, float speed_target)
{
	uint16_t duty = 0;
	uint8_t direction = MOTOR_DIR_FORWARD;
	
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
    // 速度环：控制平均速度
    SpeedPID_Init(&g_speed_pid, 5.5f, 5.1f, 5.8f, 8000.0f, -8000.0f);
    
    // 位置环：输出偏差值，叠加到速度环
    PositionPID_Init(&g_position_pid, 198.0f, 0.0f, 2280.0f, 2800.0f, 9000.0f, -9000.0f, 8.0f);
}
extern uint8_t star_car;
void PID_Control_Update(void)
{
    float current_position;
    float avg_speed;
    float speed_output;
    float position_correction;
    float left_output,right_output;
		float i_speed = 0;
		static uint8_t first_set = 0; 
		static float statr_speed = 0; 	
    // 1. 获取当前位置（从你的position变量）
    current_position = (float)position_get / 10.0f;
    
    // 2. 位置环计算（输出偏差值）
    position_correction = PositionPID_Calculate(&g_position_pid, current_position);
    
    // 3. 计算平均速度
    avg_speed = ((float)speed_left + (float)speed_right) / 2.0f;
    
		i_speed = 230 - (fmin(fabs(current_position - 8),3)/3) * 100;
    // 4. 速度环计算（输出基础速度）
		if(star_car)
    {
			if(statr_speed < i_speed && first_set == 0)
			{
				statr_speed += 0.5f;
			}
			else
			{
				statr_speed = i_speed;
				first_set  = 1;
			}
			speed_output = SpeedPID_Calculate(&g_speed_pid, statr_speed, avg_speed);
    }
		else
		{	
			statr_speed = 0;
			first_set = 0;
			g_speed_pid.last_output = 0;
			g_speed_pid.last_error = 0;
			g_speed_pid.integral = 0;
			speed_output = 0;
		}
    // 5. 叠加位置环偏差
    left_output = speed_output + position_correction;   // 左轮减
    right_output = speed_output - position_correction;  // 右轮加
    
    // 6. 设置电机
    Motor_SetSpeedWithDirection(MOTOR_L, (int16_t)left_output);
    Motor_SetSpeedWithDirection(MOTOR_R, (int16_t)right_output);
}