#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "stm32f10x.h"
#include "Motor_ctr.h"
#include "ABEncoder.h"
#include "BlackPoint_Finder.h"

/**
 * @brief PID控制器使用说明
 * 
 * 控制架构：
 * 1. 速度环（内环）：控制平均速度，反馈 = (speed_left + speed_right) / 2
 * 2. 位置环（外环）：输出偏差值，直接叠加到速度环输出上
 * 
 * 使用流程：
 * 1. 计算速度环：
 *    - 目标速度：target_speed（固定值）
 *    - 反馈速度：avg_speed = (speed_left + speed_right) / 2
 *    - 速度环输出：speed_output = SpeedPID_Calculate(&speed_pid, target_speed, avg_speed)
 * 
 * 2. 计算位置环：
 *    - 当前位置：current_position（寻点位置）
 *    - 位置环输出：position_correction = PositionPID_Calculate(&position_pid, current_position)
 * 
 * 3. 最终输出（注意极性）：
 *    - left_output = speed_output - position_correction  （位置偏右时，position_correction为正，左轮减速）
 *    - right_output = speed_output + position_correction  （位置偏右时，position_correction为正，右轮加速）
 * 
 * 极性说明：
 * - 位置偏右（position > target）：position_correction > 0，左轮减速、右轮加速，修正回中心
 * - 位置偏左（position < target）：position_correction < 0，左轮加速、右轮减速，修正回中心
 */

// ==================== 速度环PID参数 ====================
// 增量式PID参数（用于电机速度控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float output_max;   // 输出上限（占空比，0-10000）
	float output_min;   // 输出下限（占空比，-10000到0，负值为后退）
	float integral_max; // 积分限幅（防止积分饱和）
	float integral_min; // 积分下限
} SpeedPID_Param_t;

// 速度环PID控制器结构体
typedef struct
{
	SpeedPID_Param_t param;  // PID参数
	float last_error;        // 上一次误差
	float last_output;       // 上一次输出
	float integral;          // 积分项累积
} SpeedPID_Controller_t;

// ==================== 位置环PID参数 ====================
// 位置式PID参数（用于寻点位置控制）
typedef struct
{
	float kp;           // 比例系数
	float ki;           // 积分系数
	float kd;           // 微分系数
	float gyro_kd;
	float output_max;   // 输出上限（偏差值，叠加到速度环输出）
	float output_min;   // 输出下限（偏差值，叠加到速度环输出）
	float integral_max; // 积分限幅
	float integral_min; // 积分下限
	float target_position; // 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
} PositionPID_Param_t;

// 位置环PID控制器结构体
typedef struct
{
	PositionPID_Param_t param;  // PID参数
	float last_error;            // 上一次误差
	float integral;              // 积分项累积
} PositionPID_Controller_t;

// ==================== 速度环PID函数 ====================
/**
 * @brief 初始化速度环PID控制器
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（占空比，0-10000）
 * @param output_min: 输出下限（占空比，-10000到0）
 */
void SpeedPID_Init(SpeedPID_Controller_t *controller, float kp, float ki, float kd, 
                   float output_max, float output_min);

/**
 * @brief 速度环PID计算（增量式）
 * @param controller: 速度环PID控制器指针
 * @param target_speed: 目标速度（编码器值）
 * @param current_speed: 当前速度（编码器值）
 * @return 输出值（占空比，0-10000为正转，负值为后退）
 */
float SpeedPID_Calculate(SpeedPID_Controller_t *controller, float target_speed, float current_speed);

/**
 * @brief 设置速度环PID参数
 * @param controller: 速度环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void SpeedPID_SetParam(SpeedPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 重置速度环PID控制器（清零积分项和历史值）
 * @param controller: 速度环PID控制器指针
 */
void SpeedPID_Reset(SpeedPID_Controller_t *controller);

// ==================== 位置环PID函数 ====================
/**
 * @brief 初始化位置环PID控制器
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param output_max: 输出上限（偏差值，叠加到速度环输出）
 * @param output_min: 输出下限（偏差值，叠加到速度环输出）
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_Init(PositionPID_Controller_t *controller, float kp, float ki, float kd, float gyro_kd,
                      float output_max, float output_min, float target_position);

/**
 * @brief 位置环PID计算（位置式）
 * @param controller: 位置环PID控制器指针
 * @param current_position: 当前位置（寻点位置，左边为0）
 * @return 输出偏差值（直接叠加到速度环输出上，左轮减、右轮加）
 * @note 位置偏右时输出为正，左轮减速、右轮加速；位置偏左时输出为负，左轮加速、右轮减速
 */
float PositionPID_Calculate(PositionPID_Controller_t *controller, float current_position);

/**
 * @brief 设置位置环PID参数
 * @param controller: 位置环PID控制器指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 */
void PositionPID_SetParam(PositionPID_Controller_t *controller, float kp, float ki, float kd);

/**
 * @brief 设置位置环目标位置
 * @param controller: 位置环PID控制器指针
 * @param target_position: 目标位置（寻点中心位置，通常为SENSOR_COUNT/2）
 */
void PositionPID_SetTarget(PositionPID_Controller_t *controller, float target_position);

/**
 * @brief 重置位置环PID控制器（清零积分项和历史值）
 * @param controller: 位置环PID控制器指针
 */
void PositionPID_Reset(PositionPID_Controller_t *controller);

// ==================== 电机控制辅助函数 ====================
/**
 * @brief 根据速度目标值设置电机（自动处理方向和速度）
 * @param motor_id: 电机编号（MOTOR_L 或 MOTOR_R）
 * @param speed_target: 速度目标值（占空比，0-10000为正转，负值为后退）
 */
void Motor_SetSpeedWithDirection(uint8_t motor_id, float speed_target);
void PID_Init(void);
void PID_Control_Update(void);
#endif // __PID_CONTROLLER_H__

