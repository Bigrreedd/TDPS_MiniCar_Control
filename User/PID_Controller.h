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
 *    - left_output = speed_output + position_correction  （位置偏右时，position_correction为正，左轮加速，车头左转修正）
 *    - right_output = speed_output - position_correction （位置偏右时，position_correction为正，右轮减速，车头左转修正）
 *
 * 极性说明：
 * - 位置偏右（position > target）：position_correction > 0，左轮加速、右轮减速，车头左转修正回中心
 * - 位置偏左（position < target）：position_correction < 0，左轮减速、右轮加速，车头右转修正回中心
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
	float last_last_error;   // 上上次误差（用于增量式微分项）
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
	float d_filtered;            // 微分项低通滤波状态（压灰度量化尖峰）
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
 * @return 输出偏差值（直接叠加到速度环输出上，左轮加、右轮减）
 * @note 位置偏右时输出为正，左轮加速、右轮减速（车头左转修正回中心）；位置偏左时输出为负，左轮减速、右轮加速（车头右转修正回中心）
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

/**
 * @brief 获取当前速度环正在使用的目标速度(cnt/s)
 *        用于调试遥测——考虑到 BENCH_FIXED_SPEED_ENABLE 会覆盖 Path 目标，
 *        调用方不应直接查 Path_GetTargetSpeed()。
 */
float PID_GetCurrentTargetSpeed(void);

/**
 * @brief 获取丢线计数（连续丢线帧数）
 * @return 丢线计数（0=有线，>0=丢线帧数）
 */
uint16_t PID_GetLineLostTicks(void);

/**
 * @brief 获取深弯模式状态
 * @return 1=深弯模式（内侧轮保留最小速度），0=正常模式
 */
uint8_t PID_GetDeepTurnMode(void);

/* ===== P2 导航覆盖接口(06-06 雷达避障段) =====
 * 旁路寻线/路口/锁向全套机制,由上层(main 雷达状态机)直接给定运动指令：
 *   NONE    — 正常循迹(默认)
 *   HOLD    — 清洁停车保持(目标/输出清零,ApplyDeadzone(0)=0;丢线计数冻结)
 *   HEADING — 航向保持直行:corr=Kyaw×(当前-目标)yaw,限幅±50(非深弯 cap 域,
 *             floor 70 下内轮≥20 永不为负);丢线 375 自停被旁路,时长由上层预算兜底 */
#define NAV_OVERRIDE_NONE     0u
#define NAV_OVERRIDE_HOLD     1u
#define NAV_OVERRIDE_HEADING  2u
/* TURN(F47) — 写死侧深弯锐弧:corr=±320 按 PID_SetTurnDir 符号,内轮 coast(R≈6.5cm,差速不反转);
 * 在 PID 分支链最前(压 is_junction/A2);丢线 375 自停+lose_time 被旁路(非 NONE)。SEG3 专用,
 * 比赛构型(TEST_SEGMENT==0)从不置此值=行为级不变。完成由上层 IMU 到角/Δyaw 预算判定。 */
#define NAV_OVERRIDE_TURN     3u

/**
 * @brief 设置导航覆盖模式
 * @param mode: NAV_OVERRIDE_*
 * @param yaw_target_rad: HEADING 模式的目标航向(add_angle 标系,rad)
 * @param speed_cps: HEADING 模式的目标速度(cnt/s;实际受速度环 floor 70 托底)
 * @note  !is_racing(停车)时 PID 内部自动清回 NONE
 */
void PID_SetNavOverride(uint8_t mode, float yaw_target_rad, float speed_cps);

/**
 * @brief 获取当前导航覆盖模式(遥测/状态机查询用)
 */
uint8_t PID_GetNavOverride(void);

/**
 * @brief 设置 NAV_OVERRIDE_TURN 的转向符号(F47)
 * @param dir: +1=左转(yaw 增大);-1=右转。符号锁存至下次调用。SEG3 专用。
 */
void PID_SetTurnDir(int8_t dir);

/**
 * @brief 设置 SEG3 分叉偏置(F48):巡线域内(NAV_OVERRIDE_NONE)注入固定 corr 把车拐上支线。
 * @param dir: -1=左偏(corr<0) / +1=右偏(corr>0) / 0=关闭。非0时压过 is_junction 冻结+强制 deep=0
 *             (两轮都驱动的中等弧,不甩离线)。SEG3 专用;比赛构型从不置非0=行为级不变。
 */
void PID_SetBranchBias(int8_t dir);

#endif // __PID_CONTROLLER_H__

