#include "PID_Controller.h"
#include "MPU6050_Config.h"
#include "Path.h"
#include "ABEncoder.h"
#include "stm32f10x_it.h"   /* A2b-limit: add_angle(yaw 积分,rad) */
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
 * 【06-07 评审改判,旧句"上路前必须置0"作废(skeptic R3 地雷)】=1 是现役构型:
 * H3 三段律(28 / u后25 / sm14)就是当前速度调度器,全部已验证配方在此域整定。
 * 置 0 会激活 Path.c 已废弃的全量段FSM(里程刻度 6.0 错 15× + 旧布局段表)→
 * 速度规划全程错乱。禁随意置 0;Path 域复活=独立战役(改刻度 0.405 + 重填
 * DIST 表 + 台架复验深弯半径),见 PID_TUNING_LOG 06-07 评审条目。 */
#ifndef BENCH_FIXED_SPEED_ENABLE
#define BENCH_FIXED_SPEED_ENABLE 1
#endif
/* E1(06-06 用户拍板): 20→25——赛道凹凸托底处用惯性冲过(用户选择,替代机械整改)。
 * 动能 +56%;S 弯域(S_MODE 14/丢线 12)不动;U 弯差速比随外轮同升反而略紧,安全。 */
/* E2(06-06 20:46/20:49 上图): 25→28——凸起卡住两轮再现(开扇 46:34 双轮全停/关扇 49:46
 * R=3 瞬空转),继续按用户"惯性冲过"路线加动能 +25%;丢线档 22 与 sm 域 14/12 均不动。 */
#ifndef BENCH_FIXED_TARGET_CPS
#define BENCH_FIXED_TARGET_CPS 28.0f
#endif
/* F1撤回(06-07 用户纠偏):问题不是只补 S 后段,而是全图基础扭矩偏低。
 * post-U 保持 H3 的 25cps,整体余量改由 main.c 的 HOLD +30 提供。 */
#ifndef POST_U_TARGET_CPS
#define POST_U_TARGET_CPS 25.0f
#endif
#ifndef SPEED_PID_MIN_OUTPUT
/* 110→70(06-05 第9轮): 110在亏电电池上定，满电时=43cps把速度环钉死在2.2×目标。
 * 与浅弯decel_cap 90→50对偶下移：失速裕度 70-50=20 不变，深弯内轮硬下限20不变。 */
#define SPEED_PID_MIN_OUTPUT 70.0f
#endif
/* ===== S-mode 分段参数(06-05 用户方案,过Y2锁存,见 main.c g_s_mode) =====
 * 只降速不动转向：波浪S弯(30~50cm交替弯)每弯时间+43%，226差速角速度余量同比放大。
 * floor/cap 对偶下移保持失速裕度 50-30=20 不变量。 */
#ifndef S_MODE_TARGET_CPS
#define S_MODE_TARGET_CPS 14.0f
#endif
#ifndef S_MODE_MIN_OUTPUT
#define S_MODE_MIN_OUTPUT 50.0f
#endif
#ifndef S_MODE_SHALLOW_CAP
#define S_MODE_SHALLOW_CAP 30.0f
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
	// Kd保持固定(值见PID_Init)——动态Kd对"直线微偏vs入弯"区分不可靠，改用误差变化率本身
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

    // 位置环：Kp=40, Kd=550。
    // Kd 400→550 回退(06-05 第8轮): 400实测更差——温和段0.9~1.5s缩到0.3~0.6s，发车后
    // 300~600ms即触发deep。新几何下D净阻尼为正，"量化D踢是放大器"假设证伪，550重验证胜出。
    // Kp 48→40(06-05): 新几何±0.5粗台阶单步误差更大=等效每步增益更高，48已越下移后的
    // 新上限（温和摆动自发逐摆增长=增益过高型发散）。旧几何上限48（50即临界）。
    PositionPID_Init(&g_position_pid, 40.0f, 0.0f, 550.0f, 0.0f, 9000.0f, -9000.0f, (float)(SENSOR_COUNT - 1u) / 2.0f);
    g_position_pid.param.integral_max = 300.0f;  // 积分限幅降低
}
extern volatile uint8_t is_racing;
extern volatile uint8_t g_s_mode;   /* S-mode：过第二个Y后锁存的分段降速(main.c) */
extern volatile uint8_t g_u_turn_passed;   /* H3: U 弯完成锁存(main.c)——u 后段降速用 */
/* 速度样本就绪标志(定义于 main.c)：有新窗口速度时为 1 */
extern volatile uint8_t g_speed_sample_ready;
/* 速度环输出在样本间保持：无新样本时沿用上次输出 */
static float g_speed_output = 0.0f;
/* 丢线保护：连续丢线超过 1s → 强制停车 */
static uint16_t g_line_lost_ticks = 0;
/* 丢线寻线：保存上次有效修正值 */
static float g_last_valid_correction = 0.0f;
/* A2(06-06): sm区丢线再捕获去抖——连续 found 确认计数 */
static uint16_t g_reacq_run = 0;
/* A2b(06-06): 最近一次"线在边缘"的方向记忆(+1=右缘/-1=左缘/0=无)——sm 丢线找回用。
 * 19:21 实测:S 拐换边瞬间丢线时修正恰好过零,A2 锁 last_valid≈14 形同直行白丢。 */
static int8_t g_last_edge_side = 0;
/* A2c(06-06): 再捕获宽限计数——sm 丢线锁向后刚找回线的 ~100ms 内限幅修正+禁深弯,
 * 让车"滚上线"。19:30 实测乒乓极限环:catch→边缘大误差立即反向全幅 pivot→冲过线再丢,
 * 四拐过了三个半全靠运气性收敛。 */
static uint16_t g_reacq_grace = 0;
/* A2b-limit(06-06 用户拍板): 锁向旋转预算——进入锁向时的 yaw 基准(rad)与翻转计数。
 * 20:07 实测锁向连转 200°+ 未捞线(扫穿线后边缘记忆指向身后,pivot 绕圈追不上)。 */
static float   g_a2_yaw_base = 0.0f;
static uint8_t g_a2_flips = 0;
/* H2'(06-06 21:17/21:19 乒乓实测): 预算跨段持续——锁向解除后 500ms 内重锁不重置
 * 基准/flips。确认级乒乓(捞线 50ms 过 25tick 去抖→100ms 宽限扶不正→甩穿翻边重锁)
 * 每段都合法重置预算,110° 形同虚设;跨段累计后两次翻转→放弃锁向直行慢爬兜底。 */
static uint16_t g_a2_episode_cool = 0;
/* P2 导航覆盖(06-06 雷达避障段): 见 PID_Controller.h 接口注释。
 * Kyaw=300/rad、限幅 ±50:err 0.17rad(10°)即饱和,差速(out±50)在非深弯 cap 域,
 * floor 70 下内轮 70-50=20≥0 永不为负(规避反向死区踢)。 */
#define NAV_YAW_KP    300.0f
#define NAV_CORR_CAP  50.0f
static volatile uint8_t g_nav_override = NAV_OVERRIDE_NONE;
static float g_nav_yaw_target = 0.0f;
static float g_nav_speed_cps = 0.0f;
/* 深弯模式滞回状态：1=内侧轮停转模式。误差≥进入阈值置1，≤退出阈值清0，
 * 掐断弯道边缘质心量化噪声(4.19↔4.54)导致的内侧轮768↔0颤振。 */
static uint8_t g_deep_turn_mode = 0;
#ifndef PID_LINE_LOST_STOP_TICKS
#define PID_LINE_LOST_STOP_TICKS 375u  /* 750ms @ 500Hz control tick */
#endif
#ifndef REACQ_CONFIRM_TICKS
#define REACQ_CONFIRM_TICKS 25u  /* A2(06-06): 再捕获连续确认帧数(50ms@500Hz),滤甩头单帧扫过相邻S线段的假捕获 */
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
			g_reacq_run = 0;        /* A2 去抖状态同清 */
			g_last_edge_side = 0;   /* A2b 边缘记忆同清 */
			g_reacq_grace = 0;      /* A2c 宽限同清 */
			g_a2_flips = 0;         /* A2b-limit 同清 */
			g_a2_episode_cool = 0;  /* H2' 跨段窗口同清 */
			g_line_lost_ticks = 0;  /* F11(06-07 红队): 原清单漏此项——停车后冻结残留(实测 lost=266
			                         * 钉屏≥6s),污染遥测判读;且发车即丢线时陈值直撞 375 自停门。
			                         * 与主环 lose_time(StopRun 清)对齐,两计数器不再分裂。 */
			g_nav_override = NAV_OVERRIDE_NONE;  /* P2 导航覆盖同清(K2 停车即退覆盖) */
			Motor_StopAll();
			g_motor_target_l = 0.0f;
			g_motor_target_r = 0.0f;
			return;
		}
	    /* P2 HOLD 覆盖:清洁停车保持(雷达箱前停稳等决策)。目标/输出全清,
	     * ApplyDeadzone(0)=0 干净停;速度环状态归零→解除后经 R3 从 START 档干净再起步。
	     * 早退冻结丢线计数/位置环状态;解除走 HEADING(盲走),其 s_prev_junction 通道
	     * 保证最终回循迹时 R5 软启动消 D 踢。 */
	    if (g_nav_override == NAV_OVERRIDE_HOLD)
	    {
			g_motor_target_l = 0.0f;
			g_motor_target_r = 0.0f;
			g_speed_pid.last_output = 0.0f;
			g_speed_output = 0.0f;
			Motor_SetSpeedWithDirection(MOTOR_L, 0.0f);
			Motor_SetSpeedWithDirection(MOTOR_R, 0.0f);
			return;
	    }
	    /* 丢线计数更新（寻线策略延后到位置环计算后）
	     * A2(06-06 用户批准): sm区深丢线(>250ms)后,甩头单帧扫过S弯相邻线段会瞬间翻转
	     * 修正方向(17:07 波浪换边实测)——再捕获需连续 REACQ_CONFIRM_TICKS 帧 found 才
	     * 解除丢线;确认期内 g_line_lost_ticks 冻结在 >125,下方 A2 锁向分支继续生效。 */
	    if (result_BlackPoint.found) {
		if (g_s_mode && g_line_lost_ticks > 125u) {
		    g_reacq_run++;
		    if (g_reacq_run >= REACQ_CONFIRM_TICKS) {
			g_line_lost_ticks = 0;   /* 去抖通过,正式解除丢线 */
			g_reacq_run = 0;
		    }
		} else {
		    g_line_lost_ticks = 0;
		    g_reacq_run = 0;
		}
	    } else {
		g_reacq_run = 0;
		g_line_lost_ticks++;
		// 清零积分，防止丢线期间错误累积
		g_position_pid.integral = 0.0f;
	    }
	    if (g_a2_episode_cool > 0u) g_a2_episode_cool--;   /* H2': 跨段窗口倒计时(2ms/tick) */
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
		/* A2b: found 且线到边缘(|raw_err|≥1.5)时记忆方向,供 sm 丢线找回 */
		if (result_BlackPoint.found) {
			if (raw_err >= 1.5f)       g_last_edge_side = 1;
			else if (raw_err <= -1.5f) g_last_edge_side = -1;
		}
		float gain = 1.232f - 0.686f * ae + 0.564f * ae * ae;
		current_position = center + raw_err * gain;
	    }
    
#if POSITION_LOOP_ENABLE && (!BENCH_FIXED_SPEED_ENABLE || BENCH_POSITION_TEST_ENABLE)
    {
    static uint8_t s_prev_junction = 0;
    static uint8_t s_prev_a2hold = 0;
    if (g_nav_override == NAV_OVERRIDE_HEADING) {
        /* P2 盲走航向保持:旁路寻线/路口/锁向,corr=Kyaw×(当前-目标)。
         * 符号:corr>0=右转(下方差速注释),目标在左(target>yaw)时 corr<0 → 左转 ✓。
         * 限幅 ±50 见 NAV_CORR_CAP 注释;借 s_prev_junction 通道,覆盖解除首帧
         * 走 R5 软启动(对齐 last_error,消冻结期 D 踢),与路口退出同语义。 */
        position_correction = (add_angle - g_nav_yaw_target) * NAV_YAW_KP;
        if (position_correction > NAV_CORR_CAP) position_correction = NAV_CORR_CAP;
        else if (position_correction < -NAV_CORR_CAP) position_correction = -NAV_CORR_CAP;
        s_prev_junction = 1;
    } else if (result_BlackPoint.is_junction) {
        /* 路口/岔路：旁路位置环，强制走直(correction=0)。
         * 关键:不调用 PositionPID_Calculate → 环内 last_error/d_filtered 冻结在进路口前的值，
         * 且不更新 g_last_valid_correction。注意：冻结期质心若移动，退出帧 d_raw=新误差-冻结
         * 旧误差 ≠0（旧注释"d_raw≈0"过度承诺，06-05 审查 I7）——由下方 R5 软启动消除。
         * 深弯滞回在下方按 !is_junction 冻结(路口宽黑会把 raw_abs_err 顶到~2.5+误触发内侧停转)。 */
        position_correction = 0.0f;
        s_prev_junction = 1;
    } else if (g_s_mode && g_line_lost_ticks > 125u) {
        /* A2(06-06 用户批准): sm 区深丢线/再捕获确认期——锁向:按最后所见侧持续修正
         * 直至再捕获,替代旧"冻结质心上的全增益PID"(甩头扫过相邻S线段单帧翻向=波浪换边源)。
         * A2b 升级(19:21 实测): 优先朝"最后所见边缘"方向满幅找线——S 拐换边瞬间丢线时
         * 修正恰好过零,锁 last_valid≈0 形同直行;边缘记忆指向线的真实退出侧。
         * A2b-limit(20:07 实测,用户拍板): 旋转预算 110°——锁向连转 200°+ 未捞线=已扫穿线,
         * 边缘记忆指向身后;|Δyaw|>110° 翻转方向追一次;再超 110° 放弃锁向直行慢爬
         * (T 已降 12,deep 下 corr=0 两轮同速=直行),由扫线再捕获/375 自停兜底。
         * 位置环状态冻结,last_valid 不被陈旧质心覆写;解除时走下方 D 软启动。 */
        if (!s_prev_a2hold && g_a2_episode_cool == 0u) { /* 锁向进入帧:记 yaw 基准。
             * H2': 解除后 500ms 内重锁=同一事件,沿用旧基准/flips 跨段累计预算 */
            g_a2_yaw_base = add_angle;
            g_a2_flips = 0;
        }
        {
            float a2_dyaw = add_angle - g_a2_yaw_base;
            if (a2_dyaw < 0.0f) a2_dyaw = -a2_dyaw;
            if (a2_dyaw > 1.92f) {           /* 110° ≈ 1.92 rad */
                if (g_a2_flips == 0u) {
                    g_a2_flips = 1u;
                    g_a2_yaw_base = add_angle;
                    if (g_last_edge_side != 0) g_last_edge_side = (int8_t)(-g_last_edge_side);
                    else g_last_valid_correction = -g_last_valid_correction;
                } else {
                    g_a2_flips = 2u;         /* 双向都追过:放弃锁向 */
                }
            }
        }
        if (g_a2_flips >= 2u)
            position_correction = 0.0f;      /* 直行慢爬等扫线 */
        else if (g_last_edge_side != 0)
            position_correction = (float)g_last_edge_side * 320.0f;
        else
            position_correction = g_last_valid_correction;
        s_prev_a2hold = 1;
    } else {
        if (s_prev_junction || s_prev_a2hold) {
            /* R5(06-05 审查): 路口/A2锁向退出首帧 D 软启动——对齐 last_error 使本帧 d_raw=0，
             * 消除冻结期线位移造成的一次性 D 踢(原本有界但无谓,α=0.4 衰 3~4 帧)。 */
            g_position_pid.last_error = current_position - g_position_pid.param.target_position;
            s_prev_junction = 0;
            if (s_prev_a2hold) {
                g_reacq_grace = 50u;        /* A2c: 锁向找回后给 100ms 滚上线宽限 */
                g_a2_episode_cool = 250u;   /* H2': 开 500ms 跨段窗口(乒乓重锁不重置预算) */
            }
            s_prev_a2hold = 0;
        }
        // 2. 位置环计算（输出偏差值）
        position_correction = PositionPID_Calculate(&g_position_pid, current_position);

        // 保存有效修正值（供丢线寻线使用）
        g_last_valid_correction = position_correction;

        /* A2c: 宽限期内修正限幅 ±150——刚从锁向找回线,边缘大误差不许立即反向全幅,
         * 先以缓和差速滚上线;last_valid 保存未限幅值(再丢线时锁向仍走边缘记忆)。 */
        if (g_s_mode && g_reacq_grace > 0u) {
            g_reacq_grace--;
            if (position_correction > 150.0f) position_correction = 150.0f;
            else if (position_correction < -150.0f) position_correction = -150.0f;
        }

#if BENCH_FIXED_SPEED_ENABLE && BENCH_POSITION_TEST_ENABLE
        if (position_correction > BENCH_POSITION_TEST_CORRECTION_LIMIT) {
            position_correction = BENCH_POSITION_TEST_CORRECTION_LIMIT;
        } else if (position_correction < -BENCH_POSITION_TEST_CORRECTION_LIMIT) {
            position_correction = -BENCH_POSITION_TEST_CORRECTION_LIMIT;
        }
#endif
    }
    }   /* R5 s_prev_junction 作用域 */
#else
    position_correction = 0.0f;
    PositionPID_Reset(&g_position_pid);
#endif

    /* 丢线寻线策略：丢线后继续保持上次修正方向
     * P2: 导航覆盖期间整段旁路——盲走(箱内无线)不许衰减覆写 corr,更不许 375 自停;
     * 时长安全由 main 雷达状态机的每相 tick 预算兜底(超时 RD_FAIL 停车)。 */
    if (!result_BlackPoint.found && g_nav_override == NAV_OVERRIDE_NONE) {
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
			/* 架空台架：目标固定，不让 Path 状态机靠假里程一路升档。
			 * S-mode(过Y2锁存)：降到 14，波浪S弯每弯时间+43%。 */
			/* H3(06-06 21:17/21:19 上图): u 后段(U 出口→S 入口)28→25——S 进场速度
			 * 恢复 19:49 成功条件(入弯热+U 出口回稳慢 1.8s 均为 28cps 副作用);
			 * 凸起在 u 前段,保留 E2 的 28 动能;sm 域 14 不动。 */
			i_speed = g_s_mode ? S_MODE_TARGET_CPS
			        : (g_u_turn_passed ? POST_U_TARGET_CPS : (float)BENCH_FIXED_TARGET_CPS);

			// 丢线时降速：给更多时间重新找线（S-mode 下再低一档）
			// E1: 非 sm 丢线档 18→22 随基准等比上调;sm 域 12 不动
			if (!result_BlackPoint.found && g_line_lost_ticks > 10u) {
				i_speed = g_s_mode ? 12.0f : 22.0f;
			}

			/* P2 盲走:固定爬行档,不走丢线降速(箱内无线是常态非异常)。
			 * 实际速度受下方 floor 70 托底(≈20cps),几何预算按计数不按时间,不受影响。 */
			if (g_nav_override == NAV_OVERRIDE_HEADING) {
				i_speed = g_nav_speed_cps;
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
    /* 速度环下限：S-mode 50(否则 14 的目标被 70 钳死到不了)，正常 70 */
    {
        float min_out = g_s_mode ? S_MODE_MIN_OUTPUT : SPEED_PID_MIN_OUTPUT;
    if (g_speed_sample_ready)
    {
        g_speed_sample_ready = 0;
			speed_output = SpeedPID_Calculate(&g_speed_pid, i_speed, avg_speed);
			if (i_speed > 0.0f && speed_output < min_out) {
				speed_output = min_out;
				g_speed_pid.last_output = min_out;
			}
			g_speed_output = speed_output;
    }
    else
    {
        speed_output = g_speed_output;   /* 沿用最近一次速度环输出 */
        /* 发车后首个速度样本(0~250ms随机相位)未到时 g_speed_output 仍为0：
         * 若不垫底，电机命令=纯position_correction，会被死区前馈放大成原地扭
         * (12:55 Run1 实测 sent=836,-956 右轮倒转甩头)。与有样本分支同语义垫底。 */
        if (i_speed > 0.0f && speed_output < min_out) {
            speed_output = min_out;
        }
    }
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
		/* 阶梯②(06-06): sm 区深弯滞回提前 1.9/1.5→1.7/1.2(06-05 既定阶梯第②级)——
		 * 交替 S 拐反应更早,减少过渡处线滑出视场;sm 限定,U 弯/常规弯滞回不变。 */
		{
		float deep_enter = g_s_mode ? 1.7f : 1.9f;
		float deep_exit  = g_s_mode ? 1.2f : 1.5f;
		if (g_nav_override != NAV_OVERRIDE_NONE) {
			g_deep_turn_mode = 0;   /* P2 覆盖:航向保持用温和差速,禁深弯 pivot(raw_err 是噪声) */
		} else if (g_s_mode && g_reacq_grace > 0u) {
			g_deep_turn_mode = 0;   /* A2c 宽限:禁深弯 pivot,缓差速滚上线 */
		} else if (!result_BlackPoint.is_junction) {
			if (g_raw_abs_err >= deep_enter) {
				g_deep_turn_mode = 1;   /* 进入深弯:内侧轮停转 */
			} else if (g_raw_abs_err <= deep_exit) {
				g_deep_turn_mode = 0;   /* 退出深弯:内侧轮恢复前进 */
			}
		}
		}
		/* decel_cap: 深弯模式内侧减速，否则正常前进(50)。
		 * 12:00曾回退0(最大差速)，但"20不够过弯"的旧结论被滤波×0.2阈值bug污染
		 * (12:24已修，阈值0.5)。用户确认深弯内轮不许完全停转 → 回到20重新地面验证。
		 * 内轮托底由下方硬下限钳位完成(decel_cap仍=speed_output)，差速322→282(-12%)。 */
		#define MIN_INNER_WHEEL_SPEED 20.0f
		/* A1(06-06 用户拍板): sm 区深弯内轮允许干净停转——r15 几何唯一解:
		 * 爬行档(20→sent600≈实测17cps)配外轮33cps → R≈(W/2)(vo+vi)/(vo−vi)≈26cm>15,
		 * 必丢线(17:06/17:07 双实测,降速到 T=14 仍丢);停转 → 绕内轮 R≈6.5cm,裕度≥33%。
		 * U1(06-06 23:2X 双实测): D4 后爬行档=20+680=700 PWM≈实测26cps,配外轮40cps
		 * → R≈37cm,U 弯(r≈20)几何不可达——两连跑均 U 弯入口(el≈158)外甩:
		 * Run2 锁差速盘旋 197° 不复线 375 自停,Run1 楔住后弹射出图。
		 * 死区仿射映射下内轮仅两档(任意 cmd>0 ≈≥24cps / cmd=0 coast),中间档不存在,
		 * 停转授权从 sm 扩展到全部深弯(用户判"差速不够";06-05"内轮不许停转"裁定就此让位)。
		 * 安全:CLOSED_LOOP_REVERSE_ENABLE=1 下负值会经 ApplyDeadzone 放大成反向脉冲
		 * (12:55 Run1 同族)——min_inner=0 同时把 wheel_balance 负摄动钳到 0,不得绕过本钳。 */
		float min_inner = (g_s_mode || g_deep_turn_mode) ? 0.0f : MIN_INNER_WHEEL_SPEED;
		float shallow_cap = g_s_mode ? S_MODE_SHALLOW_CAP : 50.0f;
		float decel_cap = g_deep_turn_mode ? speed_output : shallow_cap;
		if (decel_cap < shallow_cap) decel_cap = shallow_cap;   /* 浅弯下限：S-mode 30/正常50，随floor对偶保持失速裕度20不变 */

		float inner_decel, outer_accel;

		if (position_correction >= 0.0f) {
			/* correction>0：右轮内侧(减速)，左轮外侧(加速)，车头右转 */
			inner_decel = (position_correction > decel_cap) ? decel_cap : position_correction;
			outer_accel = position_correction;  /* 外侧全额加速 */
			left_output  = speed_output + outer_accel + wheel_balance;
			right_output = speed_output - inner_decel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限(A1/U1: 深弯内轮=0 干净停转,全弯型) */
			if (g_deep_turn_mode && right_output < min_inner) {
				right_output = min_inner;
			}
		} else {
			/* correction<0：左轮内侧(减速)，右轮外侧(加速)，车头左转 */
			inner_decel = (-position_correction > decel_cap) ? decel_cap : (-position_correction);
			outer_accel = -position_correction;
			left_output  = speed_output - inner_decel + wheel_balance;
			right_output = speed_output + outer_accel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限(A1/U1: 深弯内轮=0 干净停转,全弯型) */
			if (g_deep_turn_mode && left_output < min_inner) {
				left_output = min_inner;
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

void PID_SetNavOverride(uint8_t mode, float yaw_target_rad, float speed_cps)
{
	g_nav_yaw_target = yaw_target_rad;
	g_nav_speed_cps = speed_cps;
	g_nav_override = mode;   /* 最后写 mode,参数先就位(主循环单线程,纯防御习惯) */
}

uint8_t PID_GetNavOverride(void)
{
	return g_nav_override;
}

uint8_t PID_GetDeepTurnMode(void)
{
    return g_deep_turn_mode;
}
