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
/* G1(06-07 晚 12-agent 议会,用户批准备码): gyro 率阻尼内环一键包——上行宏置 1 即同时
 * 获得 kd=-80(下方 PID_GYRO_KD_INIT 跟随)+内环限幅 150(gyro_term 处跟随);置 0 全部
 * 回 F24 死字段语义(字节级等价)。符号链已复核:gz 左转为正(10:21 U 左转 yw=+176 字段
 * 实证)+corr>0=右转指令+output-=gyro_kd*gz ⇒ kd 取负=阻尼。
 * ⚠烧车前台架定号仍强制(防机械装反/轴向意外):架空+传感器下垫黑线白纸,K1,绕传感器
 * 中点手转车头(质心保持≈30),左转时 pid 应右增左减(压制转动);反向→改 +80.0f。 */
#if PID_GYRO_ENABLE
#define PID_GYRO_KD_INIT  (-80.0f)   /* 台架定号若反向→ +80.0f */
#else
#define PID_GYRO_KD_INIT  (0.0f)     /* 门关=死字段回零(F23 防误导语义) */
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
	/* F20(06-07 10:02): α 0.4→0.2(τ 3.9→9ms)——Kd 550→700 实测乒乓无改善(用户"仍然
	 * 很抖动"),根因=量化 D 是跳格脉冲(α0.4 下 140PWM 仅持续 ~8ms,电机机电时间常数
	 * ~30ms 接不住);摊宽脉冲电机才吃得到,等效阻尼 ↑30~50%。这是量化 D 通道最后一档,
	 * 再不够转 gyro 内环(gyro_kd,下方 238 行现成钩子,需先台架定符号)。
	 * F23(06-07 11:0X): α 回 0.4——F20 后两轮(10:21/10:35)+F22 轮(10:59/11:00)乒乓
	 * 零改善,0.2 收益未兑现且 0.8 长记忆曾引爆 R5 出口残留踢(F22a 补丁因此而生);
	 * 回 03:30 史上最远轮的验证档。 */
	controller->d_filtered = 0.4f * d_raw + 0.6f * controller->d_filtered;
	float d_term = controller->param.kd * controller->d_filtered;
	float gyro_term = 0.0f;
#if PID_GYRO_ENABLE
	gyro_term = controller->param.gyro_kd * MPU6050_data.gz_rads;
	/* G1(06-07 晚): 限幅 output_max×0.4(=3600)→固定 150——corr 下游钳位 ±320(PC:780),
	 * 3600 等于让 gyro 独占全部 corr 预算(U pivot 会被反扭顶宽);150 留一半给 P/D。
	 * 乒乓摆速 2~4.5rad/s→|kd×gz|=160~360 触限幅,典型工作点即满阻尼。 */
	float gyro_limit = 150.0f;
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

    // 位置环：Kp=32, Kd=700。
    // Kd 550→700(06-07 09:50 F19,用户拍板"D加大一点"): F18 后自旋已绝/375干净自停,
    //   但直线沿线震荡仍在(pos 间歇满幅 0↔60,5.5m 后甩出)。方向同 06-05 400→550 胜出轮;
    //   量化体系下 D=跳格冲量(单格 ≈Kp×0.5×gain+Kd×0.5×0.4≈140→700 后 ≈177),阻尼均值
    //   仍偏小——若 700 出现直线高频抖振(贴线碎抖)或仍满幅乒乓,下一单变量=D 滤波
    //   α 0.4→0.2(摊平冲量成真阻尼,agent 审计 09:16 条目),不再加 Kd。
    // Kd 400→550 回退(06-05 第8轮): 400实测更差——温和段0.9~1.5s缩到0.3~0.6s，发车后
    // 300~600ms即触发deep。新几何下D净阻尼为正，"量化D踢是放大器"假设证伪，550重验证胜出。
    // Kp 48→40(06-05): 新几何±0.5粗台阶单步误差更大=等效每步增益更高，48已越下移后的
    // 新上限（温和摆动自发逐摆增长=增益过高型发散）。旧几何上限48（50即临界）。
    // Kp 40→32(06-07 F17a): 风机120常开定版=下压力↑抓地↑,同等差速PWM出更大yaw率=
    //   被控对象增益再次上移;08:51 带扇首跑直线复现"逐摆增长型发散"全指纹(pos 15→0/55
    //   全幅乒乓且幅值随 out 爬升递增→丢线冻结自旋90°→楔死),与 06-05 48→40 同病同药,
    //   降20%对冲;Kd 550 不动(Kp 降本身即增大阻尼比,且加Kd会放大±0.5台阶量化踢)。
    /* F21(06-07 10:21 纯电池构型首轮): 启用 gyro 角速度内环 gyro_kd=-80——量化 D 通道
     * 三连档(Kd550→700/α0.4→0.2)对乒乓收效有限,gz_rads 是唯一连续(500Hz 无量化)阻尼源。
     * 符号推导(免台架定号,全链实测锁死): U 左转实测 yw=+176(本轮) + add_angle=∫gz_rads
     * ×(+1.0)(it.c:107-111) ⇒ gz 左转为正;实测 corr>0=右转(pos=0 需左转时恒 pid_R>pid_L);
     * 阻尼=左转动给右回正 ⇒ output 须随 gz 增 ⇒ 代码 output-=gyro_kd*gz ⇒ kd 取负。
     * 量级: 摆动 1.2rad/s→96PWM 连续阻尼;U pivot 1.6rad/s→128PWM 反扭(corr 饱和 320+coast
     * 裕度内,U 半径须复验,变宽先回 -50);gyro 零偏 ~0.03rad/s→2.4PWM 可忽略。
     * 限幅 ±output_max×0.4(PC:242)既有;junction 冻结/深丢线(>125tick)期 corr 冻结,内环
     * 同步失效(既有边界,F18a 已兜自旋)。 */
    /* F23(06-07 11:0X 基线回滚,用户指令"查全史日志,用历史最好用的参数做基础"):
     * Kp 32→40 / Kd 700→550 / α 0.2→0.4(PC:238) / slew 关断(PC:438)——一次性撤销今晨
     * 四连档,回到 03:30 史上最远轮(F5,拱门2.1)+19:49 蛇形①全通+06-05 第9轮直线判定
     * 通过的同一控制核(Kp40/Kd550/α0.4/无gyro/无slew,全史成功轮全部出自它)。
     * 依据:F17(Kp32) 当轮 corr 饱和仅 −1.5% 已证伪未回退;F19(Kd700) 当轮证伪未回退;
     * F20(α)/F22c(slew) 后续四轮乒乓零改善——叠加偏离验证基线,无一兑现收益。
     * ⚠F21 勘误(本轮发现):PID_GYRO_ENABLE 全仓库无人定义为 1=恒 0(PC:11),gyro 项
     * 从未编译——-80 写进死字段,10:21 过段1与其后全部失败轮均无 gyro 参与,
     * "gyro 无辜"裁决空洞成立(空操作当然无辜)。字段回 0 防误导;gyro 内环=未测试
     * 后手,启用须 #define PID_GYRO_ENABLE 1 + 单变量轮专测。
     * 保留(非调参项):F22a/b bug修复、F18a 去抖、F16ab、SEG、风机 G 链;
     * HOLD 850/940 不随回(纯电池唯一过段验证档;770/800 的零卡滞实测全在 USB 共电
     * 披露窗内,且有 F9"太慢一直卡住"史——亏电下照搬=赌卡滞)。 */
    /* G1(06-07 晚): gyro_kd 改由 PID_GYRO_KD_INIT 跟随编译门(PC:头部)——门=0 时仍为
     * 0.0f(F23/F24 字节级等价),门=1 时 -80;不再出现"死字段写实值"的 F21 式误导。 */
    /* R4备码(06-07 晚议会): 中线偏置宏——直线稳态质心压 S2/S3(偏左1~1.5格,右弱108PWM+
     * Ki=0 常驻P误差),吃左弯裕量。默认 0.0f=字节等价;R4 单变量轮置 -0.15f 左右试探,
     * 验收=直线 pos 稳态回 28~30。偏置是改循迹目标非写死方向(红线b合规但敏感,故默认关)。 */
#ifndef PID_CENTER_TARGET_BIAS
#define PID_CENTER_TARGET_BIAS 0.0f
#endif
    PositionPID_Init(&g_position_pid, 40.0f, 0.0f, 550.0f, PID_GYRO_KD_INIT, 9000.0f, -9000.0f, (float)(SENSOR_COUNT - 1u) / 2.0f + PID_CENTER_TARGET_BIAS);
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
static int8_t g_nav_turn_dir = 0;   /* F47: NAV_OVERRIDE_TURN 转向符号(+1左/-1右);仅 SEG3 写,比赛构型恒 0 */
static int8_t g_seg3_bias = 0;      /* F48: SEG3 分叉偏置(-1左/+1右/0关);仅 SEG3 写,比赛构型恒 0 */
#ifndef SEG3_BRANCH_BIAS_CORR
#define SEG3_BRANCH_BIAS_CORR 130.0f /* F48: 分叉偏置 corr 幅度(介于 HEADING±50 太软 与 TURN±320 甩离线 之间) */
#endif
/* 深弯模式滞回状态：1=内侧轮停转模式。误差≥进入阈值置1，≤退出阈值清0，
 * 掐断弯道边缘质心量化噪声(4.19↔4.54)导致的内侧轮768↔0颤振。 */
static uint8_t g_deep_turn_mode = 0;
/* F16(06-07 07:30 实测): 非 sm deep 内轮 coast 资格门——deep 连续保持 ≥ 本阈值才允许
 * min_inner=0,否则仍走 F12 爬行档 20。两难调和:F12 动机(04:54 悬空扫边停轮,瞬扫是
 * 单帧~百 ms 级) vs U 弯几何(07:03 F13/07:30 双实测:死区仿射两档律下 sent=750 内轮
 * 仍 23~30cps,327~447 PWM 指令差速实测 R-L≈7cps,U 仅转 60° 即翻边——06-06 23:2X U1
 * 同律,降底座追不到停转点)。持续 100ms 的 deep 只在真弯出现(07:30 U 入口 deep 连续
 * ≥900 tick),质心量化抖动(deep 隔帧 0↔1)永远到不了。
 * 注意:悬空把传感器按住边路 >100ms 内轮仍会停——这是 U pivot 的必要几何行为,非故障。 */
#ifndef DEEP_COAST_CONFIRM_TICKS
#define DEEP_COAST_CONFIRM_TICKS 100u /* F51(06-08 09:11 U单测): 150(300ms)→100(200ms)。
                                       * F50 回退死区后直线已收口,但 U 入口仍 "拐弯慢/全白后才转":
                                       * 09:11 Run3 lost43/deep1 仍 sent=790,1191(内轮爬行),下一帧才
                                       * coast。回 100 取 F16/F25 几何与 F24c 护直线的折中;若直线
                                       * 单轮 sent=0 乒乓回潮即回 150。 */
/* F24c(06-07晚 12-agent议会): 50(100ms)→150(300ms)。
                                       * 18:23-25 三组实测:直线乒乓每个满幅换边帧都伴随单轮
                                       * sent=0(coast)无一例外=主功放;摆动半周 deep 连续
                                       * 数百 ms 也能拿到 100ms coast 资格。300ms 确认显著
                                       * 缩短直线段 coast 占空(瞬态 deep 只走爬行档 20);
                                       * 真 U 弯 deep 连续≥900tick(07:30 实测)不受影响。
                                       * U 入弯若变宽(内轮先爬行致半径偏大)回 50u 或折中 100u。 */
#endif
static uint16_t g_deep_hold_ticks = 0;  /* deep 连续保持计数(饱和于阈值),非 deep 即清零 */
/* F25a(06-07 19:2X 三组实测): 盲 coast 旁路——F24c 的 300ms 确认护住直线(三组直线零
 * coast)但 U 入弯付出 ~600ms 爬行档代价(19:22:44 sent=790,1208 连续两帧),弧变宽线更早
 * 出视场。鉴别量=lost:直线乒乓瞬态 deep="线在视场内的量化抖动"(lost=0);真弯深陷=
 * "线甩出视场"(lost 爬升)。deep 且盲>100ms → 立即授 coast;线可见仍走 150 确认。 */
#ifndef DEEP_BLIND_COAST_LOST_TICKS
#define DEEP_BLIND_COAST_LOST_TICKS 35u   /* F51: 50(100ms)→35(70ms),卡 09:11 lost42/43 首个全白 U 帧 */
#endif
static uint8_t g_blind_reacq_pending = 0; /* F26a: 深陷盲走episode标记——重捕首帧 D 软启动 */
#ifndef CORR_SLEW_PER_TICK
#define CORR_SLEW_PER_TICK 999.0f /* F23: 999=关断(corr∈±320,单tick最大Δ640<999 永不钳)。
                                   * F22c 原值 15.0f:10:59/11:00 两轮实测满幅乒乓如旧(0↔60 对穿,
                                   * S 全白线下穿越),止血未兑现且引入 ~36ms 反向迟滞(1.7Hz 摆频下
                                   * ≈22°相位滞后,继电器系统里是负资产);基线回滚一并撤销。
                                   * 复用=改回 15.0f(代码与清零位全保留)。 */
#endif
static float g_corr_slew_prev = 0.0f;  /* F22c: 斜率限制记忆(F11 纪律:停车/路口冻结同清) */
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
			g_deep_hold_ticks = 0;  /* F16 coast 资格计数同清(F11 教训:新增 static 必入本清单) */
			g_blind_reacq_pending = 0u;  /* F26a episode 标记同清(F11 纪律:防跨运行残留误清 d_filtered) */
			g_corr_slew_prev = 0.0f; /* F22c 斜率记忆同清 */
			g_reacq_run = 0;        /* A2 去抖状态同清 */
			g_last_edge_side = 0;   /* A2b 边缘记忆同清 */
			g_reacq_grace = 0;      /* A2c 宽限同清 */
			g_a2_flips = 0;         /* A2b-limit 同清 */
			g_a2_episode_cool = 0;  /* H2' 跨段窗口同清 */
			g_line_lost_ticks = 0;  /* F11(06-07 红队): 原清单漏此项——停车后冻结残留(实测 lost=266
			                         * 钉屏≥6s),污染遥测判读;且发车即丢线时陈值直撞 375 自停门。
			                         * 与主环 lose_time(StopRun 清)对齐,两计数器不再分裂。 */
			g_nav_override = NAV_OVERRIDE_NONE;  /* P2 导航覆盖同清(K2 停车即退覆盖) */
			g_nav_turn_dir = 0;                 /* F47: TURN 符号同清 */
			g_seg3_bias = 0;                    /* F48: 分叉偏置同清 */
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
		/* F18a(06-07 09:16): 去抖扩展到全域(原 g_s_mode 限定)——非 sm 假重捕翻边
		 * 两次实锤(07:30 U弯 pos=60 单瞥拆 72° / 09:16 [10.170] S 七路全白却报
		 * pos=60→pid 377 满舵→单帧点燃自旋 yw -44→-128)。门槛仍要求深丢线
		 * (>125tick)才启用确认,正常巡线单帧质心跳变不受影响;sm 行为不变;
		 * 真出线→lost 单调爬 375 干净自停,不再被全白伪帧反复清零横跳。 */
		if (g_line_lost_ticks > 125u) {
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
	    /* F25b(06-07 19:27 G3 实锤): 非 sm 盲走承诺转向——U 入口线从 pos=15 滑出视场
	     * (未及边缘),冻结质心 raw_err=1.5<1.9 deep 永不点火,corr 冻在温和左修 ≈-90,
	     * 车近直线盲走到黑区/375。旧"冻结质心全增益PID"只有恰冻在边缘才像样(G1/G2)。
	     * 改:深丢线(>250ms,与去抖/A2 同门槛)把冻结质心收敛到其所在侧边缘(±3 格)→
	     * deep 判据必点火 + corr 满幅(320)朝最后所见侧找线;配 F25a 盲 coast 成净 pivot。
	     * 方向来自小车自己的质心历史(红线b合规);junction 冻结/sm A2 锁向/NAV 覆盖各有
	     * 自己的分支不进此路;冻在正中(raw_err≈0,全黑改判类)不承诺保持直行;375 自停
	     * 兜底不变;重见线即走 F18a 去抖解除,回正常位置环。 */
	    if (!result_BlackPoint.found && g_line_lost_ticks > 125u &&
	        result_BlackPoint.black_count == 0u &&
	        !result_BlackPoint.is_junction && !g_s_mode &&
	        g_nav_override == NAV_OVERRIDE_NONE) {
		/* F26b(红队4b): 加 black_count==0 守卫——只对"全白盲"(线甩出视场=真弯)承诺;
		 * "全黑盲"(C4-blind,方块阵/全黑区/压宽黑)保持旧直穿语义,不许 pivot 拐进黑区。 */
		float blind_err = current_position - g_position_pid.param.target_position;
		if (blind_err > 0.05f)       current_position = g_position_pid.param.target_position + 3.0f;
		else if (blind_err < -0.05f) current_position = g_position_pid.param.target_position - 3.0f;
	    }
	    /* F26a: 标记深陷盲走episode——重捕首帧做 R5 同款 D 软启动。19:44:22 实锤:
	     * pid=252,33(pos=20 应左修却打右满舵)=盲走期 d_filtered 残值反向踢,刚捞到的
	     * 线被当帧甩掉 → U 弯"震荡卡顿"(盲转-捞线-踢丢-再盲转循环)的根因。 */
	    if (!result_BlackPoint.found && g_deep_turn_mode && !g_s_mode &&
	        g_line_lost_ticks > DEEP_BLIND_COAST_LOST_TICKS) {
		g_blind_reacq_pending = 1u;   /* F28a: 加 !g_s_mode——S 配方(19:49 验证)域内
		                               * 重捕行为保持字节级原样,A2c 宽限自管;本机制只
		                               * 服务非 sm 的 U/普通弯盲走 episode。 */
	    }
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
    if (g_nav_override == NAV_OVERRIDE_TURN) {
        /* F47 写死转弯:满幅 corr 按符号(+dir=左转⇒corr<0⇒左)。必须排在 is_junction 之前——
         * 全黑横杠会触发 is_junction 冻结(corr=0)挡住转弯;放最前确保锐弧穿过全黑杠。
         * 内轮 coast 锐弧由下方差速段 turn_arc 实现;到角/Δyaw 预算完成由上层判定。 */
        position_correction = (g_nav_turn_dir >= 0) ? -320.0f : 320.0f;
        s_prev_junction = 1;   /* 借同通道,解除首帧走 R5 软启动 */
    } else if (g_nav_override == NAV_OVERRIDE_HEADING) {
        /* P2 盲走航向保持:旁路寻线/路口/锁向,corr=Kyaw×(当前-目标)。
         * 符号:corr>0=右转(下方差速注释),目标在左(target>yaw)时 corr<0 → 左转 ✓。
         * 限幅 ±50 见 NAV_CORR_CAP 注释;借 s_prev_junction 通道,覆盖解除首帧
         * 走 R5 软启动(对齐 last_error,消冻结期 D 踢),与路口退出同语义。 */
        position_correction = (add_angle - g_nav_yaw_target) * NAV_YAW_KP;
        if (position_correction > NAV_CORR_CAP) position_correction = NAV_CORR_CAP;
        else if (position_correction < -NAV_CORR_CAP) position_correction = -NAV_CORR_CAP;
        s_prev_junction = 1;
    } else if (g_seg3_bias != 0) {
        /* F48 SEG3 分叉偏置(巡线域内,override==NONE):注入固定 corr 把车拐上支线,必须压过
         * 下方 is_junction 冻结(否则横杠处 corr=0 走直,左路口会错过左支)。-1左⇒corr<0,+1右⇒corr>0。
         * 中等幅(±130)两轮都驱动(deep 下方按 g_seg3_bias 强制 0,不内轮 coast),车不甩离线;
         * 释放(上层重捕干净单线/Δyaw 预算)。借 s_prev_junction 通道,解除首帧走 R5 软启动。 */
        position_correction = (g_seg3_bias < 0) ? -SEG3_BRANCH_BIAS_CORR : SEG3_BRANCH_BIAS_CORR;
        s_prev_junction = 1;
    } else if (result_BlackPoint.is_junction) {
        /* 路口/岔路：旁路位置环，强制走直(correction=0)。
         * 关键:不调用 PositionPID_Calculate → 环内 last_error/d_filtered 冻结在进路口前的值，
         * 且不更新 g_last_valid_correction。注意：冻结期质心若移动，退出帧 d_raw=新误差-冻结
         * 旧误差 ≠0（旧注释"d_raw≈0"过度承诺，06-05 审查 I7）——由下方 R5 软启动消除。
         * 深弯滞回在下方按 !is_junction 冻结(路口宽黑会把 raw_abs_err 顶到~2.5+误触发内侧停转)。 */
        position_correction = 0.0f;
        g_corr_slew_prev = 0.0f;   /* F22c: 冻结期 corr=0,记忆同步置 0,出口从 0 起坡 */
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
            /* F22a(10:35 [47.477] 实锤,F20 引入的回归): R5 只对齐 last_error 消 d_raw,
             * 漏清 d_filtered——α=0.2 后(0.8 保持率)冻结期残留 d_filtered 跨冻结存活,
             * 解冻首帧 Kd700×残值打出反向大踢(+271 级,盖过 pos=5 应有左修→pid=263,0
             * 右转异常帧,U 出口被推向支线)。last_error/d_filtered 必须成对复位。 */
            g_position_pid.d_filtered = 0.0f;
            s_prev_junction = 0;
            if (s_prev_a2hold) {
                g_reacq_grace = 50u;        /* A2c: 锁向找回后给 100ms 滚上线宽限 */
                g_a2_episode_cool = 250u;   /* H2': 开 500ms 跨段窗口(乒乓重锁不重置预算) */
            }
            s_prev_a2hold = 0;
        }
        /* F26a: 盲走episode重捕首帧 D 软启动(R5 同款语义):对齐 last_error+清 d_filtered,
         * 消盲走期残值的反向 D 踢;P 项保留(重捕质心的真实修正不动)。 */
        if (g_blind_reacq_pending && result_BlackPoint.found) {
            g_position_pid.last_error = current_position - g_position_pid.param.target_position;
            g_position_pid.d_filtered = 0.0f;
            g_blind_reacq_pending = 0u;
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
			if (g_nav_override == NAV_OVERRIDE_HEADING || g_nav_override == NAV_OVERRIDE_TURN) {
				i_speed = g_nav_speed_cps;   /* F47: TURN 同走 NAV 爬行速 */
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
		/* F22c(策略评审裁决,10:35 三 agent 合议): corr 斜率限制 ±15/tick——
		 * "死区两档律继电器+0.5格量化+coast bang-bang"构成固有极限环(描述函数有解),
		 * 调 Kp/Kd/gyro 数学上只能改幅度不能消环;削"瞬时满舵冲量"是当天可落地止血:
		 * 满幅 0→320 改 ~42ms 斜坡(U 入弯 1200ms 量级无感,量化跳格 ±76 级冲量被摊平)。
		 * 记忆在停车/路口冻结同清(出口从 0 起坡,与 F22a 协同);sm 域 A2 锁向同被限速,
		 * S 复验时关注。根治路线=质心 ADC 内插(BlackPoint 二值化丢幅度),另议。 */
		{
			float dc = position_correction - g_corr_slew_prev;
			if (dc > CORR_SLEW_PER_TICK)       position_correction = g_corr_slew_prev + CORR_SLEW_PER_TICK;
			else if (dc < -CORR_SLEW_PER_TICK) position_correction = g_corr_slew_prev - CORR_SLEW_PER_TICK;
			g_corr_slew_prev = position_correction;
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
		/* F39b(06-08 01:42-46 三组,F38 后 sm 实际车速 20~46cps): 高速 understeer 模式
		 * (01:44 G2: corr 满幅 322 仍掰不弯,pos 冻 20 切线滑出 S① 1/4)——误差增长率∝v,
		 * 1.7 咬合点在 40cps 下已晚半个车位。sm enter 1.7→1.5(pivot 提前),exit 1.2 不动
		 * (滞回带 0.5→0.3,仍 > 量化抖幅 0.35 的工程余量边缘,留观颤振)。
		 * 这是"占空比连升三级后 PID 链的配套刻度",非 S 配方回退。 */
		{
		float deep_enter = g_s_mode ? 1.5f : 1.9f;
		float deep_exit  = g_s_mode ? 1.2f : 1.5f;
		if (g_nav_override != NAV_OVERRIDE_NONE || g_seg3_bias != 0) {
			g_deep_turn_mode = 0;   /* P2 覆盖/F48 分叉偏置:禁深弯 pivot,中等差速两轮都驱动不甩离线 */
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
		/* F16: deep 持续计数。路口冻结期 deep 保持也照计——冻结期 corr=0 双轮同速,
		 * coast 资格不会被消费;出冻结若仍真弯则无缝接 pivot。丢线冻结期 raw_err 冻在
		 * 高位 → deep 保持 → 计数不断,07:30 式冻结 pivot 段照常享有 coast。 */
		if (g_deep_turn_mode) {
			if (g_deep_hold_ticks < DEEP_COAST_CONFIRM_TICKS) g_deep_hold_ticks++;
		} else {
			g_deep_hold_ticks = 0;
		}
		/* decel_cap: 深弯模式内侧减速，否则正常前进(50)。
		 * 06-07 F12: 悬空扫到最左/最右边路时,非 sm deep 直接 sent=0 再次被用户判定不可接受;
		 * 仅 S-mode 保留内轮 coast 几何解,非 sm deep 恢复 20 爬行下限。 */
		#define MIN_INNER_WHEEL_SPEED 20.0f
		/* A1(06-06 用户拍板): sm 区深弯内轮允许干净停转——r15 几何唯一解:
		 * 爬行档(20→sent600≈实测17cps)配外轮33cps → R≈(W/2)(vo+vi)/(vo−vi)≈26cm>15,
		 * 必丢线(17:06/17:07 双实测,降速到 T=14 仍丢);停转 → 绕内轮 R≈6.5cm,裕度≥33%。
		 * U1(06-06 23:2X 双实测): D4 后爬行档=20+680=700 PWM≈实测26cps,配外轮40cps
		 * → R≈37cm,U 弯(r≈20)几何不可达——两连跑均 U 弯入口(el≈158)外甩:
		 * Run2 锁差速盘旋 197° 不复线 375 自停,Run1 楔住后弹射出图。
		 * 死区仿射映射下内轮仅两档(任意 cmd>0 ≈≥24cps / cmd=0 coast),中间档不存在。
		 * F12 将 coast 授权收回到 sm 域:U/普通深弯不再直接停内轮,防悬空边路测试和浅弯边缘
		 * 被 deep 触发后单轮 0;若 U 弯半径变宽,按 D5 只退 U_DEEP 或复核 F12,不动 PID。
		 * 安全:CLOSED_LOOP_REVERSE_ENABLE=1 下负值会经 ApplyDeadzone 放大成反向脉冲
		 * (12:55 Run1 同族)——min_inner 仅做非负托底,不得绕过本钳。
		 * F16(06-07 07:30): 执行上注预案"复核 F12"——U 半径变宽实锤(F13 底座 730 下仍
		 * 60° 翻边,R-L≈7cps)。持续 deep ≥DEEP_COAST_CONFIRM_TICKS 恢复 coast 资格,
		 * 瞬态(悬空扫边/量化抖动)仍走爬行档 20;coast 期 cmd=0 → ApplyDeadzone 干净停,
		 * 不产生负值,反向钳语义不变。 */
		uint8_t turn_arc = (g_nav_override == NAV_OVERRIDE_TURN);   /* F47: 写死锐弧=内轮 coast(R≈6.5cm) */
		float min_inner = (turn_arc || g_s_mode || g_deep_hold_ticks >= DEEP_COAST_CONFIRM_TICKS ||
		                   (g_deep_turn_mode && g_line_lost_ticks > DEEP_BLIND_COAST_LOST_TICKS))
		                  ? 0.0f : MIN_INNER_WHEEL_SPEED;   /* F25a: 深陷盲走即授 coast;F47: TURN 同授 coast */
		float shallow_cap = g_s_mode ? S_MODE_SHALLOW_CAP : 50.0f;
		float decel_cap = (g_deep_turn_mode || turn_arc) ? speed_output : shallow_cap;   /* F47: TURN 满额减速=内轮可到0 */
		if (decel_cap < shallow_cap) decel_cap = shallow_cap;   /* 浅弯下限：S-mode 30/正常50，随floor对偶保持失速裕度20不变 */

		float inner_decel, outer_accel;

		if (position_correction >= 0.0f) {
			/* correction>0：右轮内侧(减速)，左轮外侧(加速)，车头右转 */
			inner_decel = (position_correction > decel_cap) ? decel_cap : position_correction;
			outer_accel = position_correction;  /* 外侧全额加速 */
			left_output  = speed_output + outer_accel + wheel_balance;
			right_output = speed_output - inner_decel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限(A1/U1: 深弯内轮=0 干净停转,全弯型)
			 * F22b(10:35 [48.978] pid=0 sent=870 实锤): 钳口加 0.5 凑整带——coast 资格
			 * (min_inner=0)下 (0,0.5) 浮点残差越过 EPS=0.1 触发死区前馈+boost(770+100),
			 * 破坏干净 coast;亚整数残差一并落底。 */
			if ((g_deep_turn_mode || turn_arc) && right_output < min_inner + 0.5f) {
				right_output = min_inner;   /* F47: TURN 同享内轮干净 coast 钳 */
			}
		} else {
			/* correction<0：左轮内侧(减速)，右轮外侧(加速)，车头左转 */
			inner_decel = (-position_correction > decel_cap) ? decel_cap : (-position_correction);
			outer_accel = -position_correction;
			left_output  = speed_output - inner_decel + wheel_balance;
			right_output = speed_output + outer_accel - wheel_balance;
			/* 深弯双保险：内侧轮硬下限(F22b 同右轮:钳口+0.5 凑整带防亚整数残差) */
			if ((g_deep_turn_mode || turn_arc) && left_output < min_inner + 0.5f) {
				left_output = min_inner;   /* F47: TURN 同享内轮干净 coast 钳 */
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

void PID_SetTurnDir(int8_t dir)   /* F47: TURN 模式转向符号锁存 */
{
	g_nav_turn_dir = dir;
}

void PID_SetBranchBias(int8_t dir)  /* F48: SEG3 分叉偏置符号锁存(-1左/+1右/0关) */
{
	g_seg3_bias = dir;
}

uint8_t PID_GetNavOverride(void)
{
	return g_nav_override;
}

uint8_t PID_GetDeepTurnMode(void)
{
    return g_deep_turn_mode;
}
