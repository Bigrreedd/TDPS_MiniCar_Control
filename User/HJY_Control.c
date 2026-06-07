/**
 * HJY_Control.c —— HJY 三环重构核(2026-06-07)
 *
 * 架构(方案原文见 PID_TUNING_LOG.md 同日条目):
 *   光路穷举分区 → (基速, 差速等级) ┐
 *   角度环(直行态锁航向, 1 套 PID) ┼→ 左/右轮目标速度 → 左右独立速度 PID → PWM 直驱
 *   per-state PWM 前馈表(实测标定) ┘
 *
 * 显式不用:位置环 PID、START/HOLD 静态死区前馈、coast pivot、踢腿 boost。
 * 左右电机机械不对称 → 由两套独立速度环的积分项动态补偿(HJY 方案核心主张)。
 */
#include "HJY_Control.h"

#if HJY_CORE_ENABLE

#include "PID_Controller.h"   /* SpeedPID_* 结构与算法复用 + Motor_SetSpeedWithDirection */
#include "ABEncoder.h"        /* left/right_encoder_cnt */
#include "LineSensor.h"       /* g_line_sensor_values[SENSOR_COUNT] */
#include "Motor_ctr.h"        /* MOTOR_L/R, Motor_Enable/Disable/StopAll */
#include "stm32f10x_it.h"     /* add_angle(rad), Millis_Get() */

extern volatile uint8_t is_racing;

/* ===== 遥测全局 ===== */
volatile uint8_t  g_hjy_state = (uint8_t)HJY_ST_STRAIGHT;
volatile int16_t  g_hjy_vt_l = 0, g_hjy_vt_r = 0;
volatile int16_t  g_hjy_v_l = 0,  g_hjy_v_r = 0;
volatile int16_t  g_hjy_pwm_l = 0, g_hjy_pwm_r = 0;
volatile int16_t  g_hjy_ang_corr = 0;
volatile int16_t  g_hjy_yaw_err_d10 = 0;
volatile int16_t  g_hjy_err_l = 0, g_hjy_err_r = 0;

/* ===== 三套 PID 实例 ===== */
static SpeedPID_Controller_t s_spd_l;   /* 左轮速度环(独立 Kp/Ki/Kd) */
static SpeedPID_Controller_t s_spd_r;   /* 右轮速度环(独立 Kp/Ki/Kd) */

/* 角度环:位置式 PID,误差单位 rad,输出差速 cnt/s。自带最小实现保证
 * 符号语义透明:err = add_angle - target,err>0(车头偏左) → 输出>0 → 右修。 */
typedef struct
{
    float kp, ki, kd;
    float integral;
    float last_err;
    float out_max;          /* 对称钳位 ±out_max */
} HJY_AnglePID_t;
static HJY_AnglePID_t s_ang;
static float   s_yaw_target = 0.0f;     /* 直行态锁存的目标航向(rad) */
static uint8_t s_yaw_latched = 0u;      /* 1=target 有效 */

/* ===== 状态→目标查表 ===== */
/* 差速等级(cnt/s):负=左转(vL<vR),正=右转。直行/全黑 0,微修全交角度环。
 * 初值=合理大区间中点,串口实测收敛。 */
static const float k_diff[HJY_ST_COUNT] =
{
    0.0f,    /* STRAIGHT */
    -5.0f,   /* L1 小左 */
    -10.0f,  /* L2 中左 */
    -16.0f,  /* L3 大左 */
    +5.0f,   /* R1 小右 */
    +10.0f,  /* R2 中右 */
    +16.0f,  /* R3 大右 */
    0.0f,    /* ALLBLACK 直穿 */
    0.0f     /* ALLWHITE(不用,沿用上一状态) */
};
/* 基速(cnt/s) */
static const float k_base[HJY_ST_COUNT] =
{
    HJY_BASE_SPEED,        /* STRAIGHT */
    HJY_BASE_SPEED,        /* L1 */
    HJY_BASE_SPEED_MID,    /* L2 */
    HJY_BASE_SPEED_SLOW,   /* L3 */
    HJY_BASE_SPEED,        /* R1 */
    HJY_BASE_SPEED_MID,    /* R2 */
    HJY_BASE_SPEED_SLOW,   /* R3 */
    HJY_BASE_SPEED_SLOW,   /* ALLBLACK */
    HJY_BASE_SPEED_SLOW    /* ALLWHITE(不用) */
};
/* per-state PWM 前馈表(方案第三步:每种光路状态分别标定左右电机 PWM)。
 * 初值全 0=纯闭环;串口采集各状态稳态 PWM 后回填,PID 只吃残差。 */
static int16_t s_ff_l[HJY_ST_COUNT] = {0};
static int16_t s_ff_r[HJY_ST_COUNT] = {0};

/* ===== 轮速窗口(HJY 自建,与旧 SPEED_WIN_MS 250ms 解耦) ===== */
static int32_t  s_last_cnt_l = 0, s_last_cnt_r = 0;
static int32_t  s_acc_l = 0, s_acc_r = 0;
static uint32_t s_win_t0 = 0;
static uint8_t  s_win_inited = 0u;

/* ============================================================ */
/* 光路穷举分区:任意 7 位黑白组合 → 唯一状态。
 * 实现=黑灯加权质心 c10(0..60,索引×10) 分带;每带典型组合:
 *   STRAIGHT  c10∈[25,35]: {S4},{S3,S4},{S4,S5},{S3,S4,S5}
 *   L1        c10∈[15,25): {S3},{S2,S3,S4},{S3,S4}偏组合
 *   L2        c10∈[ 5,15): {S2},{S1,S2,S3},{S2,S3}
 *   L3        c10∈[ 0, 5): {S1},{S1,S2}
 *   R1/R2/R3  镜像 (35,45]/(45,55]/(55,60]
 *   全黑(≥6 灯黑,容 1 路反光漏检) → ALLBLACK
 *   全白(0 灯黑)                  → ALLWHITE                     */
static HJY_LineState_t HJY_Classify(const volatile uint16_t *s)
{
    uint8_t  i, cnt = 0u;
    uint16_t c10 = 0u;
    uint32_t sum = 0u;

    for (i = 0u; i < SENSOR_COUNT; i++)
    {
        if (s[i] < HJY_BLACK_THRESH)
        {
            cnt++;
            sum += (uint32_t)i * 10u;
        }
    }
    if (cnt == 0u)                    return HJY_ST_ALLWHITE;
    if (cnt >= (SENSOR_COUNT - 1u))   return HJY_ST_ALLBLACK;

    c10 = (uint16_t)(sum / cnt);
    if (c10 < 5u)   return HJY_ST_L3;
    if (c10 < 15u)  return HJY_ST_L2;
    if (c10 < 25u)  return HJY_ST_L1;
    if (c10 <= 35u) return HJY_ST_STRAIGHT;
    if (c10 <= 45u) return HJY_ST_R1;
    if (c10 <= 55u) return HJY_ST_R2;
    return HJY_ST_R3;
}

static void HJY_AnglePID_Reset(void)
{
    s_ang.integral = 0.0f;
    s_ang.last_err = 0.0f;
    s_yaw_latched  = 0u;
}

void HJY_Init(void)
{
    /* 左右速度环:独立实例,初值相同;标定阶段经串口数据各自收敛。
     * 输出域=PWM(带符号),上限留 200 余量给 SAFE_MAX,反转钳 -400 轻刹。 */
    SpeedPID_Init(&s_spd_l, HJY_SPD_KP, HJY_SPD_KI, HJY_SPD_KD, HJY_PWM_MAX, -HJY_PWM_REV_MAX);
    SpeedPID_Init(&s_spd_r, HJY_SPD_KP, HJY_SPD_KI, HJY_SPD_KD, HJY_PWM_MAX, -HJY_PWM_REV_MAX);

    s_ang.kp = HJY_ANG_KP;
    s_ang.ki = HJY_ANG_KI;
    s_ang.kd = HJY_ANG_KD;
    s_ang.out_max = HJY_ANG_CORR_MAX;
    HJY_AnglePID_Reset();

    s_win_inited = 0u;
}

void HJY_ResetRun(void)
{
    SpeedPID_Reset(&s_spd_l);
    SpeedPID_Reset(&s_spd_r);
    /* 发车前馈:把增量式速度环"上次输出"播种到死区邻域,等价一次性起步
     * 前馈(双轮刻意同值,不继承 LHX 的 L/R 不对称死区);置 0 即纯积分爬升。 */
    s_spd_l.last_output = HJY_LAUNCH_FF;
    s_spd_r.last_output = HJY_LAUNCH_FF;
    HJY_AnglePID_Reset();
    s_win_inited = 0u;
    g_hjy_state = (uint8_t)HJY_ST_STRAIGHT;
    g_hjy_vt_l = g_hjy_vt_r = 0;
    g_hjy_v_l = g_hjy_v_r = 0;
    g_hjy_pwm_l = g_hjy_pwm_r = 0;
    g_hjy_ang_corr = 0;
    g_hjy_yaw_err_d10 = 0;
}

void HJY_Control_Update(void)
{
    HJY_LineState_t st;
    float base, diff, ang_out = 0.0f;
    float vt_l, vt_r;

    if (!is_racing)
    {
        Motor_StopAll();
        Motor_Disable();
        g_hjy_pwm_l = g_hjy_pwm_r = 0;
        g_hjy_vt_l = g_hjy_vt_r = 0;
        /* HJY2(16:12 架空轮): 停车后 v=35,35/ang=12 陈旧值挂死 6s+——本分支
         * 提前 return,窗口/角度环不再跑,显示全员清零防误读;窗口快照作废,
         * 下次发车(K1→ResetRun)或恢复 racing 时重新初始化。 */
        g_hjy_v_l = g_hjy_v_r = 0;
        g_hjy_ang_corr = 0;
        g_hjy_yaw_err_d10 = 0;
        g_hjy_err_l = g_hjy_err_r = 0;
        s_win_inited = 0u;
        return;
    }
    Motor_Enable();

    /* 1) 光路分区 */
    st = HJY_Classify(g_line_sensor_values);
    if (st == HJY_ST_ALLWHITE)
    {
        /* 丢线:沿用上一有效状态的目标继续走(不清不踢),
         * 超时停车由 main 主环 lose_time(375)兜底,本核不重复判停。 */
        st = (HJY_LineState_t)g_hjy_state;
        if (st == HJY_ST_ALLWHITE) st = HJY_ST_STRAIGHT;   /* 防御:不应发生 */
    }
    else
    {
        g_hjy_state = (uint8_t)st;
    }

    base = k_base[st];
    diff = k_diff[st];

    /* 2) 角度环(仅直行/全黑态):进入态沿锁存航向,误差>0(偏左)→正差速右修 */
    if (st == HJY_ST_STRAIGHT || st == HJY_ST_ALLBLACK)
    {
        float err, dterm;
        if (!s_yaw_latched)
        {
            s_yaw_target  = add_angle;
            s_yaw_latched = 1u;
            s_ang.integral = 0.0f;
            s_ang.last_err = 0.0f;
        }
        err = add_angle - s_yaw_target;
        s_ang.integral += s_ang.ki * err;
        if (s_ang.integral >  s_ang.out_max) s_ang.integral =  s_ang.out_max;
        if (s_ang.integral < -s_ang.out_max) s_ang.integral = -s_ang.out_max;
        dterm = s_ang.kd * (err - s_ang.last_err);
        s_ang.last_err = err;
        ang_out = s_ang.kp * err + s_ang.integral + dterm;
        if (ang_out >  s_ang.out_max) ang_out =  s_ang.out_max;
        if (ang_out < -s_ang.out_max) ang_out = -s_ang.out_max;
        g_hjy_yaw_err_d10 = (int16_t)(err * 572.957795f);   /* rad→0.1° */
    }
    else
    {
        /* 转向态:角度环退出,目标失效,出弯重新锁存 */
        HJY_AnglePID_Reset();
        g_hjy_yaw_err_d10 = 0;
    }
    g_hjy_ang_corr = (int16_t)(ang_out * 10.0f);

    /* 3) 轮目标速度:差速>0=右转(vL 高 vR 低,与 LHX corr 符号体系一致) */
    diff += ang_out;
    vt_l = base + diff;
    vt_r = base - diff;
    if (vt_l < 0.0f) vt_l = 0.0f;       /* 目标不为负:转向靠速度差,非反转 */
    if (vt_r < 0.0f) vt_r = 0.0f;
    g_hjy_vt_l = (int16_t)vt_l;
    g_hjy_vt_r = (int16_t)vt_r;

    /* 4) 轮速窗口:HJY_SPEED_WIN_MS 滑窗换算 cnt/s,窗口闭合时跑两套速度环 */
    if (!s_win_inited)
    {
        s_win_inited = 1u;
        s_last_cnt_l = left_encoder_cnt;
        s_last_cnt_r = right_encoder_cnt;
        s_acc_l = 0;
        s_acc_r = 0;
        s_win_t0 = Millis_Get();
    }
    else
    {
        int32_t  cl = left_encoder_cnt, cr = right_encoder_cnt;
        uint32_t now, dt;
        s_acc_l += cl - s_last_cnt_l;
        s_acc_r += cr - s_last_cnt_r;
        s_last_cnt_l = cl;
        s_last_cnt_r = cr;
        now = Millis_Get();
        dt  = now - s_win_t0;
        if (dt >= HJY_SPEED_WIN_MS)
        {
            float pwm_l, pwm_r;
            if (dt == 0u) dt = 1u;
            g_hjy_v_l = (int16_t)((s_acc_l * 1000) / (int32_t)dt);
            g_hjy_v_r = (int16_t)((s_acc_r * 1000) / (int32_t)dt);
            s_acc_l = 0;
            s_acc_r = 0;
            s_win_t0 = now;

            /* 5) 左右独立速度环(增量式,内部带 Δ200/步 与输出钳位) */
            g_hjy_err_l = (int16_t)(vt_l - (float)g_hjy_v_l);   /* HJY2: PID 实吃误差遥测 */
            g_hjy_err_r = (int16_t)(vt_r - (float)g_hjy_v_r);
            pwm_l = SpeedPID_Calculate(&s_spd_l, vt_l, (float)g_hjy_v_l);
            pwm_r = SpeedPID_Calculate(&s_spd_r, vt_r, (float)g_hjy_v_r);

            /* per-state 标定前馈叠加(初值 0;回填后注意:前馈不入 PID 状态,
             * 仅输出端平移,PID 只吃残差) */
            pwm_l += (float)s_ff_l[st];
            pwm_r += (float)s_ff_r[st];
            if (pwm_l >  HJY_PWM_MAX)     pwm_l =  HJY_PWM_MAX;
            if (pwm_l < -HJY_PWM_REV_MAX) pwm_l = -HJY_PWM_REV_MAX;
            if (pwm_r >  HJY_PWM_MAX)     pwm_r =  HJY_PWM_MAX;
            if (pwm_r < -HJY_PWM_REV_MAX) pwm_r = -HJY_PWM_REV_MAX;
            g_hjy_pwm_l = (int16_t)pwm_l;
            g_hjy_pwm_r = (int16_t)pwm_r;
        }
    }

    /* 6) PWM 直驱(无死区前馈级;窗口间保持上次 PWM)。
     * Motor_SetSpeedWithDirection 内部处理符号→方向并同步 g_motor_target_*,
     * 旧遥测 pid= 字段顺带继续有意义。 */
    Motor_SetSpeedWithDirection(MOTOR_L, (float)g_hjy_pwm_l);
    Motor_SetSpeedWithDirection(MOTOR_R, (float)g_hjy_pwm_r);
}

#endif /* HJY_CORE_ENABLE */
