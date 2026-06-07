/**
 * HJY_Control.c —— HJY 三环重构核(2026-06-07)
 *
 * 架构(方案原文见 PID_TUNING_LOG.md 同日条目;HJY7 起角度环全程生效):
 *   光路穷举分区 → (基速, 目标偏航率 w_t) ┐
 *   角度环=全程率环 diff=−[FF·w_t+KP·(w_t−gz)] ┼→ 左/右轮目标速度 → 左右独立速度 PID → PWM 直驱
 *   直行族另加航向保持分量(锁存航向→小目标率) ┘   (per-state PWM 前馈表留作标定钩子)
 *
 * 显式不用:位置环 PID、START/HOLD 静态死区前馈、coast pivot、踢腿 boost。
 * 左右电机机械不对称 → 两套独立速度环动态补偿;旋转动量 → MPU 率环全程看管。
 */
#include "HJY_Control.h"

#if HJY_CORE_ENABLE

#include "PID_Controller.h"   /* SpeedPID_* 结构与算法复用 + Motor_SetSpeedWithDirection */
#include "ABEncoder.h"        /* left/right_encoder_cnt */
#include "LineSensor.h"       /* g_line_sensor_values[SENSOR_COUNT] */
#include "Motor_ctr.h"        /* MOTOR_L/R, Motor_Enable/Disable/StopAll */
#include "stm32f10x_it.h"     /* add_angle(rad), Millis_Get() */
#include "MPU6050_Config.h"   /* HJY6: MPU6050_data.gz_rads(偏航阻尼) */

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

/* HJY7: 原"直行态专属角度位置式 PID"已并入全程率环(Update 第 3 节),
 * 此处仅保留航向保持的锁存状态。 */
static float   s_yaw_target = 0.0f;     /* 直行族锁存的目标航向(rad) */
static uint8_t s_yaw_latched = 0u;      /* 1=target 有效 */

/* ===== 状态→目标查表 ===== */
/* HJY7: 目标偏航率表 rad/s(>0=左旋,与 gz 同号)——光电分情况逻辑的输出端,
 * 角度环(率环)全程伺服。L1/L2/L3≈60/120/180°/s(U 弯需求 ~150°/s,封顶留量),
 * R 镜像;直行/全黑 0(航向保持分量另加)。 */
static const float k_rate[HJY_ST_COUNT] =
{
    0.0f,    /* STRAIGHT(保持分量另加) */
    +1.0f,   /* L1 小左 ≈60°/s */
    +2.1f,   /* L2 中左 ≈120°/s */
    +3.1f,   /* L3 大左 ≈180°/s */
    -1.0f,   /* R1 小右 */
    -2.1f,   /* R2 中右 */
    -3.1f,   /* R3 大右 */
    0.0f,    /* ALLBLACK 直穿(保持分量另加) */
    0.0f     /* ALLWHITE(不用,盲走沿记忆态) */
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
/* HJY3: K1 静置窗——s_go_at 前电机保持 0;到点一次性打不对称前馈脉冲后进闭环 */
static uint32_t s_go_at = 0u;
static uint8_t  s_went = 0u;
/* HJY4a: GO 后爬行段——双轮目标压 10cnt/s,直到两轮都计满 ALIVE_CNT(或超时)。
 * 先破粘轮被闭环按在 10 而非全速狂奔,把"单轮先动"的甩头角钳到 ~20° 内(线不丢)。 */
static uint8_t  s_creep_done = 0u;
static int32_t  s_go_cnt_l = 0, s_go_cnt_r = 0;
static uint32_t s_creep_deadline = 0u;
/* HJY5b: 破粘一次性突降标志(每轮一次) */
static uint8_t  s_broke_l = 0u, s_broke_r = 0u;
/* HJY4c: 丢线保持方向防抖——瞬时分类照常驱动,但"丢线记忆"须同态连续 25 tick
 * 才更新;16:58 实测横穿线时右缘残影 1 帧把记忆从 L 改成 R3,反向旋 250°。 */
static uint8_t  s_hold_st = (uint8_t)HJY_ST_STRAIGHT;
static uint8_t  s_st_prev = 255u;
static uint8_t  s_st_run = 0u;
/* HJY4b: 目标骤降快退绕——上窗目标,检测转向态切入 */
static float    s_prev_vt_l = 0.0f, s_prev_vt_r = 0.0f;

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

void HJY_Init(void)
{
    /* 左右速度环:独立实例,初值相同;标定阶段经串口数据各自收敛。
     * 输出域=PWM(带符号),上限留 200 余量给 SAFE_MAX,反转钳 -400 轻刹。
     * HJY7: 角度环(全程率环)无状态参数,直接用 #define;仅航向锁存需复位。 */
    SpeedPID_Init(&s_spd_l, HJY_SPD_KP, HJY_SPD_KI, HJY_SPD_KD, HJY_PWM_MAX, -HJY_PWM_REV_MAX);
    SpeedPID_Init(&s_spd_r, HJY_SPD_KP, HJY_SPD_KI, HJY_SPD_KD, HJY_PWM_MAX, -HJY_PWM_REV_MAX);

    s_yaw_latched = 0u;
    s_win_inited = 0u;
}

void HJY_ResetRun(void)
{
    SpeedPID_Reset(&s_spd_l);
    SpeedPID_Reset(&s_spd_r);
    /* HJY3: 发车前馈按轮分开(870/990,实测电机起动邻域)。16:46 事故根因=
     * 对称 900 种子在不对称电机上,首窗 1100,1100 左先动右不动→开局右甩。 */
    s_spd_l.last_output = HJY_LAUNCH_FF_L;
    s_spd_r.last_output = HJY_LAUNCH_FF_R;
    /* HJY3: K1 静置窗计时起点 */
    s_go_at = Millis_Get() + HJY_LAUNCH_HOLD_MS;
    s_went  = 0u;
    /* HJY4: 爬行段/防抖记忆/快退绕基线同清 */
    s_creep_done = 0u;
    s_hold_st = (uint8_t)HJY_ST_STRAIGHT;
    s_st_prev = 255u;
    s_st_run  = 0u;
    s_prev_vt_l = s_prev_vt_r = 0.0f;
    /* HJY5: 破粘标志同清 */
    s_broke_l = s_broke_r = 0u;
    s_yaw_latched = 0u;   /* HJY7: 航向锁存复位(原 AnglePID 已并入全程率环) */
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
    HJY_LineState_t raw;
    uint8_t blind;
    float base, w_t, diff;
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

    /* HJY3: K1 静置窗——电机压 0(车不动,人手撤离/风机起转/MPU 安定),
     * 航向锁存与轮速窗一律延后到 GO 瞬间初始化;遥测可辨签名=vt 0,0 + fn=1。 */
    if (!s_went)
    {
        if (Millis_Get() < s_go_at)
        {
            Motor_StopAll();
            g_hjy_pwm_l = g_hjy_pwm_r = 0;
            g_hjy_vt_l = g_hjy_vt_r = 0;
            g_hjy_v_l = g_hjy_v_r = 0;
            g_hjy_err_l = g_hjy_err_r = 0;
            s_win_inited = 0u;
            s_yaw_latched = 0u;   /* HJY7: 同上 */
            return;
        }
        s_went = 1u;
        /* GO 脉冲:首个速度窗闭合(~200ms)前电机就吃到各自前馈,起步即对称
         * 破粘——替代旧"首窗 0→1100,1100 对称跳变"(16:46 右甩根因)。 */
        g_hjy_pwm_l = (int16_t)HJY_LAUNCH_FF_L;
        g_hjy_pwm_r = (int16_t)HJY_LAUNCH_FF_R;
        /* HJY4a: 爬行段基线快照 */
        s_go_cnt_l = left_encoder_cnt;
        s_go_cnt_r = right_encoder_cnt;
        s_creep_deadline = Millis_Get() + HJY_CREEP_TIMEOUT_MS;
    }

    /* 1) 光路分区 */
    raw = HJY_Classify(g_line_sensor_values);
    blind = (raw == HJY_ST_ALLWHITE) ? 1u : 0u;
    st = raw;
    if (blind)
    {
        /* HJY4c: 丢线沿用"防抖记忆"而非瞬时残影——16:58 实测车横穿线时
         * 右缘 1 帧残影把保持方向从 L 族改写成 R3,反向旋 250° 触安全网。
         * 超时停车仍由 main 主环 lose_time(375)兜底,本核不重复判停。 */
        st = (HJY_LineState_t)s_hold_st;
        /* HJY5c: 盲走降档——L3/R3 速率(差速±16≈300°/s)扫线必过头(17:20 实测
         * yw 71→166 一路旋过线触安全网);盲走只许 L2/R2 档,慢扫等线回视。 */
        if (st == HJY_ST_L3)      st = HJY_ST_L2;
        else if (st == HJY_ST_R3) st = HJY_ST_R2;
    }
    else
    {
        /* 防抖记忆:同态连续 HJY_ST_DEBOUNCE_TICKS(~50ms)才有资格当丢线锚 */
        if ((uint8_t)st == s_st_prev)
        {
            if (s_st_run < 255u) s_st_run++;
        }
        else
        {
            s_st_prev = (uint8_t)st;
            s_st_run  = 1u;
        }
        if (s_st_run >= HJY_ST_DEBOUNCE_TICKS) s_hold_st = (uint8_t)st;
    }
    /* HJY5: 遥测 st 改记"实际驱动态"(丢线时=降档后的记忆态)——17:20 出现过
     * st=3 vt=12,32 表观矛盾(st 显瞬时残影,驱动用防抖记忆),统一成驱动态防误读 */
    g_hjy_state = (uint8_t)st;

    base = k_base[st];
    w_t  = k_rate[st];   /* 角度环全程目标:目标偏航率 rad/s(>0=左旋) */

    /* HJY4a: GO 后爬行段——双轮压 10cnt/s 直到都活(各 ≥2cnt)或超时。
     * HJY7: 爬行期角度环照常全程生效(w_t=0 → 率环主动扯平单轮先破粘的
     * 偏航,死轮自动多分目标);航向保持锁存延后到爬行结束。 */
    if (!s_creep_done)
    {
        if ((left_encoder_cnt  - s_go_cnt_l >= HJY_CREEP_ALIVE_CNT &&
             right_encoder_cnt - s_go_cnt_r >= HJY_CREEP_ALIVE_CNT) ||
            Millis_Get() >= s_creep_deadline)
        {
            s_creep_done = 1u;
        }
        else
        {
            base = HJY_CREEP_SPEED;
            w_t  = 0.0f;
            s_yaw_latched = 0u;
            /* HJY5a/b: 破粘检测与突降都在 tick 级(2ms),不等 200ms 窗——
             * 17:20 实测静摩擦阈(1170/1290@12.34V)≫动摩擦需求,窗级响应下
             * 破粘瞬间 40+cnt/s 暴冲 30cm。死轮 tick 爬坡加速破粘;首计数
             * 瞬间把积分一次性压回 FF−100,突奔掐在 2ms 内。 */
            if (!s_broke_l)
            {
                if (left_encoder_cnt - s_go_cnt_l >= 1)
                {
                    s_broke_l = 1u;
                    s_spd_l.last_output = HJY_LAUNCH_FF_L - HJY_BREAK_DROP_OFFSET;
                }
                else
                {
                    s_spd_l.last_output += HJY_CREEP_RAMP_PER_TICK;
                    if (s_spd_l.last_output > 1500.0f) s_spd_l.last_output = 1500.0f;
                }
            }
            if (!s_broke_r)
            {
                if (right_encoder_cnt - s_go_cnt_r >= 1)
                {
                    s_broke_r = 1u;
                    s_spd_r.last_output = HJY_LAUNCH_FF_R - HJY_BREAK_DROP_OFFSET;
                }
                else
                {
                    s_spd_r.last_output += HJY_CREEP_RAMP_PER_TICK;
                    if (s_spd_r.last_output > 1500.0f) s_spd_r.last_output = 1500.0f;
                }
            }
            /* 爬行期 tick 级直驱:窗间也跟随积分(爬坡/突降即时入电机) */
            g_hjy_pwm_l = (int16_t)s_spd_l.last_output;
            g_hjy_pwm_r = (int16_t)s_spd_r.last_output;
        }
    }

    /* 2) 航向保持(角度环"位置"分量,直行/全黑且爬行完成):锁存进入时航向,
     * 偏差折算成小目标率拉回;转向态退出、出弯重锁。盲走若记忆=直行族,
     * 继续按锁存航向走(≈直穿语义)。 */
    if (s_creep_done && (st == HJY_ST_STRAIGHT || st == HJY_ST_ALLBLACK))
    {
        float w_hold;
        if (!s_yaw_latched)
        {
            s_yaw_target  = add_angle;
            s_yaw_latched = 1u;
        }
        w_hold = HJY_HOLD_KP * (s_yaw_target - add_angle);  /* 目标在左→正(左旋)率 */
        if (w_hold >  HJY_HOLD_RATE_MAX) w_hold =  HJY_HOLD_RATE_MAX;
        if (w_hold < -HJY_HOLD_RATE_MAX) w_hold = -HJY_HOLD_RATE_MAX;
        w_t += w_hold;
        g_hjy_yaw_err_d10 = (int16_t)((add_angle - s_yaw_target) * 572.957795f);
    }
    else if (st != HJY_ST_STRAIGHT && st != HJY_ST_ALLBLACK)
    {
        s_yaw_latched = 0u;
        g_hjy_yaw_err_d10 = 0;
    }

    /* 3) 角度环主体(率环,全程生效——用户 17:5X 指令"角度环全程生效,与光电
     * 分情况逻辑紧密配合"):光电状态只产目标偏航率 w_t,轮差速一律由 MPU
     * 闭环伺服。直行 w_t=0=保持+阻尼;转向=受控转速(L3≈180°/s);盲走=记忆
     * 态目标率——旋转动量任何时刻都被看管(17:34 摆飞 +178° 的结构性根治)。
     * 法则: diff = −[FF×w_t + KP×(w_t − gz)];diff>0=右修(LHX 符号系);
     * 验算: w_t=0 时 diff=+KP×gz=纯阻尼(HJY6 等价);左转目标(w_t>0)未达率
     * 时 diff<0=vR>vL 左转 ✓。 */
    diff = -(HJY_RATE_FF * w_t + HJY_RATE_KP * (w_t - MPU6050_data.gz_rads));
    if (diff >  HJY_DIFF_MAX) diff =  HJY_DIFF_MAX;
    if (diff < -HJY_DIFF_MAX) diff = -HJY_DIFF_MAX;
    g_hjy_ang_corr = (int16_t)(diff * 10.0f);   /* 遥测 ang=角度环全程输出×10 */
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

            /* HJY4b: 目标骤降(切转向态)一次性把积分起点压到该轮起动阈下
             * (FF−100=左770/右890)——绕过增量式 Δ200/窗慢退绕:16:58 实测
             * L3 期左轮被命令 2 却以 36 跑了 ~1s(pwm 1131→931→731),车带速
             * 冲过线引发反向误捕。压到阈下轮子立即降速;回正常态后从
             * 770/890 起 1~2 窗就能重新咬合。 */
            if (vt_l + HJY_UNWIND_DROP < s_prev_vt_l &&
                s_spd_l.last_output > HJY_LAUNCH_FF_L - 100.0f)
                s_spd_l.last_output = HJY_LAUNCH_FF_L - 100.0f;
            if (vt_r + HJY_UNWIND_DROP < s_prev_vt_r &&
                s_spd_r.last_output > HJY_LAUNCH_FF_R - 100.0f)
                s_spd_r.last_output = HJY_LAUNCH_FF_R - 100.0f;
            s_prev_vt_l = vt_l;
            s_prev_vt_r = vt_r;

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
