#ifndef __HJY_CONTROL_H__
#define __HJY_CONTROL_H__

#include "stm32f10x.h"

/**
 * @brief HJY 三环重构核(2026-06-07,HJY 分支专属)
 *
 * 方案来源:HJY 整改方案(全文存档 PID_TUNING_LOG.md 同日条目)。
 * 与原 LHX 核(共用速度环+位置环+静态死区前馈)的本质区别:
 *   1. 三套独立 PID:角度环(MPU,整车 1 套) + 左轮速度环 + 右轮速度环(各 1 套);
 *   2. 循迹不走位置 PID,改"红外光路穷举分区 → 状态 → 差速等级"查表;
 *   3. 放弃静态 PWM 死区补偿(START/HOLD 不对称前馈全部不用),左右电机
 *      机械差异交给各自速度闭环动态吃掉;仅保留发车初值前馈(标定起点,可置 0)。
 *
 * 主开关 HJY_CORE_ENABLE=1 时 main.c 控制 tick 走 HJY_Control_Update(),
 * 原 PID_Control_Update() 与死区输出级整段编译旁路(代码保留,置 0 即回退)。
 */
#define HJY_CORE_ENABLE 1

/* ===== 光路穷举分区状态(7 路,索引 0=最左) =====
 * 全部 2^7 组合经"黑灯加权质心 c10(0~60)"确定性映射进唯一状态,
 * 全黑/全白单列。每档典型组合见 HJY_Control.c 分类器注释。 */
typedef enum
{
    HJY_ST_STRAIGHT = 0,  /* 中间灯组(3/4/5 邻域)黑:直行,仅角度环微修 */
    HJY_ST_L1,            /* 线偏左 小:小力度左转 */
    HJY_ST_L2,            /* 线偏左 中:中力度左转 */
    HJY_ST_L3,            /* 线偏左 大(1/2 号灯):大力度左转 */
    HJY_ST_R1,            /* 线偏右 小:小力度右转 */
    HJY_ST_R2,            /* 线偏右 中:中力度右转 */
    HJY_ST_R3,            /* 线偏右 大(6/7 号灯):大力度右转 */
    HJY_ST_ALLBLACK,      /* 全黑(岔口/横线):锁航向慢速直穿 */
    HJY_ST_ALLWHITE,      /* 全白(丢线):沿用上一状态目标,主环 375 兜底停 */
    HJY_ST_COUNT
} HJY_LineState_t;

/* ===== 初始参数(第一步"合理大区间"配置,串口实测迭代收敛) ===== */
#define HJY_BLACK_THRESH      2048    /* ADC < 阈值判黑(0/1365 黑,2730/4095 白) */
#define HJY_SPEED_WIN_MS      200u    /* 轮速窗口。HJY2(16:12 架空轮实测): 100ms 窗
                                       * 每窗仅 2.5cnt → v 分辨率 10cnt/s,PID 全程追
                                       * 量化噪声(pwm_L 510~800 来回);200ms 分辨率
                                       * 5cnt/s,响应仍快旧核 250ms 一档 */
#define HJY_BASE_SPEED        25.0f   /* 直行基速 cnt/s(沿用 E1 验证档) */
#define HJY_BASE_SPEED_MID    22.0f   /* 中偏转基速 */
#define HJY_BASE_SPEED_SLOW   18.0f   /* 大偏转/全黑基速 */
#define HJY_PWM_MAX           1800.0f /* 单轮 PWM 上限(SAFE_MAX 2000 内留 200) */
#define HJY_PWM_REV_MAX       400.0f  /* 反转钳位(允许轻刹,禁暴力倒转) */
#define HJY_LAUNCH_FF         900.0f  /* 发车初值前馈(死区邻域,双轮同值=刻意
                                       * 对称;置 0 即纯积分爬升纯净试验) */
/* 速度环(左右独立,初值同,标定后允许分道扬镳) */
#define HJY_SPD_KP            2.0f
#define HJY_SPD_KI            14.0f
#define HJY_SPD_KD            0.0f    /* 编码器粗(44cnt/m),Kd 先 0 防量化噪声 */
/* 角度环(MPU add_angle,整车 1 套;仅直行/全黑态生效,锁存进入时航向) */
#define HJY_ANG_KP            120.0f  /* 输出差速 cnt/s / rad(≈2.1 cnt/s 每°) */
#define HJY_ANG_KI            0.0f
#define HJY_ANG_KD            0.0f
#define HJY_ANG_CORR_MAX      6.0f    /* 角度环差速钳位 cnt/s(微修定位) */

void HJY_Init(void);
void HJY_ResetRun(void);       /* K1 发车/K3 复位:三套 PID+状态全清 */
void HJY_Control_Update(void); /* 主控 tick 调用(替代 PID_Control_Update+死区输出级) */

/* ===== 遥测(串口标定采集用,main.c 调试行引用) ===== */
extern volatile uint8_t  g_hjy_state;        /* 当前光路状态(HJY_LineState_t) */
extern volatile int16_t  g_hjy_vt_l;         /* 左轮目标速度 cnt/s */
extern volatile int16_t  g_hjy_vt_r;         /* 右轮目标速度 cnt/s */
extern volatile int16_t  g_hjy_v_l;          /* 左轮实测速度 cnt/s(HJY 窗口) */
extern volatile int16_t  g_hjy_v_r;          /* 右轮实测速度 cnt/s */
extern volatile int16_t  g_hjy_pwm_l;        /* 左轮最终 PWM(带符号) */
extern volatile int16_t  g_hjy_pwm_r;        /* 右轮最终 PWM */
extern volatile int16_t  g_hjy_ang_corr;     /* 角度环输出差速 cnt/s(×10 存) */
extern volatile int16_t  g_hjy_yaw_err_d10;  /* 角度环误差 0.1° 单位 */
extern volatile int16_t  g_hjy_err_l;        /* HJY2: 左速度环本窗实际误差(vt−v) */
extern volatile int16_t  g_hjy_err_r;        /* HJY2: 右速度环本窗实际误差——
                                              * 判"pwm_R 863 冻结"是混叠假象还是真死环 */

#endif /* __HJY_CONTROL_H__ */
