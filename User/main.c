#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LineSensor.h"
#include "Uart_Config.h"
#include "stdio.h"
#include "math.h"
#include "RGB_Led.h"
#include "MPU6050_Config.h"
#include "pose.h"
#include "Key_Scan.h"
#include "system_stm32f10x.h"
#include "BlackPoint_Finder.h"
#include "TelemetryScreen.h"
#include "Protocol.h"
#include "ESP32_Comm.h"
#include "Battery.h"
#include "ABEncoder.h"
#include "PID_Controller.h"
#include "Path.h"
#include "Motor_ctr.h"
#include "stm32f10x_it.h"
#if SINGLE_BOARD_LOCAL_DRIVE
#include "M3PWM.h"                 /* 单板:风扇 TIM2_CH4 本地 PWM */
#endif

/*
 * ============================================================
 *  二合一板 —— 上板（大脑）固件
 *  职责：7 路灰度循迹 + MPU6050 + OLED + 按键启动，
 *        本地运行全部控制算法：位置环 + 速度环 + Path
 *        + 15% 起步 + 丢线保护，最终通过 USART2 下发
 *        MOTOR_CMD（左/右带符号占空比 + 使能）给下板。
 *  轮速反馈来自下板的 ENC_FEEDBACK 帧。
 *  设计目标：调试时只烧上板，下板（纯执行器）烧一次永不变。
 *  注意：USART2 是“板间链路”，只发二进制帧，不输出调试文本。
 * ============================================================
 */

// 滴答定时器初始化，2ms中断一次 (500Hz)
void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 500);
}

float BDI_V = 0;
volatile uint8_t is_racing = 0;            /* 按键启动标志：1=运行，发送给下板 */
static uint8_t g_imu_init_ok = 0;
static uint8_t g_imu_who_id = 0;

// 控制环状态（从 SysTick 移到主循环）
// position_get 的 extern 声明已在 stm32f10x_it.h 中
BlackPointResult_t result_BlackPoint;
static uint32_t lose_time = 0;

// ESP32-S3 雷达数据（保留协议兼容）
volatile uint16_t radar_distance_cm = 0;

/* 起步时给速度环输出的初值(满量程 10000)。
 * 死区前馈已负责克服起步静摩擦，这里置 0，避免与速度环首拍增量叠加导致起步过冲。 */
#define START_DUTY_15PCT  0.0f

/* 电机占空比硬上限（满量程 10000）：钳住 PID 速度环的调节量(不含死区前馈)。
 * 1000 = 满量程的 10%。叠加右轮最大死区 950 后最终≈1950，仍 <下板 SAFE_MAX(2000)，
 * 故下板安全限无需改动。调速就改这一个值：想更慢往下调，想更快往上调(但需留出死区余量)。 */
#ifndef MOTOR_DUTY_HARD_CAP
#define MOTOR_DUTY_HARD_CAP  1000.0f
#endif

/* 带符号占空比限幅到 [-CAP, +CAP]，保留方向 */
static float ClampMotorDuty(float duty)
{
    if (duty >  MOTOR_DUTY_HARD_CAP) return  MOTOR_DUTY_HARD_CAP;
    if (duty < -MOTOR_DUTY_HARD_CAP) return -MOTOR_DUTY_HARD_CAP;
    return duty;
}

/* 电机起步死区前馈：实测右轮约需 10%(1000) 才起步、左轮约 8%(800)，
 * 死区以下电机不动，PID 输出落在死区内等于白调。
 * 故对带方向的非零命令垫上一个起步基值，把 PID 的工作点抬到电机能动的区间，
 * 左右分别配置以补偿起步摩擦不对称(右轮更难起步)。
 * 命令绝对值低于 EPS 视为停车(返回 0)，避免 0 附近抖动与静止蠕行。
 * 上车微调：某轮起步偏迟就调高对应 DEADZONE；起步窜得猛就调低。 */
/* D1(06-06 用户授权): START 820/940 → 870/990 对称+50。旧红线"START 不动"标定于旧机械；
 * 今日车体结构改动(摩擦+自重双增)后 19:03 实测中途双轮停转 1.2s——sent 941/1096(START档)
 * 仍拉不动,直到差速尖峰 1284 才破粘。+50 是缓解;若再现 >1s 双轮停转,下一级=停转踢腿逻辑(待批)。
 * 不对称 120 保持。 */
#ifndef MOTOR_START_DEADZONE_L
#define MOTOR_START_DEADZONE_L  870.0f
#endif
#ifndef MOTOR_START_DEADZONE_R
#define MOTOR_START_DEADZONE_R  990.0f
#endif
/* HOLD 760/880→640/760(06-05 第10轮): 旧值标定于亏电电池，满电下该前馈自身
 * 即推车到~38cps，速度环(floor=70)只能上行无法下调，T=20物理不可达。
 * 对称降120：START不动(发车行为不变)；降过头由速度环上行补回。
 * HOLD_R 760→700(06-05 第11轮): 不对称120是亏电标定的摩擦补偿，满电低占空比下
 * 过补偿——慢性右偏(pos坐30~35)/锯齿右贴deep/右pivot差速被吃(实测右136 vs 左382)
 * 三症同根。降到60: 右pivot差196(+44%)，左仍322。START不对称120保留。 */
/* HOLD 640/700→580/640(06-05 第12轮): 弯道真瓶颈=内轮减不下去——MIN_INNER20+640=
 * sent660 满电仍跑23cps，pivot指令差322实测轮速差仅~15cps。再对称降60: 内轮sent
 * 600→预计15cps，轮速差→~22cps收紧半径；基线18~25。失速风险线(用户提醒占空比过低
 * 电机不转): 直线单轮个位数轮速/顿挫 → 回+30(610/670)。不对称60与START不动。 */
/* HOLD_R 640→610(06-05 第13轮): S弯=30~50cm波浪交替弯,要求左右pivot对称；
 * 实测右196/左253——剩余不对称60还在吃右转,第一个右拐贴不住线(U弯左侧253零丢线
 * 已验证成功线)。不对称60→30: 右226/左223,两侧都到成功线90%。START不动。 */
/* B(06-06): 车体结构改动→底板蹭地摩擦增大,巡航低位已见瞬时停转帧(17:07:51 L=0@sent840
 * 靠 R3 滞回踢回)。用户授权"+3%"防停转,技术落点=HOLD 对称+30 → 610/640(全局加成会被
 * 速度闭环回吐,无效)。START 820/940 不动(两组上图发车实测正常)。不够再+30 迭代。 */
/* D2(06-06 用户授权,继 B +30 后再 +30): 610/640 → 640/670——19:03 实测巡航低位 L/R 间歇
 * 掉到 10~13cps,自重增大后保持档余量仍紧。不对称 30 保持。 */
/* D4(06-06 20:46/20:49 两轮上图,用户指示"占空比给大"): 640/670 → 680/710(+40 对称)。
 * 实证:关扇轮 U 弯内轮七八帧钉死 sent=660(HOLD_L 640+pid 20),R3/D3b 反复踢出 890/1151
 * 脉冲打乱转弯半径,U 弯只转到 83° 捞错边;开扇轮同占空比速度 23→10cps 衰减后双轮全停。
 * 看护项:深弯内轮地板抬高→拖刹差速变浅,U 弯半径若变宽即回 -20 折中。START 不动。 */
/* F2(06-07 用户纠偏):不是只补 S 后段,而是粗糙地图+低底盘导致全图移动档卡顿。
 * HOLD 680/710→710/740(+30 对称),所有闭环移动段基础占空比小幅上移;
 * START 870/990 与安全上限不动,避免起步/堵转红线扩大。 */
/* F3(06-07 现场追加):F2 后用户反馈全图仍偏慢/易卡,按既定预案再小步 +20 对称:
 * HOLD 710/740→730/760。覆盖全部闭环移动段(前段28/u后25/sm14/丢线档/RD盲走);
 * START 870/990、FINAL_CAP 1990、下板 SAFE_MAX 2000 不动;深弯内轮 cmd=0 仍 coast。
 * 回退线:U/S 半径变宽、贴不住内线或 S 乒乓 → 回 710/740(F2),再不行 700/730 折中。 */
/* F4a(06-07 03:00 实测):F3 已确认烧录(sent−pid 差=730/760 反推)。巡航 23~26cps 仍欠
 * T=28,sent 880~990 底子重;左轮 880PWM 慢磨卡滞(el 冻 233)。按梯再 +20 对称:750/780。
 * 回退链:750/780→730/760→710/740;U/S 半径变宽或 S 乒乓仍是硬回退线。 */
/* F5(06-07 03:04 实测+用户指令"还可以变大"):HOLD 域拆分,直线档与 S 配方解耦——
 * 非 sm 段(直线/U/post-S/方块圆区/RD)770/800 继续上探(F4a 750/780 未上图,直接跨档);
 * sm 段独立钉 730/760 = 03:04 轮 S① 全程首通实测档(该轮 S 中段曾单发丢线+人工救,
 * 内轮 coast 下 pivot 扫线角速度随外轮 HOLD 升——sm 档不随直线档漂移)。
 * 回退:S 再丢线 → 只回 S_MODE_HOLD 710/740→680/710;直线再卡 → 只动主 HOLD。 */
#ifndef MOTOR_HOLD_DEADZONE_L
#define MOTOR_HOLD_DEADZONE_L   770.0f
#endif
#ifndef MOTOR_HOLD_DEADZONE_R
#define MOTOR_HOLD_DEADZONE_R   800.0f
#endif
#ifndef S_MODE_HOLD_DEADZONE_L
#define S_MODE_HOLD_DEADZONE_L  730.0f
#endif
#ifndef S_MODE_HOLD_DEADZONE_R
#define S_MODE_HOLD_DEADZONE_R  760.0f
#endif
#ifndef MOTOR_HOLD_SPEED_CPS
#define MOTOR_HOLD_SPEED_CPS    10
#endif
/* R3(06-05 审查): START/HOLD 选择器滞回。旧单阈值 10cps 无滞回——内轮 pivot 减速穿越时
 * 死区前馈 580↔820 来回跳变(240 PWM 阶跃,2~4Hz)=顿挫源,深弯内轮 MIN_INNER=20 恰落该区。
 * >ENTER 进 HOLD,<EXIT 回 START,每轮独立。 */
#ifndef MOTOR_HOLD_ENTER_CPS
#define MOTOR_HOLD_ENTER_CPS    12
#endif
#ifndef MOTOR_HOLD_EXIT_CPS
#define MOTOR_HOLD_EXIT_CPS     8
#endif
static uint8_t g_dz_hold_l = 0;   /* 1=左轮处于 HOLD 死区档 */
static uint8_t g_dz_hold_r = 0;
static float g_stall_boost_l = 0.0f;  /* D3(06-06): 皱褶停转踢腿——按轮自适应死区上浮量 */
static float g_stall_boost_r = 0.0f;
/* 06-07 01:50 粗糙地图/底盘托底:只放大非 sm/deep/NAV 覆盖段脱困档。
 * S/U 深弯和雷达盲走仍封 100,防乒乓锤/弹射/盲走过冲。 */
#ifndef MOTOR_STALL_BOOST_MAX
#define MOTOR_STALL_BOOST_MAX       800.0f
#endif
#ifndef MOTOR_STALL_BOOST_SAFE_CAP
#define MOTOR_STALL_BOOST_SAFE_CAP  100.0f
#endif
#if SINGLE_BOARD_LOCAL_DRIVE && !FAN_KICK_DIAG_ENABLE
/* C1(06-06 用户批准): 风扇起转阶梯点动——K4 每按推进 20/35/50 档,点动 2s 自动归零;
 * K2 强停同时灭风扇。全程受 M3PWM 底层 FAN_DUTY_ABS_CAP=50 硬钳兜底(C0)。
 * 目的:实测最低起转占空比(5%≈0.6V 对 130 类有刷电机临界,只能实验定)。 */
static const uint16_t g_fan_ladder[3] = {20u, 35u, 50u};
static uint8_t  g_fan_step = 0u;
#endif
#if SINGLE_BOARD_LOCAL_DRIVE
static uint16_t g_fan_spot_ticks = 0u;   /* >0=kick/点动进行中,2ms tick 递减 */
static uint8_t  g_fan_on = 0u;           /* G1(06-06): 风扇锁存运行态(kick→50保持直至再按K4/K2) */
#endif
#ifndef MOTOR_CMD_EPS
#define MOTOR_CMD_EPS     0.1f
#endif
#ifndef CLOSED_LOOP_REVERSE_ENABLE
#define CLOSED_LOOP_REVERSE_ENABLE 1
#endif

/* ===== IMU 航向辅助 U 弯判定（06-05，遥测观察用，不进控制） =====
 * U 弯 = 累计航向变化 ~180°，是赛道上唯一不可伪造的大角度签名（S 弯两腿
 * 方向相反峰值 |Δyaw| 仅 ~90~110°）。K1 发车记 add_angle 基准，|Δyaw|≥
 * 阈值即锁存 u=1。配合 jc= 甄别"U 弯两端黑误报路口"：u 翻转时刻附近的
 * jc 增量大概率是 U 弯假路口。陀螺零偏已有标定+死区，慢转损耗对 150° 无碍。 */
#ifndef U_TURN_YAW_LATCH_DEG
#define U_TURN_YAW_LATCH_DEG 150.0f
#endif
static float   g_yaw_zero = 0.0f;       /* K1 发车时的 add_angle 基准(rad) */
volatile uint8_t g_u_turn_passed = 0;   /* 1=本次运行已完成 ~180° 航向变化(PID_Controller 读) */
static uint8_t g_jc_at_u = 0;           /* S1(06-06): u 锁存时刻的 jc 基线——sm 触发加固用 */
/* S-mode(06-05 用户方案)：过第二个 Y 后锁存——jc≥2 且 u=1，路标链 Y1→U→Y2 在本路线唯一，
 * 触发点距 S 弯 ~1.5m(@20cps 约 5s 直线)。效果只降速不动转向：target 20→14、
 * floor 70→50、浅弯 cap 50→30(失速裕度 20 不变)。K1 清零 → S 入口直接发车不会触发。
 * TODO: 拱门 ESP 到达信号(队友约定,两个拱门各发一次)落地后可 OR 进此锁存作第二触发源。 */
volatile uint8_t g_s_mode = 0;
/* P9 sm 出口门(06-06 三 agent 合议落码)——sm 此前永不释放,S 过后整圈被钉 14cps。
 * 主锚=拱门2.1 的 ESP 0x30 通知(S 出口后 ~30cm;队友部署确认前天然不触发);
 * 后备=三重与门:里程 Δ≥SM_EXIT_MIN_CNT + 连续 500ms 稳线居中 + 窗内航向静默。
 * 里程刻度按 06-06 实测场推算:870cm(START→Y2) ≈ 352cnt → ~2.47cm/cnt;
 * S 段几何 ~188cm ≈ 76cnt;floor 取 120cnt(~296cm)=1.6×S 长,乱甩通胀也不会在 S 内放行。
 * jc 禁用作锚(乱甩通胀 2→5 实测);yw 只用 500ms 短窗相对值(长程漂移未标定)。 */
#ifndef SM_EXIT_MIN_CNT
#define SM_EXIT_MIN_CNT  120
#endif
static uint8_t  g_s_mode_done = 0;       /* 本次运行 sm 已完整走完(防出门后 jc>基线 立即回锁) */
static int32_t  g_sm_cnt_base = 0;       /* sm 锁存帧的平均编码器绝对计数 */
static uint16_t g_sm_stable_run = 0;     /* 出口后备:连续"found 且居中"tick 计数 */
static float    g_sm_stable_yaw0 = 0.0f; /* 稳线窗起点 yaw(rad) */

/* ===== P1 终点 + P2 雷达避障段(06-06 落码 v1,设计:POST_S_STRATEGY_20260606.md) =====
 * P1: 0x30 id=2(拱门2.2)→ 2s 倒计时 → StopRun(队友沟通记录"ARCH(2)后2s停车";
 *     拱门2.2 距终点仅 ~25cm,滚动距离待实测,必要时加降速档)。
 * P2 触发: sm 走完 + 自 sm 锁存里程 Δ≥RD_ZONE_MIN_CNT + 深丢线 300ms(箱前线尽头)。
 *     误触自愈:弯中丢线达不到三门齐满;真误触也只是停车问雷达→盲走→重捕失败→安全停。
 * P2 流程: BRAKE 清洁停 → QUERY 0x03(500ms 重发,2s 超时) → 0x11(0=左过/1=右过/
 *     2=UNKNOWN→默认左,06-06 拍板) → 盲走四相 OUT(转出30°)/DIAG(斜移)/BACK(回正)/
 *     THRU(直穿+扫线) → REJOIN(重捕余量) → 回循迹。每相 tick 预算,超时=RD_FAIL 停车。
 * 几何(2.47cm/cnt 粗标,P0a 标定后回填): DIAG 24cnt≈60cm(横移≈30cm,对准空闲侧走廊
 *     中心≈箱中线偏 35cm),THRU 28cnt≈70cm(板深),REJOIN 40cnt≈100cm 余量。
 * 合规:决策与重捕全传感器驱动,盲走仅短段(禁预编程约束)。 */
#ifndef RADAR_SEGMENT_ENABLE
#define RADAR_SEGMENT_ENABLE   1
#endif
#define FINISH_STOP_TICKS      1000u   /* 2s @2ms */
#define RD_ZONE_MIN_CNT        380     /* sm锁存→箱前 ~989cm/2.47≈400;四圆出口≈371,裕量薄,P0a 后必校 */
#define RD_LOST_CONFIRM_TICKS  150u    /* 300ms 深丢确认(375 自停前先接管) */
#define RD_BRAKE_SETTLE_TICKS  50u     /* 双轮 |v|<2cps 持续 100ms = 停稳 */
#define RD_BRAKE_BUDGET        1000u   /* 2s 停不稳也发问(防滑行卡相) */
#define RD_QUERY_RESEND        250u    /* 500ms 重发 0x03 */
#define RD_QUERY_BUDGET        1000u   /* 2s 无应答 → 默认左过 */
#define RD_TURN_RAD            0.52f   /* 转出 30° */
#define RD_TURN_TOL            0.10f   /* 航向到位容差 ~5.7° */
#define RD_TURN_SETTLE         25u     /* 到位保持 50ms */
#define RD_PHASE_BUDGET        2500u   /* 每相 5s 预算 */
#define RD_DIAG_CNT            24      /* 斜移段里程 */
#define RD_THRU_CNT            28      /* 直穿段里程 */
#define RD_REJOIN_CNT          40      /* 重捕余量里程 */
#define RD_BLIND_CPS           14.0f   /* 盲走目标(实际受 floor 70 托底≈20cps) */
#define RD_REACQ_TICKS         15u     /* 连续 found 30ms = 重捕成功 */
enum { RD_OFF = 0, RD_BRAKE, RD_QUERY, RD_OUT, RD_DIAG, RD_BACK, RD_THRU, RD_REJOIN, RD_DONE, RD_FAIL };
static uint8_t  g_rd_state = RD_OFF;
static uint16_t g_rd_tick = 0;         /* 当前相计时 */
static uint16_t g_rd_settle = 0;       /* 停稳/航向到位连续计数 */
static int8_t   g_rd_dir = 1;          /* +1=左过(左转出,yaw+), -1=右过 */
static float    g_rd_yaw_base = 0.0f;  /* 停车时航向 = 箱前行进方向 */
static int32_t  g_rd_cnt_mark = 0;     /* 相起点里程 */
static uint16_t g_rd_found_run = 0;    /* 重捕连续 found 计数 */
static uint16_t g_arch_cool = 0;       /* 拱门事件连发去重窗(3s) */
static uint8_t  g_last_arch_id = 255u; /* 遥测:最近一次已消费0x30 id;255=本轮未见 */
static uint16_t g_finish_ticks = 0;    /* P1 倒计时(>0 = 进行中) */
static uint8_t  g_finish_armed = 0;    /* P1 已触发锁存 */

/* ===== 06-07 post-s-review 团队评审落码(SegmentNavigator v2 增量取向) =====
 * 终裁(skeptic《残余风险裁决》):"需重大改后方可上车"——本块落地其放行检查单。
 * 取向依据: Path.c 即"全量段FSM已试过且废弃"的实物(刻度错15×+旧布局),故走增量:
 * g_navseg=只读诊断游标(由既有锚点推进,不夺转向权);里程只做窗不做锚;
 * ESP 事件=到了就抢占、没到当不存在(ESP32_Tick 此前零调用,IsLinkAlive 恒真=假活,禁读)。
 * 行为改动各带编译开关,置 0 即回退现状。 */
#ifndef NAVSEG_T2_FORCE_LATCH
#define NAVSEG_T2_FORCE_LATCH     1   /* T2 兜底:u后里程过上界仍未锁sm→强制锁(治Y2骑岔漏检断粮) */
#endif
#ifndef NAVSEG_U3_DEEP_LATCH
#define NAVSEG_U3_DEEP_LATCH      1   /* U3(06-07 03:30 实测):Y2漏检时用S弧deep结构签名锁sm,早于T2接管 */
#endif
#ifndef NAVSEG_T3_FORCE_RELEASE
#define NAVSEG_T3_FORCE_RELEASE   1   /* T3/P9 兜底:sm里程强制释放(稳线窗死锁/早释放双模通吃) */
#endif
#ifndef NAVSEG_S2_REARM
#define NAVSEG_S2_REARM           1   /* R5:RD_DONE 边沿 re-arm sm 保护 S胶囊②(2×r15) */
#endif
#ifndef NAVSEG_FINISH_DIST_BACKUP
#define NAVSEG_FINISH_DIST_BACKUP 1   /* R2/C-4 挂科级:0x30 未部署时终点里程兜底(防跑完不停冲场);
                                       * 依赖 NAVSEG_S2_REARM(用 g_s2_active 作已过箱锚) */
#endif
#define RD_DEFAULT_DIR            1   /* 雷达 UNKNOWN/2s超时默认过侧:+1=左过(06-06 拍板)。
                                       * map-route 镜像敏感表:全固件唯一硬编码方向——赛道若镜像只翻此处。 */
#define DOWN_FORCE_LATCH_CNT    200   /* T2 上界: u锁存后 ~480cm 轮程(03:24 刻度2.4),最终兜底保持 */
#define SM_DEEP_MIN_CNT          60   /* U3 里程下界: 排除U尾deep(实测Δ<40);S入口实测Δ87~134 */
#define SM_DEEP_CONFIRM_TICKS    25   /* U3 持续门: deep 连续50ms,滤褶皱单帧踢 */
#define SM_EXIT_FORCE_CNT       240   /* T3 上界: sm锁存后 ~593cm(=3.2×S①几何76cnt,<RD下界380不挡T4),P0a 回填 */
#define FINISH_FROM_S2_CNT      110   /* R2 兜底(06-07 skeptic 验收修正): 出箱重捕(≈1929cm)→终点(≈2178cm)
                                       * =下行75+S②124+终段50≈249cm≈101cnt@2.47,取110留9cnt余量。
                                       * 宁小勿大:红区仅100cm深,冲过=出界=Task1循迹分没。
                                       * 旧值200=494cm会在终点后245cm才arm→冲出赛道尽头,形同虚设。P0a 精标后回填。 */
#if NAVSEG_FINISH_DIST_BACKUP && !NAVSEG_S2_REARM
#error "NAVSEG_FINISH_DIST_BACKUP depends on NAVSEG_S2_REARM (g_s2_active is only set by S2_REARM). Fix: enable NAVSEG_S2_REARM, or disable NAVSEG_FINISH_DIST_BACKUP."
#endif
/* g_navseg 只读段游标:遥测 sg= 字段;不参与任何转向/速度裁决(单一裁决者原则,P4)。
 * 命名 NAVSEG_* 避开 Path.h 的 PathSegment_t(SEG_*) 符号域。 */
enum { NAVSEG_START = 0, NAVSEG_U, NAVSEG_SERP1, NAVSEG_AFTER_ARCH1,
       NAVSEG_RADAR, NAVSEG_SERP2, NAVSEG_FINISH };
static uint8_t  g_navseg = NAVSEG_START;
static uint8_t  g_s2_active = 0;       /* R5: S胶囊②域标志;g_s_mode_done 语义自此="S①已完成" */
static int32_t  g_u_cnt_base = 0;      /* T2: u 锁存帧平均编码器计数(强制锁里程锚) */
static uint16_t g_u3_deep_run = 0;     /* U3: u后(里程门内)deep 连续 tick 计数 */
static uint16_t g_launch_grace = 0;    /* F6a: 发车踢腿封顶窗(K1 置 250tick=500ms) */

static float ApplyDeadzone(float duty, float deadzone)
{
    if (duty >  MOTOR_CMD_EPS) return duty + deadzone;
    if (duty < -MOTOR_CMD_EPS) return duty - deadzone;
    return 0.0f;   /* 近零命令直接停，不蠕行 */
}

static float ClampClosedLoopDuty(float duty)
{
    duty = ClampMotorDuty(duty);
#if !CLOSED_LOOP_REVERSE_ENABLE
    if (duty < MOTOR_CMD_EPS) return 0.0f;
#endif
    return duty;
}

/* 死区前馈后的最终安全上限：= PID 上限 + 最大死区，使 PID 满输出叠加死区后不被砍。
 * 仍兜一个绝对天花板防止异常值窜车。 */
#ifndef MOTOR_DUTY_FINAL_CAP
#define MOTOR_DUTY_FINAL_CAP  1990.0f
#endif
static float ClampMotorDutyFinal(float duty)
{
    if (duty >  MOTOR_DUTY_FINAL_CAP) return  MOTOR_DUTY_FINAL_CAP;
    if (duty < -MOTOR_DUTY_FINAL_CAP) return -MOTOR_DUTY_FINAL_CAP;
    return duty;
}

/* 下板经 ENC_FEEDBACK 帧回传的编码器计数（调试/里程备用） */
volatile int32_t g_link_cnt_l = 0;
volatile int32_t g_link_cnt_r = 0;
/* 新速度样本就绪标志：OnEncFeedback 算出新窗口速度时置 1，速度环消费后清 0。
 * 让速度环按反馈实际刷新率(≈20Hz)闭环，而非主控制环 500Hz 重复积分陈旧值。 */
volatile uint8_t g_speed_sample_ready = 0;
static uint8_t g_prev_racing = 0;

/* 速度环控制器（定义于 PID_Controller.c），起步时给其初始输出做前馈 */
extern SpeedPID_Controller_t g_speed_pid;
extern PositionPID_Controller_t g_position_pid;
/* 位置环/速度环计算出的带符号电机目标（定义于 PID_Controller.c） */
extern volatile float g_motor_target_l;
extern volatile float g_motor_target_r;
static int16_t g_sent_motor_l = 0;
static int16_t g_sent_motor_r = 0;

/* 下板链路监视：收到 ENC_FEEDBACK 心跳清零；丢失超时则解除运行防窜车
 * 单板:无远端心跳。g_link_alive 保留供 OLED 状态行显示(恒 0=LINK:--)；
 * g_link_lost_ticks 仅双层板看门狗用，单板下不编译以免未用变量告警。 */
static volatile uint8_t g_link_alive = 0;
#if !SINGLE_BOARD_LOCAL_DRIVE
static uint32_t g_link_lost_ticks = 0;
#endif
/* 心跳丢失阈值：下板 100Hz 回传，连续 50 个控制 tick(=100ms) 无心跳判为断链 */
#ifndef LINK_LOST_TICKS
#define LINK_LOST_TICKS 50u
#endif

/* 大端字节序解码辅助 */
#define PROTO_RD_U16(buf, i) ((int16_t)((uint16_t)(buf)[(i)] << 8 | (buf)[(i)+1]))

/* ==== 开环测试模式 ====
 * 置 1：按键直接给固定占空比、不跑 PID，用来验证"占空比→编码器读数"是否单调可信。
 *   K1=12%(1200)  K2=15%(1500)  K3=18%(1800)  K4=立即停。
 *   看 OLED 第3行 L/R 速度是否随占空比阶梯上升 → 编码器反馈可信，PID 才能闭环。
 * 置 0：恢复正常 K1 启动/K2 停/K3 复位的循迹模式。 */
#ifndef OPENLOOP_TEST_ENABLE
#define OPENLOOP_TEST_ENABLE 0
#endif
#if OPENLOOP_TEST_ENABLE
#define OPENLOOP_DUTY_12PCT  1200  /* 跨过左右电机起步死区 */
#define OPENLOOP_DUTY_15PCT  1500
#define OPENLOOP_DUTY_18PCT  1800
static int16_t g_openloop_duty   = 0;   /* 当前开环测试占空比(带符号，正=前进) */
static uint8_t g_openloop_active = 0;   /* 1=开环测试运行中 */
#endif

#ifndef OLED_TELEMETRY_ENABLE
#define OLED_TELEMETRY_ENABLE 1
#endif
#ifndef OLED_TELEMETRY_PERIOD_TICKS
#define OLED_TELEMETRY_PERIOD_TICKS 150u
#endif
/* 调试串口遥测：旧双层板经 USART3 (PB10/PB11) 发到 PC；
 * 2合1上层板(TELEMETRY_ON_USART2=1)经 USART2 (PA2/PA3→J5"通信"口) 发到 PC。
 * 与 OLED 同一周期（150 ticks ≈ 300ms），可按需关掉。 */
#ifndef DEBUG_OUT_TELEMETRY_ENABLE
#define DEBUG_OUT_TELEMETRY_ENABLE 1
#endif
/* 遥测出口路由：2合1上层板唯一串口引出=J5(USART2)，与下板协议帧共口；
 * 旧板走独立 USART3。编译期定死，零运行时开销。 */
#if TELEMETRY_ON_USART2
#define Debug_SendBuf(buf, len)  Uart2_SendBuf((buf), (len))
#else
#define Debug_SendBuf(buf, len)  Uart3_SendBuf((buf), (len))
#endif
#ifndef SENSOR_DEBUG_MIN_SPAN
#define SENSOR_DEBUG_MIN_SPAN 80u
#endif
#ifndef SENSOR_DEBUG_THRESHOLD_PERCENT
#define SENSOR_DEBUG_THRESHOLD_PERCENT 35u
#endif

static uint8_t g_sensor_min_index = 0;
static uint8_t g_sensor_max_index = 0;
static uint16_t g_sensor_min_value = 0;
static uint16_t g_sensor_max_value = 0;
static uint16_t g_sensor_span = 0;
static uint16_t g_sensor_low_mask = 0;
static uint16_t g_sensor_high_mask = 0;
static int16_t g_sensor_low_pos10 = -1;
static int16_t g_sensor_high_pos10 = -1;

static int16_t CalculateSensorDebugPos10(uint16_t mask, uint8_t low_is_target)
{
    uint8_t i;
    uint32_t weight_sum = 0;
    uint32_t position_sum = 0;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint32_t weight;
        if ((mask & (uint16_t)(1u << i)) == 0u)
        {
            continue;
        }
        if (low_is_target)
        {
            weight = (uint32_t)g_sensor_max_value - (uint32_t)g_line_sensor_values[i];
        }
        else
        {
            weight = (uint32_t)g_line_sensor_values[i] - (uint32_t)g_sensor_min_value;
        }
        weight++;
        weight_sum += weight;
        position_sum += weight * (uint32_t)i * 10u;
    }
    if (weight_sum == 0u)
    {
        return -1;
    }
    return (int16_t)((position_sum + weight_sum / 2u) / weight_sum);
}

static void UpdateSensorDebugSnapshot(void)
{
    uint8_t i;
    uint16_t low_threshold;
    uint16_t high_threshold;
    g_sensor_min_value = 0xFFFFu;
    g_sensor_max_value = 0u;
    g_sensor_low_mask = 0u;
    g_sensor_high_mask = 0u;
    g_sensor_low_pos10 = -1;
    g_sensor_high_pos10 = -1;
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value < g_sensor_min_value)
        {
            g_sensor_min_value = value;
            g_sensor_min_index = i;
        }
        if (value > g_sensor_max_value)
        {
            g_sensor_max_value = value;
            g_sensor_max_index = i;
        }
    }
    g_sensor_span = g_sensor_max_value - g_sensor_min_value;
    if (g_sensor_span < SENSOR_DEBUG_MIN_SPAN)
    {
        return;
    }
    low_threshold = g_sensor_min_value + (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    high_threshold = g_sensor_max_value - (uint16_t)(((uint32_t)g_sensor_span * SENSOR_DEBUG_THRESHOLD_PERCENT) / 100u);
    for (i = 0; i < SENSOR_COUNT; i++)
    {
        uint16_t value = (uint16_t)g_line_sensor_values[i];
        if (value <= low_threshold)
        {
            g_sensor_low_mask |= (uint16_t)(1u << i);
        }
        if (value >= high_threshold)
        {
            g_sensor_high_mask |= (uint16_t)(1u << i);
        }
    }
    g_sensor_low_pos10 = CalculateSensorDebugPos10(g_sensor_low_mask, 1u);
    g_sensor_high_pos10 = CalculateSensorDebugPos10(g_sensor_high_mask, 0u);
}

#ifndef LINE_LOST_STOP_TICKS
#define LINE_LOST_STOP_TICKS 500u  /* 1s @ 500Hz control tick */
#endif

static void StopRun(void)
{
    is_racing = 0;
    lose_time = 0;
    g_stall_boost_l = 0.0f;
    g_stall_boost_r = 0.0f;
}

#if OPENLOOP_TEST_ENABLE
static void OpenLoop_Set(uint8_t active, int16_t duty)
{
    g_openloop_active = active;
    g_openloop_duty = active ? duty : 0;
    g_motor_target_l = (float)g_openloop_duty;
    g_motor_target_r = (float)g_openloop_duty;
    RGB_SetColor(active ? RGB_COLOR_G : RGB_COLOR_R);
}

static void OpenLoop_ShowStatus(void)
{
    int spd_l = (int)speed_left;
    int spd_r = (int)speed_right;
    if (spd_l > 999) spd_l = 999;
    if (spd_l < -999) spd_l = -999;
    if (spd_r > 999) spd_r = 999;
    if (spd_r < -999) spd_r = -999;

    OLED_ClearLine(1);
    OLED_ShowString(1, 1, g_openloop_active ? "OPENLOOP RUN    " : "OPENLOOP STOP   ");

    OLED_ClearLine(2);
    OLED_ShowString(2, 1, "K:");
    OLED_ShowChar(2, 3, Key_GetState(KEY_K1) ? '1' : '0');
    OLED_ShowChar(2, 4, Key_GetState(KEY_K2) ? '2' : '0');
    OLED_ShowChar(2, 5, Key_GetState(KEY_K3) ? '3' : '0');
    OLED_ShowChar(2, 6, Key_GetState(KEY_K4) ? '4' : '0');
    OLED_ShowString(2, 8, "D:");
    OLED_ShowSignedNum(2, 10, g_openloop_duty, 4);

    OLED_ClearLine(3);
    OLED_ShowString(3, 1, "L:");
    OLED_ShowSignedNum(3, 3, spd_l, 3);
    OLED_ShowString(3, 8, "R:");
    OLED_ShowSignedNum(3, 10, spd_r, 3);

    OLED_ClearLine(4);
    OLED_ShowString(4, 1, g_link_alive ? "LINK:OK " : "LINK:-- ");
    OLED_ShowString(4, 9, g_openloop_active ? "A:1" : "A:0");
}

static void OpenLoop_HandleKeys(void)
{
    uint8_t next_active = g_openloop_active;
    int16_t next_duty = g_openloop_duty;

    if (Key_GetState(KEY_K4))
    {
        next_active = 0;
        next_duty = 0;
    }
    else if (Key_GetState(KEY_K3))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_18PCT;
    }
    else if (Key_GetState(KEY_K2))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_15PCT;
    }
    else if (Key_GetState(KEY_K1))
    {
        next_active = 1;
        next_duty = OPENLOOP_DUTY_12PCT;
    }

    if (next_active != g_openloop_active || next_duty != g_openloop_duty)
    {
        OpenLoop_Set(next_active, next_duty);
        OpenLoop_ShowStatus();
    }
}
#endif

// ========== ESP32 协议回调（保留兼容） ==========
static void OnLoraStop(const ProtoFrame_t *f)
{
    (void)f;
    StopRun();
}

static void OnRadarDist(const ProtoFrame_t *f)
{
    if (f->len < 2) return;
    radar_distance_cm = (uint16_t)PROTO_RD_U16(f->payload, 0);
}

// 下板 -> 上板 编码器反馈帧：更新轮速（供本地速度环使用）
// 速度窗口：下板 100Hz(10ms)回传，单帧增量太小(≈0.6)且量化严重。
// 用 SPEED_WIN_MS 的滑动窗口累计计数，再按实测耗时换算成"计数/秒"，
// 量化噪声降到单帧的 1/(窗口帧数)，同时仍是物理上可解释的速度量纲。
#ifndef SPEED_WIN_MS
#define SPEED_WIN_MS 250u
#endif
#if !SINGLE_BOARD_LOCAL_DRIVE
static void OnEncFeedback(const ProtoFrame_t *f)
{
    int32_t cnt_l, cnt_r, dl, dr;
    static uint8_t cnt_inited = 0;
    static int32_t last_cnt_l = 0, last_cnt_r = 0;
    static int32_t vel_accum_l = 0, vel_accum_r = 0;
    static uint32_t vel_t0 = 0;

    if (f->len < PROTO_ENC_FEEDBACK_LEN) return;

    cnt_l = (int32_t)(((uint32_t)f->payload[4] << 24) | ((uint32_t)f->payload[5] << 16) |
                      ((uint32_t)f->payload[6] << 8)  |  (uint32_t)f->payload[7]);
    cnt_r = (int32_t)(((uint32_t)f->payload[8] << 24) | ((uint32_t)f->payload[9] << 16) |
                      ((uint32_t)f->payload[10] << 8) |  (uint32_t)f->payload[11]);
    g_link_cnt_l = cnt_l;
    g_link_cnt_r = cnt_r;

    if (!cnt_inited)
    {
        cnt_inited = 1;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        vel_accum_l = 0;
        vel_accum_r = 0;
        vel_t0 = Millis_Get();
        speed_left = 0;
        speed_right = 0;
    }
    else
    {
        /* 本帧原始增量：里程按它累积(每帧恰好计一次，单位=计数) */
        dl = cnt_l - last_cnt_l;
        dr = cnt_r - last_cnt_r;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        Path_UpdateOdometer(dl, dr);

        /* 速度窗口累计，到窗口期再换算成"计数/秒"，降低低速量化噪声 */
        vel_accum_l += dl;
        vel_accum_r += dr;
        {
            uint32_t now = Millis_Get();
            uint32_t dt  = now - vel_t0;          /* 实测耗时(ms)，毫秒回绕安全 */
            if (dt >= SPEED_WIN_MS)
            {
                if (dt == 0) dt = 1;  /* 防御：SysTick 未启动时避免除零 */
                speed_left  = (int16_t)((vel_accum_l * 1000) / (int32_t)dt);
                speed_right = (int16_t)((vel_accum_r * 1000) / (int32_t)dt);
                vel_accum_l = 0;
                vel_accum_r = 0;
                vel_t0 = now;
                g_speed_sample_ready = 1;     /* 通知速度环：有新样本可闭环 */
            }
        }
    }

    g_link_alive = 1;          /* 收到下板心跳 */
    g_link_lost_ticks = 0;
}

// 下板 -> 上板 链路复位：下板刚开机/重烧，必须解除运行防止用陈旧状态窜车
static void OnLinkReset(const ProtoFrame_t *f)
{
    (void)f;
    StopRun();                 /* is_racing=0，停止下发使能 */
    SpeedPID_Reset(&g_speed_pid);
    PositionPID_Reset(&g_position_pid);
    g_link_alive = 1;
    g_link_lost_ticks = 0;
    RGB_SetColor(RGB_COLOR_R);
    OLED_ShowString(1, 1, "LOWER RESET     ");
}
#endif  /* !SINGLE_BOARD_LOCAL_DRIVE : OnEncFeedback/OnLinkReset 仅双层板用 */

#if SINGLE_BOARD_LOCAL_DRIVE
/* 单板电机输出：把双层板下板 OnMotorCmd→ApplyMotorDuty 的本地驱动逻辑搬来，
 * 带符号占空比拆成方向+幅值，幅值钳到 MOTOR_DUTY_MAX(底层 Motor_SetSpeed 再钳 SAFE_MAX)。
 * 与 lower-pid 执行器等价，duty 口径不变(原串口对接的 sent_motor_l/r 直接喂)。 */
static void ApplyMotorDutyLocal(uint8_t motor_id, int16_t duty_signed)
{
    uint8_t direction;
    uint16_t duty;
    if (duty_signed < 0)
    {
        direction = MOTOR_DIR_BACKWARD;
        duty = (uint16_t)(-(int32_t)duty_signed);
    }
    else
    {
        direction = MOTOR_DIR_FORWARD;
        duty = (uint16_t)duty_signed;
    }
    if (duty > MOTOR_DUTY_MAX) duty = MOTOR_DUTY_MAX;
    Motor_SetDirection(motor_id, direction);
    Motor_SetSpeed(motor_id, duty);
}
#endif

#if SINGLE_BOARD_LOCAL_DRIVE
/* 单板编码器本地化：把双层板 OnEncFeedback 的"计数增量→cnt/s 窗口换算"逻辑原样搬来，
 * 数据源由协议帧 payload 改为本地 ABEncoder 维护的 left/right_encoder_cnt。
 * 在主控 tick 调用(非 ISR)：先 ABEncoder_UpdateSpeed() 累计 encoder_cnt(lower-pid 原口径，
 * speed_left/right 此刻是 2ms 增量)，随后本函数把 speed_left/right 覆写为 cnt/s——
 * 同一 tick 内顺序确定、无竞态，tick 结束时 speed_left/right 即上层速度环期望的 cnt/s。
 * 窗口长度 SPEED_WIN_MS / 换算公式 / 里程接口全部沿用现值，零参数改动。 */
static void LocalEncoder_Update(void)
{
    int32_t cnt_l, cnt_r, dl, dr;
    static uint8_t cnt_inited = 0;
    static int32_t last_cnt_l = 0, last_cnt_r = 0;
    static int32_t vel_accum_l = 0, vel_accum_r = 0;
    static uint32_t vel_t0 = 0;
    /* 窗口换算出的 cnt/s 保持值：双层板 OnEncFeedback 里 speed_left/right 在窗口间
     * 自然保持上次值(无人覆写)。单板下 ABEncoder_UpdateSpeed 每 tick 把 speed_* 写成
     * 2ms 增量，故必须用 g_cps_* 缓存窗口值并每 tick 回写，复刻"保持上次 cnt/s"语义。 */
    static int16_t g_cps_l = 0, g_cps_r = 0;

    ABEncoder_UpdateSpeed();          /* lower-pid 口径：维护 encoder_cnt(+写 2ms 增量 speed_*) */

    cnt_l = left_encoder_cnt;
    cnt_r = right_encoder_cnt;
    g_link_cnt_l = cnt_l;
    g_link_cnt_r = cnt_r;

    if (!cnt_inited)
    {
        cnt_inited = 1;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        vel_accum_l = 0;
        vel_accum_r = 0;
        vel_t0 = Millis_Get();
        g_cps_l = 0;
        g_cps_r = 0;
    }
    else
    {
        dl = cnt_l - last_cnt_l;
        dr = cnt_r - last_cnt_r;
        last_cnt_l = cnt_l;
        last_cnt_r = cnt_r;
        Path_UpdateOdometer(dl, dr);

        vel_accum_l += dl;
        vel_accum_r += dr;
        {
            uint32_t now = Millis_Get();
            uint32_t dt  = now - vel_t0;
            if (dt >= SPEED_WIN_MS)
            {
                if (dt == 0) dt = 1;
                g_cps_l = (int16_t)((vel_accum_l * 1000) / (int32_t)dt);
                g_cps_r = (int16_t)((vel_accum_r * 1000) / (int32_t)dt);
                vel_accum_l = 0;
                vel_accum_r = 0;
                vel_t0 = now;
                g_speed_sample_ready = 1;
            }
        }
    }

    /* 每 tick 回写：覆盖 ABEncoder_UpdateSpeed 刚写的 2ms 增量，使下游(速度环/死区/
     * 遥测)始终读到 cnt/s 量纲。与双层板 speed_* 语义逐位等价。 */
    speed_left  = g_cps_l;
    speed_right = g_cps_r;
}
#endif

int main(void)
{
    // 外设初始化（传感/显示/交互 + 本地控制算法，电机PWM输出经串口转发）
    RGB_Init();
    OLED_Init();
    g_imu_init_ok = MPU6050_Init();
    g_imu_who_id  = MPU6050_ReadID();
    SysTick_Init();
    BlackPoint_Finder_Init();
#if SINGLE_BOARD_LOCAL_DRIVE
    /* 风扇 M3PWM 用 TIM2 部分重映射2(CH3=PB10/CH4=PB11)。必须在 LineSensor_Init 之前，
     * 由 LineSensor_Init 最后把 PB10 配回 IPU 输入(S7)；M3PWM 只初始化 CH4，不碰 CH3。 */
    Motor_Init();              /* TIM1 PA8~12 电机驱动 */
    Motor_StopAll();
    Motor_Disable();           /* 开机电机禁用，K1 发车再使能 */
    ABEncoder_Init();          /* TIM3 PA6/7 右轮 + TIM4 PB6/7 左轮 */
    M3PWM_Init();
    /* C2(06-06): 17kHz@2/3.5/5% 三档实测不起转(有声/无温升)。降频 2kHz——电流纹波
     * 峰值更高破粘滑,平均占空比不变(仍受 ABS_CAP=50 硬钳);点动期间可闻啸叫=正常。 */
    M3PWM_SetFrequency(2000);
    M3PWM_Start();
    M3PWM_SetDutyCycle(0);     /* 风扇开机 0%，底层 ABS_CAP 兜底 */
#endif
    LineSensor_Init();
    Key_Scan_Init();
    Uart2_Init(115200);
#if USART3_DEBUG_ON_PB10
    Uart3_Init(115200);       /* 调试串口 PB10/PB11 → PC */
#endif
    TelemetryScreen_Init();
    PID_Init();
    Path_Init();

    (void)g_imu_init_ok;
    (void)g_imu_who_id;
    (void)g_sensor_low_pos10;
    (void)g_sensor_high_pos10;

#if SINGLE_BOARD_LOCAL_DRIVE
    /* 单板:无远端下板，仅保留 ESP/雷达回调(物理链路另说)；电机/编码器走本地。
     * 不注册 MOTOR_CMD/ENC_FEEDBACK/LINK_RESET，不发 LinkReset 握手。 */
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);
#if ESP32_ON_USART2
    ESP32_Comm_Init();    /* A5 5A 帧解析状态复位；物理 UART 即上面的 Uart2_Init(115200) */
#endif
    is_racing = 0;
#else
    // 板间/ESP32 协议初始化
    Proto_Init();
    Proto_RegisterHandler(PROTO_CMD_LORA_STOP, OnLoraStop);
    Proto_RegisterHandler(PROTO_CMD_RADAR_DIST, OnRadarDist);
    Proto_RegisterHandler(PROTO_CMD_ENC_FEEDBACK, OnEncFeedback);
    Proto_RegisterHandler(PROTO_CMD_LINK_RESET, OnLinkReset);

    /* 上板开机/重烧：通告下板复位到安全态（停车下电、清里程），
     * 避免上板重启瞬间下板仍按陈旧指令运行。多发几帧防丢包。 */
    is_racing = 0;
    Proto_SendLinkReset();
    Delay_ms(5);
    Proto_SendLinkReset();
#endif

#if OLED_TELEMETRY_ENABLE
    uint8_t oled_due = 1;
    uint32_t oled_last_tick = add_angle_num;
#endif

    while (1)
    {
        // ---- 主循环控制环：由 SysTick 标志位驱动（2ms） ----
        if (g_control_tick)
        {
            g_control_tick = 0;

#if SINGLE_BOARD_LOCAL_DRIVE
            /* C1/G1: 风扇相位倒计时——kick(150×200ms)结束:锁存态→回落 50 持续运行(G1 跑圈
             * 构型,直至 K4 再按/K2);非锁存(点动/诊断)→归零。底层 ABS_CAP=50 硬钳兜底。 */
            if (g_fan_spot_ticks > 0u)
            {
                g_fan_spot_ticks--;
                if (g_fan_spot_ticks == 0u) M3PWM_SetDutyCycle(g_fan_on ? 50 : 0);
            }
#endif

#if SINGLE_BOARD_LOCAL_DRIVE
            /* 单板:本地编码器采样+换算(替代远端 OnEncFeedback 心跳)。每 tick 先更新，
             * 使下方速度环/死区/遥测读到的 speed_left/right 为本 tick 的最新 cnt/s。 */
            LocalEncoder_Update();
#endif

            // 7路灰度 + PA0电池电压轮询采样
            LineSensor_SampleAll();
            UpdateSensorDebugSnapshot();
#if ESP32_ON_USART2
            /* R12(06-07 评审): 链路计时器接线——此前全工程零调用,IsLinkAlive 恒真(假活)。
             * 暂无消费者(评审定:仲裁纯事件抢占,不读链路活性);接线使计时真实,供日后使用。 */
            ESP32_Tick();
#endif

            // 黑线识别 -> 循迹位置
            BlackPoint_Finder_Search(g_line_sensor_values, &result_BlackPoint);
            if (result_BlackPoint.found)
            {
                position_get = (int16_t)(result_BlackPoint.precise_position * 10.0f);
                if (lose_time > 0) lose_time--;
            }
            else if (PID_GetNavOverride() == NAV_OVERRIDE_NONE)
            {
                lose_time++;
                if (lose_time > LINE_LOST_STOP_TICKS)
                {
                    lose_time = LINE_LOST_STOP_TICKS;
                    StopRun();             /* 丢线超过 1s：本地停车 */
                }
            }
            else
            {
                /* R1/C-8(06-07 评审,挂科级): NAV 覆盖期(雷达盲走过箱无线 ≥3.5s)冻结
                 * 主环丢线杀手——否则 RD 段 1s 必自杀,雷达段结构上跑不完。
                 * PID 内部 375 自停已有覆盖旁路(PID:589),这里是独立第二杀手,同样要让位。
                 * 覆盖解除(RD_DONE/RD_FAIL)后从 0 恢复计数,语义与重捕一致。 */
                lose_time = 0;
            }

            // racing 上升沿：复位 Path/PID 并给速度环 15% 前馈起步
            if (is_racing && !g_prev_racing)
            {
                Path_StartRace();
                lose_time = 0;
                g_speed_pid.last_output = START_DUTY_15PCT;
            }
            else if (!is_racing && g_prev_racing)
            {
                Path_StopRace();
            }
            g_prev_racing = is_racing;

#if !SINGLE_BOARD_LOCAL_DRIVE
            // 下板心跳监视：运行中若下板断链(掉电/重烧/线松)，解除运行防窜车
            if (g_link_alive)
            {
                g_link_lost_ticks++;
                if (g_link_lost_ticks > LINK_LOST_TICKS)
                {
                    g_link_lost_ticks = LINK_LOST_TICKS + 1u;
                    if (is_racing)
                    {
                        StopRun();
                        SpeedPID_Reset(&g_speed_pid);
                        PositionPID_Reset(&g_position_pid);
                        RGB_SetColor(RGB_COLOR_R);
                        OLED_ShowString(1, 1, "LINK LOST STOP  ");
                    }
                    g_link_alive = 0;   /* 等待下板心跳/LINK_RESET 重新置位 */
                }
            }
#endif

            // U 弯锁存（R2，06-05 审查三方确认）：原埋在调试遥测 #if 块内且 300ms 节流——
            // 关调试串口的构建会让 S-mode 静默失效。移到控制 tick 每帧评估，遥测只读。
            {
                float dyaw_latch = (add_angle - g_yaw_zero) * 57.2957795f;
                if (dyaw_latch < 0.0f) dyaw_latch = -dyaw_latch;
                if (dyaw_latch >= U_TURN_YAW_LATCH_DEG && !g_u_turn_passed)
                {
                    g_u_turn_passed = 1;
                    /* S1: 记录 u 锁存时刻的 jc 基线，供"U 后新增路口"判据 */
                    g_jc_at_u = result_BlackPoint.junction_pass_count;
                    /* T2(06-07 评审): 同帧记里程锚,供强制锁 sm 的上界判据 */
                    g_u_cnt_base = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
                }
            }

#if NAVSEG_U3_DEEP_LATCH
            /* U3(06-07 03:30 实测): Y2 骑岔漏检(jc 全程=1)致 sm 迟到 T2(Δ201)才锁,
             * S① 前两弧以 25cps+主 HOLD 裸跑。S 弧必触 deep——用结构签名提前接管:
             * u 后 Δ≥60cnt(排除 U 尾 deep,实测 Δ<40;S 入口实测 Δ87~134)且 deep 持续
             * 50ms → 锁 sm。优先级: jc 自然判据 > U3 结构签名 > T2 里程上界(签名>里程)。 */
            if (is_racing && g_u_turn_passed && !g_s_mode && !g_s_mode_done &&
                PID_GetDeepTurnMode() &&
                ((int32_t)((g_link_cnt_l + g_link_cnt_r) / 2) - g_u_cnt_base) >= SM_DEEP_MIN_CNT)
            {
                if (g_u3_deep_run < 0xFFFFu) g_u3_deep_run++;
            }
            else
            {
                g_u3_deep_run = 0;
            }
#endif

            // S-mode 锁存：jc≥2 且 u=1 = 已过第二个 Y，S 弯在前方 ~1.5m
            // S1(06-06): 触发加固——19:03 实测发车没压起跑线时 jc 只到 1(Y2 给的)，
            // 旧判据 jc≥2 凑不满 → sm 不触发 → A1/A2 全程未上场。加 OR 支路:
            // "u 锁存后 jc 有新增"(U 后第一个路口=Y2，对摆位鲁棒)。两判据任一命中即锁存。
            if (!g_s_mode && !g_s_mode_done && is_racing && g_u_turn_passed &&
                (result_BlackPoint.junction_pass_count >= 2u ||
                 result_BlackPoint.junction_pass_count > g_jc_at_u))
            {
                g_s_mode = 1;
                g_sm_cnt_base = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);  /* P9: 里程基准 */
                g_sm_stable_run = 0;
            }
#if NAVSEG_U3_DEEP_LATCH
            else if (!g_s_mode && !g_s_mode_done && is_racing && g_u_turn_passed &&
                     g_u3_deep_run >= SM_DEEP_CONFIRM_TICKS)
            {
                g_s_mode = 1;
                g_sm_cnt_base = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
                g_sm_stable_run = 0;
                OLED_ShowString(1, 1, "SM DEEP LATCH   ");
            }
#endif
#if NAVSEG_T2_FORCE_LATCH
            /* T2 兜底(06-07 评审): Y2 骑岔漏检(D1 双段图样,19:03/06-06晚两度实证)→jc 不增
             * →sm 断粮→S① 以 25cps 裸跑必丢。u 锁存后里程过上界仍未锁 → 强制锁,
             * 保证 S① 配方(14cps/A1拖刹/A2锁向)必上场。锁早=下行段提前降速(无害);
             * 依赖 u 前置,U 弯乱甩不误触(u 未锁不计里程)。 */
            else if (!g_s_mode && !g_s_mode_done && is_racing && g_u_turn_passed &&
                     ((int32_t)((g_link_cnt_l + g_link_cnt_r) / 2) - g_u_cnt_base) >= DOWN_FORCE_LATCH_CNT)
            {
                g_s_mode = 1;
                g_sm_cnt_base = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
                g_sm_stable_run = 0;
                OLED_ShowString(1, 1, "SM FORCED LATCH ");
            }
#endif

            /* ===== 0x30 拱门事件统一消费(P9 主锚 / P1 终点) =====
             * 每 tick 至多取一次;冷却窗(3s)吸收队友同事件连发(2~3帧,≥50ms 间隔)——
             * 防 sm 释放后的残帧被终点逻辑误食(空 payload 时 id 无法区分两拱门)。 */
            {
                uint8_t arch_id = 0;
                uint8_t arch_evt = ESP32_GetArchPassed(&arch_id);
                if (g_arch_cool > 0u) { g_arch_cool--; arch_evt = 0; }   /* 冷却期:事件吸收丢弃 */
                if (arch_evt && is_racing)
                {
                    g_last_arch_id = arch_id;
                    /* R5(06-07 评审): S② 域(g_s2_active)对释放支路关闭——此时已过拱门2.1,
                     * 任何拱门事件只可能是拱门2.2,落到下方终点支路(id=2 或空payload+done)。 */
                    if (g_s_mode && arch_id <= 1u && !g_s2_active)
                    {
                        g_s_mode = 0;          /* P9 主锚:拱门2.1(id=1;空 payload 记 0 也认) */
                        g_s_mode_done = 1;
                        g_arch_cool = 1500u;
                    }
                    else if (!g_finish_armed &&
                             (arch_id == 2u || (arch_id == 0u && g_s_mode_done)))
                    {
                        /* P1 终点:拱门2.2——显式 id=2;或空 payload 但 sm 已走完(=第二次拱门事件)。
                         * 2s 倒计时后 StopRun(队友记录;终点线距拱门 ~25cm,滚动距离待实测)。 */
                        g_finish_armed = 1;
                        g_finish_ticks = FINISH_STOP_TICKS;
                        g_arch_cool = 1500u;
                        OLED_ShowString(1, 1, "FINISH IN 2S    ");
                    }
                }
            }

            /* P9 sm 出口门(后备三重与门;主锚已并入上方统一消费)。
             * 出门即置 done,sm 本次运行不再回锁;速度由 H3 三段律自动回 25(u=1,!sm)。 */
            if (g_s_mode && is_racing)
            {
                {
                    int32_t sm_dcnt = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2) - g_sm_cnt_base;
#if NAVSEG_T3_FORCE_RELEASE
                    /* T3 强制门(06-07 评审): 后备稳线窗有双故障模式——方块阵密路口帧反复
                     * 重置窗(死锁:sm 永钉 14cps 且 T4 RD-arm 断粮) / 273cm 长直提前凑齐
                     * (早放,本身无害,S② 保护由 re-arm 另管)。里程上界强制释放双模通吃,
                     * 保证 sm_done 必置位。S②(re-arm 后)同走此门,S② 短(~50cnt)稳线门
                     * 通常先放,此门是最后保险。 */
                    if (sm_dcnt >= SM_EXIT_FORCE_CNT)
                    {
                        g_s_mode = 0;
                        g_s_mode_done = 1;
                        OLED_ShowString(1, 1, "SM FORCED EXIT  ");
                    }
                    else
#endif
                    if (!result_BlackPoint.found || result_BlackPoint.is_junction ||
                        position_get < 10 || position_get > 50)
                    {
                        g_sm_stable_run = 0;   /* 丢线/路口/贴边 → 稳线窗重开 */
                    }
                    else
                    {
                        if (g_sm_stable_run == 0u) g_sm_stable_yaw0 = add_angle;
                        if (g_sm_stable_run < 0xFFFFu) g_sm_stable_run++;
                        {
                            float sm_dy = add_angle - g_sm_stable_yaw0;
                            if (sm_dy < 0.0f) sm_dy = -sm_dy;
                            if (sm_dy > 0.30f)
                            {
                                g_sm_stable_run = 0;   /* 窗内转向(S 弯内必触)→ 重开窗 */
                            }
                            else if (g_sm_stable_run >= 250u && sm_dcnt >= SM_EXIT_MIN_CNT)
                            {
                                g_s_mode = 0;
                                g_s_mode_done = 1;
                            }
                        }
                    }
                }
            }

#if RADAR_SEGMENT_ENABLE
            /* ===== P2 雷达避障段状态机(设计/几何见定义处注释) ===== */
            if (is_racing)
            {
                int32_t rd_avg = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
                switch (g_rd_state)
                {
                case RD_OFF:
                    if (g_s_mode_done &&
                        (rd_avg - g_sm_cnt_base) >= RD_ZONE_MIN_CNT &&
                        PID_GetLineLostTicks() >= RD_LOST_CONFIRM_TICKS)
                    {
                        g_rd_state = RD_BRAKE; g_rd_tick = 0; g_rd_settle = 0;
                        PID_SetNavOverride(NAV_OVERRIDE_HOLD, 0.0f, 0.0f);
                        OLED_ShowString(1, 1, "RD BRAKE        ");
                    }
                    break;
                case RD_BRAKE:
                    g_rd_tick++;
                    if (speed_left < 2 && speed_left > -2 && speed_right < 2 && speed_right > -2)
                        g_rd_settle++;
                    else
                        g_rd_settle = 0;
                    if (g_rd_settle >= RD_BRAKE_SETTLE_TICKS || g_rd_tick >= RD_BRAKE_BUDGET)
                    {
                        g_rd_yaw_base = add_angle;     /* 箱前行进方向 = 盲走基准 */
                        ESP32_SendAtPosition();
                        g_rd_state = RD_QUERY; g_rd_tick = 0;
                        OLED_ShowString(1, 1, "RD QUERY        ");
                    }
                    break;
                case RD_QUERY:
                {
                    ESP32_Decision_t rd_dec; uint8_t rd_pres; uint8_t rd_go = 0;
                    g_rd_tick++;
                    if (ESP32_GetDecision(&rd_dec, &rd_pres))
                    {
                        (void)rd_pres;
                        /* 显式决策=车体系固定映射(GO_RIGHT→右过-1/GO_LEFT→左过+1,不随镜像翻);
                         * 仅 UNKNOWN 走默认宏。待队友确认 GO_* 坐标系(车头参考 vs 场地参考,
                         * 若为场地参考则此处消费需随镜像翻——开放问题,见评审日志)。 */
                        g_rd_dir = (rd_dec == ESP32_GO_RIGHT) ? -1
                                 : (rd_dec == ESP32_GO_LEFT)  ? 1 : RD_DEFAULT_DIR;
                        rd_go = 1;
                    }
                    else if (g_rd_tick >= RD_QUERY_BUDGET)
                    {
                        g_rd_dir = RD_DEFAULT_DIR;   /* 2s 超时 → 默认过侧(06-06 拍板左) */
                        rd_go = 1;
                    }
                    else if ((g_rd_tick % RD_QUERY_RESEND) == 0u)
                    {
                        ESP32_SendAtPosition();   /* 500ms 重发 */
                    }
                    if (rd_go)
                    {
                        PID_SetNavOverride(NAV_OVERRIDE_HEADING,
                                           g_rd_yaw_base + (float)g_rd_dir * RD_TURN_RAD,
                                           RD_BLIND_CPS);
                        g_rd_state = RD_OUT; g_rd_tick = 0; g_rd_settle = 0;
                        OLED_ShowString(1, 1, (g_rd_dir > 0) ? "RD OUT LEFT     "
                                                             : "RD OUT RIGHT    ");
                    }
                    break;
                }
                case RD_OUT:
                case RD_BACK:
                {
                    float rd_tgt = (g_rd_state == RD_OUT)
                                 ? (g_rd_yaw_base + (float)g_rd_dir * RD_TURN_RAD)
                                 : g_rd_yaw_base;
                    float rd_err = add_angle - rd_tgt;
                    if (rd_err < 0.0f) rd_err = -rd_err;
                    g_rd_tick++;
                    if (rd_err < RD_TURN_TOL) g_rd_settle++;
                    else                      g_rd_settle = 0;
                    if (g_rd_settle >= RD_TURN_SETTLE)
                    {
                        g_rd_cnt_mark = rd_avg;
                        if (g_rd_state == RD_OUT)
                        {
                            g_rd_state = RD_DIAG;
                            OLED_ShowString(1, 1, "RD DIAG         ");
                        }
                        else
                        {
                            g_rd_state = RD_THRU; g_rd_found_run = 0;
                            OLED_ShowString(1, 1, "RD THRU         ");
                        }
                        g_rd_tick = 0; g_rd_settle = 0;
                    }
                    else if (g_rd_tick >= RD_PHASE_BUDGET)
                    {
                        g_rd_state = RD_FAIL;
                    }
                    break;
                }
                case RD_DIAG:
                    g_rd_tick++;
                    if ((rd_avg - g_rd_cnt_mark) >= RD_DIAG_CNT)
                    {
                        PID_SetNavOverride(NAV_OVERRIDE_HEADING, g_rd_yaw_base, RD_BLIND_CPS);
                        g_rd_state = RD_BACK; g_rd_tick = 0; g_rd_settle = 0;
                        OLED_ShowString(1, 1, "RD BACK         ");
                    }
                    else if (g_rd_tick >= RD_PHASE_BUDGET) g_rd_state = RD_FAIL;
                    break;
                case RD_THRU:
                case RD_REJOIN:
                    g_rd_tick++;
                    if (result_BlackPoint.found) g_rd_found_run++;
                    else                         g_rd_found_run = 0;
                    if (g_rd_found_run >= RD_REACQ_TICKS)
                    {
                        PID_SetNavOverride(NAV_OVERRIDE_NONE, 0.0f, 0.0f);
                        g_rd_state = RD_DONE;   /* 重捕成功 → 回循迹(R5 软启动消 D 踢) */
#if NAVSEG_S2_REARM
                        /* R5(06-07 评审,丢段级): S胶囊②(2×r15)裸奔修复——出箱重捕边沿
                         * re-arm sm,S① 全套配方(14cps/A1拖刹/A2锁向/1.7滞回/H1封顶)复用。
                         * g_s2_active 区分两域(0x30 释放支路对 S② 关闭);里程基准从重捕点
                         * 起算(P6 锚点清基准),终点兜底 FINISH_FROM_S2_CNT 同锚。 */
                        g_s_mode = 1;
                        g_s2_active = 1;
                        g_sm_cnt_base = rd_avg;
                        g_sm_stable_run = 0;
#endif
                        OLED_ShowString(1, 1, "RD DONE         ");
                    }
                    else if (g_rd_state == RD_THRU && (rd_avg - g_rd_cnt_mark) >= RD_THRU_CNT)
                    {
                        g_rd_cnt_mark = rd_avg;
                        g_rd_state = RD_REJOIN; g_rd_tick = 0;
                        OLED_ShowString(1, 1, "RD REJOIN       ");
                    }
                    else if ((g_rd_state == RD_REJOIN && (rd_avg - g_rd_cnt_mark) >= RD_REJOIN_CNT) ||
                             g_rd_tick >= RD_PHASE_BUDGET)
                    {
                        g_rd_state = RD_FAIL;
                    }
                    break;
                case RD_FAIL:
                    /* 任一相超预算/重捕失败:解除覆盖+安全停车。终态,K1/K3 复位。 */
                    PID_SetNavOverride(NAV_OVERRIDE_NONE, 0.0f, 0.0f);
                    StopRun();
                    RGB_SetColor(RGB_COLOR_R);
                    OLED_ShowString(1, 1, "RD FAIL STOP    ");
                    break;
                case RD_DONE:
                default:
                    break;   /* 终态:本次运行不再触发(单雷达箱) */
                }
            }
#endif

            /* g_navseg 只读段游标推进(06-07 评审 v2 ⑤(A)):由既有锚点事件驱动,
             * 不引入新触发源,不夺转向/速度权;用途=遥测 sg= 上图定位+日后守卫复用。 */
            if (is_racing)
            {
                switch (g_navseg)
                {
                case NAVSEG_START:       if (g_u_turn_passed)       g_navseg = NAVSEG_U;           break;
                case NAVSEG_U:           if (g_s_mode)              g_navseg = NAVSEG_SERP1;       break;
                case NAVSEG_SERP1:       if (g_s_mode_done)         g_navseg = NAVSEG_AFTER_ARCH1; break;
                case NAVSEG_AFTER_ARCH1: if (g_rd_state != RD_OFF)  g_navseg = NAVSEG_RADAR;       break;
                case NAVSEG_RADAR:       if (g_rd_state == RD_DONE) g_navseg = NAVSEG_SERP2;       break;
                case NAVSEG_SERP2:       if (g_finish_armed)        g_navseg = NAVSEG_FINISH;      break;
                default: break;
                }
            }

#if NAVSEG_FINISH_DIST_BACKUP
            /* R2/C-4(06-07 评审,挂科级): 0x30 未部署时 P1 终点门全悬空→跑完不停冲出场地。
             * 里程兜底:已过箱(g_s2_active)且自出箱重捕 Δ≥FINISH_FROM_S2_CNT → arm finish。
             * 阈值宁小勿大(02:15 Harvey 仲裁统一:110 为防冲出红区——红区仅 ~100cm 深,
             * 2s 滚动≈120cm,过大=出界;提前停留在场内损失更小);0x30 部署后拱门2.2 先到
             * 先触发,本兜底自然退化为备份。锚=g_sm_cnt_base(re-arm 置重捕点,S②释放不改)。 */
            if (is_racing && !g_finish_armed && g_s2_active &&
                ((int32_t)((g_link_cnt_l + g_link_cnt_r) / 2) - g_sm_cnt_base) >= FINISH_FROM_S2_CNT)
            {
                g_finish_armed = 1;
                g_finish_ticks = FINISH_STOP_TICKS;
                OLED_ShowString(1, 1, "FINISH DIST 2S  ");
            }
#endif

            /* P1 终点倒计时:到 0 → 全停 */
            if (g_finish_ticks > 0u)
            {
                g_finish_ticks--;
                if (g_finish_ticks == 0u)
                {
                    StopRun();
                    RGB_SetColor(RGB_COLOR_B);
                    OLED_ShowString(1, 1, "FINISH STOP     ");
                }
            }

            // 双环 PID 控制（is_racing=0 时内部自动停车并复位 g_motor_target=0）
            PID_Control_Update();

#if OPENLOOP_TEST_ENABLE
            // 开环测试：跳过 PID 输出，直接下发固定占空比，观察编码器原始响应。
            // 开环 duty 是绝对占空比，不走 PID 调节量上限(1000)，否则 1200/1500/1800 会被压成同一个值。
            g_motor_target_l = (float)g_openloop_duty;
            g_motor_target_r = (float)g_openloop_duty;
            {
                int16_t duty = (int16_t)ClampMotorDutyFinal((float)g_openloop_duty);
#if SINGLE_BOARD_LOCAL_DRIVE
                if (g_openloop_active)
                {
                    Motor_Enable();
                    ApplyMotorDutyLocal(MOTOR_L, duty);
                    ApplyMotorDutyLocal(MOTOR_R, duty);
                }
                else
                {
                    Motor_StopAll();
                    Motor_Disable();
                }
#elif TELEMETRY_ON_USART2
                /* USART2 兼作调试口：无下板心跳(台架裸板)时停发协议帧防二进制刷屏；
                 * 下板 100Hz 主动心跳，链路接通即自动恢复发送 */
                if (g_link_alive)
                    Proto_SendMotorCmd(duty, duty, g_openloop_active);
#else
                Proto_SendMotorCmd(duty, duty, g_openloop_active);
#endif
            }
#else
            // 下发电机指令到下板执行器（带符号占空比 + 使能）
            // 顺序：先对 PID 输出限幅(约束调节量) -> 加左右死区前馈(抬到电机能动区间)
            //      -> 总量再限到下板安全上限。这样 PID 的有效调节范围完整保留，死区只是平移。
            {
                /* R3: 带滞回的死区档选择（替代旧单阈值 10cps 比较） */
                if (g_dz_hold_l) { if (speed_left  < MOTOR_HOLD_EXIT_CPS)  g_dz_hold_l = 0; }
                else             { if (speed_left  > MOTOR_HOLD_ENTER_CPS) g_dz_hold_l = 1; }
                if (g_dz_hold_r) { if (speed_right < MOTOR_HOLD_EXIT_CPS)  g_dz_hold_r = 0; }
                else             { if (speed_right > MOTOR_HOLD_ENTER_CPS) g_dz_hold_r = 1; }
                /* H1(06-06 21:17/21:19 乒乓锤实测): sm 区死区禁回 START——锁向翻边瞬间
                 * 原拖刹轮 0cps 会被 R3 判"重新起步"挂 START 990,叠加 D3b+pid 成 1560/1600
                 * 暴力踢出,把捞线甩成换边乒乓。sm 区轮子从 0 起动只用 HOLD(680/710)+pid
                 * (pivot 外轮 ~1030,19:49 成功量级);非 sm 区行为不变。 */
                /* F5: sm 域独立 HOLD 档——S 配方与直线档解耦;H1 语义保持(sm 永不回 START)。 */
                float deadzone_l = g_s_mode ? S_MODE_HOLD_DEADZONE_L
                                 : (g_dz_hold_l ? MOTOR_HOLD_DEADZONE_L : MOTOR_START_DEADZONE_L);
                float deadzone_r = g_s_mode ? S_MODE_HOLD_DEADZONE_R
                                 : (g_dz_hold_r ? MOTOR_HOLD_DEADZONE_R : MOTOR_START_DEADZONE_R);
                float cmd_l = ClampClosedLoopDuty(g_motor_target_l);
                float cmd_r = ClampClosedLoopDuty(g_motor_target_r);
                /* D3(06-06 用户报告皱褶停转): 被命令运动(cmd>EPS)却近停(<3cps)的轮,死区前馈
                 * 自适应上浮(+4/tick,~125ms 到顶,封顶 +250);恢复(>12cps)或命令归零后快退(-8/tick)。
                 * 动机:赛道皱褶处 START 档(870/990)仍拉不动,R3 选择器只在两档间切换无更高档,
                 * 速度环增量(+5/帧)爬坡太慢(19:03 实测 1.2s)。按轮独立、有界、自清;
                 * sm 深弯内轮 cmd=0 天然不踢;上限受 ClampMotorDutyFinal/下层 SAFE_MAX 双重兜底。 */
                /* D3b(06-06): 两段爬升——0→250 快(+4/tick,~125ms,应对皱褶瞬滞);
                 * 250→MOTOR_STALL_BOOST_MAX 慢(+1/tick)=脱困档(19:38/01:50 托底搁浅证据);
                 * 叠加 pid 后由 FINAL_CAP 1990 钳住,不突破本地 SAFE_MAX=2000。
                 * 搁浅根因是机械(底盘托底),本档只买"能自己蹭下来"的概率,主修在机械侧。 */
                /* F4b(06-07 03:00 实测): 触发阈 3→6——左轮 880PWM 慢磨 10→3cps 区间踢腿
                 * 未及介入即被 K2 停;放宽到 <6 提早咬合慢磨型卡滞。退出滞回 >12、帽
                 * (普通 800/sm‖deep‖NAV 100)、增长斜率均不变;sm 浅弯内轮典型 ≥8cps 不受扰。 */
                if (is_racing && cmd_l > MOTOR_CMD_EPS && speed_left < 6 && speed_left > -6) {
                    g_stall_boost_l += (g_stall_boost_l < 250.0f) ? 4.0f : 1.0f;
                    if (g_stall_boost_l > MOTOR_STALL_BOOST_MAX) g_stall_boost_l = MOTOR_STALL_BOOST_MAX;
                } else if (speed_left > 12 || !is_racing || cmd_l <= MOTOR_CMD_EPS) {
                    g_stall_boost_l -= 8.0f; if (g_stall_boost_l < 0.0f) g_stall_boost_l = 0.0f;
                }
                if (is_racing && cmd_r > MOTOR_CMD_EPS && speed_right < 6 && speed_right > -6) {
                    g_stall_boost_r += (g_stall_boost_r < 250.0f) ? 4.0f : 1.0f;
                    if (g_stall_boost_r > MOTOR_STALL_BOOST_MAX) g_stall_boost_r = MOTOR_STALL_BOOST_MAX;
                } else if (speed_right > 12 || !is_racing || cmd_r <= MOTOR_CMD_EPS) {
                    g_stall_boost_r -= 8.0f; if (g_stall_boost_r < 0.0f) g_stall_boost_r = 0.0f;
                }
                /* H1: sm 区踢腿封顶 100——pivot 外轮从 0 起动属正常工况非托底,
                 * 600 级脱困档只留给非 sm 区(凸起/搁浅);封顶后 sm 最大
                 * sent≈710+320+100=1130(对照锤峰 1560/1600)。
                 * U2(06-06 23:20 Run1): 深弯同帽——U 弯楔住后双轮泵到 sent=890/1462,
                 * 脱困瞬间弹射(yw 300ms 内 197→-43)出图;U1 内轮停转后 cmd=0 本不触发
                 * 踢腿,封顶只防楔住工况。高脱困档仅留给普通循迹段;NAV 覆盖(雷达盲走)同封 100。 */
                if (g_s_mode || PID_GetDeepTurnMode() || PID_GetNavOverride() != NAV_OVERRIDE_NONE) {
                    if (g_stall_boost_l > MOTOR_STALL_BOOST_SAFE_CAP) g_stall_boost_l = MOTOR_STALL_BOOST_SAFE_CAP;
                    if (g_stall_boost_r > MOTOR_STALL_BOOST_SAFE_CAP) g_stall_boost_r = MOTOR_STALL_BOOST_SAFE_CAP;
                }
                /* F6a(06-07 03:28 实测): 发车踢腿封顶——首个速度窗样本(≤250ms)到来前读数恒 0,
                 * boost 无脑涨到 255~340(窗口相位彩票)→ START+boost≈1200/1300 弹射起步即丢线。
                 * K1 后 500ms 内同封 SAFE_CAP=100:保留 D1 级起步助推,杀掉彩票尖峰;
                 * 真起步堵转(>500ms 仍 0cps)宽限期满后 800 档照常解锁。 */
                if (g_launch_grace > 0u) {
                    g_launch_grace--;
                    if (g_stall_boost_l > MOTOR_STALL_BOOST_SAFE_CAP) g_stall_boost_l = MOTOR_STALL_BOOST_SAFE_CAP;
                    if (g_stall_boost_r > MOTOR_STALL_BOOST_SAFE_CAP) g_stall_boost_r = MOTOR_STALL_BOOST_SAFE_CAP;
                }
                float duty_l = ApplyDeadzone(cmd_l, deadzone_l + g_stall_boost_l);
                float duty_r = ApplyDeadzone(cmd_r, deadzone_r + g_stall_boost_r);
                g_sent_motor_l = (int16_t)ClampMotorDutyFinal(duty_l);
                g_sent_motor_r = (int16_t)ClampMotorDutyFinal(duty_r);
#if SINGLE_BOARD_LOCAL_DRIVE
                /* 单板:本地直驱。is_racing 使能/失能 + 下发左右占空比；停车走安全下电。 */
                if (is_racing)
                {
                    Motor_Enable();
                    ApplyMotorDutyLocal(MOTOR_L, g_sent_motor_l);
                    ApplyMotorDutyLocal(MOTOR_R, g_sent_motor_r);
                }
                else
                {
                    Motor_StopAll();
                    Motor_Disable();
                }
#elif TELEMETRY_ON_USART2
                /* 同上：无下板心跳不发协议帧（裸板台架 USART2 只走明文遥测） */
                if (g_link_alive)
                    Proto_SendMotorCmd(g_sent_motor_l, g_sent_motor_r, is_racing);
#else
                Proto_SendMotorCmd(g_sent_motor_l, g_sent_motor_r, is_racing);
#endif
            }
#endif
        }

        {
            uint32_t now_tick = add_angle_num;
#if OLED_TELEMETRY_ENABLE
            if ((uint32_t)(now_tick - oled_last_tick) >= OLED_TELEMETRY_PERIOD_TICKS)
            {
                oled_last_tick = now_tick;
                oled_due = 1;
            }
#else
            (void)now_tick;
#endif
        }

        // ---- 板间协议接收处理（编码器反馈 + ESP32/雷达兼容） ----
#if ESP32_ON_USART2
        /* ESP32 接管 USART2：环形缓冲字节喂 A5 5A 帧解析(0x11 DECISION/0x30 ARCH_PASSED)。
         * 与旧 0xAA Proto 互斥——双解析会在对方 payload 内伪同步并反向发 ACK 污染链路。 */
        while (Uart2_BytesAvailable() > 0)
            ESP32_OnByteReceived(Uart2_ReadByteBlocking());
#else
        Proto_Process();
#endif

        // ---- 人机交互：按键启动/停止 ----
        Key_Scan_Update();
        Key_Event_t *event = Key_GetEvent();
        BDI_V = (float)g_battery_adc_value * 0.00426508726f;

#if OPENLOOP_TEST_ENABLE
        OpenLoop_HandleKeys();
#endif

        if (event != NULL)
        {
#if OPENLOOP_TEST_ENABLE
            (void)event;
#else
            switch (event->key_id)
            {
            case KEY_NONE:
                break;
            case KEY_K1:
                // K1: 启动运行
#if !SINGLE_BOARD_LOCAL_DRIVE
                /* R7(06-05 审查): 看门狗对"下板从未上电"是盲区(g_link_alive 初始 0 不武装)，
                 * 今晨 30 分钟误诊根源。无心跳拒发车，当场可见。 */
                if (!g_link_alive)
                {
                    RGB_SetColor(RGB_COLOR_R);
                    OLED_ShowString(1, 1, "NO LINK! CHK PWR");
                    break;
                }
#endif
                /* 单板:电机本地直驱，无远端心跳概念，K1 直接发车(电机在 PID 输出处使能)。 */
                is_racing = 1;
                lose_time = 0;
                BlackPoint_Finder_ResetLastPosition();
                g_yaw_zero = add_angle;     /* 航向基准清零：U 弯判定从发车起算 */
                g_u_turn_passed = 0;
                g_jc_at_u = 0;              /* S1 基线同清 */
                g_s_mode = 0;
                g_s_mode_done = 0;          /* P9: 出口门状态同清 */
                g_sm_stable_run = 0;
                g_rd_state = RD_OFF;        /* P2: 雷达段状态机复位 */
                g_rd_tick = 0; g_rd_settle = 0; g_rd_found_run = 0;
                g_arch_cool = 0;
                g_last_arch_id = 255u;
                g_finish_ticks = 0;         /* P1: 终点状态复位 */
                g_finish_armed = 0;
                g_navseg = NAVSEG_START;    /* 06-07 评审: 段游标/S②域/u里程锚同清 */
                g_s2_active = 0;
                g_u_cnt_base = 0;
                g_u3_deep_run = 0;          /* U3: deep 持续计数同清 */
                g_launch_grace = 250u;      /* F6a: 发车 500ms 踢腿封顶窗 */
                PID_SetNavOverride(NAV_OVERRIDE_NONE, 0.0f, 0.0f);
                RGB_SetColor(RGB_COLOR_G);
                OLED_ShowString(1, 1, "RUN  K1 START   ");
                break;
            case KEY_K2:
                // K2: 停止运行
                StopRun();
#if SINGLE_BOARD_LOCAL_DRIVE
                g_fan_spot_ticks = 0u;       /* C1/G1: K2 一键全停含风扇(锁存态同清) */
                g_fan_on = 0u;
                M3PWM_SetDutyCycle(0);
#endif
                RGB_SetColor(RGB_COLOR_R);
                OLED_ShowString(1, 1, "STOP K2         ");
                break;
            case KEY_K3:
                // K3: 复位丢线/位置（调试用，不启动）
                StopRun();
                BlackPoint_Finder_ResetLastPosition();
                /* R4(06-05 审查): 复位语义补全——旧 K3 清 jc 不清 yaw/u/sm(半清不一致) */
                g_yaw_zero = add_angle;
                g_u_turn_passed = 0;
                g_jc_at_u = 0;              /* S1 基线同清 */
                g_s_mode = 0;
                g_s_mode_done = 0;          /* P9: 出口门状态同清 */
                g_sm_stable_run = 0;
                g_rd_state = RD_OFF;        /* P2/P1: 雷达段+终点状态复位 */
                g_rd_tick = 0; g_rd_settle = 0; g_rd_found_run = 0;
                g_arch_cool = 0;
                g_last_arch_id = 255u;
                g_finish_ticks = 0;
                g_finish_armed = 0;
                g_navseg = NAVSEG_START;    /* 06-07 评审: 段游标/S②域/u里程锚同清 */
                g_s2_active = 0;
                g_u_cnt_base = 0;
                PID_SetNavOverride(NAV_OVERRIDE_NONE, 0.0f, 0.0f);
                OLED_ShowString(1, 1, "KEY=K3 RESET    ");
                break;
            case KEY_K4:
#if SINGLE_BOARD_LOCAL_DRIVE && FAN_KICK_DIAG_ENABLE
                /* G1(06-06 用户拍板,接替 C3-kick 诊断): K4=风扇锁存开关——
                 * 开: kick 150×200ms(实测起转配方)→回落 50 保持常转(跑圈构型,负压抓地);
                 * 关: 再按 K4 或 K2。kick 进行中忽略按键(150 暴露严格 ≤200ms 不变)。 */
                if (!g_fan_on && g_fan_spot_ticks == 0u)
                {
                    g_fan_on = 1u;
                    M3PWM_SetDutyCycleKickDiag(150);
                    g_fan_spot_ticks = 100u;     /* kick 相位 200ms,倒计时毕回落 50 保持 */
                    OLED_ShowString(1, 1, "FAN ON 150>50   ");
                }
                else if (g_fan_on && g_fan_spot_ticks == 0u)
                {
                    g_fan_on = 0u;
                    M3PWM_SetDutyCycle(0);
                    OLED_ShowString(1, 1, "FAN OFF         ");
                }
#elif SINGLE_BOARD_LOCAL_DRIVE
                /* C1: 风扇起转阶梯点动 20→35→50 循环,每按 2s 自动归零 */
                {
                    uint16_t fan_duty = g_fan_ladder[g_fan_step];
                    g_fan_step = (uint8_t)((g_fan_step + 1u) % 3u);
                    g_fan_spot_ticks = 1000u;    /* 2s @ 2ms tick */
                    M3PWM_SetDutyCycle(fan_duty);
                    if (fan_duty == 20u)      OLED_ShowString(1, 1, "FAN SPOT 20 2s  ");
                    else if (fan_duty == 35u) OLED_ShowString(1, 1, "FAN SPOT 35 2s  ");
                    else                      OLED_ShowString(1, 1, "FAN SPOT 50 2s  ");
                }
#else
                OLED_ShowString(1, 1, "KEY=K4          ");
#endif
                break;
            }
#endif
        }

#if OLED_TELEMETRY_ENABLE
        if (oled_due)
        {
            oled_due = 0;
#if OPENLOOP_TEST_ENABLE
            OpenLoop_ShowStatus();
#else
            TelemetryScreen_Update();
#endif
#if DEBUG_OUT_TELEMETRY_ENABLE && (USART3_DEBUG_ON_PB10 || TELEMETRY_ON_USART2)
            {
                char dbg[320];   /* 06-07: ar/rdir/s2 后留足遥测缓冲;ESP32_SendLog 负责极端长行拆帧 */
                /* Δ航向(°)仅显示——u 锁存已移入控制 tick(R2)，此处只读 */
                float dyaw_deg = (add_angle - g_yaw_zero) * 57.2957795f;
                if (dyaw_deg > 9999.0f) dyaw_deg = 9999.0f;
                else if (dyaw_deg < -9999.0f) dyaw_deg = -9999.0f;
                int n = snprintf(dbg, sizeof(dbg),
                    /* F1(06-06): 加 bv=电池电压×10(整数,如124=12.4V)——19:55 无串口轮 S 弯复挂,
                     * 嫌疑人之一=3小时跑量电池压降(死区/floor 满电整定,battery-debt 老账)。
                     * 最坏帧长 237+6=243 < 256 仍安全。 */
                    /* P2(06-06): 加 rd=雷达段状态(0~9,RD_OFF..RD_FAIL)。最坏帧长 243+5=248,
                     * 极端情况(全字段同时满宽)超 247 由 ESP32_SendLog 拆两帧——实际典型行
                     * 150~180B 远不触及;PC 明文模式(开关=0)无此约束。 */
                    /* 06-07 评审: 加 sg=段游标(0~6,NAVSEG_*);leader复核再加 ar/rdir/s2,
                     * 用于区分拱门锚、雷达默认方向、S②域。典型行仍低于单帧上限。 */
                    "L=%d R=%d T=%d out=%d pid=%d,%d sent=%d,%d pos=%d lost=%d deep=%d junc=%d jc=%d yw=%d u=%d sm=%d rd=%d sg=%d ar=%d rdir=%d s2=%d bv=%d el=%ld er=%ld",
                    (int)speed_left, (int)speed_right,
                    (int)PID_GetCurrentTargetSpeed(),
                    (int)g_speed_pid.last_output,
                    (int)g_motor_target_l, (int)g_motor_target_r,
                    (int)g_sent_motor_l, (int)g_sent_motor_r,
                    (int)position_get,
                    (int)PID_GetLineLostTicks(),       /* 丢线计数 */
                    (int)PID_GetDeepTurnMode(),        /* 深弯模式 */
                    (int)result_BlackPoint.is_junction,  /* 路口抑制 */
                    (int)result_BlackPoint.junction_pass_count,  /* 本次运行累计穿过路口数 */
                    (int)dyaw_deg,                     /* 发车以来累计航向变化(°) */
                    (int)g_u_turn_passed,              /* 1=已完成~180°航向变化(U弯) */
                    (int)g_s_mode,                     /* S-mode 分段降速锁存 */
                    (int)g_rd_state,                   /* P2: 雷达段状态机 */
                    (int)g_navseg,                     /* 06-07: 段游标(S②域看 sm=1 且 rd=8) */
                    (int)g_last_arch_id,                /* 06-07: 最近0x30拱门id;255=未见 */
                    (int)g_rd_dir,                      /* 06-07: 雷达绕行方向(+1左/-1右) */
                    (int)g_s2_active,                   /* 06-07: S胶囊②域标志 */
                    (int)(BDI_V * 10.0f),              /* F1: 电池电压×10(压降排查) */
                    (long)g_link_cnt_l, (long)g_link_cnt_r);  /* 下板绝对累计计数(编码器CPR标定用) */
                /* S= 尾段按 SENSOR_COUNT 循环拼接：6/7 路构建通用（修复旧版7路只发6路） */
                if (n > 0 && n < (int)sizeof(dbg))
                {
                    uint8_t si;
                    for (si = 0u; si < SENSOR_COUNT; si++)
                    {
                        int m = snprintf(dbg + n, sizeof(dbg) - (size_t)n,
                                         (si == 0u) ? " S=%d" : ",%d",
                                         (int)g_line_sensor_values[si]);
                        if (m < 0 || m >= (int)(sizeof(dbg) - (size_t)n)) { n = 0; break; }  /* 截断→放弃本帧 */
                        n += m;
                    }
                    if (n > 0 && n <= (int)sizeof(dbg) - 2)
                    {
                        dbg[n++] = '\r';
                        dbg[n++] = '\n';
#if ESP32_ON_USART2
                        /* 同一行遥测文本封 0x07 帧发 ESP32→BLE→小程序面板(一行一帧,≤243B<247) */
                        ESP32_SendLog((const uint8_t *)dbg, (uint16_t)n);
#else
                        Debug_SendBuf((const uint8_t *)dbg, (uint16_t)n);
#endif
                    }
                }
            }
#endif
        }
#endif
    }
}
