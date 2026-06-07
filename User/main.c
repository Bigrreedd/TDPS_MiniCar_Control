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
#define MOTOR_START_DEADZONE_L  957.0f   /* F35: 870×1.1(用户拍板"占空比加当前基础上的10%,必须加") */
#endif
#ifndef MOTOR_START_DEADZONE_R
#define MOTOR_START_DEADZONE_R  1089.0f  /* F35: 990×1.1(同上) */
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
/* F9(06-07 F8轮口头反馈"还是有点太慢一直卡住",用户授权"已有值再大10%"——明确=现值的10%
 * 非满刻度10%):主 HOLD 770/800→850/880(+80 对称,≈+10.4%/+10.0%);不对称 30 红线保持;
 * S_MODE_HOLD 730/760 不动(S 配方冻结,T=14 可达性保命)。850 距 START_L 870 仅 20——
 * R3 双档在左轮接近合并,无功能性问题(档差只影响起步踢)。
 * 看护项:①U 弯半径回归(头号,+80 后累计自弯道验证态+170)——U 内轮 sent 应仍出现 0,
 * 入口外甩/盘旋=硬回退线;②巡航自驱超 T(HOLD+floor=920 PWM 若>28cps,速度环被 floor
 * 钳住压不下)→征兆=直线蛇摆/弯道跑宽。回退链:850/880→810/840→770/800(F5)。 */
/* F17b(06-07 08:51 风机120常开首跑): 风机债实证——速度环 out 70→245 持续 windup 仍
 * avg L/R≈15<T=28(03:30 关扇同 T 仅需 out≈70~110),末段双轮失速帧再现;按 G2 预授权
 * 协议 HOLD 对称上抬(+30:120 档吸力≈100 档预估×1.44),不对称 90(F16b)不动。
 * 看护(上方 F9 警告同源):若直线出现"out 钉 70 地板 + L/R>28 慢蛇摆"=自驱超 T,回退本档。 */
#ifndef MOTOR_HOLD_DEADZONE_L
#define MOTOR_HOLD_DEADZONE_L   935.0f   /* F35: 850×1.1(用户拍板+10%) | F18b: F17b(+30)回退——其自带回退条款命中:
                                          * 09:16 盲走帧 L/R 33~36cps > T=28/22 且 out 沿
                                          * 地板下行 = F9"自驱超 T"征兆实锤(带扇 880+floor70
                                          * =950 PWM 自驱 ≈33cps,速度环压不下来→直线蛇摆助燃) */
#endif
/* F16b(06-07 07:30 用户观察+实测指纹): 稳态巡线慢性右坐——线钉在阵列左侧 pos≈15(中心=30)。
 * 51.1~52.3 五连帧 pos=15、yw=0、L=R≈30,但 pid=30~49/101~158:速度等、航向直,PID 却要
 * 恒给右轮 +≈125 PWM 才走直 → 右传动链本占空比段偏弱(编码器 L=R 排除滑胎),Ki=0 下
 * P 环用 15 格稳态误差供养该前馈=经典 P 降落;左 U 入场余量被吃半格(07:30 60° 翻边帮凶)。
 * 旧不对称 30 系 06-05 第13轮按 pivot 对称定标——早于 F10 深弯分域;deep pivot 已走
 * U_DEEP 域,HOLD 不对称不再背 pivot 包袱,可按巡航直行重标(START 120 同向佐证)。
 * 半步 +60: 880→940(不对称 30→90);预期 pos 稳态 15→约 22~28。
 * 过补偿征兆=pos 坐 >35(06-05 第11轮 +120 过补偿同症)→回 910 折中。 */
#ifndef MOTOR_HOLD_DEADZONE_R
#define MOTOR_HOLD_DEADZONE_R   1034.0f  /* F35: 940×1.1(用户拍板+10%) | F18b: 与 L 同步回退 */
#endif
/* F33(06-08 00:07-14 三组,用户拍板"你先只加占空比,试试情况,因为之前都是惯性不够被卡住"):
 * S 配方冻结解除(冻结方=用户,拍板生效)。730/760 在褶皱备用场地三轮全卡:卡点 sent 钉
 * ~900/1000 级(760+pid+帽100)推不过褶皱脊,G2 00:13:02 自然脱困耗 2.5s。+40 对称上调到
 * 770/800 = U_DEEP 同档(03:30 清洁过 U 实证档,非 sm deep pivot 已在此档健康运行)。
 * 不对称 30 保持;T=14/帽 100/A1/A2 全不动。代价预期:S 巡线底速 14→~16,弯径略宽,留观;
 * 过冲征兆=S 弧外甩丢线 → 回 750/780 折中。 */
#ifndef S_MODE_HOLD_DEADZONE_L
#define S_MODE_HOLD_DEADZONE_L  1076.0f  /* F43: 1025×1.05(用户02:5X方块区"占空比可进一步加大5%";
                                          * ⚠治G2空转/褶皱托底,非治脱轨——脱轨根因=岔口导航不确定,见日志)
                                          * | F39:932→1025 | F38:847→932 | F35:770→847 | F33:730→770 */
#endif
#ifndef S_MODE_HOLD_DEADZONE_R
#define S_MODE_HOLD_DEADZONE_R  1118.0f  /* F43: 1065×1.05(+5%,差42≈对称) ⚠R 1118>START R 1089:
                                          * 发车走 seed_start_win 锁 START 不受影响;运行域 S>START 属预期
                                          * | F39:968→1065 | F38:880→968 | F35:800→880 */
#endif
/* F10(06-07 04:29 第三组实测): U 弯回归应验——F9 850/880 把非 sm 深弯外轮一并加热
 * (pivot 外轮 sent≈880+290=1170,03:30 清洁过 U 档为 800+pid≈1090):U 中段单帧翻边
 * 乒乓(pos 5→60)+ 后 U 蛇摆丢线 + 边缘重捕连环 deep 自旋(yw +207→-306,净转 513°)。
 * 加第三死区域:非 sm 的 deep pivot 用回 770/800(03:30 验证档);直线/浅弯保持
 * 850/880 抗卡滞;S 配方 730/760 不动。选择器优先级 sm > deep > R3(START/HOLD),
 * deep 滞回 1.9/1.5 防档位抖动。边际代价:压弯起步若首帧即 deep,发车死区 770<START
 * 870(-100),由 F6a 发车窗 boost+100 部分补偿,留观。 */
/* F13(06-07 07:04 U弯实测): 编码器修复后 U 左转 deep 丢线。
 * pos=0/5 时 sentR-sentL≈327 但实测 R-L≈0cps,左内轮 790 仍拖着跑,半径变宽后翻边。
 * 只降左内轮 deep 底座 770→730;右侧 800 保持外轮扭矩,不动 PID/主HOLD/S/T。 */
/* F16a(06-07 07:30 实测): F13 证伪——底座 730(实发 750)左内轮仍 23~30cps,差速仍≈7cps,
 * U 仅转 60° 翻边;死区仿射两档律(任意 cmd>0 ≈≥24cps)下降底座追不到停转点。
 * 真修在 PID 侧:持续 deep ≥100ms 恢复内轮 coast(见 PID_Controller.c F16 注)。
 * 本底座回 770 对齐 03:30 验证锚:爬行只剩 coast 资格期前 100ms,底座差 40 影响可忽略;
 * 右深弯外轮(L 侧)扭矩回满。 */
#ifndef U_DEEP_HOLD_DEADZONE_L
#define U_DEEP_HOLD_DEADZONE_L  847.0f   /* F35: 770×1.1(用户拍板+10%) */
#endif
#ifndef U_DEEP_HOLD_DEADZONE_R
#define U_DEEP_HOLD_DEADZONE_R  880.0f   /* F35: 800×1.1(用户拍板+10%) */
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
/* F24b(06-07晚 12-agent议会Q3实证): deep 死区档退出确认锁存——逻辑层滞回,
 * 不改 770/800/850/940 任何数值(红线a)。声明在此,逻辑在死区四档选择器处。 */
#ifndef F10_DEEP_EXIT_CONFIRM_TICKS
#define F10_DEEP_EXIT_CONFIRM_TICKS 30u   /* 60ms @ 500Hz tick */
#endif
static uint8_t  g_f10_deep_latch = 0;     /* 1=死区档锁在 U_DEEP(770/800) */
static uint16_t g_f10_deep_off_run = 0;   /* deep 退出连续 tick 计数 */
/* 06-07 01:50 粗糙地图/底盘托底:只放大非 sm/deep/NAV 覆盖段脱困档。
 * S/U 深弯和雷达盲走仍封 100,防乒乓锤/弹射/盲走过冲。 */
#ifndef MOTOR_STALL_BOOST_MAX
#define MOTOR_STALL_BOOST_MAX       800.0f
#endif
#ifndef MOTOR_STALL_BOOST_SAFE_CAP
#define MOTOR_STALL_BOOST_SAFE_CAP  100.0f
#endif
/* F31(06-07 深夜 气泡议会): 全图卡死看门狗——23:04-08 三跑实锤:线在视场卡死时(found=1,
 * lost 恒 0)双 375/1s 丢线杀手永不计数,车无声全力推到人工 K2(比赛=C-2 无远程停挂科级)。
 * 议会穷举:仅有的合法长双轮 0cps 场景=发车 grace(500ms)/FINISH 倒计时/NAV 覆盖(RD 刹停
 * 与盲走自有 2~5s 相预算),全部排除后取 3s 触发=宁长勿误停(G1/G2 自然卡死实测 ~3s)。
 * 纯 StopRun,不写任何 boost/死区/占空比 → 零红线。junction 冻结期(found 强制 1,第二
 * 无声域)同被本看门狗覆盖。计数器自清(else 每 tick 清零),免入 K1/StopRun 清单。 */
#ifndef STALL_WATCHDOG_ENABLE
#define STALL_WATCHDOG_ENABLE       1       /* 默认开:纯停车兜底 */
#endif
#ifndef STALL_WD_CPS_THRESH
#define STALL_WD_CPS_THRESH         2       /* 双轮 |v| 同时 < 此值视为卡死 */
#endif
#ifndef STALL_WD_TRIGGER_TICKS
#define STALL_WD_TRIGGER_TICKS      1500u   /* 3.0s @ 2ms tick;若日后开逃生 boost(arm 1s+
                                             * 预算 1s),3s 仍留 1s 余量不抢逃生窗 */
#endif
static uint16_t g_stall_wd_run = 0;         /* 双轮停滞连续 tick 计数(自清) */
/* F32b(06-07 23:32-37 三组+议会三补丁): 播种 sm 静止发车例外。全图中 sm 只在滚动中
 * 锁存,"sm 域静止破摩"仅 SEG2 播种态存在:730/760+帽100≈990 < 静摩阈 1170/1290
 * @12.34V 必卡死(G1 23:32:53-56 实景=同构型)。滚动确认前死区直接锁 START 870/990
 * (不走 sm/deep 三目——议会:deep 抢档会落 770/800 破摩反弱);双轮均 ≥6cps 一次即
 * 锁存滚动,例外永久关闭,回 H1 语义(sm 永不回 START)。闭环退出替代固定 500ms 开环
 * 赌注;若 START+帽100 仍破不了静摩,车钉原地由 F31 看门狗 3s 收尸=正确报告非静默。
 * 只改死区档数值选择,PID 侧全程读真 g_s_mode=1(A2 锁向/i_speed/min_inner 不受扰)。
 * 比赛构型(TEST_SEGMENT=0)本块不参编,H1 红线字节级原样。
 * (滚动锁存 static 声明在 SEGTEST_SEED_DISABLE 定义后,见 TEST_SEGMENT 宏区) */
/* F32c(06-07 23:36 G3 实证,议会裁决"落码默认关待拍板"): 单轮卡死看门狗——G3 右轮
 * er 冻 82 整 3s、sent_r 863~986 有令不动、左轮打滑 el 116→151:F31 双轮门(wd_both_dead)
 * 结构上永不满足(左轮在转每 tick 自清),只能人工 K2;比赛=无声单轮坐死。单轮判据=
 * |cps_w|<2 且 sent_w ≥ 当前死区+60(排除深弯内轮 coast sent=0 的合法长零速)持续 3s
 * 且非故意停车。红队过筛:浅弯内轮典型 ≥8cps 不触;reacq grace ≪3s 凑不满计数;
 * 编码器单通道故障(轮转读零)误停=该轮已失闭环,停车反而是安全行为。n=1 样本不敢
 * 默认开,先随双轮 WD 并行采数据,拍板后翻 1。纯 StopRun 零红线,与 F31 同构。 */
#ifndef PER_WHEEL_WD_ENABLE
#define PER_WHEEL_WD_ENABLE         1       /* F33 翻开: 00:08:08 G1 直角边第二例单轮卡死
                                             * (er 冻 209,sent_r~870 有令不动,左轮 30~46cps
                                             * 在转,双轮门永不满足)→ n=2 实证,leader 拍开。
                                             * 纯 StopRun 零红线;误停一次即回 0 复议。 */
#endif
#ifndef PW_WD_SENT_MARGIN
#define PW_WD_SENT_MARGIN           60.0f   /* sent ≥ 死区+此值才视为"有令" */
#endif
#if PER_WHEEL_WD_ENABLE
static uint16_t g_pw_wd_run_l = 0;          /* 单轮停滞连续 tick 计数(自清) */
static uint16_t g_pw_wd_run_r = 0;
#endif
/* F36(06-08 01:0X 四组实证+用户策略"增加占空比利用惯性冲过褶皱地段"): sm 域托底脱困档。
 * 议会 F30b spec(红队定版)落地: 四组卡住帧 sent 970~1180 顶帽 100 推不动(机械托底,
 * 用户定性"底盘太低被挡住"),全域再加巡航占空比的代价已现身(G2/G4 轮速 26~33cps 切线
 * 直出 S 弧)——脱困档只在卡住瞬间解锁扭矩,巡航构型零污染。时序窗: 0.5s arm →
 * 1s 预算(任一轮 >12cps 即收) → F31 看门狗 3s 兜底,三层不抢窗。 */
#ifndef STALL_ESCAPE_ENABLE
#define STALL_ESCAPE_ENABLE         1       /* F36: 默认开(用户冲褶皱策略的外科手术版) */
#endif
#ifndef STALL_ESCAPE_ARM_TICKS
#define STALL_ESCAPE_ARM_TICKS      250u    /* 双轮近停持续 0.5s 才解锁(瞬态不触) */
#endif
#ifndef STALL_ESCAPE_BUDGET_TICKS
#define STALL_ESCAPE_BUDGET_TICKS   500u    /* 1s 预算,到点未脱交还看门狗 */
#endif
#ifndef STALL_ESCAPE_CAP
#define STALL_ESCAPE_CAP            400.0f  /* 红队保守档: sm 峰≈880+320+400=1600<1990 */
#endif
static uint16_t g_escape_arm_run = 0;       /* 解锁资格连续 tick 计数(自清) */
static uint16_t g_escape_budget = 0;        /* 脱困预算倒计时(>0=解锁中) */
#if SINGLE_BOARD_LOCAL_DRIVE && !FAN_KICK_DIAG_ENABLE
/* C1(06-06 用户批准): 风扇起转阶梯点动——K4 每按推进 20/35/50 档,点动 2s 自动归零;
 * K2 强停同时灭风扇。全程受 M3PWM 底层 FAN_DUTY_ABS_CAP=50 硬钳兜底(C0)。
 * 目的:实测最低起转占空比(5%≈0.6V 对 130 类有刷电机临界,只能实验定)。 */
static const uint16_t g_fan_ladder[3] = {20u, 35u, 50u};
static uint8_t  g_fan_step = 0u;
#endif
#if SINGLE_BOARD_LOCAL_DRIVE
/* G2(06-07 用户拍板): 风机定版"常开构型"——占空比就此冻结,不再作调参变量:
 * kick 150/1000×200ms(实测唯一起转配方,06-06 19:10)→保持 50/1000(=ABS_CAP 硬钳,
 * 钳内唯一实测可保持档;>50 解锁仍欠铜皮/温升实测,见 19:00 网表审查)。
 * K1 发车自动起扇(免 K4 人因漏开),StopRun 全路径灭扇(收口 RunArch点1:
 * 丢线自停/FINISH/RD_FAIL/LORA_STOP 此前残留 50 常吹)。K4 保留=台架手动开关。 */
#ifndef FAN_AUTO_ON_RACE
#define FAN_AUTO_ON_RACE        1   /* F34(06-08 00:37 用户拍板"必须打开风扇,这是死参数"):
                                     * A/B 已出结论——关风机治"吸拱"却放大"悬空":G1 00:37:18
                                     * 起 3.5s el/yw/pos 全冻仅 er 爬 60→114=右轮悬空空转铁证
                                     * (第三类卡死,编码器在转→双看门狗结构性盲区,唯一解=
                                     * 风机下压)。占空比构型不变(kick 150×200ms→回落保持),
                                     * 自此风机=死参数,任何场地不再关断。 */
#endif
#define FAN_RACE_KICK_DUTY      150u     /* 仅走 KickDiag 专用入口(独立钳 150) */
#define FAN_RACE_KICK_TICKS     100u     /* 200ms @ 2ms tick,kick 暴露上限不变 */
#define FAN_RACE_HOLD_DUTY      120u     /* G3b: 100→120(=ABS_CAP=绝对安全天花板,堵转
                                          * 二极管 2.96A/颗·XT30 10.1A 全额定内;130 起零裕
                                          * 度),台架温升+上图过测后锁死,再不动;不过测回 50 */
static uint16_t g_fan_spot_ticks = 0u;   /* >0=kick/点动进行中,2ms tick 递减 */
static uint8_t  g_fan_on = 0u;           /* G1: 风扇锁存运行态(G2 起 K1 自动置位,StopRun/K4/K2 清) */
#define FAN_TELEM_VAL  g_fan_on
#else
#define FAN_TELEM_VAL  0u
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
#define RADAR_SEGMENT_ENABLE   0   /* F28(备用场地轮): 关——①场地无雷达箱,误武装→RD_QUERY
                                    * 无应答→RD_FAIL 白停;②RD_ZONE_MIN_CNT=380 vs 圆区出口
                                    * ≈371(注释自证"裕量薄"),备用场地矩形→圆区段还差±20cm
                                    * (±8cnt)+编码器幻速±3~6,圆区尾部贴门误武装风险实在。
                                    * ⚠回真实场地(雷达箱在位)必须改回 1。 */
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

/* ===== SEG-TEST 分段测试选择器(06-07 队友提案+用户拍板) =====
 * 0=全图(比赛构型——本特性全部 #if 包裹,=0 时不参编,比赛固件字节级不变)。
 * 1..6=单段测试:K1 在标准清单后"播种"前序段锁存/里程锚(车放该段入口发车),
 * 段出口信号命中即自停。段表(边界=现役锁存,无信号段手动 K2):
 *  1 START→U出口        种子:无                        出口:u=1 自停
 *  2 U出口→S+拱门+90°右  种子:u=1(+F32a sm播种)          出口:F41 拱门0x30武装+净转向
 *                        ≥75°持续0.5s+见线 自停(sm_done 自停已撤;兜底里程 400cnt 强停)
 *  3 S①释放→方块阵出口   种子:u=1,sm_done=1,sm锚=当前    出口:F42 IMU门自停(|Δyaw|≥75°
 *                        持续0.5s+见线;F41里程门已废,用户:编码器不可靠,状态转化用IMU)
 *                        (+SEG3_SM_RECIPE: 播 sm=1 常驻,S 配方全套,P9/T3 出口门不参编)
 *  4 方块阵出口→圆区出口  种子:同3(方块/圆之间无固件边界,靠摆放位置区分) 出口:手动 K2;
 *                        勿驶入雷达段(sm锚=发车点,Δ<RD_ZONE_MIN_CNT 则 RD 不武装,
 *                        箱前深丢线将走 C-8 丢线自停——无害但勿误判)
 *  5 圆区出口→雷达段完成  种子:同3+sm锚回拨 RD_ZONE_MIN_CNT(里程门预满足,箱前深丢线
 *                        300ms 即武装)                 出口:RD_DONE 自停(RD_FAIL 本就自停)
 *  6 雷达出口→FINISH     种子:同3+rd=DONE+S②re-arm 等效(sm=1,s2=1,sm锚=当前,lt=4;
 *                        终点里程兜底 110cnt 与全图同锚) 出口:FINISH 链自停(既有)
 * 分段 PID 说明:各段速度/死区/专属行为已全部键控于锁存(T 28/25(u后)/14(sm)、死区
 * sm>deep>R3 四域、A1/A2 等 sm 专属)——播种锁存=自动获得该段行为;不另设分段增益表,
 * 待单段数据证明需要再加(防红线组合爆炸)。比赛红线:TEST_SEGMENT 必须=0,≠0 时
 * OLED 开机/发车常显 SEG-TEST n 防误烧。 */
#ifndef TEST_SEGMENT
#define TEST_SEGMENT 3   /* F41(06-08 用户拍板"下面我们先测量矩形区域"): 切方块区单测。
                          * S 弯参数全冻结(F39 态即定版,本版零改动);S 配方借给方块区
                          * (SEG3_SM_RECIPE,用户:"s弯的pid参数可以给正方形区域参考")。
                          * 3/4 两区分开测;3 区出口=IMU 门自停(F42,里程门已废)。
                          * 段2 出口同轮重定义:拱门0x30武装+90°右转自停(回测 S 区时切 2)。
                          * 比赛/全图回 0。 */
#endif
#ifndef SEGTEST_SEED_DISABLE
#define SEGTEST_SEED_DISABLE 0  /* F26c: 1=K1不播种任何锁存,u/sm 全链自然触发(合段连测,
                                 * 从起跑线摆位);0=原语义(摆上段出口,播种前段锁存单测本段)。
                                 * F27(备用场地): 回 0——车摆 S 入口直线(Y2 后),播种 u=1,
                                 * sm 由 U3 deep 签名/jc 自然锁,S①→拱门→矩形区全链实跑。
                                 * F32(06-07 23:3X 三组实证): jc 正门全程 0 次、U3 旁门必迟到
                                 * (首弧 Δ≈55~65cnt 贴 SM_DEEP_MIN_CNT=60 下界)→ 播种升级:
                                 * 本构型下 SegTest_SeedOnStart 同时播 sm=1(lt=5),见该函数。 */
#endif
/* ===== F41(06-08 用户两连拍板) =====
 * ① "经过拱门之后,上位机会让32知道…下一个90度右转之后可以停车(对于单独的s弯道区域测量)":
 *    SEG2 出口重定义=拱门 0x30 事件武装 + 此后净航向变化 ≥75° 持续 0.5s 且当刻见线 → 自停。
 *    sm_done 自停(F40)撤——车须继续过拱门。方向不写死(红线b):用 |Δ|,拱门后唯一弯=右90°等效。
 *    ⚠场地要求:拱门必须摆在全部 S 段之后、90°右转之前,两者之间不得再有弯(否则弧内可能假凑)。
 *    兜底:拱门帧丢失 → 里程 Δ≥SEG2_STOP_BACKSTOP_CNT 强停,履约"必须停止"。
 * ② "3区域过了,到达正方形的出口就停止" + "s弯的pid参数给正方形区域参考"
 *    + "速度和现在的s弯差不多,不要用低速,底盘低会卡住特别是90度":
 *    SEG3 = S 配方常驻(播 sm=1 不释放:T=14+S域死区1025/1065 → 实跑 20~45cps,与现役
 *    S 弯完全同规,惯性冲凹凸语义保留);P9/T3 出口门不参编(长直/密路口都不许放)。
 * ③ F42(用户拍板"里程来判断终点和状态不可靠!编码器有问题,不能依赖,通过imu的角度变化
 *    来进行状态转化"): SEG3 出口=IMU 门——区内全程直穿(岔口冻结直行),发车后第一个
 *    持续 90° 级净转向只能是出口右转 → |Δyaw|≥75° 持续 0.5s 且当刻见线(转正上线)
 *    → 自停;锚=g_yaw_zero(K1 标准清单清零)。F41 里程门 SEG3_EXIT_CNT 废弃删除;
 *    jc 禁用作锚不变(S区 2→4 通胀实证);方向不写死(红线b,用 |Δ|)。 */
#ifndef SEG3_SM_RECIPE
#define SEG3_SM_RECIPE 1
#endif
#if TEST_SEGMENT == 3
static uint16_t g_seg3_stop_run = 0;    /* F42: 出口 IMU 门持续窗计数(遥测 s3w=) */
#ifndef SEG3_STOP_YAW_DEG
#define SEG3_STOP_YAW_DEG        75.0f  /* 90°弯按 5/6 提前收口(同 SEG1 150/180 比例) */
#endif
#ifndef SEG3_STOP_HOLD_TICKS
#define SEG3_STOP_HOLD_TICKS     250u   /* 0.5s 持续确认,滤岔口甩头/瞬时越阈 */
#endif
#ifndef SEG3_GUARD_MAX_LOST_TICKS
#define SEG3_GUARD_MAX_LOST_TICKS 125u  /* 见线守卫:盲旋凑角不算(同 SEG1) */
#endif
#define SEG3_TELEM_FMT  " s3w=%d"       /* F42: IMU 门窗计数上遥测(甩头假武装可观测) */
#define SEG3_TELEM_ARG  , (int)g_seg3_stop_run
#else
#define SEG3_TELEM_FMT  ""              /* 非 seg3: 帧格式字节级不变 */
#define SEG3_TELEM_ARG
#endif
#if TEST_SEGMENT == 2
static uint8_t  g_seg2_arch_seen = 0;   /* F41: 拱门事件已收(首帧锁存,重发帧不刷新基准) */
static float    g_seg2_arch_yaw0 = 0.0f;/* F41: 收帧时刻航向基准(rad) */
static uint16_t g_seg2_stop_run  = 0;   /* F41: ≥75° 持续窗计数 */
#ifndef SEG2_STOP_YAW_DEG
#define SEG2_STOP_YAW_DEG        75.0f  /* 90°弯按 5/6 提前量收口(同 SEG1 150/180 比例) */
#endif
#ifndef SEG2_STOP_HOLD_TICKS
#define SEG2_STOP_HOLD_TICKS     250u   /* 0.5s 持续确认,滤弧内瞬时越阈 */
#endif
#ifndef SEG2_GUARD_MAX_LOST_TICKS
#define SEG2_GUARD_MAX_LOST_TICKS 125u  /* 同 SEG1 守卫:盲旋凑角不算 */
#endif
#ifndef SEG2_STOP_BACKSTOP_CNT
#define SEG2_STOP_BACKSTOP_CNT   400    /* ≈9.6m(自 S 入口锚),按场地实距回填 */
#endif
#endif
#if (TEST_SEGMENT == 2 && !SEGTEST_SEED_DISABLE) || (TEST_SEGMENT == 3 && SEG3_SM_RECIPE)
static uint8_t g_seed_rolling = 0;  /* F32b: 双轮均≥6cps 一次后置 1,发车 START 例外永久关闭
                                     * (机理/红队结论见 F32b 注释块,g_stall_wd_run 声明处)。
                                     * F41: seg3 播 sm 同病同治,条件随扩。 */
#endif
#ifndef SEGTEST_EXIT_DISABLE
#define SEGTEST_EXIT_DISABLE 0  /* F27: 1=本段出口不自停(跑过 sm_done 继续后链,矩形区出口
                                 * 手动 K2 收车——段3无固件出口信号);0=原出口自停语义。
                                 * F40(06-08 用户拍板"测量完对应区域就停止,必须停止"): 回 0——
                                 * SEG2 在 sm_done 当 tick StopRun(F37 下 done 由 P9 后备门
                                 * rs=3 / T3 强释 rs=2 双门保证必置位,出口必停);停车位置
                                 * 顺带=释放点直读。段3/4 仍无固件出口信号,手动 K2 不变。 */
#endif
#if TEST_SEGMENT < 0 || TEST_SEGMENT > 6
#error "TEST_SEGMENT must be 0..6"
#endif
#if TEST_SEGMENT == 0 && !RADAR_SEGMENT_ENABLE
#error "RACE BUILD GUARD(F28b): TEST_SEGMENT=0(比赛构型)必须 RADAR_SEGMENT_ENABLE=1——F28 备用场地关断禁止带进比赛固件"
#endif
#if TEST_SEGMENT == 0 && !FAN_AUTO_ON_RACE
#error "RACE BUILD GUARD(F30a): TEST_SEGMENT=0(比赛构型)必须 FAN_AUTO_ON_RACE=1——风机常开构型(下压力)是比赛定版,备用场地关断禁止带进比赛固件"
#endif
/* F37(06-08 用户拍板"先把lora对巡线的影响关闭,容易被影响"): 0=拱门 0x30 事件遥测只读
 * (ar 照报链路监测不断),不释放 sm/不武装 FINISH——备用场地拱门摆位在 S② 之前,释放即
 * 提速 25 必甩 S②(00:58 G1 实证);sm 释放走 P9 后备门(rs=3)/T3 强释。⚠回真实场地改回 1。 */
#ifndef ARCH_CONTROL_ENABLE
#define ARCH_CONTROL_ENABLE 0
#endif
#if TEST_SEGMENT == 0 && !ARCH_CONTROL_ENABLE
#error "RACE BUILD GUARD(F37): TEST_SEGMENT=0(比赛构型)必须 ARCH_CONTROL_ENABLE=1——拱门主锚释放+终点判定都在 0x30 消费块,备用场地只读降级禁止带进比赛固件"
#endif
#if TEST_SEGMENT == 1
/* F24a(06-07晚 12-agent议会): SEG1 测试自停门守卫(逻辑见出口自停处);比赛构型(=0)零字节影响 */
#ifndef SEG1_GUARD_MAX_LOST_TICKS
#define SEG1_GUARD_MAX_LOST_TICKS 125u   /* 越150°时 lost<250ms(近期见过线)才认段1完成 */
#endif
static uint8_t g_seg1_done_latch = 0;    /* 守卫后的段1完成锁存(停车态自清,免动 K1/K3) */
#endif
static uint16_t g_u3_deep_run = 0;     /* U3: u后(里程门内)deep 连续 tick 计数 */
static uint16_t g_launch_grace = 0;    /* F6a: 发车踢腿封顶窗(K1 置 250tick=500ms) */
/* F8(06-07 团队轮): sm 锁存/释放源遥测——三锁存源(jc/U3/T2)与三释放源(0x30/T3/后备)
 * 在 sm= 上不可分,OLED 字串会被后续转移覆盖,事后无法回答"这轮谁锁的/谁放的"
 * (03:04 轮释放源至今判不出)。纯遥测,零控制消费者。 */
static uint8_t  g_sm_latch_src = 0;    /* 0=未锁 1=jc自然 2=U3 deep 3=T2里程 4=R5 re-arm 5=SEG2播种(F32a) */
static uint8_t  g_sm_rel_src = 0;      /* 0=未放 1=0x30主锚 2=T3强释 3=P9后备门 */

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
    /* G3a: G2 曾在此灭扇——但本函数被 lose_time 饱和路径(越线/台架静置)每 tick
     * 连环调用,K4 台架开扇 ≤2ms 即被掐(08:2X 实测"只转~500ms")。灭扇改挂
     * racing 下降沿(主循环 Path_StopRace 处),此处还原纯停车语义。 */
    is_racing = 0;
    lose_time = 0;
    g_stall_boost_l = 0.0f;
    g_stall_boost_r = 0.0f;
}

#if TEST_SEGMENT != 0
/* SEG-TEST: K1 标准清单之后调用,把"前序段已完成"的锁存/锚点一次性播种。
 * F11 纪律:只写 K1 清单已有变量(播种=覆盖默认值),不引入新 static;
 * 段6 的 S②re-arm 逐行镜像 R5 边沿动作(main RD_REJOIN 重捕分支)。 */
static void SegTest_SeedOnStart(void)
{
#if SEGTEST_SEED_DISABLE
    return;   /* F26c: 合段连测——不播种,从 START 起 u/sm/rd 全链自然触发,只保留本段出口自停 */
#endif
#if TEST_SEGMENT >= 2
    int32_t avg = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
    g_u_turn_passed = 1;            /* U 段视为已完成(PID 经 extern 自动切 POST_U 速度档) */
    g_u_cnt_base = avg;             /* T2 强制锁里程锚=发车点(≈U出口,与全图语义一致) */
    g_navseg = NAVSEG_U;
#endif
#if TEST_SEGMENT == 2 && !SEGTEST_SEED_DISABLE
    /* F32a(06-07 23:32-37 三组实证+议会复核): 备用场地发车点≈S入口,jc 正门(入口右侧
     * 黑斑)三组 0 次,U3 旁门结构性迟到(首弧 Δ≈55~65cnt 贴 SM_DEEP_MIN_CNT=60 下界,
     * 三组锁存帧均值 61/63/63)→ 首弧永远以 T=22~25+主HOLD 裸跑直行冲出(G2/G3)。
     * 播种 sm=1 = 诚实覆盖"车已位于 S 入口"语义,S 配方(14cps/A1/A2)从首弧即上场。
     * 预核: P9 后备门 floor 120cnt>入口直线~60cnt 不早放(yaw 静默 500ms 在 S 内不可凑);
     * T3 强释 240cnt≈S①出口外余量足; jc/U3/T2 三锁存支路被 !g_s_mode 安全短路。 */
    g_s_mode = 1;
    g_sm_cnt_base = avg + 60;       /* F34: 锚=发车点+60cnt≈真 S 入口(实测首弧 Δ55~65)。
                                     * G2 00:38:57 实证:锚=发车点时 P9 后备门(floor 120)在
                                     * S① 弧间短直凑齐三重与门提前放行(avg 124,rs=3),sm 中途
                                     * 释放→T 跳 25 甩丢线——议会"S 内不可能凑齐"被数据驳回。
                                     * +60 把 P9(120)/T3(240) 恢复到以 S 入口为基的设计几何;
                                     * 主释放锚=拱门 0x30(本轮 ar=1 实收,链路场上活体✓)。 */
    g_sm_stable_run = 0;
    g_sm_latch_src = 5;             /* lt=5: SEG2 播种(区分自然锁存源 1~4) */
    g_navseg = NAVSEG_SERP1;        /* 议会: 对齐 1453 边沿语义,sg 遥测起点即 SERP1 */
#endif
#if TEST_SEGMENT >= 3
    g_s_mode_done = 1;              /* S① 视为已走完(防 jc>基线 回锁) */
    g_sm_cnt_base = avg;            /* RD 里程门/终点兜底锚=发车点 */
    g_navseg = NAVSEG_AFTER_ARCH1;
#endif
#if TEST_SEGMENT == 3 && SEG3_SM_RECIPE
    g_s_mode = 1;                   /* F41: 方块区借 S 配方(用户拍板)——T=14/S域死区/deep1.5/
                                     * MIN_INNER=0/A1/A2/F36 全套即刻上场;P9/T3 出口门本构型
                                     * 不参编(见该块 #if),sm 全程常驻,出口=IMU 门自停(F42)。
                                     * sm_done 保持上方 >=3 块的 1(双保险防回锁,无消费冲突)。 */
    g_sm_stable_run = 0;
    g_sm_latch_src = 5;             /* lt=5: 播种(与 SEG2 同义) */
#endif
#if TEST_SEGMENT == 5
    g_sm_cnt_base = avg - RD_ZONE_MIN_CNT;  /* 圆区出口发车:RD 里程门预满足 */
#endif
#if TEST_SEGMENT == 6
    g_rd_state = RD_DONE;           /* 雷达段视为已完成(终态,单箱不再触发) */
    g_s_mode = 1;                   /* S② re-arm 等效开始(镜像 R5) */
    g_s2_active = 1;
    g_sm_stable_run = 0;
    g_sm_latch_src = 4;
    g_sm_rel_src = 0;
    g_navseg = NAVSEG_SERP2;        /* S② re-arm 等效结束 */
#endif
}
#endif

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

#if TEST_SEGMENT != 0
    /* SEG-TEST 构建警示:开机常显,防误把分段测试固件当比赛固件烧场(比赛必须 =0) */
    OLED_ShowString(1, 1, "SEG-TEST MODE   ");
    OLED_ShowChar(1, 15, (char)('0' + TEST_SEGMENT));
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
                if (g_fan_spot_ticks == 0u) M3PWM_SetDutyCycle(g_fan_on ? FAN_RACE_HOLD_DUTY : 0u);
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
            if (!is_racing) {
                ESP32_ServiceStartup();
            }
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
#if SINGLE_BOARD_LOCAL_DRIVE
                /* G3a: 灭扇=racing 下降沿事件(原 G2 放 StopRun 内,被丢线饱和路径
                 * 每 tick 连环触发,台架 K4 开扇即被掐)。下降沿恰好一次;且 PID:606
                 * 直写 is_racing=0 的旁路停车也被捕获(G2 时代要等 1s 兜底 ≤250ms,
                 * 现 ≤2ms)。台架手动关扇走 K4/K2(K2 分支无条件灭扇=总开关)。 */
                g_fan_spot_ticks = 0u;
                g_fan_on = 0u;
                M3PWM_SetDutyCycle(0);
#endif
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
                g_sm_latch_src = 1;    /* F8: jc 自然锁 */
            }
#if NAVSEG_U3_DEEP_LATCH
            else if (!g_s_mode && !g_s_mode_done && is_racing && g_u_turn_passed &&
                     g_u3_deep_run >= SM_DEEP_CONFIRM_TICKS)
            {
                g_s_mode = 1;
                g_sm_cnt_base = (int32_t)((g_link_cnt_l + g_link_cnt_r) / 2);
                g_sm_stable_run = 0;
                g_sm_latch_src = 2;    /* F8: U3 deep 签名锁 */
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
                g_sm_latch_src = 3;    /* F8: T2 里程兜底锁 */
                OLED_ShowString(1, 1, "SM FORCED LATCH ");
            }
#endif

            /* ===== 0x30 拱门事件统一消费(P9 主锚 / P1 终点) =====
             * 每 tick 至多取一次;冷却窗(3s)吸收队友同事件连发(2~3帧,≥50ms 间隔)——
             * 防 sm 释放后的残帧被终点逻辑误食(空 payload 时 id 无法区分两拱门)。
             * F37(06-08 用户拍板"先把lora对巡线的影响关闭,容易被影响"): ARCH_CONTROL_ENABLE=0
             * 时本块降级为遥测只读——事件照取照记 ar(链路监测不断),但不释放 sm/不武装
             * FINISH。备用场地拱门摆位在 S② 之前,释放即提速 25 必甩 S②(00:58 G1 实证)。
             * sm 释放改走 P9 后备门(rs=3,S 段后直线自然放行,01:04 之前多轮已验活)/T3 强释。
             * ⚠回真实场地必须改回 1(主锚释放+终点判定都在此块),比赛护栏 #error 兜底。 */
            {
                uint8_t arch_id = 0;
                uint8_t arch_evt = ESP32_GetArchPassed(&arch_id);
                if (g_arch_cool > 0u) { g_arch_cool--; arch_evt = 0; }   /* 冷却期:事件吸收丢弃 */
                if (arch_evt && is_racing)
                {
                    g_last_arch_id = arch_id;
#if TEST_SEGMENT == 2
                    /* F41: SEG2 出口武装——拱门帧(任意 id,备用场地单拱门)首帧锁存,
                     * 记当刻航向为 90°右转检测基准;ARCH_CONTROL_ENABLE=0(F37)不冲突,
                     * 本支路只武装测试自停,不碰 sm/FINISH。重发帧(无冷却)不刷新基准。 */
                    if (!g_seg2_arch_seen)
                    {
                        g_seg2_arch_seen = 1u;
                        g_seg2_arch_yaw0 = add_angle;
                        OLED_ShowString(1, 1, "SEG2 ARCH ARMED ");
                    }
#endif
#if ARCH_CONTROL_ENABLE
                    /* R5(06-07 评审): S② 域(g_s2_active)对释放支路关闭——此时已过拱门2.1,
                     * 任何拱门事件只可能是拱门2.2,落到下方终点支路(id=2 或空payload+done)。 */
                    if (g_s_mode && arch_id <= 1u && !g_s2_active)
                    {
                        g_s_mode = 0;          /* P9 主锚:拱门2.1(id=1;空 payload 记 0 也认) */
                        g_s_mode_done = 1;
                        g_arch_cool = 1500u;
                        g_sm_rel_src = 1;      /* F8: 0x30 主锚释放 */
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
#endif /* ARCH_CONTROL_ENABLE */
                }
            }

            /* P9 sm 出口门(后备三重与门;主锚已并入上方统一消费)。
             * 出门即置 done,sm 本次运行不再回锁;速度由 H3 三段律自动回 25(u=1,!sm)。 */
#if TEST_SEGMENT == 3 && SEG3_SM_RECIPE
            /* F41: 方块区 S 配方常驻——P9/T3 出口门整体不参编(方块阵长直会凑满稳线窗、
             * T3 240cnt 会中途强释,任一放行都让 T 跳 25 破坏"S 参数参考"语义)。
             * 出口自停走 IMU 门(F42,段出口自停块),与释放机制无关。 */
#else
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
                        g_sm_rel_src = 2;      /* F8: T3 强制释放 */
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
                                g_sm_rel_src = 3;  /* F8: P9 后备门(稳线+yaw静默+里程) */
                            }
                        }
                    }
                }
            }
#endif /* !(TEST_SEGMENT == 3 && SEG3_SM_RECIPE) */

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
                        /* F7(06-07 红队): 进 QUERY 前排空陈旧 0x11——decision_ready 唯一消费点在本态,
                         * 上轮 500ms 重发的迟到第二答/雷达板杂帧会跨运行锁存,首 tick 误食陈方向;
                         * 排空后保证"答案晚于本次 0x03 查询"。(0x30 无此问题:1005 每 tick 无条件排空) */
                        (void)ESP32_GetDecision(NULL, NULL);
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
                        g_sm_latch_src = 4;    /* F8: R5 re-arm(S②域) */
                        g_sm_rel_src = 0;      /* 05:13: S②重新锁存后释放源重置,防复盘误读S① rs */
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

#if (TEST_SEGMENT == 1 || TEST_SEGMENT == 2 || TEST_SEGMENT == 3 || TEST_SEGMENT == 5) && !SEGTEST_EXIT_DISABLE
            /* SEG-TEST 出口自停:命中段出口锁存→StopRun(灭扇走 racing 下降沿,G3a)。
             * is_racing 门保证只触发一次;段4 无固件出口信号(手动 K2),段6 走既有
             * FINISH 链自停,均不在此列。F27: SEGTEST_EXIT_DISABLE=1 时本块整体不参编
             * (跑过本段出口继续后链,手动 K2 收车)。
             * F41: 段2 出口重定义=拱门武装+90°右转(sm_done 自停撤);段3 新增里程门出口。 */
            if (is_racing)
            {
#if TEST_SEGMENT == 1
                /* F24a(06-07晚 12-agent议会): 自停门换守卫锁存——g_u_turn_passed 的 |yaw|≥150
                 * 无符号锁存被丢线自旋假触发(18:25:03 yw=-173+er冻结,累计第6次实锤):乒乓发散
                 * →盲旋也凑满 150°。守卫=越 150° 当刻近期见过线(lost<SEG1_GUARD_MAX_LOST_TICKS)
                 * 才认段1完成;盲旋越线时 lost≈200+ 永不武装→跑到 375 丢线自停=真实失败信号。
                 * 真 U 弯过后 |Δyaw| 保持~180,重捕线后下一 tick 即武装,不漏停。方向不写死(红线b);
                 * 比赛侧 u 锁存(上方 dyaw_latch 块)与 sm/里程锚消费链零改动。 */
                {
                    float seg1_dyaw = (add_angle - g_yaw_zero) * 57.2957795f;
                    if (seg1_dyaw < 0.0f) seg1_dyaw = -seg1_dyaw;
                    if (!g_seg1_done_latch && seg1_dyaw >= U_TURN_YAW_LATCH_DEG &&
                        PID_GetLineLostTicks() < SEG1_GUARD_MAX_LOST_TICKS)
                    {
                        g_seg1_done_latch = 1u;
                    }
                }
                if (g_seg1_done_latch)      { StopRun(); RGB_SetColor(RGB_COLOR_B); OLED_ShowString(1, 1, "SEG1 DONE u=1   "); }
#elif TEST_SEGMENT == 2
                /* F41(用户拍板): 拱门 0x30 武装后,净航向变化 ≥75° 持续 0.5s 且当刻见线
                 * → 90°右转完成 → 停。持续窗滤弧内瞬时越阈;lost 守卫滤盲旋凑角(同 SEG1)。
                 * 兜底:拱门帧丢失时里程 Δ≥SEG2_STOP_BACKSTOP_CNT(锚=S入口) 强停。 */
                if (g_seg2_arch_seen)
                {
                    float seg2_dyaw = (add_angle - g_seg2_arch_yaw0) * 57.2957795f;
                    if (seg2_dyaw < 0.0f) seg2_dyaw = -seg2_dyaw;
                    if (seg2_dyaw >= SEG2_STOP_YAW_DEG &&
                        PID_GetLineLostTicks() < SEG2_GUARD_MAX_LOST_TICKS)
                    {
                        if (g_seg2_stop_run < 0xFFFFu) g_seg2_stop_run++;
                        if (g_seg2_stop_run >= SEG2_STOP_HOLD_TICKS)
                        {
                            StopRun(); RGB_SetColor(RGB_COLOR_B); OLED_ShowString(1, 1, "SEG2 DONE A+90  ");
                        }
                    }
                    else { g_seg2_stop_run = 0u; }
                }
                if (((int32_t)((g_link_cnt_l + g_link_cnt_r) / 2) - g_sm_cnt_base) >= SEG2_STOP_BACKSTOP_CNT)
                {
                    StopRun(); RGB_SetColor(RGB_COLOR_B); OLED_ShowString(1, 1, "SEG2 BACKSTOP   ");
                }
#elif TEST_SEGMENT == 3
                /* F42(用户否决 F41 里程门:"编码器有问题,不能依赖,用imu的角度变化做状态
                 * 转化"): 出口=IMU 门。方块区内全程直穿(岔口冻结直行,yw 始终≈0),发车后
                 * 第一个持续 90° 级净转向只能是出口右转;|Δyaw|≥75° 持续 0.5s 且当刻见线
                 * (=转过弯且已转正上线,匹配"末方块上方直线转正即完成"原始口径)→ 停。
                 * 持续窗滤岔口漏检甩头(瞬时摆头难以 0.5s 稳持±75°且全程见线);
                 * 盲旋凑角被 lost 守卫滤除。锚=g_yaw_zero(K1 清零,种子不动它)。 */
                {
                    float seg3_dyaw = (add_angle - g_yaw_zero) * 57.2957795f;
                    if (seg3_dyaw < 0.0f) seg3_dyaw = -seg3_dyaw;
                    if (seg3_dyaw >= SEG3_STOP_YAW_DEG &&
                        PID_GetLineLostTicks() < SEG3_GUARD_MAX_LOST_TICKS)
                    {
                        if (g_seg3_stop_run < 0xFFFFu) g_seg3_stop_run++;
                        if (g_seg3_stop_run >= SEG3_STOP_HOLD_TICKS)
                        {
                            StopRun(); RGB_SetColor(RGB_COLOR_B); OLED_ShowString(1, 1, "SEG3 DONE EXIT  ");
                        }
                    }
                    else { g_seg3_stop_run = 0u; }
                }
#elif TEST_SEGMENT == 5
                if (g_rd_state == RD_DONE)  { StopRun(); RGB_SetColor(RGB_COLOR_B); OLED_ShowString(1, 1, "SEG5 DONE rd=8  "); }
#endif
            }
#if TEST_SEGMENT == 1
            else
            {
                g_seg1_done_latch = 0u;   /* F24a: 停车态自清,下次 K1 重新起算 */
            }
#elif TEST_SEGMENT == 2
            else
            {
                g_seg2_arch_seen = 0u;    /* F41: 停车态自清,下次 K1 重新武装 */
                g_seg2_stop_run  = 0u;
            }
#elif TEST_SEGMENT == 3
            else
            {
                g_seg3_stop_run = 0u;     /* F42: 停车态自清,下次 K1 重新起算 */
            }
#endif
#endif

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
#if STALL_WATCHDOG_ENABLE
                /* F31: 全图卡死看门狗(机理/排除清单见宏定义处)。speed_* 此处已是本窗 cnt/s。 */
                {
                    uint8_t wd_both_dead =
                        (speed_left  > -STALL_WD_CPS_THRESH && speed_left  < STALL_WD_CPS_THRESH &&
                         speed_right > -STALL_WD_CPS_THRESH && speed_right < STALL_WD_CPS_THRESH) ? 1u : 0u;
                    uint8_t wd_intentional =
                        (g_launch_grace > 0u) || (g_finish_ticks > 0u) ||
                        (PID_GetNavOverride() != NAV_OVERRIDE_NONE);   /* RD 刹停/盲走自有相预算 */
                    if (is_racing && wd_both_dead && !wd_intentional)
                    {
                        if (g_stall_wd_run < 0xFFFFu) g_stall_wd_run++;
                        if (g_stall_wd_run >= STALL_WD_TRIGGER_TICKS)
                        {
                            StopRun();
                            RGB_SetColor(RGB_COLOR_R);
                            OLED_ShowString(1, 1, "STALL WD STOP   ");
                            g_stall_wd_run = 0;
                        }
                    }
                    else
                    {
                        g_stall_wd_run = 0;   /* 任一轮在动/故意停车态/未发车:每 tick 自清 */
                    }
                }
#endif
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
                /* F10: 第三域——非 sm 深弯(U 族 pivot)外轮回 770/800(03:30 清洁过 U 档);
                 * 优先级 sm > deep > R3(START/HOLD);deep 滞回 1.9/1.5 防档位抖动。 */
                /* F24b(06-07晚 议会Q3实证): deep 档退出确认 60ms——原 f10_deep 直读瞬时
                 * deep,乒乓期 deep 逐帧 0↔1 → 死区档在 HOLD(850/940)↔U_DEEP(770/800)
                 * 间跳 80/140PWM 纯扰动脉冲(R3 选择器有滞回 1482-1485,deep 档此前没有)。
                 * 进 deep 立即(U 几何需要),退 deep 连续 30tick=60ms 才切回 HOLD,
                 * 吸收量化抖动;18:23-25 三组实测 deep 翻转≥4次/轮。四档死区数值不动。 */
                {
                    uint8_t deep_now = (PID_GetDeepTurnMode() != 0) ? 1u : 0u;
                    if (!is_racing)      { g_f10_deep_latch = 0u; g_f10_deep_off_run = 0u; }
                    else if (deep_now)   { g_f10_deep_latch = 1u; g_f10_deep_off_run = 0u; }
                    else if (g_f10_deep_latch)
                    {
                        if (g_f10_deep_off_run < F10_DEEP_EXIT_CONFIRM_TICKS) { g_f10_deep_off_run++; }
                        else { g_f10_deep_latch = 0u; g_f10_deep_off_run = 0u; }
                    }
                }
                uint8_t f10_deep = g_f10_deep_latch;
#if (TEST_SEGMENT == 2 && !SEGTEST_SEED_DISABLE) || (TEST_SEGMENT == 3 && SEG3_SM_RECIPE)
                /* F32b: 播种 sm 静止发车例外(设计/红队结论见 g_stall_wd_run 声明处注释块)。
                 * 滚动确认前死区直接锁 START(凌驾 sm/deep 三目);双轮均≥6cps 一次即永久
                 * 退出,回 H1 语义。!is_racing 自清,二次发车窗重开。
                 * F41: seg3 播 sm 静止发车同病(S域 R 地板 1065<START R 1089),条件随扩。 */
                if (!is_racing) { g_seed_rolling = 0u; }
                else if (!g_seed_rolling && speed_left >= 6 && speed_right >= 6)
                {
                    g_seed_rolling = 1u;
                }
                uint8_t seed_start_win = (is_racing && !g_seed_rolling) ? 1u : 0u;
                float deadzone_l = seed_start_win ? MOTOR_START_DEADZONE_L
                                 : (g_s_mode ? S_MODE_HOLD_DEADZONE_L
                                 : (f10_deep ? U_DEEP_HOLD_DEADZONE_L
                                 : (g_dz_hold_l ? MOTOR_HOLD_DEADZONE_L : MOTOR_START_DEADZONE_L)));
                float deadzone_r = seed_start_win ? MOTOR_START_DEADZONE_R
                                 : (g_s_mode ? S_MODE_HOLD_DEADZONE_R
                                 : (f10_deep ? U_DEEP_HOLD_DEADZONE_R
                                 : (g_dz_hold_r ? MOTOR_HOLD_DEADZONE_R : MOTOR_START_DEADZONE_R)));
#else
                float deadzone_l = g_s_mode ? S_MODE_HOLD_DEADZONE_L
                                 : (f10_deep ? U_DEEP_HOLD_DEADZONE_L
                                 : (g_dz_hold_l ? MOTOR_HOLD_DEADZONE_L : MOTOR_START_DEADZONE_L));
                float deadzone_r = g_s_mode ? S_MODE_HOLD_DEADZONE_R
                                 : (f10_deep ? U_DEEP_HOLD_DEADZONE_R
                                 : (g_dz_hold_r ? MOTOR_HOLD_DEADZONE_R : MOTOR_START_DEADZONE_R));
#endif
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
                    float sm_boost_cap = MOTOR_STALL_BOOST_SAFE_CAP;
#if STALL_ESCAPE_ENABLE
                    /* F36(设计/红队结论见宏定义处): sm 域双轮托底脱困——平时帽 100 原样
                     * (H1 防锤),双轮 |v|<2cps 持续 0.5s 且线在视场(found,排除丢线推墙)
                     * 才解锁到 400,破粘任一轮 >12cps 立即收回;1s 预算到点交还看门狗。
                     * U2 弹射工况天然排除: arming 要求 g_s_mode,U 楔住是非 sm deep。 */
                    {
                        uint8_t esc_both_dead =
                            (speed_left  > -STALL_WD_CPS_THRESH && speed_left  < STALL_WD_CPS_THRESH &&
                             speed_right > -STALL_WD_CPS_THRESH && speed_right < STALL_WD_CPS_THRESH) ? 1u : 0u;
                        if (!is_racing) { g_escape_arm_run = 0; g_escape_budget = 0; }
                        else if (g_escape_budget > 0u)
                        {
                            if (speed_left > 12 || speed_right > 12) { g_escape_budget = 0; }
                            else { g_escape_budget--; sm_boost_cap = STALL_ESCAPE_CAP; }
                        }
                        else if (g_s_mode && esc_both_dead && result_BlackPoint.found &&
                                 g_launch_grace == 0u &&
                                 PID_GetNavOverride() == NAV_OVERRIDE_NONE)
                        {
                            if (++g_escape_arm_run >= STALL_ESCAPE_ARM_TICKS)
                            {
                                g_escape_arm_run = 0;
                                g_escape_budget = STALL_ESCAPE_BUDGET_TICKS;
                                OLED_ShowString(1, 1, "ESC BOOST       ");
                            }
                        }
                        else { g_escape_arm_run = 0; }
                    }
#endif
                    if (g_stall_boost_l > sm_boost_cap) g_stall_boost_l = sm_boost_cap;
                    if (g_stall_boost_r > sm_boost_cap) g_stall_boost_r = sm_boost_cap;
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
#if PER_WHEEL_WD_ENABLE
                /* F32c: 单轮卡死看门狗(默认关,机理/红队结论见宏定义处)。"有令不动"判据
                 * 用本 tick 死区+sent 终值,深弯内轮 coast(cmd=0→sent=0)天然豁免;
                 * reacq grace ≪3s 凑不满计数,无需入排除清单(议会复核)。 */
                {
                    uint8_t pw_intentional =
                        (g_launch_grace > 0u) || (g_finish_ticks > 0u) ||
                        (PID_GetNavOverride() != NAV_OVERRIDE_NONE);
                    uint8_t pw_l_jam =
                        (speed_left  > -STALL_WD_CPS_THRESH && speed_left  < STALL_WD_CPS_THRESH &&
                         (float)g_sent_motor_l >= deadzone_l + PW_WD_SENT_MARGIN) ? 1u : 0u;
                    uint8_t pw_r_jam =
                        (speed_right > -STALL_WD_CPS_THRESH && speed_right < STALL_WD_CPS_THRESH &&
                         (float)g_sent_motor_r >= deadzone_r + PW_WD_SENT_MARGIN) ? 1u : 0u;
                    if (is_racing && pw_l_jam && !pw_intentional) { if (g_pw_wd_run_l < 0xFFFFu) g_pw_wd_run_l++; }
                    else { g_pw_wd_run_l = 0; }
                    if (is_racing && pw_r_jam && !pw_intentional) { if (g_pw_wd_run_r < 0xFFFFu) g_pw_wd_run_r++; }
                    else { g_pw_wd_run_r = 0; }
                    if (g_pw_wd_run_l >= STALL_WD_TRIGGER_TICKS || g_pw_wd_run_r >= STALL_WD_TRIGGER_TICKS)
                    {
                        StopRun();
                        RGB_SetColor(RGB_COLOR_R);
                        OLED_ShowString(1, 1, "PW WD STOP      ");
                        g_pw_wd_run_l = 0;
                        g_pw_wd_run_r = 0;
                    }
                }
#endif
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
#if ESP32_ON_USART2
                if (!ESP32_HasStarted())
                {
                    if (ESP32_IsReadyToStart())
                    {
                        ESP32_SendOk();
                    }
#if ESP32_REQUIRE_READY_BEFORE_K1
                    else
                    {
                        ESP32_ServiceStartup();
                        RGB_SetColor(RGB_COLOR_R);
                        OLED_ShowString(1, 1, "WAIT ESP READY  ");
                        break;
                    }
#endif
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
                g_sm_latch_src = 0;         /* F8: 锁存/释放源遥测同清 */
                g_sm_rel_src = 0;
#if SINGLE_BOARD_LOCAL_DRIVE && FAN_KICK_DIAG_ENABLE && FAN_AUTO_ON_RACE
                /* G2: 发车自动起扇——常开构型免 K4 人因漏开。已在转(K4 预热)不重 kick;
                 * kick 进行中(K4 后 200ms 内按 K1)不打断,150 暴露 ≤200ms 不变。 */
                if (!g_fan_on && g_fan_spot_ticks == 0u)
                {
                    g_fan_on = 1u;
                    M3PWM_SetDutyCycleKickDiag(FAN_RACE_KICK_DUTY);
                    g_fan_spot_ticks = FAN_RACE_KICK_TICKS;
                }
#endif
                PID_SetNavOverride(NAV_OVERRIDE_NONE, 0.0f, 0.0f);
#if TEST_SEGMENT != 0
                SegTest_SeedOnStart();      /* SEG-TEST: 标准清单之后播种,覆盖默认值 */
                RGB_SetColor(RGB_COLOR_G);
                OLED_ShowString(1, 1, "RUN SEG-TEST    ");
                OLED_ShowChar(1, 14, (char)('0' + TEST_SEGMENT));
#else
                RGB_SetColor(RGB_COLOR_G);
                OLED_ShowString(1, 1, "RUN  K1 START   ");
#endif
                break;
            case KEY_K2:
                // K2: 停止运行
                StopRun();
#if SINGLE_BOARD_LOCAL_DRIVE
                g_fan_spot_ticks = 0u;       /* G3a: K2 无条件灭扇(台架总开关)——比赛停车
                                              * 的灭扇在 racing 下降沿,台架态(未发车)靠这里 */
                g_fan_on = 0u;
                M3PWM_SetDutyCycle(0);
#endif
                RGB_SetColor(RGB_COLOR_R);
                OLED_ShowString(1, 1, "STOP K2         ");
                break;
            case KEY_K3:
                // K3: 复位丢线/位置（调试用，不启动）
                StopRun();
#if ESP32_ON_USART2
                ESP32_ResetStartupHandshake();
#endif
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
                g_u3_deep_run = 0;          /* F8(RunArch F2): 与 K1 对称补清 */
                g_sm_latch_src = 0;         /* F8: 锁存/释放源遥测同清 */
                g_sm_rel_src = 0;
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
                    M3PWM_SetDutyCycleKickDiag(FAN_RACE_KICK_DUTY);
                    g_fan_spot_ticks = FAN_RACE_KICK_TICKS;  /* kick 相位 200ms,倒计时毕回落 50 保持 */
                    OLED_ShowString(1, 1, "FAN ON 150>120  ");
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
                    /* F8: 加 lt=锁存源(0未锁/1jc/2U3/3T2/4re-arm/5SEG2播种) rs=释放源(0未放/1主锚/2T3/3后备)
                     * ——F6b 验收与释放源之谜(03:04)机读化。最坏 +12B 由 SendLog 拆帧兜底。 */
                    /* G2: 加 fn=风机锁存态(1=kick/50常转)——常开构型下每行日志自证风机条件,
                     * 杜绝"这轮到底开没开扇"复盘歧义。+5B,dbg=320 仍裕。 */
                    /* F42: seg3 构型追加 s3w=出口IMU门持续窗计数(0=未武装;甩头假武装直读;
                     * 非 seg3 SEG3_TELEM_FMT="" 帧格式字节级不变)。+10B 最坏仍由 SendLog 拆帧兜底。 */
                    "L=%d R=%d T=%d out=%d pid=%d,%d sent=%d,%d pos=%d lost=%d deep=%d junc=%d jc=%d yw=%d u=%d sm=%d rd=%d sg=%d ar=%d rdir=%d s2=%d lt=%d rs=%d es=%d fn=%d bv=%d el=%ld er=%ld" SEG3_TELEM_FMT,
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
                    (int)g_sm_latch_src,                /* F8: sm 锁存源 */
                    (int)g_sm_rel_src,                  /* F8: sm 释放源 */
                    (int)ESP32_GetStartupState(),        /* 06-07: ESP启动握手状态(0 ACK/1 DONE/2 OK/3 RUN) */
                    (int)FAN_TELEM_VAL,                  /* G2: 风机锁存态 */
                    (int)(BDI_V * 10.0f),              /* F1: 电池电压×10(压降排查) */
                    (long)g_link_cnt_l, (long)g_link_cnt_r SEG3_TELEM_ARG);  /* 下板绝对累计计数(编码器CPR标定用) */
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
