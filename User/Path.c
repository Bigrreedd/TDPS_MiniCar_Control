#include "Path.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "ABEncoder.h"
#include "Motor_ctr.h"
#include <math.h>

#include "stm32f10x_it.h"

/* 文件级 extern（避免函数体内重复声明） */
/* position_get 的 extern 声明已在 stm32f10x_it.h 中 */
extern BlackPointResult_t result_BlackPoint;

/* ========== 内部状态 ========== */
static PathState_t g_path;

/* 里程参数（根据实际编码器标定调整） */
#define ENCODER_TICKS_PER_CM    6.0f    /* 编码器脉冲/cm（占位值，需实测 Task#6） */

/* 计数器饱和上限 */
#define COUNT_SAT               1000

/* 段距离阈值 (cm) - 基于 OCR 实测赛道尺寸重算
 * 赛道布局（累积距离）：
 *   Start(0) → 1.1入口(165) → 1.1出口(615) → 1.2拱门(660~700)
 *   → 1.3三方框(730~880) → 1.4雷达箱(980~1170) → Finish(1270)
 */
#define DIST_START_SEARCH       30.0f    /* 起步寻线距离（保持） */
#define DIST_START_STRAIGHT     20.0f    /* 起步直行稳定（保持） */
#define DIST_U_TURN_ZONE        165.0f   /* 1.1 U弯入口：165cm */
#define DIST_S_CURVE_ZONE       660.0f   /* 1.2 拱门/S弯：660cm（U弯出口+45cm） */
#define DIST_BOX_ZONE           730.0f   /* 1.3 三方框入口：730cm */
#define DIST_CIRCLE_ZONE        880.0f   /* 四圆区（三方框出口，实际位置待确认） */
#define DIST_RADAR_APPROACH     980.0f   /* 1.4 雷达箱入口：980cm（估算，待实测调整） */
#define DIST_FINISH            1270.0f   /* 终点区：1270cm */

/* 速度定义（速度环目标值，单位=编码器计数/秒，与 main.c 速度反馈同量纲）
 * 旧版单位是“计数/控制周期”，反馈改为“计数/秒”后这些数才与反馈可比。
 * 实测低速约数十计数/秒，设定值取数十~百余；实际车速由 MOTOR_DUTY_HARD_CAP 钳住，需上车微调。 */
#define SPEED_SEARCH            90
#define SPEED_STRAIGHT          140
#define SPEED_LINE_FOLLOW       115
#define SPEED_U_TURN            80
#define SPEED_S_CURVE           90
#define SPEED_BOX               80
#define SPEED_CIRCLE            80
#define SPEED_RADAR             80
#define SPEED_FINISH            100

/* 丢线/稳线计数阈值 */
#define LINE_LOST_THRESHOLD     30
#define LINE_STABLE_THRESHOLD   10

/* ========== 弯道检测 / 赛道侧检测 内部状态（需在 Path_StartRace 中复位） ========== */
static float pos_history[8];
static uint8_t curve_idx = 0;
static int16_t side_accum = 0;

static void ResetCurveDetector(void)
{
    for (uint8_t i = 0; i < 8; i++) pos_history[i] = 0.0f;
    curve_idx = 0;
}

/* ========== 初始化 ========== */
void Path_Init(void)
{
    g_path.current_segment      = SEG_IDLE;
    g_path.seg_ticks            = 0;
    g_path.line_stable_count    = 0;
    g_path.line_lost_count      = 0;
    g_path.current_target_speed = 0;
    g_path.total_dist_cm        = 0.0f;
    g_path.seg_start_dist_cm    = 0.0f;
    g_path.track_side           = TRACK_UNKNOWN;
    g_path.curve_strength       = 0.0f;
    g_path.was_in_curve         = 0;
    g_path.seg_had_line_loss    = 0;
}

void Path_StartRace(void)
{
    Path_Init();
    ResetCurveDetector();
    side_accum = 0;
    g_path.current_segment      = SEG_START_SEARCH;
    g_path.current_target_speed = SPEED_SEARCH;
    g_path.seg_start_dist_cm    = 0.0f;
}

void Path_StopRace(void)
{
    g_path.current_segment      = SEG_IDLE;
    g_path.current_target_speed = 0;
}

/* ========== 里程更新（由 main.c OnEncFeedback 每收到一帧 ENC_FEEDBACK 调用一次） ========== */
/* 参数为左右编码器自上一帧以来的原始计数增量(单位=计数)，每个增量恰好累积一次 */
void Path_UpdateOdometer(int32_t left_pulse_delta, int32_t right_pulse_delta)
{
    /* 路径长度(非位移)累加: 两轮各走的弧长平均。
     * 原 (L+R)/2 在180° U弯两轮反向→均值≈0,恰在U弯处漏计里程。
     * 改用 (|L|+|R|)/2,保证U弯时里程正常累积(两轮路径长之和/2)。
     * 编码器增量可正可负(CLOSED_LOOP_REVERSE_ENABLE=1允许反转),取绝对值。 */
    float path_length = (fabsf((float)left_pulse_delta) + fabsf((float)right_pulse_delta)) * 0.5f;
    g_path.total_dist_cm += path_length / ENCODER_TICKS_PER_CM;
}

/* ========== 弯道检测辅助 ========== */
static float CalcCurveStrength(void)
{
    /* 使用 position_get 的方差近似弯道强度 */
    float mean = 0.0f;
    float var  = 0.0f;

    pos_history[curve_idx] = (float)position_get;
    curve_idx = (curve_idx + 1) & 0x07;

    for (uint8_t i = 0; i < 8; i++) {
        mean += pos_history[i];
    }
    mean /= 8.0f;
    for (uint8_t i = 0; i < 8; i++) {
        float d = pos_history[i] - mean;
        var += d * d;
    }
    var /= 8.0f;
    return var;
}

/* ========== 赛道侧检测 ========== */
static void DetectTrackSide(void)
{
    if (g_path.track_side == TRACK_UNKNOWN) {
        /* 需要连续多次检测到大幅偏移才判定赛道侧，避免噪声误判
         * position_get 范围 [0,50]（6路传感器，索引0-5）
         * 黑线偏右 (>40, 即 precise>4.0) -> 右赛道（车在左侧，线在右侧）
         * 黑线偏左 (<10, 即 precise<1.0) -> 左赛道（车在右侧，线在左侧） */
        if (position_get > 40) {
            side_accum++;  /* 黑线持续偏右 → 判定为右赛道 */
        } else if (position_get < 10) {
            side_accum--;  /* 黑线持续偏左 → 判定为左赛道 */
        }
        if (side_accum > 20) {
            g_path.track_side = TRACK_RIGHT;  /* 修复：side_accum++ 对应右赛道 */
        } else if (side_accum < -20) {
            g_path.track_side = TRACK_LEFT;   /* 修复：side_accum-- 对应左赛道 */
        }
    }
}

/* ========== 段转移辅助 ========== */
static void TransitionTo(PathSegment_t seg, uint16_t speed)
{
    g_path.current_segment      = seg;
    g_path.current_target_speed = speed;
    g_path.seg_ticks            = 0;
    g_path.seg_start_dist_cm    = g_path.total_dist_cm;
    g_path.seg_had_line_loss    = 0;
}

/* ========== 路径状态机主更新（主循环 2ms 调用一次） ========== */
void Path_Update(void)
{
    if (g_path.current_segment == SEG_IDLE || g_path.current_segment == SEG_FINISH) {
        return;
    }

    g_path.seg_ticks++;
    float seg_dist = g_path.total_dist_cm - g_path.seg_start_dist_cm;

    /* 弯道检测 */
    g_path.curve_strength = CalcCurveStrength();
    /* position_get∈[0,60](质心×10), 8样本方差上限≈900(4@0,4@60)。
     * 旧阈值2000永远不达→SEG_U_TURN/S_CURVE不可达。降到700:
     * 真弯道(pos在0-10或50-60震荡)方差≈625-900→触发;直线(pos≈25±5)方差≈25→不触发。
     * 关键:路口冻结 pos=25 无跳变→方差≈0,不会误触发;仅真弯道触发。 */
    uint8_t in_curve = (g_path.curve_strength > 700.0f) ? 1 : 0;

    /* 线跟踪状态 */
    if (result_BlackPoint.found) {
        if (g_path.line_lost_count > 0) g_path.line_lost_count--;
        if (g_path.line_stable_count < COUNT_SAT) g_path.line_stable_count++;
    } else {
        if (g_path.line_lost_count < COUNT_SAT) g_path.line_lost_count++;
        if (g_path.line_stable_count > 0) g_path.line_stable_count--;
        if (g_path.line_lost_count > LINE_LOST_THRESHOLD) g_path.seg_had_line_loss = 1;
    }

    /* 赛道侧检测 */
    DetectTrackSide();

    /* ---- 状态机 ---- */
    switch (g_path.current_segment) {

    case SEG_START_SEARCH:
        if (g_path.line_stable_count >= LINE_STABLE_THRESHOLD) {
            TransitionTo(SEG_START_STRAIGHT, SPEED_STRAIGHT);
        } else if (seg_dist > DIST_START_SEARCH) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_START_STRAIGHT:
        if (seg_dist > DIST_START_STRAIGHT) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_LINE_FOLLOW:
        /* 依据里程判断是否进入特殊路段 */
        if (g_path.total_dist_cm > DIST_U_TURN_ZONE &&
            g_path.total_dist_cm < DIST_U_TURN_ZONE + 60.0f &&
            in_curve && !g_path.was_in_curve) {
            TransitionTo(SEG_U_TURN, SPEED_U_TURN);
        } else if (g_path.total_dist_cm > DIST_S_CURVE_ZONE &&
                   g_path.total_dist_cm < DIST_S_CURVE_ZONE + 100.0f &&
                   in_curve) {
            TransitionTo(SEG_S_CURVE, SPEED_S_CURVE);
        } else if (g_path.total_dist_cm > DIST_BOX_ZONE &&
                   g_path.total_dist_cm < DIST_BOX_ZONE + 80.0f &&
                   g_path.line_lost_count > LINE_LOST_THRESHOLD) {
            TransitionTo(SEG_BOX_1, SPEED_BOX);
        } else if (g_path.total_dist_cm > DIST_CIRCLE_ZONE &&
                   g_path.total_dist_cm < DIST_CIRCLE_ZONE + 80.0f) {
            TransitionTo(SEG_QUAD_CIRCLES, SPEED_CIRCLE);
        } else if (g_path.total_dist_cm > DIST_RADAR_APPROACH) {
            TransitionTo(SEG_RADAR_APPROACH, SPEED_RADAR);
        }
        break;

    case SEG_U_TURN:
        /* U弯: 段内曾丢线且重新找回线 → 回到循迹 */
        if (g_path.seg_had_line_loss &&
            g_path.line_stable_count >= LINE_STABLE_THRESHOLD) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        } else if (seg_dist > 100.0f) {
            /* 安全兜底 */
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_S_CURVE:
        if (!in_curve && g_path.line_stable_count >= LINE_STABLE_THRESHOLD) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        } else if (seg_dist > 150.0f) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_BOX_1:
        if (g_path.line_stable_count >= LINE_STABLE_THRESHOLD && !in_curve) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        } else if (seg_dist > 120.0f) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_QUAD_CIRCLES:
        /* 圆圈区: 连续循迹 + 距离判断退出 */
        if (seg_dist > 120.0f && g_path.line_stable_count >= LINE_STABLE_THRESHOLD) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        } else if (seg_dist > 200.0f) {
            TransitionTo(SEG_LINE_FOLLOW, SPEED_LINE_FOLLOW);
        }
        break;

    case SEG_RADAR_APPROACH:
        if (g_path.total_dist_cm > DIST_FINISH) {
            TransitionTo(SEG_FINISH_APPROACH, SPEED_FINISH);
        }
        break;

    case SEG_FINISH_APPROACH:
        if (seg_dist > 60.0f || g_path.total_dist_cm > DIST_FINISH + 60.0f) {
            TransitionTo(SEG_FINISH, 0);
        }
        break;

    case SEG_FINISH:
    case SEG_IDLE:
    default:
        break;
    }

    g_path.was_in_curve = in_curve;
}

/* ========== 公共查询接口 ========== */
PathSegment_t Path_GetCurrentSegment(void)
{
    return g_path.current_segment;
}

float Path_GetTargetSpeed(void)
{
    return g_path.current_target_speed;
}

void Path_SetSegment(PathSegment_t seg)
{
    uint16_t speed = SPEED_LINE_FOLLOW;
    switch (seg) {
    case SEG_START_SEARCH:   speed = SPEED_SEARCH;    break;
    case SEG_START_STRAIGHT: speed = SPEED_STRAIGHT;  break;
    case SEG_U_TURN:         speed = SPEED_U_TURN;    break;
    case SEG_S_CURVE:        speed = SPEED_S_CURVE;   break;
    case SEG_BOX_1:          speed = SPEED_BOX;       break;
    case SEG_QUAD_CIRCLES:   speed = SPEED_CIRCLE;    break;
    case SEG_RADAR_APPROACH: speed = SPEED_RADAR;     break;
    case SEG_FINISH_APPROACH: speed = SPEED_FINISH;   break;
    case SEG_FINISH:         speed = 0;               break;
    case SEG_IDLE:           speed = 0;               break;
    default:                 speed = SPEED_LINE_FOLLOW; break;
    }
    TransitionTo(seg, speed);
}

TrackSide_t Path_GetTrackSide(void)
{
    return g_path.track_side;
}

float Path_GetTotalDistCm(void)
{
    return g_path.total_dist_cm;
}