#include "Path.h"
#include "BlackPoint_Finder.h"
#include "PID_Controller.h"
#include "ABEncoder.h"
#include "Motor_ctr.h"

#include "stm32f10x_it.h"

/* 文件级 extern（避免函数体内重复声明） */
/* position_get 的 extern 声明已在 stm32f10x_it.h 中 */
extern BlackPointResult_t result_BlackPoint;

/* ========== 内部状态 ========== */
static PathState_t g_path;

/* 里程参数（根据实际编码器标定调整） */
#define ENCODER_TICKS_PER_CM    6.0f    /* 编码器脉冲/cm（示例值，需实测 */

/* 计数器饱和上限 */
#define COUNT_SAT               1000

/* 段距离阈值 (cm) */
#define DIST_START_SEARCH       30.0f   /* 起步寻线距离 */
#define DIST_START_STRAIGHT     20.0f   /* 起步直行稳定 */
#define DIST_U_TURN_ZONE        80.0f   /* U弯检测区 */
#define DIST_S_CURVE_ZONE       150.0f  /* S弯检测区 */
#define DIST_BOX_ZONE           250.0f  /* 方框区 */
#define DIST_CIRCLE_ZONE        350.0f  /* 圆圈区 */
#define DIST_RADAR_APPROACH     600.0f  /* 雷达区 */
#define DIST_FINISH             750.0f  /* 终点区 */

/* 速度定义 (速度环目标值，编码器增量/控制周期)
 * 用户要求整体跑慢、越低越稳：原值基础上整体下调约 45%。
 * 下限保持 ~100，过低则速度环推力不足会走走停停。 */
#define SPEED_SEARCH            140
#define SPEED_STRAIGHT          220
#define SPEED_LINE_FOLLOW       190
#define SPEED_U_TURN            120
#define SPEED_S_CURVE           140
#define SPEED_BOX               120
#define SPEED_CIRCLE            120
#define SPEED_RADAR             100
#define SPEED_FINISH            160

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

/* ========== 里程更新（由主循环每 2ms 调用一次） ========== */
/* 参数为左右编码器在 2ms 周期内的脉冲增量（即 speed_left / speed_right） */
void Path_UpdateOdometer(int32_t left_pulse_delta, int32_t right_pulse_delta)
{
    float avg_ticks = (float)(left_pulse_delta + right_pulse_delta) * 0.5f;
    g_path.total_dist_cm += avg_ticks / ENCODER_TICKS_PER_CM;
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
         * position_get 范围 [0,60]，中心约 80（传感器8）
         * 偏右半区 (>40, 即 precise>4.0) -> 可能是右赛道
         * 偏左半区 (<20, 即 precise<2.0) -> 可能是左赛道 */
        if (position_get > 40) {
            side_accum++;
        } else if (position_get < 20) {
            side_accum--;
        }
        if (side_accum > 20) {
            g_path.track_side = TRACK_LEFT;
        } else if (side_accum < -20) {
            g_path.track_side = TRACK_RIGHT;
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
    uint8_t in_curve = (g_path.curve_strength > 2000.0f) ? 1 : 0;

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