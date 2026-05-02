#include "Path.h"
#include "BlackPoint_Finder.h"
#include "Motor_ctr.h"
#include "PID_Controller.h"
#include <math.h>

/* ========== 内部速度常量 ========== */
#define SPEED_VERY_SLOW   100.0f
#define SPEED_SLOW        150.0f
#define SPEED_NORMAL      200.0f
#define SPEED_FAST        250.0f

/* ========== 内部状态 ========== */
static PathState_t g_path;

extern int16_t    position_get;
extern uint8_t    is_racing;
extern float      add_angle;
extern BlackPointResult_t result_BlackPoint;

/* ========== 私有辅助 ========== */
static void Path_SwitchSegment(PathSegment_t seg)
{
    g_path.current_segment    = seg;
    g_path.seg_ticks          = 0;
    g_path.line_stable_count  = 0;
    g_path.line_lost_count    = 0;
}

/* ========== 公共接口 ========== */
void Path_Init(void)
{
    g_path.current_segment     = SEG_IDLE;
    g_path.seg_ticks           = 0;
    g_path.line_stable_count   = 0;
    g_path.line_lost_count     = 0;
    g_path.current_target_speed = 0.0f;
}

void Path_StartRace(void)
{
    Path_SwitchSegment(SEG_START_SEARCH);
}

void Path_StopRace(void)
{
    Path_SwitchSegment(SEG_IDLE);
    is_racing = 0;
    Motor_Disable();
}

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
    Path_SwitchSegment(seg);
}

/* ========== 核心状态机（在 PID_Control_Update 中调用，500Hz） ========== */
void Path_Update(void)
{
    g_path.seg_ticks++;

    /* 全局丢线保护 */
    if (result_BlackPoint.found) {
        g_path.line_stable_count++;
        g_path.line_lost_count = 0;
    } else {
        g_path.line_lost_count++;
        g_path.line_stable_count = 0;
    }

    /* 丢线超时 → 停车 */
    if (g_path.line_lost_count > 1000 && g_path.current_segment != SEG_IDLE) {
        Path_StopRace();
        return;
    }

    switch (g_path.current_segment) {

    case SEG_IDLE:
        g_path.current_target_speed = 0.0f;
        break;

    case SEG_START_SEARCH:
        g_path.current_target_speed = SPEED_VERY_SLOW;
        /* 稳定找到黑线 → 进入直行 */
        if (g_path.line_stable_count >= 5) {
            Path_SwitchSegment(SEG_START_STRAIGHT);
        }
        /* 10秒超时 → 停车 */
        else if (g_path.seg_ticks > 5000) {
            Path_StopRace();
        }
        break;

    case SEG_START_STRAIGHT:
        g_path.current_target_speed = SPEED_SLOW;
        /* 3秒后进入正常循迹 */
        if (g_path.seg_ticks > 1500) {
            Path_SwitchSegment(SEG_LINE_FOLLOW);
        }
        break;

    case SEG_LINE_FOLLOW:
        g_path.current_target_speed = SPEED_NORMAL;
        break;

    case SEG_CURVE:
        g_path.current_target_speed = SPEED_SLOW;
        /* 弯道结束后回到循迹 */
        if (g_path.seg_ticks > 2000) {
            Path_SwitchSegment(SEG_LINE_FOLLOW);
        }
        break;

    case SEG_FINISH:
        g_path.current_target_speed = 0.0f;
        is_racing = 0;
        Motor_Disable();
        break;

    default:
        Path_SwitchSegment(SEG_IDLE);
        break;
    }
}