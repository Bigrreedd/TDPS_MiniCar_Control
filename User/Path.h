#ifndef __PATH_H__
#define __PATH_H__

#include "stm32f10x.h"

/* ========== 路段枚举（对应 Patio 赛道 8m 全程） ========== */
typedef enum {
    SEG_IDLE = 0,
    SEG_START_SEARCH,       // 起步寻线（含摇摆搜索）
    SEG_START_STRAIGHT,     // 起步直行稳定
    SEG_LINE_FOLLOW,        // 正常循迹
    SEG_U_TURN,             // 1.1 掉头（180° U 弯）
    SEG_S_CURVE,            // 双重 S 弯
    SEG_BOX_1,              // 三方框顶角相连区（1.3）
    SEG_QUAD_CIRCLES,       // 四圆干扰区
    SEG_RADAR_APPROACH,     // Task3 雷达箱体前减速
    SEG_FINISH_APPROACH,    // 终点前最后 U 弯 + 直线
    SEG_FINISH              // 终点停车
} PathSegment_t;

/* ========== 赛道侧枚举 ========== */
typedef enum {
    TRACK_UNKNOWN = 0,
    TRACK_LEFT,             // 左半区（第一个 U 弯顺时针）
    TRACK_RIGHT             // 右半区（第一个 U 弯逆时针）
} TrackSide_t;

/* ========== 路径状态结构体 ========== */
typedef struct {
    PathSegment_t  current_segment;
    uint16_t       seg_ticks;            // 当前段内 tick 计数 (2ms/tick)
    uint16_t       line_stable_count;    // 连续检测到黑线次数
    uint16_t       line_lost_count;      // 连续丢线次数
    float          current_target_speed; // 当前段目标速度
    /* 里程 */
    float          total_dist_cm;        // 累计行驶距离 (cm)
    float          seg_start_dist_cm;    // 进入当前段时的累计距离
    /* 赛道侧 */
    TrackSide_t    track_side;           // 左/右赛道
    /* 弯道检测 */
    float          curve_strength;       // 当前弯道强度（位置偏差方差近似）
    uint8_t        was_in_curve;         // 上次 tick 是否在弯道中
    uint8_t        seg_had_line_loss;    // 本段内是否曾经历丢线
} PathState_t;

/* ========== 公共接口 ========== */
void            Path_Init(void);
void            Path_StartRace(void);
void            Path_StopRace(void);
void            Path_Update(void);              // 在主循环控制环中调用
void            Path_UpdateOdometer(int32_t left_delta, int32_t right_delta);
PathSegment_t   Path_GetCurrentSegment(void);
float           Path_GetTargetSpeed(void);
void            Path_SetSegment(PathSegment_t seg);
TrackSide_t     Path_GetTrackSide(void);
float           Path_GetTotalDistCm(void);

#endif /* __PATH_H__ */