#ifndef __PATH_H__
#define __PATH_H__

#include "stm32f10x.h"

/* ========== 路段枚举 ========== */
typedef enum {
    SEG_IDLE = 0,         // 空闲/停车
    SEG_START_SEARCH,     // 起步搜索黑线
    SEG_START_STRAIGHT,   // 起步直行
    SEG_LINE_FOLLOW,      // 正常循迹
    SEG_CURVE,            // 弯道
    SEG_FINISH            // 终点停车
} PathSegment_t;

/* ========== 路径状态结构体 ========== */
typedef struct {
    PathSegment_t  current_segment;     // 当前路段
    uint16_t       seg_ticks;           // 当前路段内计时 (2ms/tick)
    uint16_t       line_stable_count;   // 连续检测到黑线的次数
    uint16_t       line_lost_count;     // 连续丢线次数
    float          current_target_speed;// 当前目标速度
} PathState_t;

/* ========== 公共接口 ========== */
void            Path_Init(void);
void            Path_StartRace(void);
void            Path_StopRace(void);
void            Path_Update(void);
PathSegment_t   Path_GetCurrentSegment(void);
float           Path_GetTargetSpeed(void);
void            Path_SetSegment(PathSegment_t seg);

#endif /* __PATH_H__ */