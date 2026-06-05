#ifndef __M3PWM_H
#define __M3PWM_H

#include "stm32f10x.h"

// PWM相关宏定义
#define PWM_FREQUENCY_17KHZ    17000   // PWM频率17KHz
#define PWM_DUTY_CYCLE_50      50      // 默认占空比50%

/* ===== 风扇占空比硬上限（2026-06-05 功率审查，量纲 0-1000） =====
 * 预算依据(满电12.6V堵转模型,详见 TDPS_Background/hardware/motor_pwm_duty_limit_analysis.md
 * 06-05 增补章节)：系统约束链 SS54FSH续流二极管(~5A)< XT30接插件(15A总线)< 电池75C(63.75A)。
 * 轮子优先：双轮@SAFE_MAX2000 满电堵转 12.36A + 逻辑0.5A = 12.86A，XT30 仅余 2.14A。
 *  - ABS_CAP=50：二极管约束 84·D·(1-D)≤4A → D≤5%。底层无条件钳位，任何调用路径都出不去。
 *  - CONCURRENT_CAP=20：与轮子全工况并发安全(0.8×2.14A/84A)。轮 duty>1500 时的上限。
 * 解除/上调条件：换≥20A续流二极管 + 双层板铜皮温升实测 + 加电流采样，三者缺一不可。 */
#define FAN_DUTY_ABS_CAP         50u
#define FAN_DUTY_CONCURRENT_CAP  20u
#define FAN_WHEEL_CRUISE_DUTY    1500u  /* 双轮 duty 均低于此值才视为巡航(允许 50 档) */

// 函数声明
void M3PWM_Init(void);                    // PWM初始化函数
void M3PWM_SetDutyCycle(uint16_t duty);    // 设置PWM占空比 (0-1000)
void M3PWM_Start(void);                   // 启动PWM输出
void M3PWM_Stop(void);                    // 停止PWM输出
void M3PWM_SetFrequency(uint32_t freq);   // 设置PWM频率

uint16_t M3PWM_GetDutyCycle(void);        // 读取当前占空比设定值（0~1000）

#endif
